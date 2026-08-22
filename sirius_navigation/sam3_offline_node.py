#!/usr/bin/env python3
"""
SAM3 Offline ROS2 Inference Node
Subscribes to recorded stereo images (/camera/stereo_sbs/compressed) from rosbag,
computes SGM stereo depth, runs SAM3 segmentation, and publishes semantic 3D PointClouds
(/sam3/full_cloud_semantic, /sam3/obstacles, /sam3/full_cloud) for RTAB-Map and 2D mapping.
"""

import sys
import os
import time
import json
import struct
import traceback
import numpy as np
import cv2
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import String, Header
import sensor_msgs_py.point_cloud2 as pc2

try:
    import torch
    HAS_TORCH = True
except ImportError:
    torch = None
    HAS_TORCH = False

# ── Dynamic Import of SAM3 and SGM modules from sam3_zed_server ──
POSSIBLE_SERVER_PATHS = [
    os.path.expanduser("~/sam3_zed_server"),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../sam3_zed_server")),
]

SAM3_SERVER_PATH = None
for p in POSSIBLE_SERVER_PATHS:
    if os.path.isdir(p):
        SAM3_SERVER_PATH = p
        if p not in sys.path:
            sys.path.insert(0, p)
        sam3_repo = os.path.join(p, "sam3_repo_linux")
        if os.path.isdir(sam3_repo) and sam3_repo not in sys.path:
            sys.path.insert(0, sam3_repo)
        break

try:
    from sam3.model_builder import build_sam3_image_model
    from sam3.model.sam3_image_processor import Sam3Processor
    HAS_SAM3 = True
except Exception as e:
    HAS_SAM3 = False
    print(f"[SAM3 Offline Node] Warning: SAM3 import failed: {e}")

try:
    from custom_sgm import CustomSGM
    HAS_SGM = True
except Exception as e:
    HAS_SGM = False
    print(f"[SAM3 Offline Node] Warning: CustomSGM import failed: {e}")


class SAM3OfflineNode(Node):
    def __init__(self):
        super().__init__('sam3_offline_node')

        # Declare parameters
        self.declare_parameter('prompt', 'grass, tactile paving, roadway, sidewalk')
        self.declare_parameter('threshold', 0.5)
        self.declare_parameter('downsample', 4)
        self.declare_parameter('max_depth_m', 15.0)
        self.declare_parameter('frame_id', 'sirius3/zed_camera_link')
        self.declare_parameter('image_topic', '/camera/stereo_sbs/compressed')
        self.declare_parameter('params_topic', '/camera/stereo_params')
        self.declare_parameter('checkpoint_path', os.path.expanduser('~/sam3_zed_server/sam3.pt'))
        self.declare_parameter('min_interval_sec', 0.1) # Max 10 Hz inference to prevent lag

        self.prompt = self.get_parameter('prompt').get_parameter_value().string_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        self.downsample = self.get_parameter('downsample').get_parameter_value().integer_value
        self.max_depth_m = self.get_parameter('max_depth_m').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.params_topic = self.get_parameter('params_topic').get_parameter_value().string_value
        self.ckpt_path = self.get_parameter('checkpoint_path').get_parameter_value().string_value
        self.min_interval = self.get_parameter('min_interval_sec').get_parameter_value().double_value

        self.last_proc_time = 0.0
        self.device = "cuda" if (HAS_TORCH and torch.cuda.is_available()) else "cpu"

        # Default camera intrinsics (ZED 2 / Unity 110 deg FOV)
        self.camera_params = {
            'fx': 448.14, 'fy': 448.14, 'cx': 640.0, 'cy': 360.0, 'baseline': 0.12
        }

        # Semantic class mappings
        self.class_colors = {
            "grass": [0, 255, 0],
            "tactile paving": [255, 255, 0],
            "roadway": [0, 0, 255],
            "sidewalk": [128, 128, 128]
        }
        self.class_ids = {
            "unknown": 0,
            "wall": 1,
            "floor": 2,
            "grass": 3,
            "tactile paving": 4,
            "roadway": 5,
            "sidewalk": 6,
        }

        # Initialize SGM
        if HAS_SGM:
            self.sgm = CustomSGM(temporal_alpha=0.6)
        else:
            self.sgm = None

        # Initialize SAM3
        self.model = None
        self.processor = None
        self._init_sam3()

        # Publishers
        self.pub_full_cloud = self.create_publisher(PointCloud2, '/sam3/full_cloud', 10)
        self.pub_full_cloud_semantic = self.create_publisher(PointCloud2, '/sam3/full_cloud_semantic', 10)
        self.pub_obstacles = self.create_publisher(PointCloud2, '/sam3/obstacles', 10)
        self.pub_background = self.create_publisher(PointCloud2, '/sam3/background', 10)

        # Latched publisher for class colors
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.pub_class_colors = self.create_publisher(String, '/sam3/class_colors', qos_latched)
        self._publish_class_colors()

        # Subscribers
        self.sub_image = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self._image_callback,
            10
        )
        self.sub_params = self.create_subscription(
            String,
            self.params_topic,
            self._params_callback,
            10
        )

        self.get_logger().info("SAM3 Offline Node initialized.")
        self.get_logger().info(f"Device: {self.device} | Subscribing: {self.image_topic}")

    def _publish_class_colors(self):
        msg = String()
        msg.data = json.dumps(self.class_colors)
        self.pub_class_colors.publish(msg)

    def _init_sam3(self):
        if not HAS_SAM3:
            self.get_logger().error("SAM3 library not available. Pointcloud will be generated without SAM3 masks.")
            return

        if not os.path.isfile(self.ckpt_path):
            self.get_logger().warn(f"Checkpoint not found at {self.ckpt_path}. Running depth-only mode.")
            return

        try:
            self.get_logger().info(f"Loading SAM3 model from {self.ckpt_path} onto {self.device}...")
            self.model = build_sam3_image_model(
                checkpoint_path=self.ckpt_path,
                device=self.device,
                eval_mode=True
            )
            self.processor = Sam3Processor(self.model, confidence_threshold=self.threshold)
            self.get_logger().info("SAM3 model loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load SAM3 model: {e}")
            traceback.print_exc()

    def _params_callback(self, msg: String):
        try:
            p = json.loads(msg.data)
            for k in ['fx', 'fy', 'cx', 'cy', 'baseline']:
                if k in p:
                    self.camera_params[k] = float(p[k])
        except Exception as e:
            self.get_logger().warn(f"Failed to parse camera params: {e}")

    def _image_callback(self, msg: CompressedImage):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_proc_time < self.min_interval:
            return
        self.last_proc_time = now_sec

        try:
            # Decode JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img_bgr is None:
                return

            h, w = img_bgr.shape[:2]

            # Split Side-by-Side stereo (Left: [:, :w//2], Right: [:, w//2:])
            if w > h:
                w_half = w // 2
                left_bgr = img_bgr[:, :w_half]
                right_bgr = img_bgr[:, w_half:]
            else:
                left_bgr = img_bgr
                right_bgr = img_bgr

            left_rgb = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2RGB)
            right_rgb = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2RGB)

            # 1. Stereo Depth Computation (SGM)
            if self.sgm is not None:
                depth_ds, disp = self.sgm.compute(
                    left_rgb, right_rgb,
                    downsample=self.downsample,
                    camera_params=self.camera_params
                )
            else:
                depth_ds = None

            if depth_ds is None or depth_ds.size == 0:
                return

            # 2. SAM3 Semantic Segmentation Inference
            h_ds, w_ds = depth_ds.shape
            left_ds = cv2.resize(left_rgb, (w_ds, h_ds), interpolation=cv2.INTER_AREA)

            mask_2d = np.zeros((h_ds, w_ds), dtype=np.uint8)
            semantic_id_2d = np.zeros((h_ds, w_ds), dtype=np.uint16)
            color_2d = left_ds.copy()

            if self.model is not None and self.processor is not None and self.prompt:
                pil_img = Image.fromarray(left_ds)
                prompts = [p.strip() for p in self.prompt.split(",") if p.strip()]

                with torch.no_grad():
                    state = self.processor.set_image(pil_img)
                    for pr in prompts:
                        cid = self.class_ids.get(pr, 0)
                        ccolor = self.class_colors.get(pr, [255, 255, 255])
                        
                        out = self.processor.extract_masks(state, text_prompt=pr, threshold=self.threshold)
                        if out and "masks" in out and len(out["masks"]) > 0:
                            for m in out["masks"]:
                                m_np = (m.squeeze().cpu().numpy() > 0.5).astype(bool)
                                if m_np.shape != (h_ds, w_ds):
                                    m_np = cv2.resize(m_np.astype(np.uint8), (w_ds, h_ds), interpolation=cv2.INTER_NEAREST).astype(bool)
                                
                                mask_2d[m_np] = 1
                                semantic_id_2d[m_np] = cid
                                color_2d[m_np] = ccolor

            # 3. Project Depth to 3D PointCloud
            fx = self.camera_params['fx'] / self.downsample
            fy = self.camera_params['fy'] / self.downsample
            cx = self.camera_params['cx'] / self.downsample
            cy = self.camera_params['cy'] / self.downsample

            u_grid, v_grid = np.meshgrid(np.arange(w_ds), np.arange(h_ds))
            valid = (depth_ds > 0.3) & (depth_ds < self.max_depth_m) & (~np.isnan(depth_ds))

            if not np.any(valid):
                return

            z_pts = depth_ds[valid].astype(np.float32)
            x_pts = ((u_grid[valid] - cx) * z_pts / fx).astype(np.float32)
            y_pts = ((v_grid[valid] - cy) * z_pts / fy).astype(np.float32)

            r_pts = color_2d[:, :, 0][valid]
            g_pts = color_2d[:, :, 1][valid]
            b_pts = color_2d[:, :, 2][valid]

            # Pack RGB into float32 for PointCloud2
            rgb_packed = np.zeros(len(x_pts), dtype=np.uint32)
            rgb_packed |= (r_pts.astype(np.uint32) << 16)
            rgb_packed |= (g_pts.astype(np.uint32) << 8)
            rgb_packed |= b_pts.astype(np.uint32)
            rgb_float = rgb_packed.view(np.float32)

            sem_pts = semantic_id_2d[valid].astype(np.uint16)
            mask_pts = mask_2d[valid].astype(np.uint8)

            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = self.frame_id

            # 4. Construct PointCloud2 messages
            # (A) Full Semantic Cloud: x, y, z, rgb, semantic_id, is_masked
            semantic_cloud_data = np.empty(len(x_pts), dtype=[
                ('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'f4'),
                ('semantic_id', 'u2'), ('is_masked', 'u1')
            ])
            semantic_cloud_data['x'] = x_pts
            semantic_cloud_data['y'] = y_pts
            semantic_cloud_data['z'] = z_pts
            semantic_cloud_data['rgb'] = rgb_float
            semantic_cloud_data['semantic_id'] = sem_pts
            semantic_cloud_data['is_masked'] = mask_pts

            fields_semantic = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='semantic_id', offset=16, datatype=PointField.UINT16, count=1),
                PointField(name='is_masked', offset=18, datatype=PointField.UINT8, count=1),
            ]
            cloud_semantic_msg = pc2.create_cloud(header, fields_semantic, semantic_cloud_data)
            self.pub_full_cloud_semantic.publish(cloud_semantic_msg)

            # (B) Standard XYZRGB Cloud for RTAB-Map
            fields_xyzrgb = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            xyzrgb_data = np.empty(len(x_pts), dtype=[
                ('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'f4')
            ])
            xyzrgb_data['x'] = x_pts
            xyzrgb_data['y'] = y_pts
            xyzrgb_data['z'] = z_pts
            xyzrgb_data['rgb'] = rgb_float

            cloud_full_msg = pc2.create_cloud(header, fields_xyzrgb, xyzrgb_data)
            self.pub_full_cloud.publish(cloud_full_msg)

            # Obstacles (masked) vs Background
            masked_idx = mask_pts > 0
            if np.any(masked_idx):
                cloud_obs_msg = pc2.create_cloud(header, fields_xyzrgb, xyzrgb_data[masked_idx])
                self.pub_obstacles.publish(cloud_obs_msg)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = SAM3OfflineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
