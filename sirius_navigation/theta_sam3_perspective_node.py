#!/usr/bin/env python3
"""THETA dual-fisheye -> 透視投影4ビュー -> SAM3 -> 地面逆投影 -> BEVラベル。

既存Dockerサーバ（単一画像経路 + /semantic_map.png + /semantic_score.png）を使い、
4方向(前/左/後/右)の透視画像をSAM3でクラス分類し、各BEV画素の地面点を各ビューへ
投影してサンプル、スコア重み付き投票で予約IDのBEVラベル(/theta/bev_semantic)を作る。

予約ID: 0=unknown,1=wall,2=floor,3=grass,4=tactile(line-type含む),5=roadway,6=sidewalk
"""
import math
import threading
import time

import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import CompressedImage, Image
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from sirius_navigation.theta_perspective import build_perspective_remap, build_bev_to_view_maps


def quaternion_to_rpy(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw

CLASS_REGISTRY = {
    'grass': {'id': 3, 'color': [0, 255, 0]},
    'tactile paving': {'id': 4, 'color': [255, 255, 0]},
    'line-type tactile paving': {'id': 4, 'color': [255, 255, 0]},
}
PROMPTS = 'grass, tactile paving, line-type tactile paving'
MAX_CLASS_ID = 6


class ThetaSam3PerspectiveNode(Node):
    def __init__(self):
        super().__init__('theta_sam3_perspective_node')
        self.declare_parameter('calibration', str(Path(get_package_share_directory('sirius_navigation')) / 'config' / 'theta_calibration.yaml'))
        self.declare_parameter('input_topic', '/theta/dual_fisheye/image_raw/compressed')
        self.declare_parameter('output_topic', '/theta/bev_semantic')
        self.declare_parameter('output_frame', 'sirius3/base_footprint')
        self.declare_parameter('server', 'http://localhost:8080')
        self.declare_parameter('view_yaws', '0,180')
        self.declare_parameter('hfov_deg', 120.0)
        self.declare_parameter('out_width', 640)
        self.declare_parameter('out_height', 480)
        self.declare_parameter('threshold', 0.3)
        self.declare_parameter('score_min', 0.3)
        self.declare_parameter('min_radius', 1.2)
        self.declare_parameter('max_radius', 3.5)
        self.declare_parameter('infer_timeout_sec', 2.0)
        self.declare_parameter('min_interval_sec', 0.3)
        self.declare_parameter('publish_debug', False)
        # カメラ取り付け姿勢をTFから取得（theta_bev_nodeと同じ経路）。未取得時は校正YAML。
        self.declare_parameter('use_tf', True)
        self.declare_parameter('robot_frame', 'sirius3/base_footprint')
        self.declare_parameter('camera_frame', 'sirius3/theta_link')

        with open(self.get_parameter('calibration').value) as f:
            self.calibration = yaml.safe_load(f)
        self.bev_size = int(self.calibration['bev_size'])
        self.server = self.get_parameter('server').value.rstrip('/')
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.view_yaws = [float(x) for x in str(self.get_parameter('view_yaws').value).split(',') if x.strip()]
        self.hfov = float(self.get_parameter('hfov_deg').value)
        self.out_size = (int(self.get_parameter('out_width').value), int(self.get_parameter('out_height').value))
        self.threshold = float(self.get_parameter('threshold').value)
        self.score_min = float(self.get_parameter('score_min').value)
        self.min_radius = float(self.get_parameter('min_radius').value)
        self.max_radius = float(self.get_parameter('max_radius').value)
        self.infer_timeout = max(0.1, float(self.get_parameter('infer_timeout_sec').value))
        self.min_interval = max(0.0, float(self.get_parameter('min_interval_sec').value))

        self.bev_to_view = None
        self.remaps = None
        self.remap_size = None
        self.mount_from_tf = False
        self.tf_buffer = None
        self.tf_listener = None
        if bool(self.get_parameter('use_tf').value):
            self.robot_frame = self.get_parameter('robot_frame').value
            self.camera_frame = self.get_parameter('camera_frame').value
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lock = threading.Lock()
        self.latest = None  # (stamp, bgr)
        self.pub = self.create_publisher(Image, self.output_topic, 1)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.debug_pubs = {}
        self.pub_bev_color = None
        if self.publish_debug:
            for yaw in self.view_yaws:
                name = self._view_name(yaw)
                self.debug_pubs[yaw] = {
                    'view': self.create_publisher(Image, f'/theta/sam3/view_{name}', 1),
                    'mask': self.create_publisher(Image, f'/theta/sam3/mask_{name}', 1),
                    'overlay': self.create_publisher(Image, f'/theta/sam3/overlay_{name}', 1),
                }
            self.pub_bev_color = self.create_publisher(Image, '/theta/bev_semantic_color', 1)
        self.sub = self.create_subscription(
            CompressedImage, self.get_parameter('input_topic').value, self.receive, qos_profile_sensor_data)
        threading.Thread(target=self._infer_loop, daemon=True).start()
        self.get_logger().info(
            f'THETA SAM3 perspective: {self.get_parameter("input_topic").value} -> {self.output_topic} '
            f'views={self.view_yaws} hfov={self.hfov} server={self.server} debug={self.publish_debug}')

    @staticmethod
    def _view_name(yaw):
        return {0.0: 'front', 180.0: 'back', 90.0: 'left', -90.0: 'right'}.get(float(yaw), f'yaw{int(yaw)}')

    def _publish_image(self, publisher, array, stamp, encoding):
        if publisher is None:
            return
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self.output_frame
        message.height, message.width = array.shape[0], array.shape[1]
        message.encoding = encoding
        message.is_bigendian = False
        message.step = message.width * (3 if encoding in ('bgr8', 'rgb8') else 1)
        message.data = array.tobytes()
        publisher.publish(message)

    def _publish_view_debug(self, yaw, view_image, class_map, stamp):
        pubs = self.debug_pubs.get(yaw)
        if not pubs:
            return
        self._publish_image(pubs['view'], view_image, stamp, 'bgr8')
        self._publish_image(pubs['mask'], class_map, stamp, 'mono8')
        overlay = view_image.copy()
        for class_id, color_rgb in ((3, (0, 255, 0)), (4, (255, 255, 0)), (5, (0, 0, 255)), (6, (128, 128, 128))):
            mask = class_map == class_id
            if np.any(mask):
                overlay[mask] = (0.5 * overlay[mask] + 0.5 * np.array(color_rgb[::-1])).astype(np.uint8)
        self._publish_image(pubs['overlay'], overlay, stamp, 'bgr8')

    def _publish_label_color(self, label, stamp):
        if self.pub_bev_color is None:
            return
        color = np.full((label.shape[0], label.shape[1], 3), 127, np.uint8)
        for class_id, color_rgb in ((3, (0, 255, 0)), (4, (255, 255, 0)), (5, (0, 0, 255)), (6, (128, 128, 128))):
            color[label == class_id] = color_rgb[::-1]
        color[label == 1] = (0, 0, 0)
        color[label == 2] = (255, 255, 255)
        self._publish_image(self.pub_bev_color, color, stamp, 'bgr8')

    def receive(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        with self.lock:
            self.latest = (msg.header.stamp, bgr)

    def _apply_tf_mount(self):
        """カメラ取り付け姿勢をTF(base_footprint->theta_link)から取得して校正に反映する。"""
        if self.tf_buffer is None:
            return False
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame, self.camera_frame, Time(), Duration(seconds=0.05))
        except (LookupException, ConnectivityException, ExtrapolationException):
            if not getattr(self, '_tf_fallback_logged', False):
                self.get_logger().warning(
                    f'TF {self.robot_frame} <- {self.camera_frame} を取得できません'
                    f'（校正YAMLの camera_position にフォールバック）')
                self._tf_fallback_logged = True
            return False
        self._tf_fallback_logged = False
        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        roll, pitch, yaw = quaternion_to_rpy(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        position = [float(translation.x), float(translation.y), float(translation.z)]
        # build_maps/mount_matrix はUI規約(pitch正=前レンズ上向き)。ROS/SDFは逆なのでpitchを反転。
        rpy = [math.degrees(roll), -math.degrees(pitch), math.degrees(yaw)]
        if (self.calibration.get('camera_position') == position
                and np.allclose(self.calibration.get('rpy_degrees', [0.0, 0.0, 0.0]), rpy, atol=1e-6)):
            return False
        self.calibration['camera_position'] = position
        self.calibration['rpy_degrees'] = rpy
        self.mount_from_tf = True
        self.get_logger().info(f'THETA姿勢をTFから取得: 位置={position} rpy(UI度)={rpy}')
        return True

    def _ensure_remaps(self, image_size):
        changed = self._apply_tf_mount()
        if self.remaps is not None and self.remap_size == image_size and not changed:
            return
        self.remaps = [build_perspective_remap(self.calibration, yaw, image_size, self.out_size, self.hfov)
                       for yaw in self.view_yaws]
        self.bev_to_view = [build_bev_to_view_maps(self.calibration, yaw, self.bev_size, self.out_size, self.hfov,
                                                   self.min_radius, self.max_radius)
                            for yaw in self.view_yaws]
        self.remap_size = image_size
        self.get_logger().info(
            f'Built perspective remaps for input {image_size[1]}x{image_size[0]} '
            f'(mount_from_tf={self.mount_from_tf}, camera_position={self.calibration.get("camera_position")})')

    def _post(self, path, obj=None, data=None):
        try:
            requests.post(self.server + path, json=obj, data=data, timeout=2.0)
            return True
        except Exception as error:
            self.get_logger().warning(f'POST {path} failed: {error}', throttle_duration_sec=5)
            return False

    def _configure_server(self):
        self._post('/source_mode', {'mode': 'network'})
        self._post('/crop', {'mode': 'none'})
        self._post('/class_registry', {'classes': CLASS_REGISTRY})
        self._post('/prompt', {'prompt': PROMPTS})
        self._post('/threshold', {'threshold': self.threshold})

    def _debug_versions(self):
        try:
            state = requests.get(self.server + '/debug_state', timeout=2.0).json()
            return int(state.get('network_frame_version', -1)), int(state.get('sam3_inference_frame_version', -1))
        except Exception:
            return -1, -1

    def _fetch_view_maps(self, frame):
        """frame をアップロードし、その結果の class/score マップ(out_size)を返す。"""
        _, before = self._debug_versions()
        kernel = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        ok, encoded = cv2.imencode('.jpg', frame, kernel)
        if not ok:
            return None, None
        if not self._post('/upload_frame', data=encoded.tobytes()):
            return None, None
        deadline = time.time() + self.infer_timeout
        while time.time() < deadline:
            net, inf = self._debug_versions()
            if inf > before and inf >= net:
                break
            time.sleep(0.03)
        try:
            class_resp = requests.get(self.server + '/semantic_map.png', timeout=2.0)
            score_resp = requests.get(self.server + '/semantic_score.png', timeout=2.0)
        except Exception:
            return None, None
        if class_resp.status_code != 200 or score_resp.status_code != 200:
            return None, None
        class_map = cv2.imdecode(np.frombuffer(class_resp.content, np.uint8), cv2.IMREAD_GRAYSCALE)
        score_map = cv2.imdecode(np.frombuffer(score_resp.content, np.uint8), cv2.IMREAD_GRAYSCALE)
        if class_map is None or score_map is None:
            return None, None
        w, h = self.out_size
        if class_map.shape != (h, w):
            class_map = cv2.resize(class_map, (w, h), interpolation=cv2.INTER_NEAREST)
            score_map = cv2.resize(score_map, (w, h), interpolation=cv2.INTER_NEAREST)
        return class_map, score_map

    def _infer_loop(self):
        self._configure_server()
        while rclpy.ok():
            with self.lock:
                item = self.latest
            if item is None:
                time.sleep(0.1)
                continue
            stamp, fisheye = item
            self._ensure_remaps(fisheye.shape[:2])
            votes = np.zeros((self.bev_size, self.bev_size, MAX_CLASS_ID + 1), dtype=np.float32)
            any_ok = False
            for view_index, (map_x, map_y) in enumerate(self.remaps):
                view_image = cv2.remap(fisheye, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                class_map, score_map = self._fetch_view_maps(view_image)
                if class_map is None:
                    continue
                any_ok = True
                if self.publish_debug:
                    self._publish_view_debug(self.view_yaws[view_index], view_image, class_map, stamp)
                u_map, v_map, valid = self.bev_to_view[view_index]
                rows, cols = np.nonzero(valid)
                if rows.size == 0:
                    continue
                uu = np.clip(np.round(u_map[rows, cols]).astype(np.int32), 0, self.out_size[0] - 1)
                vv = np.clip(np.round(v_map[rows, cols]).astype(np.int32), 0, self.out_size[1] - 1)
                cls = class_map[vv, uu].astype(np.int32)
                score = score_map[vv, uu].astype(np.float32) / 255.0
                keep = (cls >= 3) & (score >= self.score_min)
                np.add.at(votes, (rows[keep], cols[keep], cls[keep]), score[keep])
            if not any_ok:
                time.sleep(self.min_interval)
                continue
            best = np.argmax(votes, axis=2).astype(np.uint8)
            best_score = np.max(votes, axis=2)
            label = np.where(best_score > 0.0, best, 0).astype(np.uint8)
            out = Image()
            out.header.stamp = stamp
            out.header.frame_id = self.output_frame
            out.height, out.width = label.shape[0], label.shape[1]
            out.encoding = 'mono8'
            out.step = label.shape[1]
            out.data = label.tobytes()
            self.pub.publish(out)
            if self.publish_debug:
                self._publish_label_color(label, stamp)
            time.sleep(self.min_interval)


def main():
    rclpy.init()
    node = ThetaSam3PerspectiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
