#!/usr/bin/env python3
"""3D LiDAR点群(VLP-16)を RICOH THETA(デュアルフィッシュアイ)で色付ける。

OmniColor (arXiv:2404.04693) の考え方を参考にした「まず動かす」最小構成。
カメラ姿勢の大域最適化は行わず、LiDAR点をカメラフレームへ変換して魚眼へ投影し、
その画素色を点に付与する（point-centricな投影＋色サンプリング）。

入力:
  - /velodyne_points                         (sensor_msgs/PointCloud2)
  - /theta/dual_fisheye/image_raw[/compressed] (RICOH THETA デュアルフィッシュアイ)
出力:
  - /velodyne_points_colored                 (sensor_msgs/PointCloud2, x,y,z,intensity,rgb)

外部パラメータ（LiDAR->THETA）は優先順で解決する:
  1. TF  camera_frame <- cloud.frame_id      (use_tf=true, 既定)
  2. パラメータ theta_xyz (+ theta_rpy_degrees): THETAのLiDARフレームでの姿勢
実機は静的TFが未配信のことが多いため 2 のフォールバックを持つ。
"""
from collections import deque
from pathlib import Path
import time

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import (Buffer, TransformListener, LookupException,
                     ConnectivityException, ExtrapolationException)
from ament_index_python.packages import get_package_share_directory

from sirius_navigation.theta_perspective import (project_camera_frame_points,
                                                 rpy_to_matrix)


def quaternion_to_matrix(x, y, z, w):
    """Quaternion -> 3x3 rotation matrix."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


class ThetaColorizeCloudNode(Node):
    def __init__(self):
        super().__init__('theta_colorize_cloud')
        share = Path(get_package_share_directory('sirius_navigation'))
        self.declare_parameter('calibration', str(share / 'config' / 'theta_calibration_real.yaml'))
        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points_colored')
        # 'compressed' は /theta/dual_fisheye/image_raw/compressed、'raw' は生Image。
        self.declare_parameter('image_transport', 'compressed')
        self.declare_parameter('image_topic', '/theta/dual_fisheye/image_raw/compressed')
        self.declare_parameter('raw_image_topic', '/theta/dual_fisheye/image_raw')
        # 外部パラメータの解決
        self.declare_parameter('use_tf', True)
        self.declare_parameter('camera_frame', 'sirius3/theta_link')
        # フォールバック: THETAのLiDARフレームにおける位置[m]と姿勢[deg]（ZYX）
        self.declare_parameter('theta_xyz', [0.0, 0.0, 0.285])
        self.declare_parameter('theta_rpy_degrees', [0.0, 0.0, 0.0])
        # 対応させる画像の最大時間差[s]。これを超えたら色を付けない（既定は最新画像にフォールバック）
        self.declare_parameter('max_image_age', 0.35)
        self.declare_parameter('fallback_to_latest_image', True)
        # 近傍/遠方の除外[m]
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 80.0)
        # 画像の有効画素下限（暗すぎる画素は色として採用しない）。0で無効
        self.declare_parameter('min_brightness', 0)
        # 画像キャッシュ枚数
        self.declare_parameter('image_cache_size', 20)

        with open(self.get_parameter('calibration').value) as f:
            self.calibration = yaml.safe_load(f)
        self.calib_image_size = (int(self.calibration['image_size'][0]),
                                 int(self.calibration['image_size'][1]))
        self.theta_xyz = np.asarray(self.get_parameter('theta_xyz').value, dtype=np.float64)
        self.theta_rpy = [float(v) for v in self.get_parameter('theta_rpy_degrees').value]
        self.use_tf = bool(self.get_parameter('use_tf').value)
        self.camera_frame = self.get_parameter('camera_frame').value
        self.max_image_age = max(0.0, float(self.get_parameter('max_image_age').value))
        self.fallback_latest = bool(self.get_parameter('fallback_to_latest_image').value)
        self.min_range = max(0.0, float(self.get_parameter('min_range').value))
        self.max_range = max(0.0, float(self.get_parameter('max_range').value))
        self.min_brightness = int(self.get_parameter('min_brightness').value)
        cache_size = max(1, int(self.get_parameter('image_cache_size').value))

        self.images = deque(maxlen=cache_size)  # (stamp_float, bgr)
        self.last_warn = 0.0
        self.tf_buffer = None
        self.tf_listener = None
        if self.use_tf:
            self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        self.dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                               ('intensity', '<f4'), ('rgb', '<f4')])

        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value,
                                         qos_profile_sensor_data)
        self.lidar_sub = self.create_subscription(
            PointCloud2, self.get_parameter('lidar_topic').value, self.receive_cloud,
            qos_profile_sensor_data)
        transport = str(self.get_parameter('image_transport').value).lower()
        if transport == 'raw':
            self.image_sub = self.create_subscription(
                Image, self.get_parameter('raw_image_topic').value, self.receive_raw,
                qos_profile_sensor_data)
        else:
            self.image_sub = self.create_subscription(
                CompressedImage, self.get_parameter('image_topic').value, self.receive_compressed,
                qos_profile_sensor_data)
        self.get_logger().info(
            f'colorize: {self.get_parameter("lidar_topic").value} + THETA({transport}) '
            f'-> {self.get_parameter("output_topic").value} (use_tf={self.use_tf})')

    # ------------------------------------------------------------------ images
    def _store_image(self, stamp, bgr):
        stamp_float = stamp.sec + stamp.nanosec * 1e-9 if stamp is not None else time.time()
        self.images.append((stamp_float, bgr))

    def receive_compressed(self, msg):
        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        except cv2.error as error:
            self.get_logger().warning(f'JPEG decode failed: {error}', throttle_duration_sec=5)
            return
        if frame is not None:
            self._store_image(msg.header.stamp, frame)

    def receive_raw(self, msg):
        if msg.encoding not in ('bgr8', 'rgb8'):
            self.get_logger().warning(f'Unsupported encoding: {msg.encoding}', throttle_duration_sec=5)
            return
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            frame = frame[:, :, ::-1].copy()
        else:
            frame = frame[:, :, :3].copy()
        self._store_image(msg.header.stamp, frame)

    def _select_image(self, stamp_float):
        if not self.images:
            return None
        if stamp_float is None or stamp_float <= 0.0:
            return self.images[-1][1]
        best, best_dt = None, self.max_image_age
        for img_stamp, bgr in self.images:
            dt = abs(img_stamp - stamp_float)
            if dt <= best_dt:
                best, best_dt = bgr, dt
        if best is not None:
            return best
        return self.images[-1][1] if self.fallback_latest else None

    # --------------------------------------------------------------- transform
    def _extrinsic(self, cloud):
        """cloudのframe -> カメラフレームへの (R, t)。p_cam = R @ p + t。"""
        frame = cloud.header.frame_id or self.camera_frame
        if self.tf_buffer is not None:
            stamp = cloud.header.stamp
            lookup_time = stamp if (stamp.sec or stamp.nanosec) else Time()
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.camera_frame, frame, lookup_time, Duration(seconds=0.05))
            except (LookupException, ConnectivityException, ExtrapolationException) as error:
                now = time.time()
                if now - self.last_warn > 5.0:
                    self.get_logger().warning(
                        f'TF {self.camera_frame} <- {frame} を取得できません。'
                        f'パラメータ theta_xyz にフォールバックします: {error}')
                    self.last_warn = now
            else:
                t = tf_msg.transform.translation
                q = tf_msg.transform.rotation
                return quaternion_to_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])
        # フォールバック: THETAはLiDARフレームで theta_xyz にある。
        # p_cam = R_lidar_cam^T @ (p_lidar - t)
        r_lidar_cam = rpy_to_matrix(self.theta_rpy[0], self.theta_rpy[1], self.theta_rpy[2])
        return r_lidar_cam.T, -r_lidar_cam.T @ self.theta_xyz

    # ------------------------------------------------------------------ cloud
    def receive_cloud(self, msg):
        stamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        image = self._select_image(stamp_float)
        if image is None:
            self.get_logger().warning('THETA画像が未受信のため色付けできません',
                                      throttle_duration_sec=5)
            return
        points = pc2.read_points(msg, field_names=None, skip_nans=False)
        xyz = np.stack([points['x'], points['y'], points['z']], axis=-1).astype(np.float64)
        intensity = (points['intensity'].astype(np.float32)
                     if 'intensity' in points.dtype.names else np.zeros(len(xyz), np.float32))
        count = xyz.shape[0]
        if count == 0:
            return

        rotation, translation = self._extrinsic(msg)
        points_cam = xyz @ rotation.T + translation

        rgb = np.zeros(count, dtype=np.uint32)
        valid = np.ones(count, dtype=bool)
        if self.min_range > 0.0 or self.max_range > 0.0:
            dist = np.linalg.norm(xyz, axis=-1)
            if self.min_range > 0.0:
                valid &= dist >= self.min_range
            if self.max_range > 0.0:
                valid &= dist <= self.max_range
        uv, proj_ok = project_camera_frame_points(points_cam, self.calibration, image.shape[:2])
        valid &= proj_ok
        valid_idx = np.flatnonzero(valid)
        if valid_idx.size:
            bgr, keep = self._sample_rgb(image, uv[valid_idx])
            sel = valid_idx[keep]
            b = bgr[keep, 0].astype(np.uint32)
            g = bgr[keep, 1].astype(np.uint32)
            r = bgr[keep, 2].astype(np.uint32)
            rgb[sel] = (r << 16) | (g << 8) | b

        out = np.empty(count, dtype=self.dtype)
        out['x'], out['y'], out['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        out['intensity'] = intensity
        out['rgb'] = rgb.view(np.float32)

        header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)
        cloud = PointCloud2()
        cloud.header = header
        cloud.height, cloud.width = 1, count
        cloud.fields = self.fields
        cloud.is_bigendian = False
        cloud.point_step = self.dtype.itemsize
        cloud.row_step = self.dtype.itemsize * count
        cloud.is_dense = False
        cloud.data = out.tobytes()
        self.pub.publish(cloud)

    def _sample_rgb(self, image, uv):
        """BGR画像から双線形補間で色を引く。

        Returns: (bgr uint8 (N,3), keep bool (N,))。keep は輝度下限を満たす点。
        """
        h, w = image.shape[:2]
        u = uv[:, 0]
        v = uv[:, 1]
        u0 = np.clip(np.floor(u).astype(np.int64), 0, w - 1)
        v0 = np.clip(np.floor(v).astype(np.int64), 0, h - 1)
        u1 = np.clip(u0 + 1, 0, w - 1)
        v1 = np.clip(v0 + 1, 0, h - 1)
        du = np.clip(u - u0, 0.0, 1.0)
        dv = np.clip(v - v0, 0.0, 1.0)
        w00 = ((1 - du) * (1 - dv))[:, None]
        w01 = (du * (1 - dv))[:, None]
        w10 = ((1 - du) * dv)[:, None]
        w11 = (du * dv)[:, None]
        c00 = image[v0, u0].astype(np.float32)
        c01 = image[v0, u1].astype(np.float32)
        c10 = image[v1, u0].astype(np.float32)
        c11 = image[v1, u1].astype(np.float32)
        bgr = np.clip(c00 * w00 + c01 * w01 + c10 * w10 + c11 * w11, 0, 255).astype(np.uint8)
        if self.min_brightness > 0:
            keep = bgr.max(axis=1) >= self.min_brightness
        else:
            keep = np.ones(bgr.shape[0], dtype=bool)
        return bgr, keep


def main():
    rclpy.init()
    node = ThetaColorizeCloudNode()
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
