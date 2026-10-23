#!/usr/bin/env python3
"""3D LiDAR点群(VLP-16)を RICOH THETA(デュアルフィッシュアイ)で色付ける。

固定外部校正・カメラ視点の遮蔽・画像時刻への運動補償を使う。
ロボット共通方針は theta_colorization.yaml。元のXYZ/ring/timeを保持する。

入力:
  - /velodyne_points                         (sensor_msgs/PointCloud2)
  - /theta/dual_fisheye/image_raw[/compressed] (RICOH THETA デュアルフィッシュアイ)
出力:
  - /velodyne_points_colored                 (sensor_msgs/PointCloud2, x,y,z,intensity,rgb)

外部パラメータ（LiDAR->THETA）は優先順で解決する:
  1. 共通方針の camera_in_lidar（測定・校正値を明示した場合）
  2. TF  camera_frame <- cloud.frame_id      (use_tf=true, 既定)
  3. パラメータ theta_xyz (+ theta_rpy_degrees): THETAのLiDARフレームでの姿勢
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
from sirius_navigation.theta_color_projection import load_color_policy, camera_in_lidar, visible_colors


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



def color_cloud_preserving_fields(msg, rgb):
    """Add packed RGB while preserving XYZ, ring, time, order and every input field."""
    import copy
    count = msg.width * msg.height
    existing = next((f for f in msg.fields if f.name == 'rgb'), None)
    if existing is not None and (existing.datatype != PointField.FLOAT32 or existing.count != 1):
        raise ValueError('Existing rgb field must be FLOAT32 count=1')
    offset = existing.offset if existing is not None else (msg.point_step + 3) // 4 * 4
    step = msg.point_step if existing is not None else offset+4
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.row_step)
    original = rows[:, :msg.width*msg.point_step].copy().reshape(count, msg.point_step)
    data = np.zeros((count, step), dtype=np.uint8)
    data[:, :msg.point_step] = original
    packed = np.ndarray((count,), dtype='>u4' if msg.is_bigendian else '<u4', buffer=data, offset=offset, strides=(step,))
    packed[:] = np.asarray(rgb, dtype=np.uint32).reshape(-1)
    fields = copy.deepcopy(msg.fields)
    if existing is None:
        fields.append(PointField(name='rgb', offset=offset, datatype=PointField.FLOAT32, count=1))
    return PointCloud2(header=msg.header, height=msg.height, width=msg.width, fields=fields,
                       is_bigendian=msg.is_bigendian, point_step=step, row_step=step*msg.width,
                       is_dense=msg.is_dense, data=data.tobytes())

class ThetaColorizeCloudNode(Node):
    def __init__(self):
        super().__init__('theta_colorize_cloud')
        share = Path(get_package_share_directory('sirius_navigation'))
        self.declare_parameter('calibration', str(share / 'config' / 'theta_calibration_real.yaml'))
        self.declare_parameter('colorization_config', str(share / 'config' / 'theta_colorization.yaml'))
        self.color_policy = load_color_policy(self.get_parameter('colorization_config').value)
        self.mount_override = camera_in_lidar(self.color_policy)
        self.declare_parameter('fixed_frame', 'sirius3/odom')
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
        # 最大時間差[s]。古い最新画像へのフォールバックは既定で無効。
        self.declare_parameter('max_image_age', float(self.color_policy['max_image_age_s']))
        self.declare_parameter('fallback_to_latest_image', False)
        # 近傍/遠方の除外[m]
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 80.0)
        # 画像の有効画素下限（暗すぎる画素は色として採用しない）。0で無効
        self.declare_parameter('min_brightness', 0)
        # 画像キャッシュ枚数
        self.declare_parameter('image_cache_size', 20)
        # Wait for the next camera image and delayed odometry TF before choosing
        # the closest image. Sensor timestamps and point geometry stay unchanged.
        self.declare_parameter('image_wait_timeout', 0.3)

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
        self.image_wait_timeout = max(0.0, float(self.get_parameter('image_wait_timeout').value))
        self.pending_clouds = deque()
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
        self.pending_timer = self.create_timer(0.02, self._flush_pending_clouds)
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
        if stamp_float is None:
            return None
        best, best_dt = None, self.max_image_age
        for img_stamp, bgr in self.images:
            img_stamp += self.color_policy['image_time_offset_s']
            dt = abs(img_stamp - stamp_float)
            if dt <= best_dt:
                best, best_dt = (img_stamp, bgr), dt
        if best is not None:
            return best
        if self.fallback_latest:
            t, bgr = self.images[-1]
            return t+self.color_policy['image_time_offset_s'], bgr
        return None

    # --------------------------------------------------------------- transform
    def _extrinsic(self, cloud):
        """cloudのframe -> カメラフレームへの (R, t)。p_cam = R @ p + t。"""
        frame = cloud.header.frame_id or self.camera_frame
        if self.mount_override is not None:
            if frame.lstrip('/') != self.color_policy['lidar_frame'].lstrip('/') or self.camera_frame != self.color_policy['camera_frame']:
                raise ValueError('固定色付け校正と点群/カメラのframeが一致しません')
            rot = self.mount_override[:3, :3].T
            return rot, -rot @ self.mount_override[:3, 3]
        if self.tf_buffer is not None:
            stamp = cloud.header.stamp
            lookup_time = Time.from_msg(stamp) if (stamp.sec or stamp.nanosec) else Time()
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
        r_lidar_cam = rpy_to_matrix(self.theta_rpy[0], self.theta_rpy[1], self.theta_rpy[2], pitch_sign=1.)
        return r_lidar_cam.T, -r_lidar_cam.T @ self.theta_xyz

    def _image_transform(self, cloud, image_stamp):
        rotation, translation = self._extrinsic(cloud)
        cloud_stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec*1e-9
        if abs(image_stamp-cloud_stamp) < 1.e-6:
            return rotation, translation
        if self.tf_buffer is not None:
            try:
                motion = self.tf_buffer.lookup_transform_full(
                    cloud.header.frame_id, Time(nanoseconds=int(image_stamp*1.e9)),
                    cloud.header.frame_id, Time.from_msg(cloud.header.stamp),
                    self.get_parameter('fixed_frame').value, Duration(seconds=0.))
                q, t = motion.transform.rotation, motion.transform.translation
                delta = quaternion_to_matrix(q.x, q.y, q.z, q.w)
                return rotation @ delta, rotation @ np.array([t.x, t.y, t.z])+translation
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
        if abs(image_stamp-cloud_stamp) > self.color_policy['max_uncompensated_age_s']:
            return None
        return rotation, translation

    # ------------------------------------------------------------------ cloud
    def receive_cloud(self, msg):
        if self.image_wait_timeout <= 0:
            self._colorize_cloud(msg)
            return
        # Bound memory even if image decoding/color projection falls behind.
        if len(self.pending_clouds) >= 20:
            self._colorize_cloud(self.pending_clouds.popleft()[1])
        self.pending_clouds.append((time.monotonic(), msg))

    def _flush_pending_clouds(self):
        now = time.monotonic()
        while self.pending_clouds and now - self.pending_clouds[0][0] >= self.image_wait_timeout:
            self._colorize_cloud(self.pending_clouds.popleft()[1])

    def _colorize_cloud(self, msg):
        stamp_float = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        selected = self._select_image(stamp_float)
        points = pc2.read_points(msg, field_names=None, skip_nans=False).reshape(-1)
        xyz = np.stack([points['x'], points['y'], points['z']], axis=-1).astype(np.float64)
        intensity = (points['intensity'].astype(np.float32)
                     if 'intensity' in points.dtype.names else np.zeros(len(xyz), np.float32))
        count = xyz.shape[0]
        if count == 0:
            return

        rgb = np.full(count, 0x808080, dtype=np.uint32)
        if selected is not None:
            image_stamp, image = selected
            resolved = self._image_transform(msg, image_stamp)
            if resolved is not None:
                rotation, translation = resolved
                points_cam = xyz @ rotation.T + translation
                sampled, confidence, _ = visible_colors(points_cam, image, self.calibration, self.color_policy)
                distance = np.linalg.norm(xyz, axis=1)
                valid = (confidence > 0) & (distance >= self.min_range)
                if self.max_range:
                    valid &= distance <= self.max_range
                if self.min_brightness:
                    brightness = np.maximum.reduce([(sampled>>16)&255, (sampled>>8)&255, sampled&255])
                    valid &= brightness >= self.min_brightness
                rgb[valid] = sampled[valid]
        else:
            self.get_logger().warning('同期できるTHETA画像なし。形状を保持して灰色で配信します', throttle_duration_sec=5)

        cloud = color_cloud_preserving_fields(msg, rgb)
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
