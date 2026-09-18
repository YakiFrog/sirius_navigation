#!/usr/bin/env python3
"""Compressed dual-fisheye -> calibrated planar BEV, with cached remap tables.

カメラ姿勢（外部パラメータ）の解決優先順位:
  1. TF  robot_frame -> camera_frame  （SDF/URDF由来。sim・実機共通の正式経路）
  2. 専用トピック pose_topic          （アプリ同期。simの補助・フォールバック）
  3. calibration YAML の camera_position / rpy_degrees（最終フォールバック）
レンズ内部パラメータ（front/back, image_size, bev_*）は常にYAMLから取得する。
"""
from pathlib import Path
import json
import math
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from ament_index_python.packages import get_package_share_directory
from sirius_navigation.theta_bev_projection import build_blend_maps


def quaternion_to_rpy(x, y, z, w):
    """Quaternion -> (roll, pitch, yaw) [rad] in the ROS ZYX (yaw-pitch-roll) convention."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class ThetaBevNode(Node):
    def __init__(self):
        super().__init__('theta_bev')
        self.declare_parameter('calibration', str(Path(get_package_share_directory('sirius_navigation')) / 'config' / 'theta_calibration.yaml'))
        self.declare_parameter('input_topic', '/theta/dual_fisheye/image_raw/compressed')
        self.declare_parameter('output_topic', '/theta/bev/image_raw')
        self.declare_parameter('output_frame', 'sirius3/base_footprint')
        # RViz2のImage表示は生Imageのみ。compressed_image_transport未導入でも見られるよう、
        # デコード済みDual Fisheyeを生Imageとして再配信する。
        self.declare_parameter('publish_raw', True)
        self.declare_parameter('raw_topic', '/theta/dual_fisheye/image_raw')
        # 任意: アプリのTHETA姿勢同期トピック（JSON String, m/deg）。TFが取れない時のフォールバック。
        self.declare_parameter('pose_topic', '/theta/mount_pose')
        # TFからカメラ姿勢を取得する（SDF由来）。取れなければ pose_topic -> YAML の順にフォールバック。
        self.declare_parameter('use_tf', True)
        self.declare_parameter('robot_frame', 'sirius3/base_footprint')
        self.declare_parameter('camera_frame', 'sirius3/theta_link')
        self.declare_parameter('tf_timeout', 0.05)
        # build_mapsは pitch を反転して使う（UI規約: 正=前レンズが上向き）。ROS/SDF規約は逆なので符号を合わせる。
        self.declare_parameter('tf_pitch_sign', -1.0)
        # 前後レンズのつなぎ目（ロボット真横, θ≈90°）をクロスフェードする帯の半角[deg]。
        # 0で無効（従来のハード切り替え）。大きいほど広くブレンドする。
        self.declare_parameter('blend_half_deg', 6.0)

        with open(self.get_parameter('calibration').value) as f:
            self.base_calibration = yaml.safe_load(f)

        self.use_tf = bool(self.get_parameter('use_tf').value)
        self.robot_frame = self.get_parameter('robot_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tf_timeout = Duration(seconds=float(self.get_parameter('tf_timeout').value))
        self.tf_pitch_sign = float(self.get_parameter('tf_pitch_sign').value)
        self.blend_half_deg = float(self.get_parameter('blend_half_deg').value)

        self.topic_pose = None
        self.applied_pose = None
        self.calibration = self.base_calibration
        self.maps = None
        self.shape = None
        self.buffer = None
        self.listener = None
        if self.use_tf:
            self.buffer = Buffer(cache_time=Duration(seconds=60.0))
            self.listener = TransformListener(self.buffer, self)

        self.pub = self.create_publisher(Image, self.get_parameter('output_topic').value, 1)
        self.raw_pub = self.create_publisher(Image, self.get_parameter('raw_topic').value, 1) if self.get_parameter('publish_raw').value else None
        self.sub = self.create_subscription(CompressedImage, self.get_parameter('input_topic').value, self.receive, qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(String, self.get_parameter('pose_topic').value, self.receive_pose, 1)
        source = f'TF {self.robot_frame} <- {self.camera_frame}' if self.use_tf else 'topic/YAML'
        self.get_logger().info(f'Dual Fisheye JPEG -> BEV 準備完了（平面地面仮定, 姿勢取得元: {source}）')

    def receive_pose(self, msg):
        try:
            pose = json.loads(msg.data)
            self.topic_pose = (
                [float(pose['x']), float(pose['y']), float(pose['z'])],
                [float(pose.get('roll', 0.0)), float(pose.get('pitch', 0.0)), float(pose.get('yaw', 0.0))],
            )
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(f'Invalid theta pose message: {error}', throttle_duration_sec=5)

    def resolve_pose(self, stamp):
        """(position, rpy_degrees, source) を優先順位付きで返す。"""
        if self.buffer is not None:
            try:
                tf_msg = self.buffer.lookup_transform(self.robot_frame, self.camera_frame, stamp, self.tf_timeout)
                t, q = tf_msg.transform.translation, tf_msg.transform.rotation
                roll, pitch, yaw = quaternion_to_rpy(q.x, q.y, q.z, q.w)
                return ([t.x, t.y, t.z],
                        [math.degrees(roll), self.tf_pitch_sign * math.degrees(pitch), math.degrees(yaw)],
                        'tf')
            except (LookupException, ConnectivityException, ExtrapolationException) as error:
                self.get_logger().warning(
                    f'TF {self.robot_frame} <- {self.camera_frame} を取得できません（校正YAMLへフォールバック）: {error}',
                    throttle_duration_sec=5)
        if self.topic_pose is not None:
            return (self.topic_pose[0], self.topic_pose[1], 'topic')
        return (self.base_calibration['camera_position'], self.base_calibration['rpy_degrees'], 'yaml')

    def apply_pose(self, resolved):
        position, rpy, source = resolved
        pose = (list(position), list(rpy))
        if self.applied_pose == pose:
            return
        self.calibration = dict(self.base_calibration)
        self.calibration['camera_position'] = [float(v) for v in position]
        self.calibration['rpy_degrees'] = [float(v) for v in rpy]
        self.applied_pose = pose
        self.shape = None  # 次のフレームで再投影マップを作り直す
        source_ja = {'tf': 'TF', 'topic': '姿勢トピック(/theta/mount_pose)', 'yaml': '校正YAML'}.get(source, source)
        pose_text = f'位置={self.calibration["camera_position"]} rpy={self.calibration["rpy_degrees"]}'
        if source == 'yaml':
            self.get_logger().warning(f'THETA姿勢: TF/トピックが無いため校正YAMLにフォールバック（{pose_text}）')
        else:
            self.get_logger().info(f'THETA姿勢を{source_ja}から取得: {pose_text}')

    def receive(self, msg):
        try:
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError('Invalid JPEG')
            stamp = msg.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                stamp = Time()  # スタンプ未設定なら最新のTFを使う
            self.apply_pose(self.resolve_pose(stamp))
            if self.raw_pub is not None:
                raw = Image()
                raw.header.stamp = msg.header.stamp
                raw.header.frame_id = msg.header.frame_id or 'sirius3/theta_link'
                raw.height, raw.width = frame.shape[:2]
                raw.encoding = 'bgr8'
                raw.is_bigendian = False
                raw.step = raw.width * 3
                raw.data = frame.tobytes()
                self.raw_pub.publish(raw)
            if self.shape != frame.shape:
                self.maps = build_blend_maps(self.calibration, frame.shape[1], frame.shape[0], self.blend_half_deg)
                self.shape = frame.shape
            mx_f, my_f, mx_b, my_b, alpha = self.maps
            front = cv2.remap(frame, mx_f, my_f, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            back = cv2.remap(frame, mx_b, my_b, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            a = alpha[..., None]
            bev = np.clip(front.astype(np.float32) * a + back.astype(np.float32) * (1.0 - a), 0, 255).astype(np.uint8)
            out = Image()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.get_parameter('output_frame').value
            out.height, out.width = bev.shape[:2]
            out.encoding = 'bgr8'
            out.is_bigendian = False
            out.step = out.width * 3
            out.data = bev.tobytes()
            self.pub.publish(out)
        except (ValueError, KeyError, cv2.error) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5)


def main():
    rclpy.init()
    node = ThetaBevNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
