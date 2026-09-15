#!/usr/bin/env python3
"""BEV画像(地面投影, sensor_msgs/Image) -> 地面点群(PointCloud2)。

BEVは機体基準(base_footprint)で前方が上・左が左・8m四方・中心が機体原点なので、
各画素を z=0 の地面点へ戻すだけで3D点群になる。THETAオフライン路面マッピングで
RTAB-Mapの scan_cloud 入力として使う。SAM3やステレオ深度は不要。

SAM3ラベル画像(/theta/bev_semantic, mono8, 予約ID)が来た場合は、**同じ撮影時刻(stamp)の
BEVフレームにだけ**semantic_idを付与する（時刻ずれによる誤配置を防ぐ）。ラベルが
semantic_wait_sec 以内に来なければ、そのフレームはsemantic_id=0で出力する。
"""
import time
from collections import OrderedDict
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from ament_index_python.packages import get_package_share_directory


class ThetaGroundCloudNode(Node):
    def __init__(self):
        super().__init__('theta_ground_cloud')
        self.declare_parameter('calibration', str(Path(get_package_share_directory('sirius_navigation')) / 'config' / 'theta_calibration.yaml'))
        self.declare_parameter('input_topic', '/theta/bev/image_raw')
        self.declare_parameter('output_topic', '/theta/ground_cloud')
        self.declare_parameter('frame_id', 'sirius3/base_footprint')
        self.declare_parameter('stride', 4)
        self.declare_parameter('min_value', 8)
        self.declare_parameter('min_radius', 1.2)
        self.declare_parameter('max_radius', 3.5)
        self.declare_parameter('semantic_topic', '/theta/bev_semantic')
        # SAM3ラベルが同じstampで来るのを待つ上限[s]。過ぎたらsemantic無しで出力。
        self.declare_parameter('semantic_wait_sec', 1.0)
        self.declare_parameter('max_pending', 200)

        with open(self.get_parameter('calibration').value) as f:
            calibration = yaml.safe_load(f)
        self.bev_size = int(calibration['bev_size'])
        self.extent = float(calibration['bev_extent_m'])
        self.frame_id = self.get_parameter('frame_id').value
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.min_value = int(self.get_parameter('min_value').value)
        self.min_radius = max(0.0, float(self.get_parameter('min_radius').value))
        self.max_radius = max(0.0, float(self.get_parameter('max_radius').value))
        self.semantic_wait = max(0.0, float(self.get_parameter('semantic_wait_sec').value))
        self.max_pending = max(1, int(self.get_parameter('max_pending').value))

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='semantic_id', offset=16, datatype=PointField.UINT8, count=1),
        ]
        self.pending = OrderedDict()  # stamp_key -> (stamp, frame_id, image, arrival)
        self.labels = {}              # stamp_key -> mono8 label
        self.warned_shape = False
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, 1)
        self.sub = self.create_subscription(Image, self.get_parameter('input_topic').value, self.receive, qos_profile_sensor_data)
        self.semantic_sub = self.create_subscription(
            Image, self.get_parameter('semantic_topic').value, self.receive_semantic, qos_profile_sensor_data)
        self.create_timer(0.1, self._flush_stale)
        self.get_logger().info(
            f'BEV {self.bev_size}x{self.bev_size} ({self.extent}m) -> {self.get_parameter("output_topic").value} '
            f'frame={self.frame_id} stride={self.stride} min_radius={self.min_radius} max_radius={self.max_radius} '
            f'semantic={self.get_parameter("semantic_topic").value}')

    @staticmethod
    def _key(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def receive(self, msg):
        if msg.height != self.bev_size or msg.width != self.bev_size:
            if not self.warned_shape:
                self.get_logger().warning(
                    f'BEV size mismatch: got {msg.width}x{msg.height}, expected {self.bev_size}x{self.bev_size}')
                self.warned_shape = True
            return
        if msg.encoding not in ('bgr8', 'rgb8'):
            self.get_logger().warning(f'Unsupported encoding: {msg.encoding}', throttle_duration_sec=5)
            return
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            image = image[:, :, ::-1].copy()
        key = self._key(msg)
        self.pending[key] = (msg.header.stamp, msg.header.frame_id or self.frame_id, image, time.time())
        while len(self.pending) > self.max_pending:
            self._emit(*self.pending.popitem(last=False))
        self._process_if_ready(key)

    def receive_semantic(self, msg):
        if msg.height != self.bev_size or msg.width != self.bev_size:
            return
        if msg.encoding not in ('mono8', '8UC1'):
            self.get_logger().warning(f'Unsupported semantic encoding: {msg.encoding}', throttle_duration_sec=5)
            return
        key = self._key(msg)
        self.labels[key] = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)
        if len(self.labels) > self.max_pending:
            for old in list(self.labels)[:-self.max_pending]:
                del self.labels[old]
        self._process_if_ready(key)

    def _process_if_ready(self, key):
        if key in self.pending and key in self.labels:
            stamp, frame_id, image, _ = self.pending.pop(key)
            self._emit(stamp, frame_id, image, self.labels.pop(key))

    def _flush_stale(self):
        if self.semantic_wait <= 0.0:
            wait = 0.0
        else:
            wait = self.semantic_wait
        now = time.time()
        for key in list(self.pending):
            stamp, frame_id, image, arrival = self.pending[key]
            if self.semantic_wait <= 0.0 or now - arrival > wait:
                self.pending.pop(key)
                self.labels.pop(key, None)
                self._emit(stamp, frame_id, image, None)

    def _emit(self, stamp, frame_id, image, semantic):
        height, width = image.shape[:2]
        step = self.stride
        rows = np.arange(0, height, step)
        cols = np.arange(0, width, step)
        grid_cols, grid_rows = np.meshgrid(cols, rows)
        u = (grid_cols + 0.5) / width
        v = (grid_rows + 0.5) / height
        x = (0.5 - v) * self.extent  # 前(+)
        y = (0.5 - u) * self.extent  # 左(+)
        colors = image[grid_rows, grid_cols]
        b, g, r = colors[..., 0].astype(np.uint32), colors[..., 1].astype(np.uint32), colors[..., 2].astype(np.uint32)
        valid = colors.max(axis=2) >= self.min_value
        radius_sq = x * x + y * y
        if self.min_radius > 0.0:
            valid &= radius_sq >= self.min_radius ** 2
        if self.max_radius > 0.0:
            valid &= radius_sq <= self.max_radius ** 2
        rgb = ((r << 16) | (g << 8) | b)
        rgb_float = rgb.view(np.float32)
        label_ok = semantic is not None and semantic.shape == (height, width)
        points = [
            (float(x[row, col]), float(y[row, col]), 0.0, float(rgb_float[row, col]),
             int(semantic[row, col]) if label_ok else 0)
            for row, col in zip(*np.nonzero(valid))
        ]
        if not points:
            return
        header = Header(stamp=stamp, frame_id=frame_id)
        self.pub.publish(pc2.create_cloud(header, self.fields, points))


def main():
    rclpy.init()
    node = ThetaGroundCloudNode()
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
