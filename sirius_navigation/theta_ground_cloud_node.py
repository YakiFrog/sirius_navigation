#!/usr/bin/env python3
"""BEV画像(地面投影, sensor_msgs/Image) -> 地面点群(PointCloud2)。

BEVは機体基準(base_footprint)で前方が上・左が左・8m四方・中心が機体原点なので、
各画素を z=0 の地面点へ戻すだけで3D点群になる。THETAオフライン路面マッピングで
RTAB-Mapの scan_cloud 入力として使う。SAM3やステレオ深度は不要。
"""
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
        # 全画素は重いので間引く。stride=4 で約6cm間隔。
        self.declare_parameter('stride', 4)
        # 外周の黒（BEVのBORDER_CONSTANT=0）を除外するための最小輝度。
        self.declare_parameter('min_value', 8)
        # 機体自身・直下の映り込みを除外する中心マスク半径[m]。0で無効。
        # 実機THETAも機体が写るため、sim/実機共通でこの半径で除外する。
        self.declare_parameter('min_radius', 1.2)
        # BEV外周の低品質な点を除外する最大半径[m]。0で無効。
        self.declare_parameter('max_radius', 3.5)

        with open(self.get_parameter('calibration').value) as f:
            calibration = yaml.safe_load(f)
        self.bev_size = int(calibration['bev_size'])
        self.extent = float(calibration['bev_extent_m'])
        self.frame_id = self.get_parameter('frame_id').value
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.min_value = int(self.get_parameter('min_value').value)
        self.min_radius = max(0.0, float(self.get_parameter('min_radius').value))
        self.max_radius = max(0.0, float(self.get_parameter('max_radius').value))

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.warned_shape = False
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, 1)
        self.sub = self.create_subscription(Image, self.get_parameter('input_topic').value, self.receive, qos_profile_sensor_data)
        self.get_logger().info(
            f'BEV {self.bev_size}x{self.bev_size} ({self.extent}m) -> {self.get_parameter("output_topic").value} '
            f'frame={self.frame_id} stride={self.stride} min_radius={self.min_radius} max_radius={self.max_radius}')

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
        step = self.stride
        rows = np.arange(0, msg.height, step)
        cols = np.arange(0, msg.width, step)
        grid_cols, grid_rows = np.meshgrid(cols, rows)
        u = (grid_cols + 0.5) / msg.width
        v = (grid_rows + 0.5) / msg.height
        x = (0.5 - v) * self.extent  # 前(+)
        y = (0.5 - u) * self.extent  # 左(+)
        colors = image[grid_rows, grid_cols]
        b, g, r = colors[..., 0].astype(np.uint32), colors[..., 1].astype(np.uint32), colors[..., 2].astype(np.uint32)
        valid = colors.max(axis=2) >= self.min_value
        radius_sq = x * x + y * y
        if self.min_radius > 0.0:
            valid &= radius_sq >= self.min_radius ** 2  # 機体自身・直下の映り込みを除外
        if self.max_radius > 0.0:
            valid &= radius_sq <= self.max_radius ** 2  # BEV外周の低品質点を除外
        rgb = ((r << 16) | (g << 8) | b)
        rgb_float = rgb.view(np.float32)
        points = [
            (float(x[row, col]), float(y[row, col]), 0.0, float(rgb_float[row, col]))
            for row, col in zip(*np.nonzero(valid))
        ]
        if not points:
            return
        header = Header(stamp=msg.header.stamp, frame_id=self.frame_id)
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
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
