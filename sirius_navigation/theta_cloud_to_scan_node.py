#!/usr/bin/env python3
"""3D色付き点群 -> 2D LaserScan（水平スライス）。SLAM Toolboxへ渡すため。

/velodyne_points_colored から各方位ビンの最小距離を取って sensor_msgs/LaserScan を作る。
z帯（センサ基準）で壁・什器の高さだけを残し、床・天井を除外する。

使い方:
  ros2 run sirius_navigation theta_cloud_to_scan_node --ros-args \
      -p input_topic:=/velodyne_points_colored -p output_topic:=/scan_from_cloud
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2


class ThetaCloudToScanNode(Node):
    def __init__(self):
        super().__init__('theta_cloud_to_scan')
        self.declare_parameter('input_topic', '/velodyne_points_colored')
        self.declare_parameter('output_topic', '/scan_from_cloud')
        self.declare_parameter('z_min', -0.5)
        self.declare_parameter('z_max', 0.5)
        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 60.0)
        self.declare_parameter('angle_increment_deg', 0.5)
        self.declare_parameter('frame_id', '')  # 空なら点群のframeを使う

        g = self.get_parameter
        self.z_min = float(g('z_min').value)
        self.z_max = float(g('z_max').value)
        self.range_min = float(g('range_min').value)
        self.range_max = float(g('range_max').value)
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_inc = math.radians(float(g('angle_increment_deg').value))
        self.num_bins = int(round((self.angle_max - self.angle_min) / self.angle_inc))
        self.frame_override = g('frame_id').value

        self.pub = self.create_publisher(LaserScan, g('output_topic').value, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, g('input_topic').value, self.on_cloud,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f'cloud->scan: {g("input_topic").value} -> {g("output_topic").value} '
            f'z=[{self.z_min},{self.z_max}] bins={self.num_bins}')

    def on_cloud(self, msg):
        pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return
        x = pts['x'].astype(np.float64)
        y = pts['y'].astype(np.float64)
        z = pts['z'].astype(np.float64)
        r = np.hypot(x, y)
        keep = (z >= self.z_min) & (z <= self.z_max) & (r >= self.range_min) & (r <= self.range_max)
        if not np.any(keep):
            return
        x, y, r = x[keep], y[keep], r[keep]
        ang = np.arctan2(y, x)
        idx = np.floor((ang - self.angle_min) / self.angle_inc).astype(np.int64)
        valid = (idx >= 0) & (idx < self.num_bins)
        ranges = np.full(self.num_bins, np.inf, dtype=np.float32)
        np.minimum.at(ranges, idx[valid], r[valid].astype(np.float32))

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.frame_override or msg.header.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + self.angle_inc * self.num_bins
        scan.angle_increment = self.angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = ThetaCloudToScanNode()
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
