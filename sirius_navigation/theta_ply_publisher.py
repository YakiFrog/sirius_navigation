#!/usr/bin/env python3
"""PLY(colored点群) -> sensor_msgs/PointCloud2 を配信してRViz2で表示できるようにする。

RViz2はPLYファイルを直接読めないため、本ノードで常駐配信する。
ply_file 未指定なら ~/rosbag2_data/*/colored_map.ply の最新を自動選択。

使い方:
  ros2 run sirius_navigation theta_ply_publisher --ros-args \
      -p ply_file:=$HOME/rosbag2_data/xxx/colored_map.ply -p frame_id:=sirius3/odom
"""
import glob
import csv
from decimal import Decimal, InvalidOperation
import os
import struct
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage


def _latest_ply(rosbag_dir):
    files = glob.glob(os.path.join(rosbag_dir, '*', 'colored_map.ply'))
    if not files:
        return ''
    return max(files, key=os.path.getmtime)


def read_ply(path):
    """binary_little_endian / ascii の vertex を持ち、x,y,z(とred,green,blue) を返す。"""
    with open(path, 'rb') as stream:
        if stream.readline().strip() != b'ply':
            raise ValueError('PLYヘッダが不正です')
        fmt = 'ascii'
        n_vertex = 0
        props = []
        in_vertex = False
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PLYヘッダの終端が見つかりません')
            text = line.decode('ascii', 'ignore').strip()
            if text.startswith('format '):
                fmt = text.split()[1]
            elif text.startswith('element '):
                parts = text.split()
                in_vertex = (parts[1] == 'vertex')
                if in_vertex:
                    n_vertex = int(parts[2])
            elif text.startswith('property ') and in_vertex:
                props.append(text.split()[1:])
            elif text == 'end_header':
                break
        names = [p[1] for p in props]
        dtypes = []
        for p in props:
            typ = p[0]
            dtypes.append({'float': '<f4', 'float32': '<f4', 'double': '<f8',
                           'uchar': 'u1', 'uint8': 'u1', 'char': 'i1',
                           'int': '<i4', 'int32': '<i4', 'short': '<i2', 'ushort': '<u2'}.get(typ))
        if n_vertex == 0:
            return np.empty((0, 3), np.float32), np.empty(0, np.uint32)
        if fmt == 'ascii':
            data = np.loadtxt(stream, dtype=np.float64, max_rows=n_vertex, ndmin=2)
            arr = {name: data[:, i] for i, name in enumerate(names)}
        else:
            dtype = np.dtype([(n, d) for n, d in zip(names, dtypes)])
            raw = stream.read(n_vertex * dtype.itemsize)
            rec = np.frombuffer(raw, dtype=dtype, count=n_vertex)
            arr = {name: rec[name] for name in names}
        xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=-1).astype(np.float32)
        if 'red' in arr:
            rgb = np.stack([arr['red'], arr['green'], arr['blue']], axis=-1)
            rgb = np.clip(rgb, 0, 255).astype(np.uint32)
        else:
            rgb = np.zeros((n_vertex, 3), dtype=np.uint32)
        packed = ((rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]).astype(np.uint32)
        return xyz, packed


def read_trajectory(path, frame_id):
    """Read saved map-frame BASE poses; retain XYZ, tilt and recording timestamps."""
    poses = []
    previous = -1
    with open(path, newline='') as stream:
        rows = csv.DictReader(stream)
        if not set(('stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')).issubset(rows.fieldnames or ()):
            raise ValueError('軌跡CSVの列が不足しています')
        for row in rows:
            try:
                ns = int((Decimal(row['stamp'])*1_000_000_000).to_integral_value())
                xyz = np.array([float(row[k]) for k in ('x', 'y', 'z')])
                quat = np.array([float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')])
            except (ValueError, TypeError, InvalidOperation, OverflowError) as error:
                raise ValueError('軌跡CSVに不正な数値があります') from error
            if (not np.isfinite(xyz).all() or not np.isfinite(quat).all()
                    or not np.isclose(np.linalg.norm(quat), 1., atol=1.e-3)
                    or ns < previous or not 0 <= ns < 2**31*1_000_000_000):
                raise ValueError('軌跡CSVの座標・姿勢・時刻が不正です')
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = map(float, quat)
            poses.append(pose)
            previous = ns
    return poses


class PlyPublisher(Node):
    def __init__(self):
        super().__init__('theta_ply_publisher')
        home = os.path.expanduser('~')
        self.declare_parameter('ply_file', '')
        self.declare_parameter('topic', '/colored_map')
        self.declare_parameter('geometry_topic', '')
        self.declare_parameter('removed_topic', '')
        self.declare_parameter('removed_ply_file', '')
        self.declare_parameter('fragment_topic', '')
        self.declare_parameter('trajectory_topic', '')
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('frame_id', 'sirius3/odom')
        self.declare_parameter('period_sec', 3.0)
        self.declare_parameter('z_min', -1.e9)
        self.declare_parameter('z_max', 1.e9)
        self.declare_parameter('rosbag_dir', os.path.join(home, 'rosbag2_data'))
        valid = self.validate_z_parameters([])
        if not valid.successful:
            self.get_logger().error(valid.reason)
            raise SystemExit(1)

        path = self.get_parameter('ply_file').value
        if not path:
            path = _latest_ply(self.get_parameter('rosbag_dir').value)
        if not path or not os.path.exists(path):
            self.get_logger().error(f'PLYが見つかりません: "{path}"')
            raise SystemExit(1)
        xyz, packed = read_ply(path)
        self.get_logger().info(f'PLY読込: {path} ({xyz.shape[0]} points)')

        dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
        self.data = np.empty(xyz.shape[0], dtype=dtype)
        self.data['x'], self.data['y'], self.data['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        self.data['rgb'] = packed.view(np.float32)
        self.count = xyz.shape[0]
        self.frame_id = self.get_parameter('frame_id').value

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('topic').value, qos)
        geometry_topic = self.get_parameter('geometry_topic').value
        self.geometry_pub = self.create_publisher(PointCloud2, geometry_topic, qos) if geometry_topic else None
        self.removed_pub = None
        self.removed_data = np.empty(0, dtype=dtype)
        removed_topic = self.get_parameter('removed_topic').value
        if removed_topic:
            self.removed_pub = self.create_publisher(PointCloud2, removed_topic, qos)
            removed_path = self.get_parameter('removed_ply_file').value
            if not removed_path:
                removed_path = os.path.join(os.path.dirname(path), 'dynamic_candidates.ply')
            if os.path.isfile(removed_path):
                removed_xyz, _ = read_ply(removed_path)
                self.removed_data = np.empty(len(removed_xyz), dtype=dtype)
                for i, name in enumerate(('x', 'y', 'z')):
                    self.removed_data[name] = removed_xyz[:, i]
                # Original RGB remains in the PLY; the diagnostic overlay is red.
                self.removed_data['rgb'] = np.uint32(0xff3030).view(np.float32)
                self.get_logger().info(f'削除対象: {removed_path} ({len(removed_xyz)} points) -> {removed_topic} (赤)')
            else:
                self.get_logger().info('削除対象PLYなし: 空の点群を配信します')
        self.fragment_pub = None
        self.full_fragment_data = np.empty(0, dtype=dtype)
        fragment_topic = self.get_parameter('fragment_topic').value
        if fragment_topic:
            self.fragment_pub = self.create_publisher(PointCloud2, fragment_topic, qos)
            fragment_path = os.path.join(os.path.dirname(path), 'dynamic_fragments.ply')
            if os.path.isfile(fragment_path):
                fragment_xyz, _ = read_ply(fragment_path)
                self.full_fragment_data = np.empty(len(fragment_xyz), dtype=dtype)
                for i, name in enumerate(('x', 'y', 'z')):
                    self.full_fragment_data[name] = fragment_xyz[:, i]
                self.full_fragment_data['rgb'] = np.uint32(0xffa000).view(np.float32)
            self.get_logger().info(f'追加の輪郭除去: {len(self.full_fragment_data)}点 -> {fragment_topic} (オレンジ)')
        self.trajectory_pub = None
        self.trajectory = PathMessage()
        trajectory_topic = self.get_parameter('trajectory_topic').value
        if trajectory_topic:
            trajectory_file = (self.get_parameter('trajectory_file').value
                               or os.path.join(os.path.dirname(path), 'trajectory.csv'))
            self.trajectory.header.frame_id = self.frame_id
            if os.path.isfile(trajectory_file):
                self.trajectory.poses = read_trajectory(trajectory_file, self.frame_id)
                self.get_logger().info(f'推定軌跡: {trajectory_file} ({len(self.trajectory.poses)} poses) -> {trajectory_topic}')
            else:
                self.get_logger().info('trajectory.csvなし: 空の軌跡を配信します')
            self.trajectory_pub = self.create_publisher(PathMessage, trajectory_topic, qos)
        # Keep full clouds in memory so a wider range restores points immediately.
        self.full_data, self.full_removed_data = self.data, self.removed_data
        self.apply_z_filter(publish=False)
        self.add_on_set_parameters_callback(self.validate_z_parameters)
        self.add_post_set_parameters_callback(self.z_parameters_changed)
        period = max(0.5, float(self.get_parameter('period_sec').value))
        self.create_timer(period, self.publish)
        self.publish()
        self.get_logger().info(f'配信中: {self.get_parameter("topic").value} frame={self.frame_id}')

    def validate_z_parameters(self, parameters):
        values = {name: self.get_parameter(name).value for name in ('z_min', 'z_max')}
        for parameter in parameters:
            if parameter.name in values:
                values[parameter.name] = parameter.value
        valid = all(isinstance(v, (int, float)) and not isinstance(v, bool) and np.isfinite(v)
                    for v in values.values())
        if not valid or values['z_min'] > values['z_max']:
            return SetParametersResult(successful=False, reason='z_min <= z_max の有限の高さ[m]を指定してください')
        return SetParametersResult(successful=True)

    def z_parameters_changed(self, parameters):
        if any(p.name in ('z_min', 'z_max') for p in parameters):
            self.apply_z_filter()

    def apply_z_filter(self, publish=True):
        lower, upper = (self.get_parameter(name).value for name in ('z_min', 'z_max'))
        self.data = self.full_data[(self.full_data['z'] >= lower) & (self.full_data['z'] <= upper)]
        self.removed_data = self.full_removed_data[(self.full_removed_data['z'] >= lower)
                                                  & (self.full_removed_data['z'] <= upper)]
        self.fragment_data = self.full_fragment_data[(self.full_fragment_data['z'] >= lower)
                                                     & (self.full_fragment_data['z'] <= upper)]
        self.count = len(self.data)
        self.get_logger().info(f'表示Z範囲 [{lower:g}, {upper:g}] m ({self.frame_id}): '
                               f'保持 {self.count}/{len(self.full_data)}点、'
                               f'削除対象 {len(self.removed_data)}/{len(self.full_removed_data)}点')
        if publish:
            self.publish()

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        msg = self.make_cloud(self.data, stamp)
        self.pub.publish(msg)
        if self.geometry_pub is not None:
            # The RGB bytes become padding; XYZ and point order stay identical.
            msg.fields = msg.fields[:3]
            self.geometry_pub.publish(msg)
        if self.removed_pub is not None:
            self.removed_pub.publish(self.make_cloud(self.removed_data, stamp))
        if self.fragment_pub is not None:
            self.fragment_pub.publish(self.make_cloud(self.fragment_data, stamp))
        if self.trajectory_pub is not None:
            self.trajectory.header.stamp = stamp
            self.trajectory_pub.publish(self.trajectory)

    def make_cloud(self, data, stamp):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = 1, len(data)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = data.dtype.itemsize
        msg.row_step = data.dtype.itemsize * len(data)
        msg.is_dense = True
        msg.data = data.tobytes()
        return msg


def main():
    rclpy.init()
    try:
        node = PlyPublisher()
    except SystemExit:
        if rclpy.ok():
            rclpy.shutdown()
        return
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
