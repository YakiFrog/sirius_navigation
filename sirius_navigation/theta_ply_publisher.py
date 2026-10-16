#!/usr/bin/env python3
"""PLY(colored点群) -> sensor_msgs/PointCloud2 を配信してRViz2で表示できるようにする。

RViz2はPLYファイルを直接読めないため、本ノードで常駐配信する。
ply_file 未指定なら ~/rosbag2_data/*/colored_map.ply の最新を自動選択。

使い方:
  ros2 run sirius_navigation theta_ply_publisher --ros-args \
      -p ply_file:=$HOME/rosbag2_data/xxx/colored_map.ply -p frame_id:=sirius3/odom
"""
import glob
import os
import struct
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


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


class PlyPublisher(Node):
    def __init__(self):
        super().__init__('theta_ply_publisher')
        home = os.path.expanduser('~')
        self.declare_parameter('ply_file', '')
        self.declare_parameter('topic', '/colored_map')
        self.declare_parameter('geometry_topic', '')
        self.declare_parameter('frame_id', 'sirius3/odom')
        self.declare_parameter('period_sec', 3.0)
        self.declare_parameter('rosbag_dir', os.path.join(home, 'rosbag2_data'))

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
        period = max(0.5, float(self.get_parameter('period_sec').value))
        self.create_timer(period, self.publish)
        self.publish()
        self.get_logger().info(f'配信中: {self.get_parameter("topic").value} frame={self.frame_id}')

    def publish(self):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = 1, self.count
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = self.data.dtype.itemsize
        msg.row_step = self.data.dtype.itemsize * self.count
        msg.is_dense = True
        msg.data = self.data.tobytes()
        self.pub.publish(msg)
        if self.geometry_pub is not None:
            # The RGB bytes become padding; XYZ and point order stay identical.
            msg.fields = msg.fields[:3]
            self.geometry_pub.publish(msg)


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
