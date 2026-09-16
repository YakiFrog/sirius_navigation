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
        self.declare_parameter('stride', 1)
        self.declare_parameter('min_value', 8)
        self.declare_parameter('min_radius', 1.2)
        self.declare_parameter('max_radius', 4.8)
        self.declare_parameter('semantic_topic', '/theta/bev_semantic')
        # SAM3ラベルが同じstampで来るのを待つ上限[s]。過ぎたらsemantic無しで出力。
        # 遅延が大きいとTFキャッシュ(既定10s)を超えて取りこぼすため控えめにする
        # （bag --rate 2.0 では実時間x2の遅延になる点に注意）。
        self.declare_parameter('semantic_wait_sec', 2.5)
        self.declare_parameter('max_pending', 600)
        # デバッグ用: クラス色にした点群のトピック（空文字で無効）
        self.declare_parameter('semantic_debug_topic', '')

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
        debug_topic = self.get_parameter('semantic_debug_topic').value
        self.debug_pub = self.create_publisher(PointCloud2, debug_topic, 1) if debug_topic else None
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
        x = ((0.5 - v) * self.extent).astype(np.float32)  # 前(+)
        y = ((0.5 - u) * self.extent).astype(np.float32)  # 左(+)
        colors = image[grid_rows, grid_cols]
        b = colors[..., 0].astype(np.uint32)
        g = colors[..., 1].astype(np.uint32)
        r = colors[..., 2].astype(np.uint32)
        valid = colors.max(axis=2) >= self.min_value
        radius_sq = x.astype(np.float64) ** 2 + y.astype(np.float64) ** 2
        if self.min_radius > 0.0:
            valid &= radius_sq >= self.min_radius ** 2
        if self.max_radius > 0.0:
            valid &= radius_sq <= self.max_radius ** 2
        rgb_float = (((r << 16) | (g << 8) | b).astype(np.uint32)).view(np.float32)
        label_ok = semantic is not None and semantic.shape == (height, width)
        if label_ok:
            sid_full = semantic[grid_rows, grid_cols]
        else:
            sid_full = np.zeros_like(grid_rows, dtype=np.uint8)

        xv, yv, rv, sv = x[valid], y[valid], rgb_float[valid], sid_full[valid].astype(np.uint8)
        count = xv.size
        if count == 0:
            return
        dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4'), ('semantic_id', 'u1')])
        cloud = np.empty(count, dtype=dtype)
        cloud['x'], cloud['y'], cloud['z'] = xv, yv, 0.0
        cloud['rgb'], cloud['semantic_id'] = rv, sv
        header = Header(stamp=stamp, frame_id=frame_id)
        msg = PointCloud2()
        msg.header = header
        msg.height, msg.width = 1, count
        msg.fields = self.fields
        msg.is_bigendian = False
        msg.point_step = dtype.itemsize
        msg.row_step = dtype.itemsize * count
        msg.is_dense = True
        msg.data = cloud.tobytes()
        self.pub.publish(msg)

        if self.debug_pub is not None:
            debug_rgb = rv.copy()
            for class_id, (red, green, blue) in ((3, (0, 255, 0)), (4, (255, 255, 0)),
                                                 (5, (0, 0, 255)), (6, (128, 128, 128))):
                mask = sv == class_id
                if np.any(mask):
                    packed = np.array([(red << 16) | (green << 8) | blue], np.uint32).view(np.float32)[0]
                    debug_rgb[mask] = packed
            debug_cloud = np.empty(count, dtype=dtype)
            debug_cloud['x'], debug_cloud['y'], debug_cloud['z'] = xv, yv, 0.0
            debug_cloud['rgb'], debug_cloud['semantic_id'] = debug_rgb, sv
            debug_msg = PointCloud2()
            debug_msg.header = header
            debug_msg.height, debug_msg.width = 1, count
            debug_msg.fields = self.fields
            debug_msg.is_bigendian = False
            debug_msg.point_step = dtype.itemsize
            debug_msg.row_step = dtype.itemsize * count
            debug_msg.is_dense = True
            debug_msg.data = debug_cloud.tobytes()
            self.debug_pub.publish(debug_msg)


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
