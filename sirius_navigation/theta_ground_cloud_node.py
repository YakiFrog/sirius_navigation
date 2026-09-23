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
from sensor_msgs.msg import Image, PointCloud2, PointField, LaserScan
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
        # Scan3(2D LiDAR)連動: フリースペース内の地面点だけを残す（壁の向こう/角の裏を除外）
        self.declare_parameter('lidar_topic', '')
        self.declare_parameter('lidar_gate', False)
        self.declare_parameter('lidar_margin_m', 0.25)
        self.declare_parameter('lidar_keep_outside_fov', True)
        self.declare_parameter('lidar_offset', [0.0, 0.0])
        # 自機ボディ等の近距離ビームは障害物とみなさない（誤除外防止）
        self.declare_parameter('lidar_min_obstacle_m', 0.4)
        # 「明確な遮蔽のみ」を除外する比。beam < ratio*distance のときだけ遮蔽扱い
        # （机0.85m等でTHETAは見える地面を誤除外しないため。0.6推奨）
        self.declare_parameter('lidar_occlusion_ratio', 0.6)
        # ゲートで残る点の割合がこれ未満なら、そのフレームはゲートを無効化（過剰除外の防止）
        # 実測の生保持率は約0.55前後なので、それを許容する0.4を既定にする。
        self.declare_parameter('lidar_gate_min_keep_ratio', 0.4)
        # デバッグ用: クラス色にした点群のトピック（空文字で無効）
        self.declare_parameter('semantic_debug_topic', '')
        # semantic待ちをした「色+semantic付き」点群のトピック（空文字で無効）。
        # output_topic は semantic待ちせず即配信する（RTAB等の遅延敏感な消費者向け）。
        self.declare_parameter('semantic_cloud_topic', '/theta/ground_cloud_semantic')
        # 観測品質の重み（入射角cosのべき乗）。地面点に weight(float32) を付与し、
        # 地図側(theta_indexed_map_node)が近距離・斜め視の品質差を重み付けできるようにする。
        # w = (h / sqrt(ρ² + h²)) ** quality_power  （ρ=機体からの水平距離, h=カメラ高）
        self.declare_parameter('quality_weight_enable', True)
        self.declare_parameter('quality_power', 2.0)
        # 0より大きいと weight < floor の点を配信から除外する（既定0=全点保持）
        self.declare_parameter('quality_weight_floor', 0.0)

        with open(self.get_parameter('calibration').value) as f:
            calibration = yaml.safe_load(f)
        self.bev_size = int(calibration['bev_size'])
        self.extent = float(calibration['bev_extent_m'])
        self.camera_height = float(calibration.get('camera_position', [0.0, 0.0, 1.0])[2])
        self.frame_id = self.get_parameter('frame_id').value
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.min_value = int(self.get_parameter('min_value').value)
        self.min_radius = max(0.0, float(self.get_parameter('min_radius').value))
        self.max_radius = max(0.0, float(self.get_parameter('max_radius').value))
        self.semantic_wait = max(0.0, float(self.get_parameter('semantic_wait_sec').value))
        self.max_pending = max(1, int(self.get_parameter('max_pending').value))
        self.lidar_gate = bool(self.get_parameter('lidar_gate').value)
        self.lidar_margin = max(0.0, float(self.get_parameter('lidar_margin_m').value))
        self.lidar_keep_outside_fov = bool(self.get_parameter('lidar_keep_outside_fov').value)
        self.lidar_offset = [float(v) for v in self.get_parameter('lidar_offset').value]
        self.lidar_min_obstacle = max(0.0, float(self.get_parameter('lidar_min_obstacle_m').value))
        self.lidar_occlusion_ratio = float(self.get_parameter('lidar_occlusion_ratio').value)
        self.lidar_min_keep = max(0.0, float(self.get_parameter('lidar_gate_min_keep_ratio').value))
        self.quality_weight_enable = bool(self.get_parameter('quality_weight_enable').value)
        self.quality_power = max(0.0, float(self.get_parameter('quality_power').value))
        self.quality_weight_floor = max(0.0, float(self.get_parameter('quality_weight_floor').value))
        self.latest_scan = None  # (angle_min, angle_increment, ranges, range_max)
        self.scans = {}          # stamp_key -> scan（ゲートの時刻合わせ用）

        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='weight', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='semantic_id', offset=20, datatype=PointField.UINT8, count=1),
        ]
        self.pending = OrderedDict()  # stamp_key -> (stamp, frame_id, image, arrival)
        self.labels = {}              # stamp_key -> mono8 label
        self.warned_shape = False
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, 1)
        semantic_cloud_topic = self.get_parameter('semantic_cloud_topic').value
        self.semantic_pub = (self.create_publisher(PointCloud2, semantic_cloud_topic, 1)
                             if semantic_cloud_topic else None)
        debug_topic = self.get_parameter('semantic_debug_topic').value
        self.debug_pub = self.create_publisher(PointCloud2, debug_topic, 1) if debug_topic else None
        self.sub = self.create_subscription(Image, self.get_parameter('input_topic').value, self.receive, qos_profile_sensor_data)
        self.semantic_sub = self.create_subscription(
            Image, self.get_parameter('semantic_topic').value, self.receive_semantic, qos_profile_sensor_data)
        lidar_topic = self.get_parameter('lidar_topic').value
        if lidar_topic:
            self.create_subscription(LaserScan, lidar_topic, self._on_scan, qos_profile_sensor_data)
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
        # 遅延敏感な消費者（RTAB等）へは semantic を待たず即配信する。
        # semantic待ちで stamp が古くなると TF 外挿で棄却され、開始地点で塗りが止まるため。
        self._emit(msg.header.stamp, msg.header.frame_id or self.frame_id, image, None,
                   publish_main=True, publish_semantic=False)
        while len(self.pending) > self.max_pending:
            old_key, entry = self.pending.popitem(last=False)
            old_stamp, old_frame, old_image, _ = entry
            self._emit(old_stamp, old_frame, old_image, self.labels.pop(old_key, None),
                       publish_main=False, publish_semantic=True)
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
            self._emit(stamp, frame_id, image, self.labels.pop(key),
                       publish_main=False, publish_semantic=True)

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
                self._emit(stamp, frame_id, image, self.labels.pop(key, None),
                           publish_main=False, publish_semantic=True)

    def _on_scan(self, msg):
        data = (msg.angle_min, msg.angle_increment,
                np.asarray(msg.ranges, dtype=np.float32), float(msg.range_max))
        self.latest_scan = data
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.scans[key] = data
        while len(self.scans) > 80:
            self.scans.pop(next(iter(self.scans)))

    def _scan_for(self, stamp):
        """点群のstampに最も近いスキャンを返す（ゲートの時刻ズレ対策）。"""
        if stamp is not None and self.scans:
            target = stamp.sec + stamp.nanosec * 1e-9
            best, best_diff = None, 1e9
            for (s, ns), data in self.scans.items():
                diff = abs((s + ns * 1e-9) - target)
                if diff < best_diff:
                    best, best_diff = data, diff
            if best is not None and best_diff <= 1.0:
                return best
        return self.latest_scan

    def _lidar_free_mask(self, xg, yg, stamp=None):
        """地面点(x,y)[base_footprint] が LiDAR のフリースペース内かを返す。
        ビームが点より手前で障害物に当たっている（beam < range）点と、LiDAR範囲外を除外する。"""
        scan = self._scan_for(stamp)
        if scan is None:
            return np.ones_like(xg, dtype=bool)
        angle_min, angle_inc, ranges, range_max = scan
        if angle_inc == 0.0 or ranges.size == 0:
            return np.ones_like(xg, dtype=bool)
        px = xg - self.lidar_offset[0]
        py = yg - self.lidar_offset[1]
        distances = np.hypot(px, py)
        angles = np.arctan2(py, px)
        index = np.round(np.mod(angles - angle_min, 2.0 * np.pi) / angle_inc).astype(np.int64)
        inside = index < ranges.size
        beam = np.full(distances.shape, np.inf, dtype=np.float32)
        valid = np.isfinite(ranges) & (ranges >= self.lidar_min_obstacle)
        beam[inside] = np.where(valid[index[inside]], ranges[index[inside]], np.inf)
        blocked = (beam < (distances - self.lidar_margin)) & (
            beam < self.lidar_occlusion_ratio * distances)
        out_of_range = distances > range_max
        keep = ~blocked & ~out_of_range
        if self.lidar_keep_outside_fov:
            keep = np.where(inside, keep, True)
        # 過剰除外の防止: 残る割合が低すぎるならこのフレームはゲートしない
        raw_keep = float(keep.mean())
        if self.lidar_min_keep > 0.0 and raw_keep < self.lidar_min_keep:
            self.get_logger().info(
                f'LiDAR gate: raw keep {raw_keep:.2f} < {self.lidar_min_keep:.2f} -> disabled (all kept)',
                throttle_duration_sec=3.0)
            return np.ones_like(xg, dtype=bool)
        self.get_logger().info(
            f'LiDAR gate: raw keep {raw_keep:.2f} (applied)',
            throttle_duration_sec=3.0)
        return keep

    def _emit(self, stamp, frame_id, image, semantic, publish_main=True, publish_semantic=True):
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
        weight = np.ones_like(x, dtype=np.float32)
        if self.quality_weight_enable and self.camera_height > 0.0:
            weight = (self.camera_height / np.sqrt(radius_sq + self.camera_height ** 2)).astype(np.float32)
            if self.quality_power != 1.0:
                weight = np.power(weight, self.quality_power).astype(np.float32)
            if self.quality_weight_floor > 0.0:
                valid &= weight >= self.quality_weight_floor
        if self.lidar_gate and self.latest_scan is not None:
            before = int(np.count_nonzero(valid))
            valid &= self._lidar_free_mask(x, y, stamp)
            self.get_logger().info(
                f'LiDAR gate: kept {int(np.count_nonzero(valid))}/{before} points',
                throttle_duration_sec=3.0)
        rgb_float = (((r << 16) | (g << 8) | b).astype(np.uint32)).view(np.float32)
        label_ok = semantic is not None and semantic.shape == (height, width)
        if label_ok:
            sid_full = semantic[grid_rows, grid_cols]
        else:
            sid_full = np.zeros_like(grid_rows, dtype=np.uint8)

        xv, yv, rv, sv, wv = (x[valid], y[valid], rgb_float[valid],
                              sid_full[valid].astype(np.uint8), weight[valid])
        count = xv.size
        if count == 0:
            return
        dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4'), ('weight', '<f4'), ('semantic_id', 'u1')])
        cloud = np.empty(count, dtype=dtype)
        cloud['x'], cloud['y'], cloud['z'] = xv, yv, 0.0
        cloud['rgb'], cloud['weight'], cloud['semantic_id'] = rv, wv, sv
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
        if publish_main:
            self.pub.publish(msg)
        if publish_semantic and self.semantic_pub is not None:
            self.semantic_pub.publish(msg)

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
            debug_cloud['rgb'], debug_cloud['weight'], debug_cloud['semantic_id'] = debug_rgb, wv, sv
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
