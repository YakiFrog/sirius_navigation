#!/usr/bin/env python3
"""色付き点群をキーフレーム化して蓄積し、SLAMの map->odom でその都度「全体を再描画」する。

通常のオンライン蓄積だと、SLAMのループ閉じ込めで map->odom がジャンプしても
過去の点は直らず継ぎ目が残る。本ノードは:
  - 各スキャンを滑らかな odom フレームでキーフレーム保存（ジャンプ無し）
  - 再描画時に「現在の map->odom」を全キーフレームへ一括適用
    -> ループ閉じ込め後は地図全体が最新の最適化姿勢で作り直される
これにより壁の2重（ドリフト）が解消される。

使い方（例）:
  ros2 run sirius_navigation theta_keyframe_map_node --ros-args \
    -p cloud_topic:=/velodyne_points_colored -p odom_frame:=sirius3/odom \
    -p map_frame:=map -p save_path:=$HOME/colored_map_slam.ply
  （map_frame を空文字にすると odom のまま＝SLAM無し）
SIGINTで最終再描画してPLY保存。
"""
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import (Buffer, TransformListener, LookupException,
                     ConnectivityException, ExtrapolationException)

VOX_OFFSET = 1 << 20
VOX_LIMIT = (1 << 21) - 1


def quat_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([[1 - (yy + zz), xy - wz, xz + wy],
                     [xy + wz, 1 - (xx + zz), yz - wx],
                     [xz - wy, yz + wx, 1 - (xx + yy)]])


def yaw_of(rot):
    return float(np.arctan2(rot[1, 0], rot[0, 0]))


def voxel_reduce(xyz, colors, voxel):
    if xyz.shape[0] == 0:
        return xyz, colors
    q = np.floor(xyz / voxel).astype(np.int64) + VOX_OFFSET
    np.clip(q, 0, VOX_LIMIT, out=q)
    keys = (q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2]
    uniq, inv = np.unique(keys, return_inverse=True)
    inv = inv.reshape(-1)
    m = uniq.shape[0]
    cnt = np.bincount(inv, minlength=m).astype(np.float64)
    out_xyz = np.empty((m, 3))
    out_col = np.empty((m, 3))
    for i in range(3):
        out_xyz[:, i] = np.bincount(inv, weights=xyz[:, i], minlength=m) / cnt
        out_col[:, i] = np.bincount(inv, weights=colors[:, i], minlength=m) / cnt
    return out_xyz, out_col


def write_ply(path, xyz, rgb_u8):
    n = xyz.shape[0]
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {n}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                      ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    arr = np.empty(n, dtype=dtype)
    arr['x'], arr['y'], arr['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    arr['red'], arr['green'], arr['blue'] = rgb_u8[:, 0], rgb_u8[:, 1], rgb_u8[:, 2]
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(arr.tobytes())


class ThetaKeyframeMapNode(Node):
    def __init__(self):
        super().__init__('theta_keyframe_map')
        home = os.path.expanduser('~')
        self.declare_parameter('cloud_topic', '/velodyne_points_colored')
        self.declare_parameter('odom_frame', 'sirius3/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('output_topic', '/colored_map_3d')
        self.declare_parameter('voxel', 0.05)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 60.0)
        self.declare_parameter('min_keyframe_dist', 0.25)
        self.declare_parameter('min_keyframe_yaw_deg', 8.0)
        self.declare_parameter('min_keyframe_interval', 0.4)
        self.declare_parameter('rebuild_period', 5.0)
        self.declare_parameter('autosave_period', 0.0)  # >0 で定期保存
        self.declare_parameter('save_path', os.path.join(home, 'colored_map_slam.ply'))
        self.declare_parameter('max_keyframes', 3000)

        g = self.get_parameter
        self.odom_frame = g('odom_frame').value
        self.map_frame = g('map_frame').value
        self.voxel = float(g('voxel').value)
        self.min_range = float(g('min_range').value)
        self.max_range = float(g('max_range').value)
        self.min_dist = float(g('min_keyframe_dist').value)
        self.min_yaw = np.deg2rad(float(g('min_keyframe_yaw_deg').value))
        self.min_interval = float(g('min_keyframe_interval').value)
        self.save_path = g('save_path').value
        self.max_keyframes = int(g('max_keyframes').value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.listener = TransformListener(self.tf_buffer, self)

        self.key_xyz = deque()   # np arrays in odom frame
        self.key_col = deque()
        self.last_pose = None
        self.last_kf_time = 0.0
        self.scans = 0
        self.skipped_tf = 0
        self.last_global = None  # 直近の正常な map->odom（終了時にTFが取れない場合の保険）
        self.dirty = False       # 新しいキーフレームが追加されたか

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PointCloud2, g('output_topic').value, qos)
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.create_subscription(PointCloud2, g('cloud_topic').value, self.on_cloud,
                                 qos_profile_sensor_data)
        rp = max(1.0, float(g('rebuild_period').value))
        self.create_timer(rp, self.rebuild)
        asp = float(g('autosave_period').value)
        if asp > 0:
            self.create_timer(asp, lambda: self.save(self.save_path))
        self.get_logger().info(
            f'keyframe map: {g("cloud_topic").value} odom={self.odom_frame} '
            f'map="{self.map_frame}" voxel={self.voxel} -> {self.save_path}')

    def _lookup(self, target, source, stamp):
        try:
            tr = self.tf_buffer.lookup_transform(target, source, stamp, Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        return quat_to_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def on_cloud(self, msg):
        pts = pc2.read_points(msg, field_names=None, skip_nans=True)
        if pts.shape[0] == 0:
            return
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1).astype(np.float64)
        packed = np.asarray(pts['rgb'], dtype=np.float32).view(np.uint32)
        colors = np.stack([((packed >> 16) & 255), ((packed >> 8) & 255), (packed & 255)],
                          axis=-1).astype(np.float64) / 255.0
        if self.min_range > 0 or self.max_range > 0:
            dist = np.linalg.norm(xyz, axis=-1)
            keep = np.ones(xyz.shape[0], bool)
            if self.min_range > 0:
                keep &= dist >= self.min_range
            if self.max_range > 0:
                keep &= dist <= self.max_range
            xyz, colors = xyz[keep], colors[keep]
        if xyz.shape[0] == 0:
            return
        src = msg.header.frame_id or (self.map_frame or self.odom_frame)
        stamp = Time.from_msg(msg.header.stamp)
        # オフライン再生では雲とTFの到達順が前後して補間が外れることがあるため、
        # まずスキャン時刻、失敗したら最新TFにフォールバックする。
        tfm = self._lookup(self.odom_frame, src, stamp)
        if tfm is None:
            tfm = self._lookup(self.odom_frame, src, Time())
        if tfm is None:
            self.skipped_tf += 1
            if self.skipped_tf % 50 == 1:
                self.get_logger().warning(
                    f'TF {self.odom_frame} <- {src} 取得待ち ({self.skipped_tf})')
            return
        rot, trans = tfm
        xyz_odom = xyz @ rot.T + trans
        now = time.time()
        if not self._is_keyframe(rot, trans, now):
            return
        sx, sc = voxel_reduce(xyz_odom, colors, self.voxel)
        self.key_xyz.append(sx)
        self.key_col.append(sc)
        while len(self.key_xyz) > self.max_keyframes:
            self.key_xyz.popleft(); self.key_col.popleft()
        self.last_pose = (rot, trans)
        self.last_kf_time = now
        self.scans += 1
        self.dirty = True

    def _is_keyframe(self, rot, trans, now):
        if self.last_pose is None:
            return True
        if now - self.last_kf_time < self.min_interval:
            return False
        lrot, lt = self.last_pose
        dist = float(np.linalg.norm(trans - lt))
        dyaw = abs(yaw_of(rot) - yaw_of(lrot))
        dyaw = min(dyaw, 2 * np.pi - dyaw)
        return dist >= self.min_dist or dyaw >= self.min_yaw

    def _global_transform(self):
        """map->odom を返す。map_frame が空なら恒等（SLAM無し=odomのまま）。"""
        if not self.map_frame:
            return np.eye(3), np.zeros(3)
        tfm = self._lookup(self.map_frame, self.odom_frame, Time())
        if tfm is None:
            return self.last_global
        self.last_global = tfm
        return tfm

    def _assemble(self):
        if not self.key_xyz:
            return None, None
        tfm = self._global_transform()
        if tfm is None:
            return None, None
        rot, trans = tfm
        xs = [k @ rot.T + trans for k in self.key_xyz]
        xyz = np.concatenate(xs)
        col = np.concatenate(self.key_col)
        return voxel_reduce(xyz, col, self.voxel)

    def rebuild(self):
        if not self.dirty:
            return
        xyz, col = self._assemble()
        if xyz is None:
            return
        self.dirty = False
        rgb = np.clip(np.round(col * 255.0), 0, 255).astype(np.uint32)
        packed = ((rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]).astype(np.uint32).view(np.float32)
        dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
        data = np.empty(xyz.shape[0], dtype=dtype)
        data['x'], data['y'], data['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        data['rgb'] = packed
        msg = PointCloud2()
        msg.header = Header(stamp=self.get_clock().now().to_msg(),
                            frame_id=self.map_frame or self.odom_frame)
        msg.height, msg.width = 1, xyz.shape[0]
        msg.fields = self.fields
        msg.is_bigendian = False
        msg.point_step = dtype.itemsize
        msg.row_step = dtype.itemsize * xyz.shape[0]
        msg.is_dense = True
        msg.data = data.tobytes()
        self.pub.publish(msg)
        self.get_logger().info(
            f'rebuild: keyframes={len(self.key_xyz)} points={xyz.shape[0]} (scans={self.scans})',
            throttle_duration_sec=3.0)

    def save(self, path=None):
        path = path or self.save_path
        xyz, col = self._assemble()
        if xyz is None:
            self.get_logger().warning('保存対象がありません')
            return
        rgb = np.clip(np.round(col * 255.0), 0, 255).astype(np.uint8)
        write_ply(path, xyz, rgb)
        self.get_logger().info(f'SAVED: {path} ({xyz.shape[0]} points)')


def main():
    rclpy.init()
    node = ThetaKeyframeMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save()
        except Exception as error:  # noqa: BLE001
            node.get_logger().error(f'保存失敗: {error}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
