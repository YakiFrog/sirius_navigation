#!/usr/bin/env python3
"""THETA 路面の indexed color map ノード（ZED/SAM3 と同形式）。

ZEDの sam3_indexed_map_node と同じ「代表色パレット」方式:
  - RTAB-Map の /rtabmap/grid_map を構造として格子を作り、wall/floor を反映
  - /theta/ground_cloud（色付き地面点群）を map フレームへ変換し、
    RGB を 5段量子化した代表色パレットの index として格子へ描画
  - 実RGBは重み付き平均で .texture.png に蓄積

保存（/theta/save_indexed_map にパスを publish）:
  <path>.colored.pgm  … パレットindex (class_id)
  <path>.colored.json … palette / labels / semantic_encoding=class_id
  <path>.texture.png  … 実RGBテクスチャ

これにより既存の sam3_map_colorizer.py・rebase_semantic_map_to_slam.py が
そのまま使える。将来 SAM3 を載せたら semantic_id フィールドを描画するだけで
セマンティック地図へ拡張できる。
"""
import json
import os
import struct

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

# 0=unknown, 1=wall, 2=floor, 3+=semantic class + 代表色（ZEDと同一）
SEMANTIC_CLASSES = {
    0: {"name": "unknown", "color": [127, 127, 127]},
    1: {"name": "wall", "color": [0, 0, 0]},
    2: {"name": "floor", "color": [255, 255, 255]},
    3: {"name": "grass", "color": [0, 255, 0]},
    4: {"name": "tactile paving", "color": [255, 255, 0]},
    5: {"name": "roadway", "color": [0, 0, 255]},
    6: {"name": "sidewalk", "color": [128, 128, 128]},
}


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ThetaIndexedMapNode(Node):
    def __init__(self):
        super().__init__('theta_indexed_map_node')
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('cloud_topic', '/theta/ground_cloud')
        self.declare_parameter('save_topic', '/theta/save_indexed_map')
        self.declare_parameter('grid_topic', '/theta/colored_map_grid')
        self.declare_parameter('struct_map_topic', '/rtabmap/grid_map')
        self.declare_parameter('min_cloud_interval_sec', 0.0)

        self.res = self.get_parameter('grid_resolution').value
        self.map_frame = self.get_parameter('map_frame').value
        self.min_cloud_interval = max(0.0, float(self.get_parameter('min_cloud_interval_sec').value))

        self.grid = None
        self.struct_grid = None
        self.origin = [0.0, 0.0]
        self.width = 0
        self.height = 0
        self.texture_sum = None
        self.texture_count = None
        self.dirty = False
        self.last_cloud_time = 0.0

        self.palette = self._generate_default_palette()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, self.get_parameter('struct_map_topic').value,
                                 self._grid_callback, map_qos)
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                 self._cloud_callback, 10)
        self.create_subscription(String, self.get_parameter('save_topic').value,
                                 self._save_callback, 10)
        self.pub_indexed_grid = self.create_publisher(OccupancyGrid, self.get_parameter('grid_topic').value, 10)
        self.create_timer(2.0, self._timer_callback)
        self.get_logger().info(
            f'THETA indexed map ready: {self.get_parameter("cloud_topic").value} -> '
            f'{self.get_parameter("grid_topic").value} (palette {len(self.palette)} colors)')

    def _generate_default_palette(self):
        steps = [0, 64, 128, 192, 255]
        reserved = [SEMANTIC_CLASSES[i]["color"] for i in sorted(SEMANTIC_CLASSES)]
        colors = []
        for r in steps:
            for g in steps:
                for b in steps:
                    color = [r, g, b]
                    if color not in reserved:
                        colors.append(color)
        palette = np.array(reserved + colors, dtype=np.uint8)
        # 5段量子化RGB -> パレットindex のLUT
        self.color_lut = np.zeros((5, 5, 5), dtype=np.uint8)
        palette_sem = palette[len(reserved):]
        for ri, rv in enumerate(steps):
            for gi, gv in enumerate(steps):
                for bi, bv in enumerate(steps):
                    dist = np.sum((palette_sem - np.array([rv, gv, bv], dtype=np.float32)) ** 2, axis=1)
                    self.color_lut[ri, gi, bi] = np.argmin(dist) + len(reserved)
        return palette

    def _grid_callback(self, msg):
        new_width, new_height = msg.info.width, msg.info.height
        new_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        if (self.grid is None or self.width != new_width or self.height != new_height
                or not np.allclose(self.origin, new_origin, atol=1e-3)):
            old = (self.grid, self.texture_sum, self.texture_count, self.width, self.height, self.origin)
            self.width, self.height = new_width, new_height
            self.origin = new_origin
            self.res = msg.info.resolution
            self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
            self.texture_sum = np.zeros((self.height, self.width, 3), dtype=np.float64)
            self.texture_count = np.zeros((self.height, self.width), dtype=np.uint16)
            if old[0] is not None:
                dx = int(round((old[5][0] - self.origin[0]) / self.res))
                dy = int(round((old[5][1] - self.origin[1]) / self.res))
                sx0, sy0 = max(0, -dx), max(0, -dy)
                sx1, sy1 = min(old[3], self.width - dx), min(old[4], self.height - dy)
                ddx0, ddy0 = max(0, dx), max(0, dy)
                if sx1 > sx0 and sy1 > sy0:
                    src = np.s_[sy0:sy1, sx0:sx1]
                    dst = np.s_[ddy0:ddy0 + (sy1 - sy0), ddx0:ddx0 + (sx1 - sx0)]
                    self.grid[dst] = old[0][src]
                    self.texture_sum[dst] = old[1][src]
                    self.texture_count[dst] = old[2][src]
            self.get_logger().info(f'Grid resized: {self.width}x{self.height}')
        self.struct_grid = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        self.grid[self.struct_grid == 100] = 1
        self.grid[(self.struct_grid == 0) & (self.grid < 3)] = 2
        self.dirty = True

    def _transform_to_map(self, x, y, z, header):
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, header.frame_id, header.stamp, Duration(seconds=0.1))
        except TransformException:
            # 画像/点群のstampが直前のTFより僅かに未来だと外挿エラーになるため、最新TFで再試行する。
            try:
                transform = self.tf_buffer.lookup_transform(self.map_frame, header.frame_id, Time(), Duration(seconds=0.1))
            except TransformException as error:
                self.get_logger().warning(f'TF {self.map_frame} <- {header.frame_id} unavailable: {error}',
                                          throttle_duration_sec=5)
                return None
        t, q = transform.transform.translation, transform.transform.rotation
        rotation = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        points = np.column_stack([x, y, z])
        return points @ rotation.T + np.array([t.x, t.y, t.z])

    def _cloud_callback(self, msg):
        if self.grid is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.min_cloud_interval > 0.0 and now - self.last_cloud_time < self.min_cloud_interval:
            return
        self.last_cloud_time = now
        offsets = {f.name: f.offset for f in msg.fields}
        if 'x' not in offsets or 'y' not in offsets:
            return
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        x = data[:, offsets['x']:offsets['x'] + 4].copy().view(np.float32).flatten()
        y = data[:, offsets['y']:offsets['y'] + 4].copy().view(np.float32).flatten()
        z = (data[:, offsets['z']:offsets['z'] + 4].copy().view(np.float32).flatten()
             if 'z' in offsets else np.zeros_like(x))
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z, data = x[finite], y[finite], z[finite], data[finite]
        if len(x) == 0:
            return
        mapped = self._transform_to_map(x, y, z, msg.header)
        if mapped is None:
            return
        gx = ((mapped[:, 0] - self.origin[0]) / self.res).astype(np.int32)
        gy = ((mapped[:, 1] - self.origin[1]) / self.res).astype(np.int32)
        in_bounds = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)
        gx, gy, data = gx[in_bounds], gy[in_bounds], data[in_bounds]
        if len(gx) == 0:
            return
        color_off = offsets.get('rgb', offsets.get('rgba'))
        if color_off is None:
            return
        r = data[:, color_off + 2].astype(np.float32)
        g = data[:, color_off + 1].astype(np.float32)
        b = data[:, color_off + 0].astype(np.float32)
        ri = np.clip(np.round(r / 63.75).astype(np.int32), 0, 4)
        gi = np.clip(np.round(g / 63.75).astype(np.int32), 0, 4)
        bi = np.clip(np.round(b / 63.75).astype(np.int32), 0, 4)
        indices = self.color_lut[ri, gi, bi]
        not_wall = self.grid[gy, gx] != 1
        self.grid[gy[not_wall], gx[not_wall]] = indices[not_wall]
        # 実RGBの平均を蓄積（.texture.png 用）
        np.add.at(self.texture_sum, (gy, gx), np.column_stack([r, g, b]))
        np.add.at(self.texture_count, (gy, gx), 1)
        self.dirty = True
        self.get_logger().info(f'Painted {int(np.sum(not_wall))} ground points onto grid.',
                               throttle_duration_sec=5.0)

    def _timer_callback(self):
        if self.grid is not None and self.dirty:
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.map_frame
            msg.info.resolution = self.res
            msg.info.width, msg.info.height = self.width, self.height
            msg.info.origin.position.x, msg.info.origin.position.y = self.origin[0], self.origin[1]
            msg.info.origin.orientation.w = 1.0
            msg.data = self.grid.astype(np.int8).flatten().tolist()
            self.pub_indexed_grid.publish(msg)
            self.dirty = False

    def _save_callback(self, msg):
        if self.grid is None:
            return
        path = msg.data
        if not path.startswith('/'):
            path = os.path.join(os.path.expanduser('~/sirius_jazzy_ws/maps_waypoints/maps/'), path)
        # ZED(sam3_indexed_map_node)と同じ流儀: path は "<base>.colored" で渡され、
        # ここで ".pgm" / ".json" を付け、テクスチャは "<base>.texture.png" にする。
        cv2.imwrite(path + ".pgm", self.grid[::-1, :])
        labels = {str(idx): info for idx, info in SEMANTIC_CLASSES.items()}
        meta = {
            "resolution": self.res,
            "origin": self.origin,
            "width": self.width,
            "height": self.height,
            "palette": self.palette.tolist(),
            "labels": labels,
            "semantic_encoding": "class_id",
        }
        with open(path + ".json", 'w') as stream:
            json.dump(meta, stream, indent=4)
        observed = self.texture_count > 0
        if np.any(observed):
            texture_rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            texture_rgb[observed] = np.clip(
                np.round(self.texture_sum[observed] / self.texture_count[observed, None]), 0, 255).astype(np.uint8)
            texture_bgra = np.dstack([texture_rgb[:, :, ::-1], (observed * 255).astype(np.uint8)])
            texture_base = path[:-len('.colored')] if path.endswith('.colored') else path
            cv2.imwrite(texture_base + ".texture.png", texture_bgra[::-1, :])
        self.get_logger().info(f'SUCCESS: Saved indexed map to {path}.pgm / .json')


def main():
    rclpy.init()
    node = ThetaIndexedMapNode()
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
