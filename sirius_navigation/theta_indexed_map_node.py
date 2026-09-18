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
from scipy import ndimage
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
MAX_CLASS_ID = 6


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
        # 保存時、自機マスク等で残った小さな穴(floor/unknown)を周囲の路面クラスで埋める最大距離[m]。
        # 0で無効。画像端に連結した本物の未知/床は対象外（囲まれた穴のみ）。
        self.declare_parameter('fill_hole_m', 0.75)
        # テクスチャに使う直近観測の枚数K（移動平均）。大きいほど滑らかだがブレが累積しやすい。
        self.declare_parameter('texture_samples', 5)
        # 直近K観測の集約方法: 'median' / 'mean' /
        # 'weighted'（視点品質で重み付け: cos(入射角)/(1+r^2), 斜めボケに強い）/
        # 'best'（最良視点=最大重みの観測を採用, 最もシャープ）/
        # 'soft'（直前K観測を視点重み w^γ で重み付き平均。γ大でbest寄り。段差を平滑化）。
        self.declare_parameter('texture_aggregate', 'soft')
        # 'soft' の重み強調指数γ。1=weighted（滑らか）, 大きいほど最良観測寄り（シャープ）。
        self.declare_parameter('texture_soft_gamma', 6.0)
        # 重み計算に使うセンサ高さ[m]（カメラ高さ）。camera_position zと一致させる。
        self.declare_parameter('sensor_height', 1.135)

        self.res = self.get_parameter('grid_resolution').value
        self.map_frame = self.get_parameter('map_frame').value
        self.min_cloud_interval = max(0.0, float(self.get_parameter('min_cloud_interval_sec').value))
        self.fill_hole_m = max(0.0, float(self.get_parameter('fill_hole_m').value))
        self.texture_k = max(1, int(self.get_parameter('texture_samples').value))
        self.texture_aggregate = str(self.get_parameter('texture_aggregate').value).lower()
        self.sensor_height = max(1e-3, float(self.get_parameter('sensor_height').value))

        self.grid = None
        self.struct_grid = None
        self.struct_origin = None
        self.origin = [0.0, 0.0]
        self.width = 0
        self.height = 0
        self.texture_ring = None   # (H,W,K,3) uint8 直近K枚
        self.texture_sum = None    # (H,W,3) 直近K枚の合計
        self.texture_count = None  # (H,W) 総観測数
        self.texture_weight_ring = None  # (H,W,K) 直近K枚の視点重み
        self.texture_weight_sum = None   # (H,W) 直近K枚の重み合計
        self.texture_wcolor_sum = None   # (H,W,3) 直近K枚の weighted color 合計
        self.best_weight = None          # (H,W) 最良視点の重み
        self.best_rgb = None             # (H,W,3) 最良視点の色
        self.semantic_votes = None
        self.dirty = False
        self.last_cloud_time = 0.0

        self.palette = self._generate_default_palette()
        # 地面点群はsemantic_waitで遅延するため、高レート再生でもTFが引けるよう
        # tf2バッファのキャッシュを拡大（既定10sだと 遅延×rate で外挿エラーになる）。
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
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

    def _alloc_canvas(self, ox, oy, width, height, res):
        self.origin = [float(ox), float(oy)]
        self.res = float(res)
        self.width = int(max(1, width))
        self.height = int(max(1, height))
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self.texture_ring = np.zeros((self.height, self.width, self.texture_k, 3), dtype=np.uint8)
        self.texture_sum = np.zeros((self.height, self.width, 3), dtype=np.float64)
        self.texture_count = np.zeros((self.height, self.width), dtype=np.int32)
        self.texture_weight_ring = np.zeros((self.height, self.width, self.texture_k), dtype=np.float32)
        self.texture_weight_sum = np.zeros((self.height, self.width), dtype=np.float64)
        self.texture_wcolor_sum = np.zeros((self.height, self.width, 3), dtype=np.float64)
        self.best_weight = np.zeros((self.height, self.width), dtype=np.float64)
        self.best_rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.semantic_votes = np.zeros((self.height, self.width, MAX_CLASS_ID + 1), dtype=np.float32)

    def _expand_canvas(self, ox, oy, width, height):
        """キャンバスを拡大（既存の描画データを保持）。縮小はしない。"""
        old = (self.grid, self.texture_ring, self.texture_sum, self.texture_count, self.semantic_votes,
               self.texture_weight_ring, self.texture_weight_sum, self.texture_wcolor_sum,
               self.best_weight, self.best_rgb,
               self.origin[0], self.origin[1], self.width, self.height)
        dx = int(round((old[10] - ox) / self.res))
        dy = int(round((old[11] - oy) / self.res))
        self._alloc_canvas(ox, oy, width, height, self.res)
        sx0, sy0 = max(0, -dx), max(0, -dy)
        sx1, sy1 = min(old[12], self.width - dx), min(old[13], self.height - dy)
        ddx0, ddy0 = max(0, dx), max(0, dy)
        if sx1 > sx0 and sy1 > sy0:
            src = np.s_[sy0:sy1, sx0:sx1]
            dst = np.s_[ddy0:ddy0 + (sy1 - sy0), ddx0:ddx0 + (sx1 - sx0)]
            self.grid[dst] = old[0][src]
            self.texture_ring[dst] = old[1][src]
            self.texture_sum[dst] = old[2][src]
            self.texture_count[dst] = old[3][src]
            self.semantic_votes[dst] = old[4][src]
            self.texture_weight_ring[dst] = old[5][src]
            self.texture_weight_sum[dst] = old[6][src]
            self.texture_wcolor_sum[dst] = old[7][src]
            self.best_weight[dst] = old[8][src]
            self.best_rgb[dst] = old[9][src]

    def _grid_callback(self, msg):
        # 構造（壁/フリー）は別配列に保持するだけにし、描画キャンバスには触れない。
        # （RTABのgrid_mapはサイズ/原点が変動するため、共有すると描画が壊れる）
        self.struct_grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.struct_origin = [float(msg.info.origin.position.x), float(msg.info.origin.position.y)]
        self.dirty = True

    def _ensure_canvas_for(self, xs, ys, margin_m=1.0):
        """描画する地図座標の範囲を含むようキャンバスを拡大（縮小はしない）。"""
        if xs.size == 0:
            return
        res = self.res if self.res else 0.05
        mnx, mxx = float(xs.min()), float(xs.max())
        mny, mxy = float(ys.min()), float(ys.max())
        if self.grid is None:
            ox, oy = mnx - margin_m, mny - margin_m
            w = int(round((mxx - ox + margin_m) / res)) + 1
            h = int(round((mxy - oy + margin_m) / res)) + 1
            self._alloc_canvas(ox, oy, w, h, res)
            self.get_logger().info(f'Grid canvas created: {self.width}x{self.height}')
            return
        x0, y0 = self.origin
        x1, y1 = x0 + self.width * res, y0 + self.height * res
        ux0 = min(x0, mnx - margin_m)
        uy0 = min(y0, mny - margin_m)
        ux1 = max(x1, mxx + margin_m)
        uy1 = max(y1, mxy + margin_m)
        if ux0 < x0 - 1e-9 or uy0 < y0 - 1e-9 or ux1 > x1 + 1e-9 or uy1 > y1 + 1e-9:
            self._expand_canvas(ux0, uy0, int(round((ux1 - ux0) / res)), int(round((uy1 - uy0) / res)))
            self.get_logger().info(f'Grid canvas expanded: {self.width}x{self.height}')

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
        if not getattr(self, '_first_cloud_logged', False):
            self._first_cloud_logged = True
            self.get_logger().info(
                f'[cloud] first ground cloud received (frame={msg.header.frame_id}, '
                f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec})')
        if self.grid is None and not getattr(self, '_no_tf_logged', False):
            self._no_tf_logged = True
            self.get_logger().info('[cloud] painting directly (grid=None will be allocated on first successful transform)')
        now = self.get_clock().now().nanoseconds / 1e9
        if self.min_cloud_interval > 0.0 and now - self.last_cloud_time < self.min_cloud_interval:
            return
        self.last_cloud_time = now
        if not hasattr(self, 'color_lut'):
            self.palette = self._generate_default_palette()
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
        self._ensure_canvas_for(mapped[:, 0], mapped[:, 1])
        gx = ((mapped[:, 0] - self.origin[0]) / self.res).astype(np.int32)
        gy = ((mapped[:, 1] - self.origin[1]) / self.res).astype(np.int32)
        in_bounds = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)
        gx, gy, data, xr, yr = (gx[in_bounds], gy[in_bounds], data[in_bounds],
                                x[in_bounds], y[in_bounds])
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
        rgb_u8 = np.column_stack([r, g, b]).astype(np.uint8)
        # 代表色パレットで路面を描く（wallは保護）。セマンティックは別途フレーム間投票で集約。
        not_wall = self.grid[gy, gx] != 1
        self.grid[gy[not_wall], gx[not_wall]] = indices[not_wall]
        # 視点品質の重み: cos(入射角)/(1+r^2)。近く・正対した観測ほど高重み（斜めボケに強い）。
        h2 = self.sensor_height ** 2
        r2 = xr.astype(np.float64) ** 2 + yr.astype(np.float64) ** 2
        weight = (self.sensor_height / np.sqrt(r2 + h2)) / (1.0 + r2)
        # 直近K枚の移動平均テクスチャ。同一フレーム内の複数点はセルごとに重み付き平均し、
        # 1セル=1観測としてリングへ入れる（点数で重複加算して白飛びするのを防ぐ）。
        flat = gy.astype(np.int64) * self.width + gx
        uniq, inverse = np.unique(flat, return_inverse=True)
        frame_counts = np.bincount(inverse)
        rgb_f = rgb_u8.astype(np.float64)
        frame_wcolor = np.zeros((uniq.size, 3), dtype=np.float64)
        frame_wnorm = np.zeros(uniq.size, dtype=np.float64)
        np.add.at(frame_wcolor, inverse, weight[:, None] * rgb_f)
        np.add.at(frame_wnorm, inverse, weight)
        np.maximum(frame_wnorm, 1e-12, out=frame_wnorm)
        cell_rgb = np.clip(np.round(frame_wcolor / frame_wnorm[:, None]), 0, 255).astype(np.uint8)
        cell_w = frame_wnorm / np.maximum(frame_counts, 1)
        uy, ux = np.divmod(uniq, self.width)
        np.add.at(self.texture_count, (uy, ux), 1)
        slot = (self.texture_count[uy, ux] - 1) % self.texture_k
        previous = self.texture_ring[uy, ux, slot].astype(np.float64)
        self.texture_ring[uy, ux, slot] = cell_rgb
        np.add.at(self.texture_sum, (uy, ux), cell_rgb.astype(np.float64) - previous)
        prev_w = self.texture_weight_ring[uy, ux, slot].astype(np.float64)
        self.texture_weight_ring[uy, ux, slot] = cell_w
        np.add.at(self.texture_weight_sum, (uy, ux), cell_w - prev_w)
        np.add.at(self.texture_wcolor_sum, (uy, ux),
                  cell_w[:, None] * cell_rgb.astype(np.float64) - prev_w[:, None] * previous)
        # 最良視点（最大重み）の色を保持（winner-take-all）
        better = cell_w > self.best_weight[uy, ux]
        if np.any(better):
            self.best_weight[uy[better], ux[better]] = cell_w[better]
            self.best_rgb[uy[better], ux[better]] = cell_rgb[better]
        # セマンティックはフレーム間でクラスIDの投票（予約ID>=3）
        if 'semantic_id' in offsets:
            sid = data[:, offsets['semantic_id']].astype(np.int32)
            keep = sid >= 3
            if np.any(keep):
                np.add.at(self.semantic_votes, (gy[keep], gx[keep], sid[keep]), 1.0)
        self.dirty = True

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

    def _fill_small_holes(self, grid):
        """自機マスク等で残った小さな穴(0/2)を、囲んでいる路面クラス(>=3)で埋める。"""
        fill_px = int(round(self.fill_hole_m / self.res)) if self.fill_hole_m > 0 else 0
        if fill_px <= 0:
            return grid
        fillable = (grid == 0) | (grid == 2)
        if not np.any(fillable) or not np.any(~fillable):
            return grid
        dist, indices = ndimage.distance_transform_edt(fillable, return_distances=True, return_indices=True)
        nearest = grid[tuple(indices)]
        # 画像端に連結した背景（本物の未知/床）は除外し、囲まれた穴だけ対象にする
        components, _ = ndimage.label(fillable)
        border = np.concatenate([components[0, :], components[-1, :], components[:, 0], components[:, -1]])
        enclosed = fillable & ~np.isin(components, np.unique(border))
        hole = enclosed & (dist <= fill_px) & (nearest >= 3)
        if not np.any(hole):
            return grid
        filled = grid.copy()
        filled[hole] = nearest[hole]
        return filled

    def _aggregate_texture(self):
        """直近K観測(RGB)をセルごとに集約する。
        median: エッジ/二重像に強い / mean: 単純平均 /
        weighted: 視点品質 w=cos(入射角)/(1+r^2) の重み付き平均（斜めボケに強い）/
        best: 最良視点（最大重み）の色を採用（ゴースト最小）/
        soft: 直近K観測を w^γ で重み付き平均（γ大でbest寄り、選択の段差を平滑化）。"""
        out = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        mode = str(self.get_parameter('texture_aggregate').value).lower()
        if mode == 'best':
            observed = self.best_weight > 0.0
            out[observed] = self.best_rgb[observed]
            return out
        if mode == 'weighted':
            observed = self.texture_weight_sum > 1e-9
            if np.any(observed):
                out[observed] = np.clip(
                    np.round(self.texture_wcolor_sum[observed] / self.texture_weight_sum[observed, None]),
                    0, 255).astype(np.uint8)
            return out
        if mode == 'soft':
            gamma = max(1.0, float(self.get_parameter('texture_soft_gamma').value))
            n = np.minimum(self.texture_count, self.texture_k)
            observed = n > 0
            if not np.any(observed):
                return out
            num = np.zeros((self.height, self.width, 3), dtype=np.float32)
            den = np.zeros((self.height, self.width), dtype=np.float32)
            slot_axis = np.arange(self.texture_k)
            for k in range(self.texture_k):
                w = np.where(slot_axis[k] < n, self.texture_weight_ring[:, :, k], 0.0).astype(np.float32)
                wk = w ** gamma
                num += wk[:, :, None] * self.texture_ring[:, :, k].astype(np.float32)
                den += wk
            safe = np.maximum(den, 1e-12)
            out[observed] = np.clip(np.round(num[observed] / safe[observed, None]), 0, 255).astype(np.uint8)
            return out
        count = self.texture_count
        observed = count > 0
        if not np.any(observed):
            return out
        n = np.minimum(count, self.texture_k)
        if mode != 'median':
            denominator = n.astype(np.float64)
            out[observed] = np.clip(
                np.round(self.texture_sum[observed] / denominator[observed, None]), 0, 255).astype(np.uint8)
            return out
        # 下側中央値: 無効スロットを +inf 相当(=1000)にして昇順ソートすると、
        # 有効な n 個が先頭に並ぶので index (n-1)//2 が下側中央値になる。
        slot_axis = np.arange(self.texture_k)
        valid = slot_axis[None, None, :] < n[:, :, None]
        vals = np.where(valid[:, :, :, None], self.texture_ring.astype(np.int16), np.int16(1000))
        vals.sort(axis=2)
        med_index = np.clip((n - 1) // 2, 0, self.texture_k - 1)
        rows, cols = np.indices((self.height, self.width))
        median_rgb = vals[rows, cols, med_index]
        out[observed] = np.clip(median_rgb[observed], 0, 255).astype(np.uint8)
        return out

    def _save_callback(self, msg):
        self.get_logger().info(
            f'[save] request received: {msg.data} '
            f'(grid={"set" if self.grid is not None else "None"}, '
            f'painted={int(np.sum(self.grid >= 3)) if self.grid is not None else 0})')
        if self.grid is None:
            self.get_logger().warning('[save] grid is None（描画前に保存要求）。保存をスキップします。')
            return
        path = msg.data
        if not path.startswith('/'):
            path = os.path.join(os.path.expanduser('~/sirius_jazzy_ws/maps_waypoints/maps/'), path)
        # ZED(sam3_indexed_map_node)と同じ流儀: path は "<base>.colored" で渡され、
        # ここで ".pgm" / ".json" を付け、テクスチャは "<base>.texture.png" にする。
        # セマンティック投票と代表色を合成（投票があるセルは予約IDで上書き）
        out_grid = self.grid.copy()
        if self.semantic_votes is not None:
            best = np.argmax(self.semantic_votes, axis=2).astype(np.uint8)
            total = np.sum(self.semantic_votes, axis=2)
            semantic_mask = (total >= 1.0) & (best >= 3)
            out_grid[semantic_mask] = best[semantic_mask]
        out_grid = self._fill_small_holes(out_grid)
        cv2.imwrite(path + ".pgm", out_grid[::-1, :])
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
        # ロバストテクスチャ（texture_aggregate=median なら直近Kの下側中央値）
        observed = self.texture_count > 0
        if np.any(observed):
            texture_rgb = self._aggregate_texture()
            texture_bgra = np.dstack([texture_rgb[:, :, ::-1], (observed * 255).astype(np.uint8)])
            texture_base = path[:-len('.colored')] if path.endswith('.colored') else path
            cv2.imwrite(texture_base + ".texture.png", texture_bgra[::-1, :])
        self.get_logger().info(f'SUCCESS: Saved indexed map to {path}.pgm / .json')
        self.get_logger().info(
            f'[save] grid={self.width}x{self.height} origin={self.origin} '
            f'painted(>=3)={int(np.sum(self.grid >= 3))} '
            f'saved_painted(>=3)={int(np.sum(out_grid >= 3))} '
            f'struct_idx={getattr(self, "struct_grid", None) is not None}')


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
