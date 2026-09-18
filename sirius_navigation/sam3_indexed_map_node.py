import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import String, Header
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import numpy as np
import struct
import json
import os
import cv2

from sirius_navigation.sam3_texture_fusion import (
    estimate_overlap_exposure_gain,
    fill_small_texture_holes,
    robust_color_residual_weights,
    weighted_texture_from_statistics,
)
from sirius_navigation.semantic_costs import load_semantic_costs, costs_for

# クラス名 → 代表色。コストは config/semantic_costs.yaml から読み込む。
SEMANTIC_CLASSES = {
    0: {"name": "unknown", "color": [127, 127, 127]},
    1: {"name": "wall", "color": [0, 0, 0]},
    2: {"name": "floor", "color": [255, 255, 255]},
    3: {"name": "grass", "color": [0, 255, 0]},
    4: {"name": "tactile paving", "color": [255, 255, 0]},
    5: {"name": "roadway", "color": [0, 0, 255]},
    6: {"name": "sidewalk", "color": [128, 128, 128]},
}


def robust_texture_from_statistics(color_sum, color_min, color_max, count):
    """Return a per-cell trimmed mean, rejecting one bright and dark view."""
    result = np.zeros(color_sum.shape, dtype=np.uint8)
    observed = count > 0
    trimmed = count > 2
    regular = observed & ~trimmed

    if np.any(regular):
        result[regular] = np.clip(
            np.round(color_sum[regular] / count[regular, None]),
            0,
            255,
        ).astype(np.uint8)
    if np.any(trimmed):
        result[trimmed] = np.clip(
            np.round(
                (
                    color_sum[trimmed]
                    - color_min[trimmed]
                    - color_max[trimmed]
                )
                / (count[trimmed, None] - 2)
            ),
            0,
            255,
        ).astype(np.uint8)
    return result


class SAM3IndexedMapNode(Node):
    def __init__(self):
        super().__init__('sam3_indexed_map_node')
        
        # Parameters
        self.declare_parameter('palette_size', 256)
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('semantic_cloud_topic', '/sam3/full_cloud_semantic')
        self.declare_parameter('accumulate_real_texture', True)
        self.declare_parameter('texture_min_height_m', -0.15)
        self.declare_parameter('texture_max_height_m', 0.25)
        self.declare_parameter('texture_min_range_m', 0.5)
        self.declare_parameter('texture_max_range_m', 7.0)
        self.declare_parameter('texture_exposure_compensation', True)
        self.declare_parameter('texture_robust_delta', 35.0)
        self.declare_parameter('texture_max_fusion_weight', 8.0)
        self.declare_parameter('texture_fill_small_holes', True)
        self.declare_parameter('min_cloud_interval_sec', 2.0)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        
        self.res = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.semantic_cloud_topic = self.get_parameter('semantic_cloud_topic').get_parameter_value().string_value
        self.accumulate_real_texture = (
            self.get_parameter('accumulate_real_texture').get_parameter_value().bool_value
        )
        self.texture_min_height = (
            self.get_parameter('texture_min_height_m').get_parameter_value().double_value
        )
        self.texture_max_height = (
            self.get_parameter('texture_max_height_m').get_parameter_value().double_value
        )
        self.texture_min_range = (
            self.get_parameter('texture_min_range_m').get_parameter_value().double_value
        )
        self.texture_max_range = (
            self.get_parameter('texture_max_range_m').get_parameter_value().double_value
        )
        self.texture_exposure_compensation = (
            self.get_parameter('texture_exposure_compensation').get_parameter_value().bool_value
        )
        self.texture_robust_delta = (
            self.get_parameter('texture_robust_delta').get_parameter_value().double_value
        )
        self.texture_max_fusion_weight = (
            self.get_parameter('texture_max_fusion_weight').get_parameter_value().double_value
        )
        self.texture_fill_small_holes = (
            self.get_parameter('texture_fill_small_holes').get_parameter_value().bool_value
        )
        
        # Internal State
        self.grid = None       # uint8 array (indexed map)
        self.struct_grid = None # Original occupancy grid for reference
        self.origin = [0.0, 0.0]
        self.width = 0
        self.height = 0
        self.texture_sum = None
        self.texture_min = None
        self.texture_max = None
        self.texture_count = None
        self.texture_weight = None
        self.texture_reference_luma = None
        self.current_motion_weight = 1.0
        self.previous_texture_pose = None
        self.dirty = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Performance tuning: Rate limit for cloud processing
        self.last_semantic_cloud_time = 0.0
        self.last_legacy_cloud_time = 0.0
        self.semantic_cloud_received = False
        self.min_cloud_interval = max(
            0.0,
            self.get_parameter('min_cloud_interval_sec').get_parameter_value().double_value,
        )

        # QoS for persistent map topics
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.sub_grid = self.create_subscription(OccupancyGrid, '/rtabmap/grid_map', self._grid_callback, map_qos)
        self.sub_cloud = self.create_subscription(PointCloud2, '/cloud_map', self._cloud_callback, map_qos)
        self.sub_semantic_cloud = self.create_subscription(
            PointCloud2,
            self.semantic_cloud_topic,
            self._cloud_callback,
            10,
        )
        self.sub_save = self.create_subscription(String, '/sam3/save_indexed_map', self._save_callback, 10)
        
        # Publishers
        self.pub_indexed_grid = self.create_publisher(OccupancyGrid, '/sam3/colored_map_grid', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(2.0, self._timer_callback)
        self.latest_stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info('SAM3 Mapping Node: Reverted to Stable Reliable Mode.')

    def _grid_callback(self, msg):
        """Handle incoming structural map."""
        new_width = msg.info.width
        new_height = msg.info.height
        new_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        
        # Re-initialize if dimensions or origin changed significantly
        if (self.grid is None or 
            self.width != new_width or self.height != new_height or 
            not np.allclose(self.origin, new_origin, atol=1e-3)):
            
            old_grid, old_width, old_height, old_origin = (
                self.grid,
                self.width,
                self.height,
                self.origin,
            )
            old_texture = (
                self.texture_sum,
                self.texture_min,
                self.texture_max,
                self.texture_count,
                self.texture_weight,
            )
            
            self.width, self.height = new_width, new_height
            self.origin = new_origin
            self.res = msg.info.resolution
            self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
            self.texture_sum = np.zeros((self.height, self.width, 3), dtype=np.float64)
            self.texture_min = np.full(
                (self.height, self.width, 3), 255, dtype=np.uint8
            )
            self.texture_max = np.zeros(
                (self.height, self.width, 3), dtype=np.uint8
            )
            self.texture_count = np.zeros(
                (self.height, self.width), dtype=np.uint16
            )
            self.texture_weight = np.zeros(
                (self.height, self.width), dtype=np.float32
            )
            
            # Blit old data (Persistence logic)
            if old_grid is not None:
                dx = int(round((old_origin[0] - self.origin[0]) / self.res))
                dy = int(round((old_origin[1] - self.origin[1]) / self.res))
                src_x0, src_y0 = max(0, -dx), max(0, -dy)
                src_x1, src_y1 = min(old_width, self.width - dx), min(old_height, self.height - dy)
                dst_x0, dst_y0 = max(0, dx), max(0, dy)
                dst_x1, dst_y1 = min(self.width, old_width + dx), min(self.height, old_height + dy)
                if src_x1 > src_x0 and src_y1 > src_y0:
                    src = np.s_[src_y0:src_y1, src_x0:src_x1]
                    dst = np.s_[dst_y0:dst_y1, dst_x0:dst_x1]
                    self.grid[dst] = old_grid[src]
                    if old_texture[0] is not None:
                        self.texture_sum[dst] = old_texture[0][src]
                        self.texture_min[dst] = old_texture[1][src]
                        self.texture_max[dst] = old_texture[2][src]
                        self.texture_count[dst] = old_texture[3][src]
                        if old_texture[4] is not None:
                            self.texture_weight[dst] = old_texture[4][src]
            
            self.get_logger().info(f'Grid resized: {self.width}x{self.height}')

        # Store structural data
        self.struct_grid = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        
        # Update structural elements (protecting semantic colors)
        wall_mask = (self.struct_grid == 100)
        self.grid[wall_mask] = 1
        floor_mask = (self.struct_grid == 0) & (self.grid < 3)
        self.grid[floor_mask] = 2
        
        self.latest_stamp = msg.header.stamp
        self.dirty = True

    def _cloud_callback(self, msg):
        """Handle optimized global cloud with robust, high-speed NumPy vectorization."""
        if self.grid is None: return
        has_semantic_field = any(field.name == 'semantic_id' for field in msg.fields)
        now = self.get_clock().now().nanoseconds / 1e9
        if has_semantic_field:
            if now - self.last_semantic_cloud_time < self.min_cloud_interval:
                return
            self.last_semantic_cloud_time = now
            if not self.semantic_cloud_received:
                self.semantic_cloud_received = True
                if self.struct_grid is not None:
                    self.grid[self.struct_grid == 0] = 2
                    self.grid[self.struct_grid == 100] = 1
        else:
            # The optimized /cloud_map is retained as an old-format fallback,
            # but must never block or overwrite semantic_id observations.
            if self.semantic_cloud_received:
                return
            if now - self.last_legacy_cloud_time < self.min_cloud_interval:
                return
            self.last_legacy_cloud_time = now

        try:
            if not hasattr(self, 'color_lut'): 
                self.get_logger().info("Initializing Palette and Color LUT...")
                self.palette = self._generate_default_palette()
            
            # 1. Access raw bytes for zero-loss bit manipulation
            field_offsets = {f.name: f.offset for f in msg.fields}
            field_datatypes = {f.name: f.datatype for f in msg.fields}
            
            x_off = field_offsets.get('x')
            y_off = field_offsets.get('y')
            z_off = field_offsets.get('z')
            color_off = field_offsets.get('rgb')
            if color_off is None:
                color_off = field_offsets.get('rgba')
            semantic_id_off = field_offsets.get('semantic_id')
            
            if x_off is None or y_off is None:
                self.get_logger().warn(f"Required fields (x,y) not found. Got: {list(field_offsets.keys())}")
                return
            
            # Efficiently reshape data buffer
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
            
            # 2. Extract XYZ as float32
            # Use data slicing to avoid non-contiguous view issues
            x = data[:, x_off:x_off+4].copy().view(np.float32).flatten()
            y = data[:, y_off:y_off+4].copy().view(np.float32).flatten()
            z = (
                data[:, z_off:z_off+4].copy().view(np.float32).flatten()
                if z_off is not None
                else np.zeros_like(x)
            )
            
            # Filter NaNs and handle grid projection
            mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            x, y, z, data = x[mask], y[mask], z[mask], data[mask]
            
            if len(x) == 0:
                self.get_logger().debug("No finite points in cloud.")
                return
            
            range_m = np.sqrt(x * x + y * y + z * z)
            x_map, y_map, z_map = self._transform_xyz_to_map(x, y, z, msg.header)
            if x_map is None:
                return

            gx = ((x_map - self.origin[0]) / self.res).astype(np.int32)
            gy = ((y_map - self.origin[1]) / self.res).astype(np.int32)
            
            # Bounds checking
            in_bounds = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)
            gx, gy, z_map, range_m, data = (
                gx[in_bounds],
                gy[in_bounds],
                z_map[in_bounds],
                range_m[in_bounds],
                data[in_bounds],
            )
            
            if len(gx) == 0:
                self.get_logger().debug(f"Points outside grid bounds. Origin: {self.origin}, Res: {self.res}")
                return

            if semantic_id_off is not None:
                semantic_ids = self._extract_semantic_ids(data, semantic_id_off, field_datatypes.get('semantic_id'))
                semantic_mask = semantic_ids > 0
                current_pixel_values = self.grid[gy, gx]
                not_wall = (current_pixel_values != 1)
                update_mask = semantic_mask & not_wall
                self.grid[gy[update_mask], gx[update_mask]] = semantic_ids[update_mask]

                if int(np.sum(update_mask)) > 0:
                    self.get_logger().info(
                        f'Painted {int(np.sum(update_mask))} semantic_id points onto grid.',
                        throttle_duration_sec=5.0,
                    )
            # 3. Robust Color Extraction & Quantization fallback for older clouds.
            elif color_off is not None:
                # Directly extract bytes to avoid alignment errors from .view(np.uint32)
                # To match iterative logic: (packed >> 16) is byte 2, (>> 8) is byte 1, mask is byte 0
                r = data[:, color_off + 2].astype(np.float32)
                g = data[:, color_off + 1].astype(np.float32)
                b = data[:, color_off + 0].astype(np.float32)
                
                # Correct quantization to nearest step (0,1,2,3,4) with rounding
                ri = np.clip(np.round(r / 63.75).astype(np.int32), 0, 4)
                gi = np.clip(np.round(g / 63.75).astype(np.int32), 0, 4)
                bi = np.clip(np.round(b / 63.75).astype(np.int32), 0, 4)
                
                # Use pre-computed LUT for perfect index matching
                indices = self.color_lut[ri, gi, bi]
                
                # 4. Persistent Masking (Protect Walls)
                current_pixel_values = self.grid[gy, gx]
                not_wall = (current_pixel_values != 1)
                
                # Batch update only non-wall pixels
                self.grid[gy[not_wall], gx[not_wall]] = indices[not_wall]
                
                if len(indices[not_wall]) > 0:
                    self.get_logger().info(f'Painted {len(indices[not_wall])} colored points onto grid.')

            if (
                self.accumulate_real_texture
                and semantic_id_off is not None
                and color_off is not None
            ):
                self._accumulate_texture(data, gx, gy, z_map, range_m, color_off)
            
            self.dirty = True
        except Exception as e:
            self.get_logger().error(f"Vectorized cloud projection error: {e}")

    def _accumulate_texture(self, data, gx, gy, z_map, range_m, color_off):
        """Fuse exposure-corrected, distance-weighted RGB into free cells."""
        if (
            self.struct_grid is None
            or self.texture_sum is None
            or self.texture_weight is None
        ):
            return

        floor = self.struct_grid[gy, gx] == 0
        height_ok = (
            (z_map >= self.texture_min_height)
            & (z_map <= self.texture_max_height)
        )
        range_ok = (
            (range_m >= self.texture_min_range)
            & (range_m <= self.texture_max_range)
        )
        selected = floor & height_ok & range_ok
        if not np.any(selected):
            return

        selected_data = data[selected]
        selected_gx = gx[selected]
        selected_gy = gy[selected]
        selected_range = range_m[selected]
        # Packed 00RRGGBB is little-endian in PointCloud2: B, G, R, padding.
        rgb = np.column_stack([
            selected_data[:, color_off + 2],
            selected_data[:, color_off + 1],
            selected_data[:, color_off + 0],
        ]).astype(np.float32)

        flat = selected_gy.astype(np.int64) * self.width + selected_gx
        unique, inverse = np.unique(flat, return_inverse=True)
        # Near floor samples are more stable than the distant edge of a camera
        # frustum. One cloud still contributes only one view per grid cell,
        # regardless of point density.
        sample_quality = 1.0 / (1.0 + (selected_range / 3.0) ** 2)
        sample_quality = np.clip(sample_quality, 0.08, 1.0)
        quality_per_cell = np.bincount(inverse, weights=sample_quality)
        cell_rgb = np.column_stack([
            np.bincount(inverse, weights=rgb[:, channel] * sample_quality)
            / quality_per_cell
            for channel in range(3)
        ])
        samples_per_cell = np.bincount(inverse)
        cell_quality = quality_per_cell / np.maximum(samples_per_cell, 1)
        cell_quality *= self.current_motion_weight
        cell_y = unique // self.width
        cell_x = unique % self.width

        can_update = self.texture_count[cell_y, cell_x] < np.iinfo(np.uint16).max
        cell_y = cell_y[can_update]
        cell_x = cell_x[can_update]
        cell_rgb = cell_rgb[can_update]
        cell_quality = cell_quality[can_update]
        if len(cell_y) == 0:
            return

        old_weight = self.texture_weight[cell_y, cell_x].astype(np.float64)
        old_rgb = np.zeros_like(cell_rgb)
        observed = old_weight > 1e-6
        if np.any(observed):
            old_rgb[observed] = (
                self.texture_sum[cell_y[observed], cell_x[observed]]
                / old_weight[observed, None]
            )

        if self.texture_exposure_compensation:
            gain = estimate_overlap_exposure_gain(old_rgb, cell_rgb, observed)
            cell_rgb = np.clip(cell_rgb * gain, 0, 255)
            if abs(gain - 1.0) > 0.03:
                self.get_logger().info(
                    f'Texture exposure gain: {gain:.3f}',
                    throttle_duration_sec=5.0,
                )

        # Huber-style robust weighting prevents a slightly misregistered pass
        # or in-place turn from smearing an already consistent tile texture.
        if np.any(observed):
            cell_quality *= robust_color_residual_weights(
                old_rgb,
                cell_rgb,
                observed,
                delta=self.texture_robust_delta,
            )

        # Bound historical influence so a later close, stable observation can
        # still improve a cell that was first seen at the frustum edge.
        retained_weight = np.minimum(
            old_weight,
            np.maximum(self.texture_max_fusion_weight - cell_quality, 0.0),
        )
        scale = np.divide(
            retained_weight,
            old_weight,
            out=np.zeros_like(retained_weight),
            where=old_weight > 1e-6,
        )
        self.texture_sum[cell_y, cell_x] *= scale[:, None]
        self.texture_sum[cell_y, cell_x] += cell_rgb * cell_quality[:, None]
        self.texture_weight[cell_y, cell_x] = retained_weight + cell_quality
        cell_rgb_u8 = np.clip(np.round(cell_rgb), 0, 255).astype(np.uint8)
        self.texture_min[cell_y, cell_x] = np.minimum(
            self.texture_min[cell_y, cell_x], cell_rgb_u8
        )
        self.texture_max[cell_y, cell_x] = np.maximum(
            self.texture_max[cell_y, cell_x], cell_rgb_u8
        )
        self.texture_count[cell_y, cell_x] += 1

    def _extract_semantic_ids(self, data, offset, datatype):
        if datatype == 2:  # UINT8
            return data[:, offset].astype(np.uint8)
        if datatype == 4:  # UINT16
            return data[:, offset:offset + 2].copy().view(np.uint16).flatten().astype(np.uint8)
        if datatype == 6:  # UINT32
            return np.clip(
                data[:, offset:offset + 4].copy().view(np.uint32).flatten(),
                0,
                255,
            ).astype(np.uint8)
        if datatype == 7:  # FLOAT32
            return np.clip(
                np.round(data[:, offset:offset + 4].copy().view(np.float32).flatten()),
                0,
                255,
            ).astype(np.uint8)
        return np.zeros(data.shape[0], dtype=np.uint8)

    def _transform_xyz_to_map(self, x, y, z, header):
        source_frame = header.frame_id
        if not source_frame or source_frame == self.map_frame:
            self.current_motion_weight = 1.0
            return x, y, z

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                Time.from_msg(header.stamp),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as e:
            self.get_logger().warn(
                f'No transform from {source_frame} to {self.map_frame}; semantic cloud skipped: {e}',
                throttle_duration_sec=5.0,
            )
            return None, None, None

        q = tf.transform.rotation
        t = tf.transform.translation
        self.current_motion_weight = self._texture_motion_weight(t, q, header)
        rotation = np.array([
            [
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                2.0 * (q.x * q.y - q.z * q.w),
                2.0 * (q.x * q.z + q.y * q.w),
            ],
            [
                2.0 * (q.x * q.y + q.z * q.w),
                1.0 - 2.0 * (q.x * q.x + q.z * q.z),
                2.0 * (q.y * q.z - q.x * q.w),
            ],
            [
                2.0 * (q.x * q.z - q.y * q.w),
                2.0 * (q.y * q.z + q.x * q.w),
                1.0 - 2.0 * (q.x * q.x + q.y * q.y),
            ],
        ])
        transformed = np.column_stack([x, y, z]) @ rotation.T
        transformed[:, 0] += t.x
        transformed[:, 1] += t.y
        transformed[:, 2] += t.z
        return transformed[:, 0], transformed[:, 1], transformed[:, 2]

    def _texture_motion_weight(self, translation, rotation, header):
        """Down-weight fast turns and TF jumps while retaining their coverage."""
        stamp = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
        yaw = np.arctan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        pose = (stamp, float(translation.x), float(translation.y), float(yaw))
        previous = self.previous_texture_pose
        self.previous_texture_pose = pose
        if previous is None:
            return 1.0

        dt = stamp - previous[0]
        if dt <= 1e-3 or dt > 5.0:
            return 1.0
        yaw_delta = np.arctan2(
            np.sin(yaw - previous[3]),
            np.cos(yaw - previous[3]),
        )
        yaw_rate = abs(float(yaw_delta)) / dt
        translation_rate = np.hypot(
            pose[1] - previous[1],
            pose[2] - previous[2],
        ) / dt
        rotation_weight = min(1.0, 0.30 / max(yaw_rate, 1e-6))
        translation_weight = min(1.0, 1.20 / max(translation_rate, 1e-6))
        return float(np.clip(rotation_weight * translation_weight, 0.20, 1.0))

    def _generate_default_palette(self):
        # Index 0: Unknown, 1: Wall, 2: Floor, 3+: semantic class ids.
        reserved = [SEMANTIC_CLASSES[i]["color"] for i in sorted(SEMANTIC_CLASSES)]
        colors = []
        steps = [0, 64, 128, 192, 255]
        for r in steps:
            for g in steps:
                for b in steps:
                    c = [r, g, b]
                    if c not in reserved: colors.append(c)
        
        palette = np.array(reserved + colors, dtype=np.uint8)
        
        # Generate LUT for O(1) vectorized indexing
        # This maps any of the 125 quantized combinations to the closest index in palette[3:]
        self.color_lut = np.zeros((5, 5, 5), dtype=np.uint8)
        palette_sem = palette[3:]
        
        for ri, rv in enumerate(steps):
            for gi, gv in enumerate(steps):
                for bi, bv in enumerate(steps):
                    rgb = np.array([rv, gv, bv], dtype=np.float32)
                    dist = np.sum((palette_sem - rgb)**2, axis=1)
                    self.color_lut[ri, gi, bi] = np.argmin(dist) + 3
            
        return palette

    def _timer_callback(self):
        if self.grid is not None and self.dirty:
            self._publish_grid()
            self.dirty = False

    def _publish_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.latest_stamp
        msg.header.frame_id = self.map_frame
        msg.info = MapMetaData(resolution=self.res, width=self.width, height=self.height)
        msg.info.origin.position.x, msg.info.origin.position.y = self.origin[0], self.origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.astype(np.int8).flatten().tolist()
        self.pub_indexed_grid.publish(msg)

    def _save_callback(self, msg):
        if self.grid is None: return
        path = msg.data
        if not path.startswith('/'):
            path = os.path.join(os.path.expanduser('~/sirius_jazzy_ws/maps_waypoints/maps/'), path)
        
        cv2.imwrite(path + ".pgm", self.grid[::-1, :])
        semantic_costs, _ = load_semantic_costs()
        labels = {}
        for idx, info in SEMANTIC_CLASSES.items():
            cat = costs_for(semantic_costs, info["name"])
            labels[str(idx)] = {
                "name": info["name"],
                "color": info["color"],
                "global_cost": cat["global_cost"],
                "local_cost": cat["local_cost"],
            }
        meta = {
            "resolution": self.res,
            "origin": self.origin,
            "width": self.width,
            "height": self.height,
            "palette": self.palette.tolist(),
            "labels": labels,
            "semantic_encoding": "class_id",
        }
        with open(path + ".json", 'w') as f: json.dump(meta, f, indent=4)

        if self.accumulate_real_texture and self.texture_count is not None:
            texture_rgb = weighted_texture_from_statistics(
                self.texture_sum,
                self.texture_weight,
            )
            texture_bgra = np.zeros((self.height, self.width, 4), dtype=np.uint8)
            valid = self.texture_weight > 1e-6
            floor_mask = np.ones_like(valid)
            if self.struct_grid is not None:
                floor_mask = self.struct_grid == 0
                valid &= floor_mask
            if self.texture_fill_small_holes:
                texture_rgb, valid = fill_small_texture_holes(
                    texture_rgb,
                    valid,
                    floor_mask,
                )
            texture_bgra[:, :, :3] = texture_rgb[:, :, ::-1]
            texture_bgra[:, :, 3][valid] = 255
            texture_base = path[:-len('.colored')] if path.endswith('.colored') else path
            texture_path = texture_base + ".texture.png"
            cv2.imwrite(texture_path, texture_bgra[::-1, :])
            robust_cells = int(np.sum(self.texture_count > 2))
            self.get_logger().info(
                f'Saved robust RGB texture to {texture_path}: '
                f'{int(np.sum(valid))} observed cells, '
                f'{robust_cells} cells fused from 3+ views.'
            )
        self.get_logger().info(f'SUCCESS: Saved to {path}.pgm')

def main(args=None):
    rclpy.init(args=args)
    node = SAM3IndexedMapNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__': main()
