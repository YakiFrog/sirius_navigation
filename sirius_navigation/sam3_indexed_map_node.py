import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import String, Header
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import numpy as np
import struct
import json
import os
import cv2
import cv2

class SAM3IndexedMapNode(Node):
    def __init__(self):
        super().__init__('sam3_indexed_map_node')
        
        # Parameters
        self.declare_parameter('palette_size', 256)
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('map_frame', 'map')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        
        self.res = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        
        # Internal State
        self.grid = None       # uint8 array (indexed map)
        self.struct_grid = None # Original occupancy grid for reference
        self.origin = [0.0, 0.0]
        self.width = 0
        self.height = 0
        self.dirty = False
        
        # Performance tuning: Rate limit for cloud processing
        self.last_cloud_time = 0.0
        self.min_cloud_interval = 2.0  # seconds

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
            
            old_grid, old_width, old_height, old_origin = self.grid, self.width, self.height, self.origin
            
            self.width, self.height = new_width, new_height
            self.origin = new_origin
            self.res = msg.info.resolution
            self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
            
            # Blit old data (Persistence logic)
            if old_grid is not None:
                dx = int(round((old_origin[0] - self.origin[0]) / self.res))
                dy = int(round((old_origin[1] - self.origin[1]) / self.res))
                src_x0, src_y0 = max(0, -dx), max(0, -dy)
                src_x1, src_y1 = min(old_width, self.width - dx), min(old_height, self.height - dy)
                dst_x0, dst_y0 = max(0, dx), max(0, dy)
                dst_x1, dst_y1 = min(self.width, old_width + dx), min(self.height, old_height + dy)
                if src_x1 > src_x0 and src_y1 > src_y0:
                    self.grid[dst_y0:dst_y1, dst_x0:dst_x1] = old_grid[src_y0:src_y1, src_x0:src_x1]
            
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
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_cloud_time < self.min_cloud_interval: return 
        self.last_cloud_time = now

        try:
            if not hasattr(self, 'color_lut'): 
                self.get_logger().info("Initializing Palette and Color LUT...")
                self.palette = self._generate_default_palette()
            
            # 1. Access raw bytes for zero-loss bit manipulation
            field_offsets = {f.name: f.offset for f in msg.fields}
            
            x_off = field_offsets.get('x')
            y_off = field_offsets.get('y')
            color_off = field_offsets.get('rgb') or field_offsets.get('rgba')
            
            if x_off is None or y_off is None:
                self.get_logger().warn(f"Required fields (x,y) not found. Got: {list(field_offsets.keys())}")
                return
            
            # Efficiently reshape data buffer
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
            
            # 2. Extract X, Y as float32
            # Use data slicing to avoid non-contiguous view issues
            x = data[:, x_off:x_off+4].copy().view(np.float32).flatten()
            y = data[:, y_off:y_off+4].copy().view(np.float32).flatten()
            
            # Filter NaNs and handle grid projection
            mask = np.isfinite(x) & np.isfinite(y)
            x, y, data = x[mask], y[mask], data[mask]
            
            if len(x) == 0:
                self.get_logger().debug("No finite points in cloud.")
                return
            
            gx = ((x - self.origin[0]) / self.res).astype(np.int32)
            gy = ((y - self.origin[1]) / self.res).astype(np.int32)
            
            # Bounds checking
            in_bounds = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)
            gx, gy, data = gx[in_bounds], gy[in_bounds], data[in_bounds]
            
            if len(gx) == 0:
                self.get_logger().debug(f"Points outside grid bounds. Origin: {self.origin}, Res: {self.res}")
                return

            # 3. Robust Color Extraction & Quantization
            if color_off is not None:
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
            
            self.dirty = True
        except Exception as e:
            self.get_logger().error(f"Vectorized cloud projection error: {e}")

    def _generate_default_palette(self):
        # Index 0: Unknown (Gray), Index 1: Wall (Black), Index 2: Floor (White)
        reserved = [[127, 127, 127], [0, 0, 0], [255, 255, 255]]
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
        meta = {"resolution": self.res, "origin": self.origin, "width": self.width, "height": self.height, "palette": self.palette.tolist()}
        with open(path + ".json", 'w') as f: json.dump(meta, f, indent=4)
        self.get_logger().info(f'SUCCESS: Saved to {path}.pgm')

def main(args=None):
    rclpy.init(args=args)
    node = SAM3IndexedMapNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()
