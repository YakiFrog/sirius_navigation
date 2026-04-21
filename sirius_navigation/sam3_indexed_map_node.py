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
from sklearn.cluster import KMeans

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
        # 1. Structural Grid (Perfect reference)
        self.sub_grid = self.create_subscription(OccupancyGrid, '/rtabmap/grid_map', self._grid_callback, map_qos)
        # 2. Optimized Colored Cloud (Perfect color)
        self.sub_cloud = self.create_subscription(PointCloud2, '/cloud_map', self._cloud_callback, map_qos)
        # 3. Save Trigger
        self.sub_save = self.create_subscription(String, '/sam3/save_indexed_map', self._save_callback, 10)
        
        # Publishers
        self.pub_indexed_grid = self.create_publisher(OccupancyGrid, '/sam3/indexed_grid', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(2.0, self._timer_callback)
        self.latest_stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info('SAM3 Cloud-to-Grid Node initialized from scratch (Loop-closure Corrected).')

    def _grid_callback(self, msg):
        """Handle incoming structural map."""
        new_width = msg.info.width
        new_height = msg.info.height
        new_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        
        # Re-initialize if dimensions or origin changed significantly
        if (self.grid is None or 
            self.width != new_width or self.height != new_height or 
            not np.allclose(self.origin, new_origin, atol=1e-3)):
            
            old_grid = self.grid
            old_width, old_height = self.width, self.height
            old_origin = self.origin
            
            self.width, self.height = new_width, new_height
            self.origin = new_origin
            self.res = msg.info.resolution
            self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
            
            # Blit old data into new grid
            if old_grid is not None:
                dx = int(round((old_origin[0] - self.origin[0]) / self.res))
                dy = int(round((old_origin[1] - self.origin[1]) / self.res))
                
                # Copy overlap
                src_x0, src_y0 = max(0, -dx), max(0, -dy)
                src_x1, src_y1 = min(old_width, self.width - dx), min(old_height, self.height - dy)
                dst_x0, dst_y0 = max(0, dx), max(0, dy)
                dst_x1, dst_y1 = min(self.width, old_width + dx), min(self.height, old_height + dy)
                
                if src_x1 > src_x0 and src_y1 > src_y0:
                    self.grid[dst_y0:dst_y1, dst_x0:dst_x1] = old_grid[src_y0:src_y1, src_x0:src_x1]
            
            self.get_logger().info(f'Canvas adjusted: {self.width}x{self.height} @ {self.origin}')

        # Store structural data
        self.struct_grid = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        
        # Baseline sync: Map structural cells to base indices
        # index 1: Wall (Priority), 2: Floor (Only if no color)
        
        # 1. Update walls (Structural priority)
        wall_mask = (self.struct_grid == 100)
        self.grid[wall_mask] = 1
        
        # 2. Update floor only if it doesn't already have a semantic color (index >= 3)
        # This prevents flickering and preserves cloud-based colors
        floor_update_mask = (self.struct_grid == 0) & (self.grid < 3)
        self.grid[floor_update_mask] = 2
        
        self.latest_stamp = msg.header.stamp
        self.dirty = True

    def _cloud_callback(self, msg):
        """Handle optimized global cloud."""
        if self.grid is None: return
        
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_cloud_time < self.min_cloud_interval:
            return 
        self.last_cloud_time = now

        self.get_logger().info(f'Projecting color from perfect /cloud_map...')
        
        try:
            fields = {f.name: f for f in msg.fields}
            color_field = "rgb" if "rgb" in fields else ("rgba" if "rgba" in fields else None)
            
            read_fields = ["x", "y"]
            if color_field: read_fields.append(color_field)
            
            # Using NumPy for faster projection if many points
            points = list(pc2.read_points(msg, field_names=read_fields, skip_nans=True))
            if not points: return

            inv_res = 1.0 / self.res
            ox, oy = self.origin[0], self.origin[1]
            h, w = self.height, self.width

            # Default simple palette
            if not hasattr(self, 'palette'):
                self.palette = self._generate_default_palette()
            palette = self.palette

            for p in points:
                gx = int((p[0] - ox) * inv_res)
                gy = int((p[1] - oy) * inv_res)
                
                if 0 <= gx < w and 0 <= gy < h:
                    if self.grid[gy, gx] == 1: continue 

                    if color_field:
                        val = p[2]
                        try:
                            packed = struct.unpack('I', struct.pack('f', val))[0]
                            rgb = ((packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF)
                            dist = np.sum((palette[3:] - rgb)**2, axis=1)
                            idx = np.argmin(dist) + 3
                            self.grid[gy, gx] = idx
                        except: pass
            
            self.dirty = True
        except Exception as e:
            self.get_logger().error(f"Cloud projection error: {e}")

    def _generate_default_palette(self):
        reserved = [[127, 127, 127], [0, 0, 0], [255, 255, 255]]
        colors = []
        steps = [0, 128, 255]
        for r in steps:
            for g in steps:
                for b in steps:
                    c = [r, g, b]
                    if c not in reserved: colors.append(c)
        while len(colors) < 253: colors.append([128, 128, 128])
        return np.array(reserved + colors, dtype=np.uint8)

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
        
        self.get_logger().info(f'Saving indexed map to {path}...')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        cv2.imwrite(path + ".pgm", self.grid[::-1, :])
        meta = {
            "resolution": self.res,
            "origin": self.origin,
            "width": self.width,
            "height": self.height,
            "palette": self.palette.tolist()
        }
        with open(path + ".json", 'w') as f:
            json.dump(meta, f, indent=4)
        self.get_logger().info(f'SUCCESS: Saved to {path}.pgm')

def main(args=None):
    rclpy.init(args=args)
    node = SAM3IndexedMapNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()
