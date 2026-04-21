import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import String, Header
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose, TransformStamped
import tf2_ros
import tf2_geometry_msgs
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
        self.grid = None  # uint8 array
        self.origin = [0.0, 0.0]
        self.width = 0
        self.height = 0
        self.dirty = False # Flag for change detection
        
        # Color Palette
        self.palette = self._generate_default_palette()
        self.color_reservoir = set()
        self.max_reservoir_size = 10000
        
        # TF Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS setup
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.sub_grid = self.create_subscription(OccupancyGrid, '/rtabmap/grid_map', self._grid_callback, map_qos)
        self.sub_cloud = self.create_subscription(PointCloud2, '/sam3/obstacles', self._cloud_callback, 10)
        self.sub_save = self.create_subscription(String, '/sam3/save_indexed_map', self._save_callback, 10)
        
        # Publisher
        self.pub_indexed_grid = self.create_publisher(OccupancyGrid, '/sam3/indexed_grid', 10)
        
        # Timer for publishing grid (Rate limit: 1Hz for efficiency)
        self.timer = self.create_timer(1.0, self._timer_callback)
        self.latest_stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info('SAM3 Indexed Map Node started (Optimized).')

    def _generate_default_palette(self):
        reserved = [[127, 127, 127], [0, 0, 0], [255, 255, 255]]
        colors = []
        steps = [0, 51, 102, 153, 204, 255]
        for r in steps:
            for g in steps:
                for b in steps:
                    c = [r, g, b]
                    if c not in reserved: colors.append(c)
        colors = colors[:253]
        while len(colors) < 253: colors.append([128, 128, 128])
        colors.sort(key=lambda c: 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2])
        return np.array(reserved + colors, dtype=np.uint8)

    def _get_nearest_index(self, palette, r, g, b):
        dist = np.sum((palette - [r, g, b])**2, axis=1)
        return np.argmin(dist)

    def _grid_callback(self, msg):
        """Update structure."""
        if self.width != msg.info.width or self.height != msg.info.height:
            self.width, self.height = msg.info.width, msg.info.height
            self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
            self.res = msg.info.resolution
            self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
            self.get_logger().info(f'Initialized grid: {self.width}x{self.height}')

        occ = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        wall_mask = (occ == 100)
        floor_mask = (occ == 0)
        
        # Update walls (structural priority)
        if np.any(self.grid[wall_mask] != 1):
            self.grid[wall_mask] = 1
            self.dirty = True
        
        # Update floor only if currently empty
        floor_update_mask = (self.grid < 3) & floor_mask
        if np.any(self.grid[floor_update_mask] != 2):
            self.grid[floor_update_mask] = 2
            self.dirty = True

    def _cloud_callback(self, msg):
        """Update colors."""
        if self.grid is None: return
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception: return

        t, q = transform.transform.translation, transform.transform.rotation
        inv_res = 1.0 / self.res
        
        # Use vectorized sampling for the reservoir to keep performance
        points = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
        if not points: return

        # Transform and Project
        for p in points:
            # Simple rotation p' = qpq* (inline for speed)
            vx = 2.0 * (q.w * (q.y * p[2] - q.z * p[1]) + q.x * (q.x * p[0] + q.y * p[1] + q.z * p[2])) + p[0] * (q.w**2 - q.x**2 - q.y**2 - q.z**2)
            vy = 2.0 * (q.w * (q.z * p[0] - q.x * p[2]) + q.y * (q.x * p[0] + q.y * p[1] + q.z * p[2])) + p[1] * (q.w**2 - q.x**2 - q.y**2 - q.z**2)
            
            gx = int((vx + t.x - self.origin[0]) * inv_res)
            gy = int((vy + t.y - self.origin[1]) * inv_res)
            
            if 0 <= gx < self.width and 0 <= gy < self.height:
                if self.grid[gy, gx] != 1: # Don't overwrite structural walls
                    packed = struct.unpack('I', struct.pack('f', p[3]))[0]
                    # Colors are packed differently in some systems, but SAM3 is RGB
                    rgb = ((packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF)
                    
                    if len(self.color_reservoir) < self.max_reservoir_size:
                        self.color_reservoir.add(rgb)
                    
                    idx = self._get_nearest_index(self.palette[3:], *rgb) + 3
                    if self.grid[gy, gx] != idx:
                        self.grid[gy, gx] = idx
                        self.dirty = True
        self.latest_stamp = msg.header.stamp

    def _timer_callback(self):
        if self.grid is not None and self.dirty:
            self._publish_indexed_grid(self.latest_stamp)
            self.dirty = False


    def _publish_indexed_grid(self, stamp):
        msg = OccupancyGrid()
        msg.header.stamp, msg.header.frame_id = stamp, self.map_frame
        msg.info = MapMetaData(resolution=self.res, width=self.width, height=self.height)
        msg.info.origin.position.x, msg.info.origin.position.y = self.origin[0], self.origin[1]
        msg.info.origin.orientation.w = 1.0
        # Optimization: directly assign flatten result (numpy to list)
        msg.data = self.grid.astype(np.int8).flatten().tolist()
        self.pub_indexed_grid.publish(msg)

    def _save_callback(self, msg):
        if self.grid is None:
            self.get_logger().error('Cannot save map: Grid is not initialized. Is /rtabmap/grid_map being published?')
            return
        path = msg.data
        if not path.startswith('/'):
            path = os.path.join(os.path.expanduser('~/sirius_jazzy_ws/maps_waypoints/maps/'), path)
        
        self.get_logger().info(f'Saving map with GIF-like quantization...')
        reserved = np.array([[127, 127, 127], [0, 0, 0], [255, 255, 255]], dtype=np.uint8)
        
        if len(self.color_reservoir) > 10:
            data = np.array(list(self.color_reservoir))
            kmeans = KMeans(n_clusters=min(253, len(data)), n_init=1, random_state=42).fit(data)
            semantic = sorted(kmeans.cluster_centers_.astype(np.uint8).tolist(), key=lambda c: 0.299*c[0]+0.587*c[1]+0.114*c[2])
            palette = np.vstack([reserved, semantic])
        else: palette = self.palette

        os.makedirs(os.path.dirname(path), exist_ok=True)
        cv2.imwrite(path + ".pgm", self.grid[::-1, :])
        meta = {"resolution": self.res, "origin": self.origin, "width": self.width, "height": self.height, "palette": palette.tolist()}
        with open(path + ".json", 'w') as f: json.dump(meta, f, indent=4)
        self.get_logger().info(f'SUCCESS: Saved to {path}.pgm')

def main(args=None):
    rclpy.init(args=args)
    node = SAM3IndexedMapNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
