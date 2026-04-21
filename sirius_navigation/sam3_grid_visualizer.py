import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import struct

class SAM3GridVisualizer(Node):
    def __init__(self):
        super().__init__('sam3_grid_visualizer')
        
        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.declare_parameter('z_offset', -0.20)
        self.z_offset = self.get_parameter('z_offset').get_parameter_value().double_value
        
        # Internal State
        self.palette = self._generate_default_palette()
        
        # Subscriber
        self.sub_grid = self.create_subscription(
            OccupancyGrid, 
            '/sam3/indexed_grid', 
            self.grid_callback, 
            10
        )
        
        # Publisher
        self.pub_cloud = self.create_publisher(PointCloud2, '/sam3/colored_map_cloud', 10)
        
        self.get_logger().info('SAM3 Grid Visualizer started.')

    def _generate_default_palette(self):
        # Index 0: Unknown (Gray), Index 1: Wall (Black), Index 2: Floor (White)
        reserved = [[127, 127, 127], [0, 0, 0], [255, 255, 255]]
        colors = []
        # 5 steps for 5x5x5 = 125 colors (R,G,B 0,64,128,192,255)
        # Total index stays <= 127, safe for int8 occupancy grid.
        steps = [0, 64, 128, 192, 255]
        for r in steps:
            for g in steps:
                for b in steps:
                    c = [r, g, b]
                    if c not in reserved: colors.append(c)
        
        # Fill exactly up to 256 to ensure array shape stability
        while len(reserved + colors) < 256:
            colors.append([128, 128, 128])
            
        return np.array(reserved + colors, dtype=np.uint8)

    def grid_callback(self, msg):
        """Convert indexed grid to PointCloud2 (Optimized)."""
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        
        grid_data = np.array(msg.data, dtype=np.uint8).reshape((height, width))
        
        # Identify non-unknown cells
        rows, cols = np.where(grid_data > 0)
        if len(rows) == 0: return

        num_points = len(rows)
        indices = grid_data[rows, cols]
        
        # Build coordinates
        x = cols * res + ox
        y = rows * res + oy
        z = np.full(num_points, self.z_offset, dtype=np.float32)

        # Get colors from palette
        colors = self.palette[indices] # [N, 3] (R, G, B)
        
        # Pack RGB into float32 (Big-Endian equivalent for PointCloud2)
        # PointCloud2 expects 4 bytes for RGB: [B, G, R, A] or [A, R, G, B] depending on system
        # Here we pack it similar to how RTAB-Map does it.
        rgb_packed = np.zeros(num_points, dtype=np.uint32)
        rgb_packed |= colors[:, 0].astype(np.uint32) << 16 # R
        rgb_packed |= colors[:, 1].astype(np.uint32) << 8  # G
        rgb_packed |= colors[:, 2].astype(np.uint32)       # B
        
        # Prepare structured array for PointCloud2
        datatype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)]
        cloud_arr = np.empty(num_points, dtype=datatype)
        cloud_arr['x'] = x
        cloud_arr['y'] = y
        cloud_arr['z'] = z
        cloud_arr['rgb'] = rgb_packed.view(np.float32)

        # Create cloud
        cloud_msg = pc2.create_cloud(
            header=Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id),
            fields=[
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            ],
            points=cloud_arr
        )
        self.pub_cloud.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SAM3GridVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
