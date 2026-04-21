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

    def grid_callback(self, msg):
        """Convert indexed grid to PointCloud2."""
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        
        # Prepare grid data
        grid_data = np.array(msg.data, dtype=np.uint8).reshape((height, width))
        
        # Find known cells (indices > 0)
        rows, cols = np.where(grid_data > 0)
        if len(rows) == 0: return

        # Map indices to RGB
        indices = grid_data[rows, cols]
        rgb_list = self.palette[indices]
        
        # Map indices to coordinates
        x_coords = cols * res + origin[0]
        y_coords = rows * res + origin[1]
        
        points = []
        for i in range(len(rows)):
            r, g, b = rgb_list[i]
            # Pack RGB into a single float
            rgb_packed = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
            points.append([float(x_coords[i]), float(y_coords[i]), 0.0, rgb_packed])
        
        # Create cloud
        cloud_msg = pc2.create_cloud(
            header=Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id),
            fields=[
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            ],
            points=points
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
