import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
from std_msgs.msg import Header
import json
import os
import struct

class SAM3ColoredMapLoader(Node):
    def __init__(self):
        super().__init__('sam3_colored_map_loader')
        
        # Parameters
        self.declare_parameter('map_path', '')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('map_frame', 'map')
        
        self.map_path = self.get_parameter('map_path').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        
        # Publisher
        self.pub_cloud = self.create_publisher(PointCloud2, '/sam3/static_colored_map_cloud', 10)
        
        self.cloud_msg = None
        
        if self.map_path:
            self.load_map(self.map_path)
        else:
            self.get_logger().warn('No map_path provided. Use: ros2 run sirius_navigation sam3_colored_map_loader --ros-args -p map_path:=/path/to/my_map')

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def load_map(self, path):
        # Handle extension-less path
        if path.endswith('.pgm') or path.endswith('.json'):
            base_path = os.path.splitext(path)[0]
        else:
            base_path = path

        pgm_file = base_path + ".pgm"
        json_file = base_path + ".json"

        if not os.path.exists(pgm_file) or not os.path.exists(json_file):
            self.get_logger().error(f'File not found: {pgm_file} or {json_file}')
            return

        self.get_logger().info(f'Loading map from {base_path}...')
        
        # Load PGM (indices)
        grid = cv2.imread(pgm_file, cv2.IMREAD_UNCHANGED)
        if grid is None:
            self.get_logger().error(f'Failed to read PGM: {pgm_file}')
            return
        
        # RTAB-Map convention / our saver convention: flipped vertically?
        # Our saver did: cv2.imwrite(path + ".pgm", self.grid[::-1, :])
        # So we flip it back
        grid = grid[::-1, :]
        
        # Load JSON (metadata)
        try:
            with open(json_file, 'r') as f:
                meta = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to read JSON: {e}')
            return

        res = meta.get('resolution', 0.05)
        origin = meta.get('origin', [0.0, 0.0])
        palette = np.array(meta.get('palette', []), dtype=np.uint8)
        
        if len(palette) == 0:
            self.get_logger().error('Palette is empty in JSON')
            return

        self.get_logger().info(f'Map Loaded: {grid.shape[1]}x{grid.shape[0]} at {res}m/pix')

        # Convert to PointCloud2
        rows, cols = np.where(grid > 0) # Skip unknown (0)
        if len(rows) == 0:
            self.get_logger().warn('No known cells (indices > 0) found in map.')
            return

        indices = grid[rows, cols]
        
        # Map indices to RGB
        rgb_list = palette[indices]
        
        # Map indices to coordinates
        x_coords = cols * res + origin[0]
        y_coords = rows * res + origin[1]
        
        points = []
        for i in range(len(rows)):
            r, g, b = rgb_list[i]
            # Pack RGB into a single float (standard ROS pattern)
            rgb_packed = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
            points.append([float(x_coords[i]), float(y_coords[i]), 0.0, rgb_packed])

        self.cloud_msg = pc2.create_cloud(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.map_frame),
            fields=[
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
            ],
            points=points
        )
        self.get_logger().info(f'Converted {len(points)} points to cloud.')

    def timer_callback(self):
        if self.cloud_msg is not None:
            # Update timestamp for RViz
            self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_cloud.publish(self.cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SAM3ColoredMapLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
