import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import json
import websocket
import threading
import numpy as np
import time
import struct

class SAM3ROSBridge(Node):
    def __init__(self):
        super().__init__('sam3_ros_bridge')
        
        # Parameters
        self.declare_parameter('server_url', 'ws://localhost:8080/ws_3d')
        self.declare_parameter('frame_id', 'sirius3/zed_camera_link')
        self.declare_parameter('mask_threshold', 0.5)
        
        self.url = self.get_parameter('server_url').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.mask_threshold = self.get_parameter('mask_threshold').get_parameter_value().double_value
        
        # Publisher
        self.pub = self.create_publisher(PointCloud2, '/sam3/obstacles', 10)
        
        # WebSocket setup
        self.running = True
        self.ws = None
        self.ws_thread = threading.Thread(target=self._ws_loop, daemon=True)
        self.ws_thread.start()
        
        self.get_logger().info(f'SAM3 ROS Bridge started. Connecting to {self.url}...')

    def _ws_loop(self):
        while self.running:
            try:
                # websocket-client uses a different callback signature for on_close in newer versions
                self.ws = websocket.WebSocketApp(
                    self.url,
                    on_message=self._on_message,
                    on_error=self._on_error,
                    on_close=self._on_close
                )
                self.ws.run_forever()
            except Exception as e:
                self.get_logger().error(f'WebSocket run_forever error: {e}')
            
            if self.running:
                self.get_logger().info('Attempting to reconnect in 3 seconds...')
                time.sleep(3)

    def _on_message(self, ws, message):
        # Stats message is JSON (str), Point Cloud is binary (bytes)
        if isinstance(message, str):
            try:
                # Optional: parse stats like FPS
                # stats = json.loads(message)
                pass
            except: pass
            return
        
        # Binary data decoding
        # Format: [x, y, z, r, g, b, is_masked] (7 * float32)
        try:
            raw_data = np.frombuffer(message, dtype=np.float32)
            if len(raw_data) % 7 != 0:
                return
                
            data = raw_data.reshape(-1, 7)
            
            # Filter for masked points (segmented objects)
            mask = data[:, 6] > self.mask_threshold
            masked_points = data[mask]
            
            if len(masked_points) == 0:
                return

            # ROS_X = -ZED_Z (Forward)
            # ROS_Y = -ZED_X (Left)
            # ROS_Z =  ZED_Y (Up)
            
            # Prepare points with color: [x, y, z, rgb_packed]
            points_with_color = []
            for i in range(len(masked_points)):
                x = -masked_points[i, 2]
                y = -masked_points[i, 0]
                z =  masked_points[i, 1]
                
                # RGB floats (0.0-1.0) to uint8 (0-255)
                r = int(masked_points[i, 3] * 255)
                g = int(masked_points[i, 4] * 255)
                b = int(masked_points[i, 5] * 255)
                
                # Pack RGB into a single float (ROS standard)
                # Format: 00RR GGBB
                rgb = (r << 16) | (g << 8) | b
                # Pack as unsigned int, then unpack as float
                rgb_packed = struct.unpack('f', struct.pack('I', rgb))[0]
                
                points_with_color.append([x, y, z, rgb_packed])
            
            # Prepare PointCloud2 Header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.frame_id
            
            # Define fields
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            
            # Create ROS message
            cloud_msg = pc2.create_cloud(header, fields, points_with_color)
            self.pub.publish(cloud_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def _on_error(self, ws, error):
        self.get_logger().error(f'WebSocket Error: {error}')

    def _on_close(self, ws, close_status_code, close_msg):
        self.get_logger().warn(f'WebSocket connection closed: {close_status_code} - {close_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = SAM3ROSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Runtime error: {e}')
    finally:
        node.running = False
        if node.ws:
            node.ws.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
