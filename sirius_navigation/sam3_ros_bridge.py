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
        self.declare_parameter('downsample_factor', 4)
        
        self.url = self.get_parameter('server_url').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.mask_threshold = self.get_parameter('mask_threshold').get_parameter_value().double_value
        self.downsample_factor = self.get_parameter('downsample_factor').get_parameter_value().integer_value
        
        # Publishers
        self.pub_obstacles = self.create_publisher(PointCloud2, '/sam3/obstacles', 10)
        self.pub_background = self.create_publisher(PointCloud2, '/sam3/background', 10)
        
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
            
            # Filter for masked points (segmented objects) vs background
            is_masked = data[:, 6] > self.mask_threshold
            masked_data = data[is_masked]
            background_data = data[~is_masked]
            
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
            
            # Filter for masked points (segmented objects) vs background
            is_masked = data[:, 6] > self.mask_threshold
            masked_data = data[is_masked]
            background_data = data[~is_masked]
            
            # --- Optimization: Downsample background ---
            # Most of the lag comes from the density of the background.
            # Use the factor from parameters (default: 4)
            if self.downsample_factor > 1:
                background_data = background_data[::self.downsample_factor]
            
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
            
            # Vectorized packing function using Numpy structured arrays
            def pack_cloud_data_fast(points):
                if len(points) == 0:
                    return None
                
                N = len(points)
                # Define structured array dtype matching PointField offsets
                # x:f4(4), y:f4(4), z:f4(4), rgb:f4(4) = 16 bytes
                cloud_arr = np.empty(N, dtype=[
                    ('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'f4')
                ])
                
                # Vectorized axis conversion: X=-Z, Y=-X, Z=Y
                cloud_arr['x'] = -points[:, 2]
                cloud_arr['y'] = -points[:, 0]
                cloud_arr['z'] =  points[:, 1]
                
                # Vectorized color packing
                r = (points[:, 3] * 255).astype(np.uint32)
                g = (points[:, 4] * 255).astype(np.uint32)
                b = (points[:, 5] * 255).astype(np.uint32)
                
                # Pack into 00RRGGBB.
                rgb = (r << 16) | (g << 8) | b
                cloud_arr['rgb'] = rgb.view(np.float32)
                
                return cloud_arr

            def create_cloud_from_arr(header, fields, arr):
                if arr is None or len(arr) == 0:
                    return pc2.create_cloud(header, fields, [])
                
                # Direct buffer to PointCloud2 message for performance
                msg = PointCloud2()
                msg.header = header
                msg.height = 1
                msg.width = len(arr)
                msg.fields = fields
                msg.is_bigendian = False
                msg.point_step = 16 # 4 * 4
                msg.row_step = msg.point_step * msg.width
                msg.is_dense = False
                msg.data = arr.tobytes()
                return msg

            # Publish Obstacles
            masked_arr = pack_cloud_data_fast(masked_data)
            cloud_msg_obs = create_cloud_from_arr(header, fields, masked_arr)
            self.pub_obstacles.publish(cloud_msg_obs)
                
            # Publish Background
            background_arr = pack_cloud_data_fast(background_data)
            cloud_msg_bg = create_cloud_from_arr(header, fields, background_arr)
            self.pub_background.publish(cloud_msg_bg)
            
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
