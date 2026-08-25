#!/usr/bin/env python3
"""Feed rosbag stereo images to the SAM3 server.

Subscribes to recorded stereo images (/camera/stereo_sbs/compressed) from rosbag playback,
and feeds them via HTTP POST to the GPU-accelerated sam3_zed_server (Docker on port 8080).
Works completely without requiring PyTorch/CUDA in the host ROS2 environment!
"""

import json
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class SAM3RosbagPlayer(Node):
    def __init__(self):
        super().__init__('sam3_rosbag_player')

        self.declare_parameter('server_url', 'http://localhost:8080/upload_frame')
        self.declare_parameter('image_topic', '/camera/stereo_sbs/compressed')
        self.declare_parameter('params_topic', '/camera/stereo_params')
        self.declare_parameter('min_interval_sec', 0.05)  # Max 20 FPS feed rate
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.server_url = self.get_parameter('server_url').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.params_topic = self.get_parameter('params_topic').get_parameter_value().string_value
        self.min_interval = (
            self.get_parameter('min_interval_sec').get_parameter_value().double_value
        )

        self.last_feed_time = 0.0
        self.camera_params = {
            'fx': 448.14, 'fy': 448.14, 'cx': 640.0, 'cy': 360.0, 'baseline': 0.12
        }
        self.camera_params_received = False
        self.first_frame_sent = False

        # Subscribers
        # rosbag2 replays sensor-data topics as BEST_EFFORT. A RELIABLE
        # subscription is incompatible and silently receives no images, so use
        # the standard sensor-data QoS profile explicitly.
        self.sub_image = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data
        )
        self.sub_params = self.create_subscription(
            String,
            self.params_topic,
            self._params_callback,
            10
        )

        self.get_logger().info(
            f'SAM3 Rosbag Player started. Feeding {self.image_topic} to {self.server_url}'
        )

    def _params_callback(self, msg: String):
        try:
            p = json.loads(msg.data)
            for k in ['fx', 'fy', 'cx', 'cy', 'baseline']:
                if k in p:
                    self.camera_params[k] = float(p[k])
            self.camera_params_received = all(
                key in p for key in ['fx', 'fy', 'cx', 'cy', 'baseline']
            )
            if self.camera_params_received:
                source = p.get('source', 'unknown')
                serial = p.get('serial_number', 'unknown')
                fx = self.camera_params['fx']
                baseline = self.camera_params['baseline']
                self.get_logger().info(
                    f'Camera calibration ready: source={source}, serial={serial}, '
                    f'fx={fx:.2f}, baseline={baseline:.4f}m',
                    once=True,
                )
        except Exception as e:
            self.get_logger().warning(f'Failed to parse camera params: {e}')

    def _image_callback(self, msg: CompressedImage):
        if not self.camera_params_received:
            self.get_logger().warning(
                'Camera image received before /camera/stereo_params; frame is not sent to SAM3.',
                throttle_duration_sec=5.0,
            )
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_feed_time < self.min_interval:
            return
        self.last_feed_time = now_sec

        try:
            # Stamp to simulation time float
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            headers = {
                'Content-Type': 'image/jpeg',
                'X-Unity-Sim-Time': str(stamp_sec),
                'X-Camera-Fx': str(self.camera_params['fx']),
                'X-Camera-Fy': str(self.camera_params['fy']),
                'X-Camera-Cx': str(self.camera_params['cx']),
                'X-Camera-Cy': str(self.camera_params['cy']),
                'X-Camera-Baseline': str(self.camera_params['baseline']),
            }

            req = urllib.request.Request(
                self.server_url,
                data=msg.data,
                headers=headers,
                method='POST'
            )

            with urllib.request.urlopen(req, timeout=1.0) as response:
                if response.status != 200:
                    raise RuntimeError(f'SAM3 server returned HTTP {response.status}')

            if not self.first_frame_sent:
                self.first_frame_sent = True
                fx = self.camera_params['fx']
                self.get_logger().info(
                    f'First rosbag camera frame delivered to SAM3: '
                    f'stamp={stamp_sec:.6f}, frame_id={msg.header.frame_id}, '
                    f'fx={fx:.2f}'
                )

        except urllib.error.URLError as e:
            self.get_logger().warning(
                f'Failed to deliver camera frame to SAM3: {e}',
                throttle_duration_sec=5.0,
            )
        except Exception as e:
            self.get_logger().error(
                f'Error posting frame to SAM3 server: {e}',
                throttle_duration_sec=5.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = SAM3RosbagPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
