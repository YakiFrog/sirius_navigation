#!/usr/bin/env python3
"""
Unity Stereo Image ROS Bridge (Lightweight HTTP Receiver)
Receives Side-by-Side stereo frames posted by Unity's StereoImageSender.cs (port 8080)
and publishes them directly as ROS2 CompressedImage topics for lightweight Rosbag recording.
"""

import sys
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Header


class StereoHTTPHandler(BaseHTTPRequestHandler):
    """Handles HTTP POST /upload_frame from Unity StereoImageSender.cs"""
    
    bridge_node = None  # Reference to the ROS2 node

    def do_POST(self):
        if self.path == '/upload_frame' or self.path == '/upload_sbs':
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_response(400)
                self.end_headers()
                return

            post_data = self.rfile.read(content_length)

            # Extract Unity camera parameters & sim time from HTTP headers
            sim_time_str = self.headers.get("X-Unity-Sim-Time", "")
            fx_str = self.headers.get("X-Camera-Fx", "448.14")
            fy_str = self.headers.get("X-Camera-Fy", "448.14")
            cx_str = self.headers.get("X-Camera-Cx", "640.0")
            cy_str = self.headers.get("X-Camera-Cy", "360.0")
            baseline_str = self.headers.get("X-Camera-Baseline", "0.12")

            sim_time = float(sim_time_str) if sim_time_str else None
            camera_params = {
                "fx": float(fx_str),
                "fy": float(fy_str),
                "cx": float(cx_str),
                "cy": float(cy_str),
                "baseline": float(baseline_str),
                "sim_time": sim_time
            }

            if StereoHTTPHandler.bridge_node is not None:
                StereoHTTPHandler.bridge_node.publish_frame(post_data, camera_params)

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status": "ok"}')
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # Suppress standard HTTP request logs to keep terminal clean
        pass


class UnityStereoBridge(Node):
    def __init__(self):
        super().__init__('unity_stereo_bridge')

        self.declare_parameter('port', 8080)
        self.declare_parameter('frame_id', 'sirius3/zed_camera_link')
        self.declare_parameter('image_topic', '/camera/stereo_sbs/compressed')
        self.declare_parameter('params_topic', '/camera/stereo_params')

        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.params_topic = self.get_parameter('params_topic').get_parameter_value().string_value

        self.pub_image = self.create_publisher(CompressedImage, self.image_topic, 10)
        self.pub_params = self.create_publisher(String, self.params_topic, 10)

        self.last_params_json = ""
        self.frame_count = 0
        self.last_report_time = self.get_clock().now()

        StereoHTTPHandler.bridge_node = self

        self.server = HTTPServer(('0.0.0.0', self.port), StereoHTTPHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

        self.get_logger().info(f"Unity Stereo Bridge started on port {self.port}")
        self.get_logger().info(f"Publishing to {self.image_topic} and {self.params_topic}")

    def publish_frame(self, jpeg_bytes: bytes, camera_params: dict):
        now = self.get_clock().now()

        # 1. Publish CompressedImage with accurate simulation timestamp
        msg = CompressedImage()
        sim_t = camera_params.get("sim_time")
        if sim_t is not None and sim_t > 0:
            from builtin_interfaces.msg import Time
            t_msg = Time()
            t_msg.sec = int(sim_t)
            t_msg.nanosec = int((sim_t - int(sim_t)) * 1e9)
            msg.header.stamp = t_msg
        else:
            msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = jpeg_bytes
        self.pub_image.publish(msg)

        # 2. Publish Camera Params if changed
        params_json = json.dumps(camera_params)
        if params_json != self.last_params_json:
            self.last_params_json = params_json
            param_msg = String()
            param_msg.data = params_json
            self.pub_params.publish(param_msg)

        self.frame_count += 1
        elapsed = (now - self.last_report_time).nanoseconds / 1e9
        if elapsed >= 5.0:
            fps = self.frame_count / elapsed
            self.get_logger().info(f"Streaming Active: {fps:.1f} FPS (Frame size: {len(jpeg_bytes) // 1024} KB)")
            self.frame_count = 0
            self.last_report_time = now

    def destroy_node(self):
        if self.server:
            self.server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnityStereoBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
