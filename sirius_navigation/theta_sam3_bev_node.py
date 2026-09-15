#!/usr/bin/env python3
"""THETA BEV -> SAM3(既存Dockerサーバ) -> 予約IDラベル画像 /theta/bev_semantic (mono8)。

既存サーバ(sam3_zed_server.py)は内部で全クラスの semantic_id_map を計算しており、
追加エンドポイント GET /semantic_map.png がそれを8bitグレースケールPNGで返す。
本ノードは BEV をサーバへ送り、そのマップを取得して publish するだけ。

RSJ2026規則（予約ID, サーバの class_ids と一致）:
  0=unknown, 1=wall, 2=floor, 3=grass, 4=tactile paving, 5=roadway, 6=sidewalk
セマンティックに該当しない路面は theta_indexed_map_node が代表色(量子化)で補う。
"""
import threading
import time

import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PROMPTS = 'grass, tactile paving, line-type tactile paving, roadway, sidewalk'
# 予約IDと代表色（line-type tactile paving は tactile paving と同じID4=黄）
CLASS_REGISTRY = {
    'grass': {'id': 3, 'color': [0, 255, 0]},
    'tactile paving': {'id': 4, 'color': [255, 255, 0]},
    'line-type tactile paving': {'id': 4, 'color': [255, 255, 0]},
    'roadway': {'id': 5, 'color': [0, 0, 255]},
    'sidewalk': {'id': 6, 'color': [128, 128, 128]},
}


class ThetaSam3BevNode(Node):
    def __init__(self):
        super().__init__('theta_sam3_bev_node')
        self.declare_parameter('server', 'http://localhost:8080')
        self.declare_parameter('input_topic', '/theta/bev/image_raw')
        self.declare_parameter('output_topic', '/theta/bev_semantic')
        self.declare_parameter('output_frame', 'sirius3/base_footprint')
        self.declare_parameter('prompts', PROMPTS)
        self.declare_parameter('threshold', 0.3)
        self.declare_parameter('infer_timeout_sec', 1.0)
        self.declare_parameter('min_interval_sec', 0.3)

        self.server = self.get_parameter('server').value.rstrip('/')
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.prompts = self.get_parameter('prompts').value
        self.threshold = float(self.get_parameter('threshold').value)
        self.infer_timeout = max(0.1, float(self.get_parameter('infer_timeout_sec').value))
        self.min_interval = max(0.0, float(self.get_parameter('min_interval_sec').value))

        self.lock = threading.Lock()
        self.latest = None  # (stamp, bgr)
        self.frame_version = -1

        self.pub = self.create_publisher(Image, self.output_topic, 1)
        self.sub = self.create_subscription(
            Image, self.get_parameter('input_topic').value, self.receive, qos_profile_sensor_data)
        threading.Thread(target=self._infer_loop, daemon=True).start()
        self.get_logger().info(
            f'THETA SAM3 semantic (server mode): {self.get_parameter("input_topic").value} -> '
            f'{self.output_topic} (server {self.server})')

    def receive(self, msg):
        if msg.encoding not in ('bgr8', 'rgb8'):
            return
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            image = image[:, :, ::-1].copy()
        with self.lock:
            self.latest = (msg.header.stamp, image)

    def _post(self, path, obj=None, data=None):
        try:
            requests.post(self.server + path, json=obj, data=data, timeout=2.0)
            return True
        except Exception as error:
            self.get_logger().warning(f'POST {path} failed: {error}', throttle_duration_sec=5)
            return False

    def _infer_loop(self):
        self._post('/source_mode', {'mode': 'network'})
        self._post('/crop', {'mode': 'none'})
        self._post('/class_registry', {'classes': CLASS_REGISTRY})
        self._post('/prompt', {'prompt': self.prompts})
        self._post('/threshold', {'threshold': self.threshold})
        while rclpy.ok():
            with self.lock:
                item = self.latest
            if item is None:
                time.sleep(0.1)
                continue
            stamp, bev = item
            ok, encoded = cv2.imencode('.jpg', bev, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if not ok:
                continue
            started_at = time.time()
            if not self._post('/upload_frame', data=encoded.tobytes()):
                time.sleep(self.min_interval)
                continue
            label = self._wait_semantic_map(bev.shape[1], bev.shape[0], started_at)
            if label is None:
                time.sleep(self.min_interval)
                continue
            out = Image()
            out.header.stamp = stamp
            out.header.frame_id = self.output_frame
            out.height, out.width = label.shape[0], label.shape[1]
            out.encoding = 'mono8'
            out.step = label.shape[1]
            out.data = label.tobytes()
            self.pub.publish(out)
            time.sleep(self.min_interval)

    def _wait_semantic_map(self, width, height, started_at):
        deadline = time.time() + self.infer_timeout
        while time.time() < deadline:
            try:
                response = requests.get(self.server + '/semantic_map.png', timeout=2.0)
            except Exception:
                time.sleep(0.05)
                continue
            if response.status_code == 200 and response.content:
                array = np.frombuffer(response.content, np.uint8)
                image = cv2.imdecode(array, cv2.IMREAD_GRAYSCALE)
                if image is not None:
                    if image.shape != (height, width):
                        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_NEAREST)
                    return image
            time.sleep(0.05)
        return None


def main():
    rclpy.init()
    node = ThetaSam3BevNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
