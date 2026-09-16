#!/usr/bin/env python3
"""THETA S (HDMI -> USBキャプチャ) -> CompressedImage 配信。

`/dev/theta_capture`（udevシンボリックリンク）から MJPG で取得し、JPEGで
`/theta/dual_fisheye/image_raw/compressed`（既定 frame `sirius3/theta_link`）へ配信する。
既存のオフライン路面マッピング（theta_bev_node 以降）にそのまま載せられる。

- 映像はequirect/dual-fisheyeのどちらでもよい（下流の校正に合わせる）。
- 実機はwall clock（use_sim_time=false）を使用。
- デバイスが無い/切れた場合は再接続を試みる（もう一台のキャプチャに変わっても device を変えればOK）。
"""
import threading
import time
import os

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class ThetaCaptureNode(Node):
    def __init__(self):
        super().__init__('theta_capture')
        self.declare_parameter('device', '/dev/theta_capture')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        # I-O DATA HDPC-UT は MJPG が一様フレームになるため YUYV 既定。
        self.declare_parameter('fourcc', 'YUYV')
        # 間引きレート[Hz]。範囲 0.1〜30.0（上限30は THETA S の HDMI ライブ出力の上限）。
        # 既定 5.0 fps（bag容量/負荷対策）。根拠: RTAB-Mapの取り込みは
        # DetectionRate=2Hz、SAM3透視投影は約1.5Hz、画像が容量の大半。RVizプレビューを
        # 滑らかにしたい場合は fps:=10〜15 に上げる（録画は5推奨）。実測はCPU律速で
        # 1920x1080 YUYV取得+JPEG/生配信のため目標より下がることがある（例: 5→約3.4fps）。
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('output_topic', '/theta/dual_fisheye/image_raw/compressed')
        self.declare_parameter('frame_id', 'sirius3/theta_link')
        self.declare_parameter('jpeg_quality', 85)
        # RViz2のImage表示は生Imageのみなので、デコード済みを生Imageでも配信する。
        self.declare_parameter('publish_raw', True)
        self.declare_parameter('raw_topic', '/theta/dual_fisheye/image_raw')

        # 実機THETA SのHDMI出力は各レンズが光軸周りに回転している（左=前CCW90°/右=後CW90°）。
        # ただし向き補正は「投影時」に行う（校正YAMLの image_roll_degrees。theta_bev_projection /
        # theta_perspective が適用）。画素を回す方式は近似中心/ROIで円がずれ残像が出るため既定オフ。
        # どうしても画素側で回したい場合のみ rotate_* を指定（CCW正[deg], 中心は画像比(u,v)）。
        self.declare_parameter('rotate_front', 0)
        self.declare_parameter('rotate_back', 0)
        self.declare_parameter('front_center', [0.2677, 0.4407])
        self.declare_parameter('back_center', [0.7536, 0.4528])

        self.device = self.get_parameter('device').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fourcc = str(self.get_parameter('fourcc').value)
        self.fps = min(30.0, max(0.1, float(self.get_parameter('fps').value)))
        self.out_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.rotate_front = int(self.get_parameter('rotate_front').value)
        self.rotate_back = int(self.get_parameter('rotate_back').value)
        self.front_center = [float(x) for x in self.get_parameter('front_center').value]
        self.back_center = [float(x) for x in self.get_parameter('back_center').value]

        self.cap = None
        self.active_device = None
        self.last_publish = 0.0
        self.pub = self.create_publisher(CompressedImage, self.out_topic, qos_profile_sensor_data)
        # RVizのImage表示(Reliable)に合わせて生ImageはReliableで配信する。
        self.pub_raw = self.create_publisher(Image, self.get_parameter('raw_topic').value, 1) if self.publish_raw else None
        threading.Thread(target=self._loop, daemon=True).start()
        self.get_logger().info(
            f'THETA capture: {self.device} {self.width}x{self.height} @ {self.fps}fps '
            f'-> {self.out_topic} (frame={self.frame_id}) '
            f'rotate front={self.rotate_front}deg back={self.rotate_back}deg')

    def _device_candidates(self):
        candidates = [self.device]
        # udev未設定などで既定パスが無い場合のフォールバック
        for fallback in ('/dev/theta_capture', '/dev/video0', '/dev/video1'):
            if fallback not in candidates:
                candidates.append(fallback)
        return candidates

    def _open(self):
        for device in self._device_candidates():
            if device.startswith('/dev/') and not os.path.exists(device):
                continue
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not cap.isOpened():
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if device != self.device:
                self.get_logger().warning(f'{self.device} が無いため {device} を使用します')
            self.active_device = device
            return cap
        return None

    def _rotate_lens(self, frame, center, angle):
        if angle % 360 == 0:
            return
        h, w = frame.shape[:2]
        cx, cy = int(round(center[0] * w)), int(round(center[1] * h))
        half = int(min(cx, w - cx, cy, h - cy))
        if half < 8:
            self.get_logger().warning(
                f'レンズ回転補正をスキップ（中心 {center} が画像端に近すぎます）', throttle_duration_sec=10)
            return
        roi = frame[cy - half:cy + half, cx - half:cx + half]
        a = angle % 360
        if a == 90:
            out = cv2.rotate(roi, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif a == 270:
            out = cv2.rotate(roi, cv2.ROTATE_90_CLOCKWISE)
        elif a == 180:
            out = cv2.rotate(roi, cv2.ROTATE_180)
        else:
            m = cv2.getRotationMatrix2D((half, half), angle, 1.0)
            out = cv2.warpAffine(roi, m, (2 * half, 2 * half))
        frame[cy - half:cy + half, cx - half:cx + half] = out

    def _apply_rotations(self, frame):
        self._rotate_lens(frame, self.front_center, self.rotate_front)
        self._rotate_lens(frame, self.back_center, self.rotate_back)
        return frame

    def _loop(self):
        warned = False
        while rclpy.ok():
            if self.cap is None or not self.cap.isOpened():
                self.cap = self._open()
                if self.cap is None:
                    if not warned:
                        self.get_logger().warning(
                            f'{self.device} を開けません（信号待ち/占有中）。再接続を試みます。',
                            throttle_duration_sec=10)
                        warned = True
                    time.sleep(1.0)
                    continue
                warned = False
                self.get_logger().info(f'{self.active_device} を開きました')
            ret, frame = self.cap.read()
            if not ret or frame is None or frame.size == 0:
                self.get_logger().warning('フレーム取得失敗。再接続します。', throttle_duration_sec=5)
                self.cap.release()
                self.cap = None
                time.sleep(0.5)
                continue
            frame = self._apply_rotations(frame)
            now = time.time()
            if now - self.last_publish < 1.0 / self.fps:
                continue
            self.last_publish = now
            ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                continue
            message = CompressedImage()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.frame_id
            message.format = 'jpeg'
            message.data = encoded.tobytes()
            self.pub.publish(message)
            if self.pub_raw is not None:
                raw = Image()
                raw.header = message.header
                raw.height, raw.width = frame.shape[:2]
                raw.encoding = 'bgr8'
                raw.is_bigendian = False
                raw.step = raw.width * 3
                raw.data = frame.tobytes()
                self.pub_raw.publish(raw)


def main():
    rclpy.init()
    node = ThetaCaptureNode()
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
