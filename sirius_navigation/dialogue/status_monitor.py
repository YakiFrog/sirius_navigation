#!/usr/bin/env python3
import sys
import math
import time
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QFrame, QScrollArea, QGridLayout, QMessageBox
)
from PyQt5.QtCore import QTimer, Qt, QPointF
from PyQt5.QtGui import QFont, QPalette, QColor, QClipboard, QPainter, QPen, QBrush

try:
    from .face_client import FaceClient
except ImportError:
    from face_client import FaceClient


class _SilentLogger:
    def warning(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class SiriusStatusMonitor(Node):
    KNOWLEDGE_ITEMS = [
        "現在座標 (TF map -> base_footprint)",
        "現在Yaw角",
        "目標座標 (/goal_pose)",
        "目標までの残距離",
        "直近の制御 (/nav_control)",
        "現在速度 (/odom)",
        "周囲人数 (/target_detector/target_markers)",
        "顔サーバー接続状態",
        "バッテリー残量 (Face GetStatus)",
        "表情状態",
        "直前の動作状態",
        "経路実行中かどうか",
        "障害物で停止中かどうか",
    ]

    MOVE_CAPABILITIES = [
        "前進 / 後退",
        "右向き / 左向き",
        "その場旋回",
        "絶対方位に向く",
        "座標へ移動",
        "速度変更",
        "表情変更",
        "停止 / 再開 / キャンセル",
    ]

    def __init__(self):
        super().__init__('sirius_status_monitor')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.face_client = FaceClient(logger=_SilentLogger())

        self.current_vel_x = 0.0
        self.current_vel_theta = 0.0
        self.surrounding_people_count = 0
        self.current_expression = "unknown"
        self.face_active = True
        self.last_nav_control = "idle"
        self.last_action_status = "n/a"
        self.last_action_type = "n/a"
        self.active_goal = None
        self.goal_distance = None
        self.current_xy_tolerance = 0.50
        self.is_stuck = False

        self._battery_cache_text = "[unknown]"
        self._battery_cache_time = 0.0

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(MarkerArray, '/target_detector/target_markers', self.marker_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(String, '/nav_control', self.nav_control_callback, 10)
        self.create_subscription(Bool, '/sirius_is_stuck', self.stuck_callback, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10)
        self.last_costmap_time = 0.0
        self.obstacle_distances = {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0}

    def odom_callback(self, msg):
        self.current_vel_x = msg.twist.twist.linear.x
        self.current_vel_theta = msg.twist.twist.angular.z

    def marker_callback(self, msg):
        tracked_ids = set()
        for marker in msg.markers:
            if marker.ns == 'tracked_targets' and marker.action == Marker.ADD:
                tracked_ids.add(marker.id)
        self.surrounding_people_count = len(tracked_ids)

    def goal_callback(self, msg):
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        self.active_goal = (msg.pose.position.x, msg.pose.position.y, yaw_deg)

    def nav_control_callback(self, msg):
        self.last_nav_control = msg.data.strip() or "idle"

    def stuck_callback(self, msg):
        self.is_stuck = msg.data

    def costmap_callback(self, msg):
        now = time.time()
        if now - self.last_costmap_time < 0.95:
            return
        self.last_costmap_time = now

        try:
            trans = self.tf_buffer.lookup_transform(
                'sirius3/base_footprint',
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except Exception:
            return

        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        res = msg.info.resolution
        width = msg.info.width
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        front_dist = 999.0
        left_dist = 999.0
        right_dist = 999.0
        back_dist = 999.0

        for idx, cost in enumerate(msg.data):
            if cost >= 99:
                row = idx // width
                col = idx % width
                
                mx = origin_x + (col + 0.5) * res
                my = origin_y + (row + 0.5) * res
                
                x_robot = cos_yaw * mx - sin_yaw * my + tx
                y_robot = sin_yaw * mx + cos_yaw * my + ty
                
                dist = math.sqrt(x_robot**2 + y_robot**2)
                if dist == 0.0:
                    continue
                angle_deg = math.degrees(math.atan2(y_robot, x_robot))
                angle_deg = (angle_deg + 180) % 360 - 180
                
                if -20.0 <= angle_deg <= 20.0:
                    if dist < front_dist:
                        front_dist = dist
                elif 20.0 < angle_deg <= 80.0:
                    if dist < left_dist:
                        left_dist = dist
                elif -80.0 <= angle_deg < -20.0:
                    if dist < right_dist:
                        right_dist = dist
                elif angle_deg > 135.0 or angle_deg < -135.0:
                    if dist < back_dist:
                        back_dist = dist

        self.obstacle_distances = {
            "front": front_dist,
            "left": left_dist,
            "right": right_dist,
            "back": back_dist
        }

    def get_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'sirius3/base_footprint', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
            return x, y, yaw_deg
        except Exception:
            return 0.0, 0.0, 0.0

    def get_status_data(self):
        now = time.time()
        if now - self._battery_cache_time > 10.0:
            self._battery_cache_text = self.face_client.get_battery_level_string()
            self._battery_cache_time = now

        x, y, yaw_deg = self.get_pose()
        self.face_active = self.face_client.face_server_active
        goal_str = "none"
        dist_str = "n/a"
        if self.active_goal is not None:
            gx, gy, gyaw = self.active_goal
            goal_str = f"X={gx:.2f}, Y={gy:.2f}, Yaw={gyaw:+.1f}°"
            self.goal_distance = math.sqrt((gx - x) ** 2 + (gy - y) ** 2)
            dist_str = f"{self.goal_distance:.2f}m"
        else:
            self.goal_distance = None

        status = "STUCK" if self.is_stuck else "OK"
        can_move = self._assess_moveability()

        return {
            "time": time.strftime('%Y-%m-%d %H:%M:%S'),
            "pose": f"X={x:.2f}, Y={y:.2f}, Yaw={yaw_deg:+.1f}°",
            "goal": goal_str,
            "distance": dist_str,
            "status": status,
            "can_move": can_move,
            "action": self.last_nav_control,
            "last_action": f"type={self.last_action_type} status={self.last_action_status}",
            "velocity": f"linear={self.current_vel_x:.2f}m/s angular={self.current_vel_theta:.2f}rad/s",
            "people": str(self.surrounding_people_count),
            "face": f"{'on' if self.face_active else 'off'} / {self.current_expression}",
            "battery": self._battery_cache_text,
        }

    def _assess_moveability(self):
        if self.is_stuck:
            return "No - blocked / stuck"
        if self.face_active is False:
            return "Yes - motion possible, face server offline"
        if self.active_goal is not None and self.goal_distance is not None:
            return f"Yes - target active, remaining {self.goal_distance:.2f}m"
        if self.last_nav_control in ["cancel", "pause"]:
            return f"Probably not - last control was {self.last_nav_control}"
        return "Yes - idle and ready"


class RadarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.obstacle_distances = {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0}
        self.setMinimumSize(220, 220)

    def set_distances(self, distances):
        self.obstacle_distances = distances
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()
        center_x = width / 2.0
        center_y = height / 2.0
        radius = min(width, height) / 2.0 - 12.0
        
        if radius <= 0:
            return

        max_range = 3.0
        
        # Dark circular background
        painter.setPen(QPen(QColor(40, 44, 52), 1.2, Qt.SolidLine))
        painter.setBrush(QBrush(QColor(18, 18, 22)))
        painter.drawEllipse(QPointF(center_x, center_y), radius, radius)
        
        # Inner circular range grids
        painter.setPen(QPen(QColor(40, 44, 52), 0.8, Qt.SolidLine))
        painter.drawEllipse(QPointF(center_x, center_y), radius * 2.0 / 3.0, radius * 2.0 / 3.0)
        painter.drawEllipse(QPointF(center_x, center_y), radius * 1.0 / 3.0, radius * 1.0 / 3.0)
        
        # Draw labels for ranges
        painter.setPen(QPen(QColor(110, 115, 130), 1, Qt.SolidLine))
        painter.setFont(QFont("Arial", 8))
        painter.drawText(int(center_x + 5), int(center_y - radius * 1.0 / 3.0 + 4), "1.0m")
        painter.drawText(int(center_x + 5), int(center_y - radius * 2.0 / 3.0 + 4), "2.0m")
        painter.drawText(int(center_x + 5), int(center_y - radius + 12), "3.0m")

        # Draw cross axes
        painter.setPen(QPen(QColor(40, 44, 52), 1, Qt.DashLine))
        painter.drawLine(int(center_x - radius), int(center_y), int(center_x + radius), int(center_y))
        painter.drawLine(int(center_x), int(center_y - radius), int(center_x), int(center_y + radius))

        # Direction indicators
        painter.setPen(QPen(QColor(140, 145, 160), 1, Qt.SolidLine))
        painter.drawText(int(center_x - 18), 16, "FRONT")
        painter.drawText(int(center_x - 15), int(height - 6), "BACK")
        painter.drawText(6, int(center_y + 4), "LEFT")
        painter.drawText(int(width - 38), int(center_y + 4), "RIGHT")

        # Sectors definition mapping standard angles
        sectors = {
            "front": (45, 90),
            "left": (135, 90),
            "back": (225, 90),
            "right": (315, 90)
        }
        
        for direction, (start_angle, span_angle) in sectors.items():
            dist = self.obstacle_distances.get(direction, 999.0)
            if dist < max_range:
                # Color code
                if dist < 0.8:
                    color = QColor(235, 47, 6, 200)   # Vivid Red
                elif dist < 1.5:
                    color = QColor(241, 196, 15, 180) # Orange-Yellow
                else:
                    color = QColor(46, 204, 113, 160)  # Green
                
                painter.setPen(QPen(color, 3.5, Qt.SolidLine))
                r_dist = radius * (dist / max_range)
                if r_dist < 10:
                    r_dist = 10
                
                rect_x = center_x - r_dist
                rect_y = center_y - r_dist
                painter.drawArc(int(rect_x), int(rect_y), int(r_dist * 2), int(r_dist * 2), int(start_angle * 16), int(span_angle * 16))
                
                # Draw small fill slice
                color_fill = QColor(color)
                color_fill.setAlpha(30)
                painter.setBrush(QBrush(color_fill))
                painter.setPen(Qt.NoPen)
                painter.drawPie(int(rect_x), int(rect_y), int(r_dist * 2), int(r_dist * 2), int(start_angle * 16), int(span_angle * 16))

        # Center dot (robot)
        painter.setBrush(QBrush(QColor(0, 168, 255)))
        painter.setPen(QPen(QColor(255, 255, 255), 1.5))
        painter.drawEllipse(QPointF(center_x, center_y), 7, 7)


class StatusMonitorWidget(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        # PyQt5 ウィンドウの基本設定
        self.setWindowTitle("Sirius Telemetry Status Monitor")
        self.resize(900, 600)

        # プレミアムダークテーマスタイルシート
        self.setStyleSheet("""
            QWidget {
                background-color: #121214;
                color: #e3e3e6;
                font-family: 'Helvetica Neue', Helvetica, Arial, sans-serif;
            }
            QLabel {
                font-size: 14px;
            }
            QFrame#card {
                background-color: #1e1e24;
                border-radius: 8px;
                padding: 10px;
            }
            QPushButton {
                background-color: #00a8ff;
                color: #ffffff;
                border-radius: 4px;
                padding: 8px 16px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #0097e6;
            }
            QPushButton:pressed {
                background-color: #007cc0;
            }
            QScrollArea {
                border: none;
                background-color: transparent;
            }
        """)

        # メインレイアウト
        main_layout = QVBoxLayout(self)

        # ヘッダーエリア
        header_layout = QHBoxLayout()
        title_label = QLabel("SIRIUS TELEMETRY MONITOR")
        title_label.setStyleSheet("font-size: 20px; font-weight: bold; color: #00a8ff; letter-spacing: 1px;")
        header_layout.addWidget(title_label)

        # 状態バッジ
        self.status_badge = QLabel("STATUS: OK")
        self.status_badge.setAlignment(Qt.AlignCenter)
        self.status_badge.setFixedSize(140, 30)
        self.status_badge.setStyleSheet("""
            background-color: #2ecc71;
            color: #ffffff;
            font-weight: bold;
            border-radius: 15px;
            font-size: 13px;
        """)
        header_layout.addWidget(self.status_badge)
        main_layout.addLayout(header_layout)

        # メインスプリットレイアウト
        split_layout = QHBoxLayout()

        # 左側: ステータスグリッド
        self.left_card = QFrame()
        self.left_card.setObjectName("card")
        left_layout = QVBoxLayout(self.left_card)
        left_layout.setContentsMargins(15, 15, 15, 15)

        self.grid_layout = QGridLayout()
        self.grid_layout.setHorizontalSpacing(20)
        self.grid_layout.setVerticalSpacing(12)

        # 表示項目の作成
        self.value_labels = {}
        items = [
            ("Time", "time"),
            ("Pose", "pose"),
            ("Goal", "goal"),
            ("Distance", "distance"),
            ("Can Move", "can_move"),
            ("Action", "action"),
            ("Last Act", "last_action"),
            ("Velocity", "velocity"),
            ("People", "people"),
            ("Face State", "face"),
            ("Battery", "battery")
        ]

        for idx, (label_text, key) in enumerate(items):
            title_lbl = QLabel(label_text + " :")
            title_lbl.setStyleSheet("font-weight: bold; color: #8f92a1;")
            val_lbl = QLabel("Fetching...")
            val_lbl.setStyleSheet("color: #00d2ff; font-weight: 500;")
            val_lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.value_labels[key] = val_lbl

            self.grid_layout.addWidget(title_lbl, idx, 0)
            self.grid_layout.addWidget(val_lbl, idx, 1)

        left_layout.addLayout(self.grid_layout)
        left_layout.addStretch()

        # アクションエリア
        button_layout = QHBoxLayout()
        self.copy_btn = QPushButton("📋 Copy Status Data")
        self.copy_btn.clicked.connect(self.copy_to_clipboard)
        button_layout.addWidget(self.copy_btn)
        left_layout.addLayout(button_layout)

        split_layout.addWidget(self.left_card, stretch=3)

        # 右側: ナレッジと能力表示
        right_card = QFrame()
        right_card.setObjectName("card")
        right_layout = QVBoxLayout(right_card)

        # 障害物レーダーの表示
        radar_title = QLabel("🛰️ Proximity Obstacle Radar")
        radar_title.setStyleSheet("font-size: 15px; font-weight: bold; color: #00a8ff;")
        right_layout.addWidget(radar_title)
        
        self.radar_widget = RadarWidget()
        right_layout.addWidget(self.radar_widget)
        right_layout.addSpacing(10)

        # ナレッジスクロールエリア
        know_title = QLabel("💡 Monitor Target Knowledge")
        know_title.setStyleSheet("font-size: 15px; font-weight: bold; color: #fbc531;")
        right_layout.addWidget(know_title)

        know_scroll = QScrollArea()
        know_widget = QWidget()
        know_box = QVBoxLayout(know_widget)
        know_box.setContentsMargins(0, 5, 0, 5)
        for item in self.node.KNOWLEDGE_ITEMS:
            item_lbl = QLabel(f"• {item}")
            item_lbl.setStyleSheet("font-size: 12px; color: #dcdde1; padding: 2px;")
            know_box.addWidget(item_lbl)
        know_widget.setLayout(know_box)
        know_scroll.setWidget(know_widget)
        know_scroll.setWidgetResizable(True)
        right_layout.addWidget(know_scroll)

        # 能力スクロールエリア
        cap_title = QLabel("🤖 Movement Capabilities")
        cap_title.setStyleSheet("font-size: 15px; font-weight: bold; color: #fbc531; margin-top: 15px;")
        right_layout.addWidget(cap_title)

        cap_scroll = QScrollArea()
        cap_widget = QWidget()
        cap_box = QVBoxLayout(cap_widget)
        cap_box.setContentsMargins(0, 5, 0, 5)
        for item in self.node.MOVE_CAPABILITIES:
            item_lbl = QLabel(f"• {item}")
            item_lbl.setStyleSheet("font-size: 12px; color: #dcdde1; padding: 2px;")
            cap_box.addWidget(item_lbl)
        cap_widget.setLayout(cap_box)
        cap_scroll.setWidget(cap_widget)
        cap_scroll.setWidgetResizable(True)
        right_layout.addWidget(cap_scroll)

        split_layout.addWidget(right_card, stretch=2)
        main_layout.addLayout(split_layout)

        # タイマー（100ms周期でROS 2のメッセージをスピン & 画面リフレッシュ）
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.ros_spin_once)
        self.ros_timer.start(50)  # 20Hz

        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self.refresh_gui)
        self.gui_timer.start(1000)  # 1Hz

        self.refresh_gui()

    def ros_spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def refresh_gui(self):
        data = self.node.get_status_data()
        for key, val_lbl in self.value_labels.items():
            val_lbl.setText(data.get(key, "n/a"))

        # 状態バッジとカラーの動的変更
        status = data.get("status", "OK")
        self.status_badge.setText(f"STATUS: {status}")
        if status == "STUCK":
            self.status_badge.setStyleSheet("""
                background-color: #ff4d4d;
                color: #ffffff;
                font-weight: bold;
                border-radius: 15px;
                font-size: 13px;
            """)
        else:
            self.status_badge.setStyleSheet("""
                background-color: #2ecc71;
                color: #ffffff;
                font-weight: bold;
                border-radius: 15px;
                font-size: 13px;
            """)

        # レーダー表示の更新
        self.radar_widget.set_distances(self.node.obstacle_distances)

    def copy_to_clipboard(self):
        data = self.node.get_status_data()
        text_lines = [
            "=== Sirius Status Telemetry ===",
            f"Time      : {data['time']}",
            f"Pose      : {data['pose']}",
            f"Goal      : {data['goal']}",
            f"Distance  : {data['distance']}",
            f"Status    : {data['status']}",
            f"Can Move  : {data['can_move']}",
            f"Action    : {data['action']}",
            f"Last Act  : {data['last_action']}",
            f"Velocity  : {data['velocity']}",
            f"People    : {data['people']}",
            f"Face      : {data['face']}",
            f"Battery   : {data['battery']}",
        ]
        text_content = "\n".join(text_lines)
        clipboard = QApplication.clipboard()
        clipboard.setText(text_content)
        self.copy_btn.setText("✓ Copied to Clipboard!")
        QTimer.singleShot(2000, lambda: self.copy_btn.setText("📋 Copy Status Data"))

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.gui_timer.stop()
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    
    node = SiriusStatusMonitor()
    widget = StatusMonitorWidget(node)
    widget.show()
    
    try:
        sys.exit(app.exec_())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
