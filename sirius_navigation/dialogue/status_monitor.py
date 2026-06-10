#!/usr/bin/env python3
import sys
import os
import ast
import json
import math
import time
from collections import deque
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QPushButton, QFrame, QScrollArea, QGridLayout, QMessageBox
)
from PyQt5.QtCore import QTimer, Qt, QPointF
from PyQt5.QtGui import QFont, QPalette, QColor, QClipboard, QPainter, QPen, QBrush, QPolygonF

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
    RADAR_MAX_RANGE = 2.0

    KNOWLEDGE_ITEMS = [
        "現在座標 (TF map -> base_footprint)",
        "現在Yaw角",
        "目標座標 (/goal_pose)",
        "目標までの残距離",
        "直近の制御 (/nav_control)",
        "現在速度 (/odom)",
        "周囲人数 (/target_detector/target_markers)",
        "ランドマーク一覧 (/sirius/landmark_status)",
        "顔サーバー接続状態",
        "バッテリー残量 (Face GetStatus)",
        "表情状態",
        "直前の動作状態",
        "経路実行中かどうか",
        "障害物で停止中かどうか",
        "何を覚えているか / 何を忘れているか",
    ]

    MOVE_CAPABILITIES = [
        "前進 / 後退",
        "右向き / 左向き",
        "その場旋回",
        "絶対方位に向く",
        "座標へ移動",
        "ランドマーク名で移動",
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
        self.landmark_status = {"map": "unknown", "count": 0, "names": [], "file": ""}
        self.goal_distance = None
        self.current_xy_tolerance = 0.50
        self.is_stuck = False
        self.robot_base_frame = 'sirius3/base_footprint'
        self.robot_footprint = self._load_nav2_footprint()

        self._battery_cache_text = "[unknown]"
        self._battery_cache_time = 0.0

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(MarkerArray, '/target_detector/target_markers', self.marker_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(String, '/nav_control', self.nav_control_callback, 10)
        self.create_subscription(Bool, '/sirius_is_stuck', self.stuck_callback, 10)
        landmark_qos = QoSProfile(depth=1)
        landmark_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        landmark_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(String, '/sirius/landmark_status', self.landmark_status_callback, landmark_qos)
        self.active_command = None
        self.command_queue = []
        self.command_history = []
        self.create_subscription(String, '/sirius/command_queue', self.queue_callback, landmark_qos)
        self.create_subscription(LaserScan, '/hokuyo_scan', lambda msg: self.scan_callback(msg, '/hokuyo_scan'), 10)
        self.create_subscription(LaserScan, '/scan3', lambda msg: self.scan_callback(msg, '/scan3'), 10)
        self.obstacle_distances = {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0}
        self.obstacle_points = []
        self._scan_obstacle_distances = {}
        self._scan_obstacle_points = {}
        self._last_scan_times = {}
        self._scan_hz_samples = {
            '/hokuyo_scan': deque(maxlen=40),
            '/scan3': deque(maxlen=40),
        }
        self._last_scan_hz_time = {}

    def queue_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.active_command = data.get("active")
            self.command_queue = data.get("queue", [])
            self.command_history = data.get("history", [])
        except Exception:
            pass

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

    def landmark_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.landmark_status = {
                    "map": str(data.get("map", "unknown")),
                    "count": int(data.get("count", 0)),
                    "names": [str(name) for name in data.get("names", [])],
                    "file": str(data.get("file", "")),
                }
        except Exception:
            self.landmark_status = {"map": "unknown", "count": 0, "names": [], "file": ""}

    def _load_nav2_footprint(self):
        fallback = [[0.50, 0.30], [0.50, -0.30], [-0.70, -0.30], [-0.70, 0.30]]
        candidate_paths = [
            os.environ.get('NAV2_PARAMS_FILE'),
            os.path.expanduser('~/sirius_jazzy_ws/params/nav2_params.yaml'),
            os.path.expanduser('~/sirius_jazzy_ws/params/nav2_params_sim.yaml'),
            '/home/kotantu-desktop/sirius_jazzy_ws/params/nav2_params.yaml',
            '/home/kotantu-desktop/sirius_jazzy_ws/params/nav2_params_sim.yaml',
        ]

        for path in candidate_paths:
            if not path or not os.path.exists(path):
                continue
            footprint = self._extract_footprint_from_yaml(path)
            if footprint:
                self.get_logger().info(f"Loaded radar footprint from {path}: {footprint}")
                return footprint

        self.get_logger().warning(f"Using fallback radar footprint: {fallback}")
        return fallback

    def _extract_footprint_from_yaml(self, path):
        in_local_costmap = False
        in_ros_parameters = False
        local_indent = None
        params_indent = None

        try:
            with open(path, 'r', encoding='utf-8') as yaml_file:
                for raw_line in yaml_file:
                    line = raw_line.split('#', 1)[0].rstrip()
                    if not line.strip():
                        continue

                    indent = len(line) - len(line.lstrip(' '))
                    stripped = line.strip()

                    if indent == 0:
                        in_local_costmap = stripped == 'local_costmap:'
                        local_indent = indent if in_local_costmap else None
                        in_ros_parameters = False
                        params_indent = None
                        continue

                    if in_local_costmap and stripped == 'ros__parameters:':
                        in_ros_parameters = True
                        params_indent = indent
                        continue

                    if in_local_costmap and local_indent is not None and indent <= local_indent:
                        in_local_costmap = False
                        in_ros_parameters = False
                        continue

                    if in_ros_parameters and params_indent is not None and indent <= params_indent:
                        in_ros_parameters = False
                        continue

                    if in_ros_parameters and stripped.startswith('footprint:'):
                        _, value = stripped.split(':', 1)
                        value = value.strip().strip('"').strip("'")
                        footprint = ast.literal_eval(value)
                        if self._is_valid_footprint(footprint):
                            return [[float(x), float(y)] for x, y in footprint]
        except Exception as exc:
            self.get_logger().warning(f"Failed to load radar footprint from {path}: {exc}")

        return None

    def _is_valid_footprint(self, footprint):
        return (
            isinstance(footprint, list)
            and len(footprint) >= 3
            and all(isinstance(point, (list, tuple)) and len(point) == 2 for point in footprint)
        )

    def scan_callback(self, msg, topic_name):
        now = time.time()
        self._record_scan_hz(topic_name, now)

        topic_key = topic_name
        if now - self._last_scan_times.get(topic_key, 0.0) < 0.08:
            return
        self._last_scan_times[topic_key] = now

        try:
            trans = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
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

        distances = {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0}
        points = []
        angle = msg.angle_min

        for scan_range in msg.ranges:
            if math.isfinite(scan_range) and msg.range_min <= scan_range <= msg.range_max:
                sensor_x = scan_range * math.cos(angle)
                sensor_y = scan_range * math.sin(angle)
                robot_x = cos_yaw * sensor_x - sin_yaw * sensor_y + tx
                robot_y = sin_yaw * sensor_x + cos_yaw * sensor_y + ty

                center_dist = math.hypot(robot_x, robot_y)
                if center_dist > 0.0:
                    angle_rad = math.atan2(robot_y, robot_x)
                    angle_deg = (math.degrees(angle_rad) + 180.0) % 360.0 - 180.0
                    footprint_dist = self._footprint_radius_at(angle_rad)
                    clearance = max(0.0, center_dist - footprint_dist)
                    if center_dist <= self.RADAR_MAX_RANGE and center_dist >= footprint_dist:
                        points.append((robot_x, robot_y))

                    direction = self._direction_from_angle(angle_deg)
                    if direction and clearance < distances[direction]:
                        distances[direction] = clearance
            angle += msg.angle_increment

        self._scan_obstacle_distances[topic_key] = (now, distances)
        self._scan_obstacle_points[topic_key] = (now, points)
        self._merge_scan_obstacle_distances(now)

    def _record_scan_hz(self, topic_name, now):
        last_time = self._last_scan_hz_time.get(topic_name)
        if last_time is not None:
            interval = now - last_time
            if interval > 0.0:
                self._scan_hz_samples.setdefault(topic_name, deque(maxlen=40)).append(interval)
        self._last_scan_hz_time[topic_name] = now

    def _format_scan_hz(self, now):
        parts = []
        for topic_name in ['/hokuyo_scan', '/scan3']:
            last_time = self._last_scan_hz_time.get(topic_name)
            samples = self._scan_hz_samples.get(topic_name)
            if not last_time or not samples or now - last_time > 1.5:
                hz_text = "--"
            else:
                avg_interval = sum(samples) / len(samples)
                hz_text = f"{(1.0 / avg_interval):.1f}"
            parts.append(f"{topic_name}={hz_text}Hz")
        return " / ".join(parts)

    def _format_footprint(self):
        return " ".join(f"({x:+.2f},{y:+.2f})" for x, y in self.robot_footprint)

    def _format_landmarks(self):
        names = self.landmark_status.get("names", [])
        count = self.landmark_status.get("count", len(names))
        map_name = self.landmark_status.get("map", "unknown")
        if not names:
            return f"map={map_name} / 0 loaded"
        joined = ", ".join(names[:6])
        if len(names) > 6:
            joined += f", ... +{len(names) - 6}"
        return f"map={map_name} / {count} loaded: {joined}"

    def _format_memory(self):
        landmarks = self.landmark_status.get("names", [])
        active = self.active_command.get("name", "idle") if isinstance(self.active_command, dict) else "idle"
        queue_len = len(self.command_queue)
        history_len = len(self.command_history)
        goal_text = "none" if self.active_goal is None else "active"
        landmark_text = "none" if not landmarks else f"{len(landmarks)} landmarks"
        return f"goal={goal_text} / active={active} / queue={queue_len} / history={history_len} / landmarks={landmark_text}"

    def _direction_from_angle(self, angle_deg):
        if -20.0 <= angle_deg <= 20.0:
            return "front"
        if 20.0 < angle_deg <= 80.0:
            return "left"
        if -80.0 <= angle_deg < -20.0:
            return "right"
        if angle_deg > 135.0 or angle_deg < -135.0:
            return "back"
        return None

    def _footprint_radius_at(self, angle_rad):
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        best = None
        points = self.robot_footprint

        for idx, (x1, y1) in enumerate(points):
            x2, y2 = points[(idx + 1) % len(points)]
            edge_x = x2 - x1
            edge_y = y2 - y1
            denom = dx * edge_y - dy * edge_x
            if abs(denom) < 1.0e-9:
                continue

            ray_t = (x1 * edge_y - y1 * edge_x) / denom
            edge_t = (x1 * dy - y1 * dx) / denom
            if ray_t >= 0.0 and 0.0 <= edge_t <= 1.0:
                if best is None or ray_t < best:
                    best = ray_t

        return best if best is not None else 0.0

    def _merge_scan_obstacle_distances(self, now):
        merged = {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0}
        merged_points = []
        stale_keys = []

        for key, (stamp, distances) in self._scan_obstacle_distances.items():
            if now - stamp > 0.7:
                stale_keys.append(key)
                continue
            for direction, distance in distances.items():
                if distance < merged[direction]:
                    merged[direction] = distance

        for key in stale_keys:
            self._scan_obstacle_distances.pop(key, None)
            self._scan_obstacle_points.pop(key, None)

        for stamp, points in self._scan_obstacle_points.values():
            if now - stamp <= 0.7:
                merged_points.extend(points)

        self.obstacle_distances = merged
        self.obstacle_points = merged_points

    def get_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', self.robot_base_frame, rclpy.time.Time())
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
            "scan_hz": self._format_scan_hz(now),
            "footprint": self._format_footprint(),
            "landmarks": self._format_landmarks(),
            "memory": self._format_memory(),
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
        self.obstacle_points = []
        self.footprint = []
        self.setMinimumSize(220, 220)

    def set_distances(self, distances):
        self.obstacle_distances = distances
        self.update()

    def set_points(self, points):
        self.obstacle_points = points
        self.update()

    def set_footprint(self, footprint):
        self.footprint = footprint
        self.update()

    def _footprint_radius_at(self, angle_rad):
        if not self.footprint:
            return 0.0

        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        best = None

        for idx, (x1, y1) in enumerate(self.footprint):
            x2, y2 = self.footprint[(idx + 1) % len(self.footprint)]
            edge_x = x2 - x1
            edge_y = y2 - y1
            denom = dx * edge_y - dy * edge_x
            if abs(denom) < 1.0e-9:
                continue

            ray_t = (x1 * edge_y - y1 * edge_x) / denom
            edge_t = (x1 * dy - y1 * dx) / denom
            if ray_t >= 0.0 and 0.0 <= edge_t <= 1.0:
                if best is None or ray_t < best:
                    best = ray_t

        return best if best is not None else 0.0

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

        max_range = SiriusStatusMonitor.RADAR_MAX_RANGE
        scale = radius / max_range
        
        # Dark circular background
        painter.setPen(QPen(QColor(40, 44, 52), 1.2, Qt.SolidLine))
        painter.setBrush(QBrush(QColor(18, 18, 22)))
        painter.drawEllipse(QPointF(center_x, center_y), radius, radius)
        
        # Inner circular range grids
        painter.setPen(QPen(QColor(40, 44, 52), 0.8, Qt.SolidLine))
        painter.drawEllipse(QPointF(center_x, center_y), radius * 1.0 / 2.0, radius * 1.0 / 2.0)
        painter.drawEllipse(QPointF(center_x, center_y), radius * 1.0 / 4.0, radius * 1.0 / 4.0)
        
        # Draw labels for ranges
        painter.setPen(QPen(QColor(110, 115, 130), 1, Qt.SolidLine))
        painter.setFont(QFont("Arial", 8))
        painter.drawText(int(center_x + 5), int(center_y - radius * 1.0 / 4.0 + 4), "0.5m")
        painter.drawText(int(center_x + 5), int(center_y - radius * 1.0 / 2.0 + 4), "1.0m")
        painter.drawText(int(center_x + 5), int(center_y - radius + 12), "2.0m")

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
            "front": (45, 90, 0.0),
            "left": (135, 90, math.pi / 2.0),
            "back": (225, 90, math.pi),
            "right": (315, 90, -math.pi / 2.0)
        }
        
        for direction, (start_angle, span_angle, robot_angle) in sectors.items():
            clearance = self.obstacle_distances.get(direction, 999.0)
            footprint_radius = self._footprint_radius_at(robot_angle)
            center_range = clearance + footprint_radius
            if center_range < max_range:
                # Color code
                if clearance < 0.8:
                    color = QColor(235, 47, 6, 200)   # Vivid Red
                elif clearance < 1.5:
                    color = QColor(241, 196, 15, 180) # Orange-Yellow
                else:
                    color = QColor(46, 204, 113, 160)  # Green
                
                painter.setPen(QPen(color, 3.5, Qt.SolidLine))
                r_dist = radius * (center_range / max_range)
                if r_dist < 10:
                    r_dist = 10
                
                rect_x = center_x - r_dist
                rect_y = center_y - r_dist
                painter.drawArc(int(rect_x), int(rect_y), int(r_dist * 2), int(r_dist * 2), int(start_angle * 16), int(span_angle * 16))

        if self.obstacle_points:
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(QColor(241, 196, 15, 210)))
            for x, y in self.obstacle_points:
                px = center_x - y * scale
                py = center_y - x * scale
                painter.drawEllipse(QPointF(px, py), 2.0, 2.0)

        # Center dot (robot)
        if self.footprint:
            polygon = QPolygonF()
            for x, y in self.footprint:
                polygon.append(QPointF(center_x - y * scale, center_y - x * scale))
            painter.setBrush(QBrush(QColor(0, 168, 255, 45)))
            painter.setPen(QPen(QColor(0, 168, 255, 180), 1.5, Qt.SolidLine))
            painter.drawPolygon(polygon)

        painter.setBrush(QBrush(QColor(0, 168, 255)))
        painter.setPen(QPen(QColor(255, 255, 255), 1.5))
        painter.drawEllipse(QPointF(center_x, center_y), 7, 7)


class StatusMonitorWidget(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        # PyQt5 ウィンドウの基本設定
        self.setWindowTitle("Sirius Telemetry Status Monitor")
        self.resize(930, 680)

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
            ("LiDAR Hz", "scan_hz"),
            ("Footprint", "footprint"),
            ("Landmarks", "landmarks"),
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
        
        # Command Queue Visualizer Section
        queue_sec = QFrame()
        queue_sec.setStyleSheet("background-color: #1a1a1c; border-radius: 6px; padding: 10px; margin-top: 10px;")
        queue_sec_layout = QVBoxLayout(queue_sec)
        
        queue_title = QLabel("📋 Execution Command Queue / History")
        queue_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #fbc531;")
        queue_sec_layout.addWidget(queue_title)
        
        self.active_cmd_lbl = QLabel("Active: Idle")
        self.active_cmd_lbl.setStyleSheet("""
            background-color: #272822;
            color: #75715e;
            font-weight: bold;
            font-size: 12px;
            padding: 6px;
            border-radius: 4px;
            border: 1px solid #3e3d32;
        """)
        queue_sec_layout.addWidget(self.active_cmd_lbl)
        
        queue_scroll = QScrollArea()
        queue_scroll.setFixedHeight(120)
        queue_scroll.setWidgetResizable(True)
        
        self.queue_list_widget = QWidget()
        self.queue_list_layout = QVBoxLayout(self.queue_list_widget)
        self.queue_list_layout.setContentsMargins(0, 0, 0, 0)
        self.queue_list_layout.setSpacing(4)
        self.queue_list_layout.addStretch() # bottom anchor
        
        queue_scroll.setWidget(self.queue_list_widget)
        queue_sec_layout.addWidget(queue_scroll)
        
        left_layout.addWidget(queue_sec)

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
        self.radar_widget.set_footprint(self.node.robot_footprint)
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
        self.radar_widget.set_points(self.node.obstacle_points)

        # コマンドキュー/履歴の表示更新
        active = self.node.active_command
        if active:
            self.active_cmd_lbl.setText(f"Active: {active.get('name', 'Executing')} [{active.get('time', '')}]")
            self.active_cmd_lbl.setStyleSheet("""
                background-color: #1e272c;
                color: #2ecc71;
                font-weight: bold;
                font-size: 12px;
                padding: 6px;
                border-radius: 4px;
                border: 1px solid #2ecc71;
            """)
        else:
            self.active_cmd_lbl.setText("Active: Idle")
            self.active_cmd_lbl.setStyleSheet("""
                background-color: #272822;
                color: #75715e;
                font-weight: bold;
                font-size: 12px;
                padding: 6px;
                border-radius: 4px;
                border: 1px solid #3e3d32;
            """)

        # Clear queue scroll layout children (except the stretch at bottom)
        while self.queue_list_layout.count() > 1:
            child = self.queue_list_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

        # Add upcoming queue items
        for cmd in self.node.command_queue:
            lbl = QLabel(f"  ➜ {cmd.get('name', 'Command')}")
            lbl.setStyleSheet("color: #00d2ff; font-size: 11px; font-weight: bold;")
            self.queue_list_layout.insertWidget(self.queue_list_layout.count() - 1, lbl)

        # Add command history items (most recent first)
        for cmd in reversed(self.node.command_history):
            status = cmd.get("status", "unknown")
            time_str = cmd.get("time", "")
            name_str = cmd.get("name", "Command")
            
            if status == "completed":
                lbl = QLabel(f"  ✓ {name_str} ({time_str})")
                lbl.setStyleSheet("color: #2ecc71; font-size: 11px;")
            elif status in ["cancelled", "cleared"]:
                lbl = QLabel(f"  ✗ {name_str} ({time_str})")
                lbl.setStyleSheet("color: #7f8c8d; font-size: 11px; text-decoration: line-through;")
            elif status == "failed_stuck":
                lbl = QLabel(f"  ⚠️ {name_str} ({time_str}) - Stuck/Blocked")
                lbl.setStyleSheet("color: #e74c3c; font-size: 11px; font-weight: bold;")
            else:
                lbl = QLabel(f"  • {name_str} ({time_str}) - {status}")
                lbl.setStyleSheet("color: #95a5a6; font-size: 11px;")
            
            self.queue_list_layout.insertWidget(self.queue_list_layout.count() - 1, lbl)

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
            f"LiDAR Hz  : {data['scan_hz']}",
            f"Footprint : {data['footprint']}",
            f"Landmarks : {data['landmarks']}",
            f"Memory    : {data['memory']}",
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
