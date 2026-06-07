#!/usr/bin/env python3
import math
import time
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

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
        self.create_timer(1.0, self.refresh_view)

        print("=== Sirius Status Monitor ===")
        print("Ctrl+Cで終了できます")

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

    def refresh_view(self):
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
            goal_str = f"X={gx:.2f}, Y={gy:.2f}, Yaw={gyaw:+.1f}deg"
            self.goal_distance = math.sqrt((gx - x) ** 2 + (gy - y) ** 2)
            dist_str = f"{self.goal_distance:.2f}m"
        else:
            self.goal_distance = None

        status = "STUCK" if self.is_stuck else "OK"
        can_move = self._assess_moveability()
        self._render(
            x=x, y=y, yaw_deg=yaw_deg,
            goal_str=goal_str, dist_str=dist_str,
            status=status,
            can_move=can_move
        )

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

    def _render(self, x, y, yaw_deg, goal_str, dist_str, status, can_move):
        lines = [
            "=== Sirius Status Monitor ===",
            f"Time      : {time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"Pose      : X={x:.2f}, Y={y:.2f}, Yaw={yaw_deg:+.1f}deg",
            f"Goal      : {goal_str}",
            f"Distance  : {dist_str}",
            f"Status    : {status}",
            f"Can Move  : {can_move}",
            f"Action    : {self.last_nav_control}",
            f"Last Act  : type={self.last_action_type} status={self.last_action_status}",
            f"Velocity  : linear={self.current_vel_x:.2f}m/s angular={self.current_vel_theta:.2f}rad/s",
            f"People    : {self.surrounding_people_count}",
            f"Face      : {'on' if self.face_active else 'off'} / {self.current_expression}",
            f"Battery   : {self._battery_cache_text}",
            "Knowledge  :",
        ]
        for item in self.KNOWLEDGE_ITEMS:
            lines.append(f"  - {item}")
        lines.append("Capabilities:")
        for item in self.MOVE_CAPABILITIES:
            lines.append(f"  - {item}")
        print("\n".join(lines))
        print("-" * 48)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SiriusStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
