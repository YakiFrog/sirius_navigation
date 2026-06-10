# -*- coding: utf-8 -*-
import json
import math
import time
import threading
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

try:
    from ..local_parser import DIALOGUE_TEMPLATES
except ImportError:
    from local_parser import DIALOGUE_TEMPLATES

class TeleopHandler:
    def __init__(self, node):
        self.node = node

    def handle_manual_teleop_instruction(self, instruction: str) -> bool:
        """Remote Controller のタッチ操縦JSONをTwistへ変換する。"""
        stripped = instruction.strip()
        if not stripped.startswith("{"):
            return False

        try:
            payload = json.loads(stripped)
        except json.JSONDecodeError:
            return False

        if payload.get("type") != "manual_teleop":
            return False

        assisted_value = payload.get("assisted", True)
        if isinstance(assisted_value, str):
            assisted = assisted_value.lower() not in ["0", "false", "no", "off"]
        else:
            assisted = bool(assisted_value)
        linear = max(-0.35, min(0.35, float(payload.get("linear", 0.0) or 0.0)))
        angular = max(-1.2, min(1.2, float(payload.get("angular", 0.0) or 0.0)))

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        self.node.publish_nav_control("pause_silent")
        teleop_subscribers = self.node.cmd_vel_teleop_pub.get_subscription_count()
        route = "cmd_vel_direct"
        if assisted:
            self.node.nav_ctrl.ensure_assisted_teleop_goal()
            self.node.cmd_vel_teleop_pub.publish(twist)
            route = "cmd_vel_teleop"
            if teleop_subscribers == 0:
                route = "cmd_vel_teleop unavailable"
                self.node.get_logger().warning(
                    "[AssistedTeleop] /cmd_vel_teleop has no subscribers. "
                    "Not falling back to direct velocity because obstacle avoidance would be bypassed."
                )
        else:
            self.node.cmd_vel_direct_pub.publish(twist)

        if abs(linear) < 0.001 and abs(angular) < 0.001:
            mode_label = "AssistedTeleop" if assisted else "DirectTeleop"
            self.node.get_logger().info(f"[{mode_label}] stop route={route} teleop_subscribers={teleop_subscribers}")
        else:
            self.node.get_logger().debug(
                f"[{'AssistedTeleop' if assisted else 'DirectTeleop'}] "
                f"linear={linear:.2f} angular={angular:.2f} "
                f"route={route} teleop_subscribers={teleop_subscribers}"
            )
        return True

    def publish_assisted_drive_twist(self, twist: Twist):
        """前後移動の速度指令を publish する。"""
        self.node.cmd_vel_teleop_pub.publish(twist)
        self.node.cmd_vel_direct_pub.publish(twist)

        teleop_subscribers = self.node.cmd_vel_teleop_pub.get_subscription_count()
        direct_subscribers = self.node.cmd_vel_direct_pub.get_subscription_count()
        now = self.node.get_clock().now().nanoseconds / 1e9
        with self.node.lock:
            should_log = now - self.node.assisted_drive_last_route_log_time > 1.0
            if should_log:
                self.node.assisted_drive_last_route_log_time = now

        if should_log:
            if direct_subscribers > 0:
                self.node.get_logger().info(
                    f"[AssistDrive] publishing to cmd_vel_direct "
                    f"(direct_subscribers={direct_subscribers}, "
                    f"teleop_subscribers={teleop_subscribers}, linear.x={twist.linear.x:.2f})"
                )
            elif teleop_subscribers == 0:
                self.node.get_logger().warning(
                    "[AssistDrive] no subscribers on cmd_vel_direct or cmd_vel_teleop. "
                    "Velocity command may not reach the robot."
                )
            else:
                self.node.get_logger().info(
                    f"[AssistDrive] publishing to cmd_vel_teleop "
                    f"(subscribers={teleop_subscribers}, linear.x={twist.linear.x:.2f})"
                )

    def get_assisted_drive_position(self):
        """前後移動の距離測定用位置を取得する。/odom を優先する。"""
        with self.node.lock:
            if self.node.has_odom_pose:
                return self.node.current_odom_x, self.node.current_odom_y, "odom"

        try:
            trans = self.node.tf_buffer.lookup_transform(
                'map',
                'sirius3/base_footprint',
                rclpy.time.Time()
            )
            return trans.transform.translation.x, trans.transform.translation.y, "map_tf"
        except Exception:
            return None, None, "unavailable"

    def start_assisted_drive(self, direction: float, distance: float, should_speak: bool = True):
        """前後移動を assisted teleop 系の速度指示に切り替える。"""
        self.node.nav_ctrl.ensure_assisted_teleop_goal()

        try:
            if isinstance(distance, dict):
                distance = distance.get("value", 1.0)
            distance = float(distance)
        except (TypeError, ValueError, AttributeError):
            distance = 1.0

        distance = max(abs(distance), 0.3)
        direction = 1.0 if direction >= 0.0 else -1.0
        speed = min(max(distance * 0.35, 0.12), 0.22)
        duration = distance / speed
        start_x, start_y, distance_source = self.get_assisted_drive_position()
        now = self.node.get_clock().now().nanoseconds / 1e9
        preempt_delay = 1.1

        self.node.publish_nav_control("pause_silent")
        stop_msg = Bool()
        stop_msg.data = True
        self.node.stop_pub.publish(stop_msg)

        def reset_manual_drive_stop():
            msg = Bool()
            msg.data = False
            self.node.stop_pub.publish(msg)
        threading.Timer(preempt_delay, reset_manual_drive_stop).start()

        with self.node.lock:
            self.node.assisted_drive_active = True
            self.node.assisted_drive_direction = direction
            self.node.assisted_drive_start_time = now + preempt_delay
            self.node.assisted_drive_end_time = now + preempt_delay + duration + 3.0
            self.node.assisted_drive_speed = speed
            self.node.assisted_drive_distance = distance
            self.node.assisted_drive_start_x = start_x
            self.node.assisted_drive_start_y = start_y
            self.node.assisted_drive_distance_source = distance_source
            self.node.assisted_drive_blocked_reported = False
            self.node.assisted_drive_last_route_log_time = 0.0
            self.node.assisted_drive_last_commanded_x = 0.0

        if should_speak:
            if direction > 0.0:
                self.node.send_sirius_speak(DIALOGUE_TEMPLATES["forward_start"].format(distance=distance))
            else:
                self.node.send_sirius_speak(DIALOGUE_TEMPLATES["backward_start"].format(distance=distance))

        self.node.get_logger().info(
            f"[AssistDrive] direction={'forward' if direction > 0.0 else 'backward'}, "
            f"target_distance={distance:.2f}m speed={speed:.2f}m/s "
            f"preempt_delay={preempt_delay:.1f}s timeout={duration + 3.0:.2f}s "
            f"distance_source={distance_source}"
        )

    def stop_assisted_drive(self):
        should_publish_stop = False
        with self.node.lock:
            should_publish_stop = self.node.assisted_drive_active
            self.node.assisted_drive_active = False
            self.node.assisted_drive_direction = 0.0
            self.node.assisted_drive_end_time = None
            self.node.assisted_drive_start_time = 0.0
            self.node.assisted_drive_speed = 0.0
            self.node.assisted_drive_distance = 0.0
            self.node.assisted_drive_start_x = None
            self.node.assisted_drive_start_y = None
            self.node.assisted_drive_distance_source = "unknown"
            self.node.assisted_drive_blocked_reported = False
            self.node.assisted_drive_last_commanded_x = 0.0

        if should_publish_stop:
            self.publish_assisted_drive_twist(Twist())

    def timer_assisted_drive_publisher(self):
        with self.node.lock:
            active = self.node.assisted_drive_active
            end_time = self.node.assisted_drive_end_time
            start_time = self.node.assisted_drive_start_time
            direction = self.node.assisted_drive_direction
            speed = self.node.assisted_drive_speed
            target_distance = self.node.assisted_drive_distance
            start_x = self.node.assisted_drive_start_x
            start_y = self.node.assisted_drive_start_y
            distance_source = self.node.assisted_drive_distance_source
            blocked_reported = self.node.assisted_drive_blocked_reported
            last_commanded_x = self.node.assisted_drive_last_commanded_x
            last_type = self.node.last_active_cmd_type
            obs_dists = dict(getattr(self.node, 'obstacle_distances', {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0}))

        if not active:
            return

        now = self.node.get_clock().now().nanoseconds / 1e9
        if start_time and now < start_time:
            return

        if start_x is None or start_y is None:
            current_x, current_y, current_source = self.get_assisted_drive_position()
            if current_x is not None and current_y is not None:
                with self.node.lock:
                    self.node.assisted_drive_start_x = current_x
                    self.node.assisted_drive_start_y = current_y
                    self.node.assisted_drive_distance_source = current_source
                start_x = current_x
                start_y = current_y
                distance_source = current_source

        nearest_front = obs_dists.get("front", 999.0)
        nearest_back = obs_dists.get("back", 999.0)
        blocked_dist = nearest_back if direction < 0.0 else nearest_front
        blocked_side = "後方" if direction < 0.0 else "前方"
        hard_stop_dist = 0.34
        slow_start_dist = 0.85 if direction < 0.0 else 0.70
        min_creep_speed = 0.055
        target_speed = speed
        if blocked_dist < slow_start_dist:
            denom = max(slow_start_dist - hard_stop_dist, 0.01)
            scale = max(0.0, min(1.0, (blocked_dist - hard_stop_dist) / denom))
            target_speed = max(min_creep_speed, speed * scale)

        if blocked_dist <= hard_stop_dist:
            if not blocked_reported:
                with self.node.lock:
                    self.node.assisted_drive_blocked_reported = True
                    self.node.is_stuck = True
                    self.node.last_action_status = "failed_stuck"
                self.node.send_sirius_speak(f"[sad]{blocked_side} {blocked_dist:.1f}メートルに障害物があって、これ以上動けないのだ！")
                self.node.get_logger().warning(
                    f"[前後移動停止] 障害物で停止: 側={blocked_side}, 距離={blocked_dist:.2f}m, last_type={last_type}"
                )
            self.stop_assisted_drive()
            return

        current_x, current_y, current_source = self.get_assisted_drive_position()
        if start_x is not None and start_y is not None and current_x is not None and current_y is not None:
            traveled = math.hypot(current_x - start_x, current_y - start_y)
            if traveled >= target_distance:
                self.node.get_logger().info(
                    f"[AssistDrive] target reached by odometry feedback: "
                    f"traveled={traveled:.2f}m target={target_distance:.2f}m source={distance_source}"
                )
                self.node.send_sirius_speak("[happy]ここまで進んだのだ！")
                self.stop_assisted_drive()
                return
        elif current_source == "unavailable":
            self.node.get_logger().warning("[AssistDrive] odometry/TF position unavailable; distance feedback is paused.")

        if end_time is not None and now >= end_time:
            self.node.get_logger().warning(
                f"[前後移動停止] 距離目標前にタイムアウト: target={target_distance:.2f}m source={distance_source}"
            )
            self.node.send_sirius_speak("[sad]距離を確認できないので、いったん止まるのだ。")
            self.stop_assisted_drive()
            return

        twist = Twist()
        target_x = target_speed * direction
        max_delta_per_tick = 0.025
        delta = target_x - last_commanded_x
        if abs(delta) > max_delta_per_tick:
            target_x = last_commanded_x + math.copysign(max_delta_per_tick, delta)

        with self.node.lock:
            self.node.assisted_drive_last_commanded_x = target_x

        twist.linear.x = target_x
        self.publish_assisted_drive_twist(twist)
