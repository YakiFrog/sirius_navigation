#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import json
import time
import re
import urllib.request
import urllib.error
import threading
import unicodedata
import os
import yaml

os.environ.setdefault("RCUTILS_COLORIZED_OUTPUT", "1")
os.environ.setdefault("RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{time}] [{name}]: {message}")

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool, String
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import Spin, AssistedTeleop
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from visualization_msgs.msg import Marker, MarkerArray

_COLOR_ENABLED = os.environ.get("NO_COLOR", "").lower() not in ["1", "true", "yes"]
_RESET = "\033[0m" if _COLOR_ENABLED else ""
_CYAN = "\033[36m" if _COLOR_ENABLED else ""
_GREEN = "\033[32m" if _COLOR_ENABLED else ""
_YELLOW = "\033[33m" if _COLOR_ENABLED else ""
_MAGENTA = "\033[35m" if _COLOR_ENABLED else ""


def color_text(text, color):
    return f"{color}{text}{_RESET}" if color else text


def print_command_prompt():
    print(color_text("Command > ", _CYAN), end="", flush=True)


try:
    from .face_client import FaceClient
    from .local_parser import parse_local_rules, DIALOGUE_TEMPLATES, normalize_instruction_text, style_sirius_speak, DEFAULT_HUMOR_LEVEL, clamp_humor_level
except ImportError:
    from face_client import FaceClient
    from local_parser import parse_local_rules, DIALOGUE_TEMPLATES, normalize_instruction_text, style_sirius_speak, DEFAULT_HUMOR_LEVEL, clamp_humor_level

try:
    from .modules.llm_client import LlmClient
    from .modules.landmark_manager import LandmarkManager
    from .modules.nav_controller import NavController
    from .modules.teleop_handler import TeleopHandler
    from .modules.command_executor import CommandExecutor
    from .modules.http_server import HttpInstructionServer
except ImportError:
    from modules.llm_client import LlmClient
    from modules.landmark_manager import LandmarkManager
    from modules.nav_controller import NavController
    from modules.teleop_handler import TeleopHandler
    from modules.command_executor import CommandExecutor
    from modules.http_server import HttpInstructionServer


class LlmDynamicGoal(Node):
    def __init__(self):
        super().__init__('llm_dynamic_goal')
        
        # パラメータ設定
        self.declare_parameter('lm_studio_url', 'http://localhost:1234/v1/chat/completions')
        self.declare_parameter('model_name', 'meta-llama-3-8b-instruct')
        self.declare_parameter('current_map_state_file', os.path.expanduser('~/.sirius_nav2_current_map.yaml'))
        
        self.lm_studio_url = self.get_parameter('lm_studio_url').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.current_map_state_file = self.get_parameter('current_map_state_file').get_parameter_value().string_value
        
        # TFのリスナーを初期化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # パブリッシャーの定義
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/dynamic_goal_marker', 10)
        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        self.landmark_marker_pub = self.create_publisher(MarkerArray, '/sirius/landmark_markers', marker_qos)
        self.landmark_status_pub = self.create_publisher(String, '/sirius/landmark_status', marker_qos)
        self.stop_pub = self.create_publisher(Bool, 'stop', 10)
        self.nav_control_pub = self.create_publisher(String, '/nav_control', 10)
        self.cmd_vel_teleop_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.cmd_vel_direct_pub = self.create_publisher(Twist, 'cmd_vel_direct', 10)
        self.stuck_pub = self.create_publisher(Bool, '/sirius_is_stuck', 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10)
        self.last_costmap_time = 0.0
        self.obstacle_distances = {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0}
        
        # オドメトリサブスクライバー（リアルタイム速度取得用）
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.current_vel_x = 0.0
        self.current_vel_theta = 0.0
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.has_odom_pose = False
        
        # アクションキャンセルのためのサービス
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.assisted_teleop_client = ActionClient(self, AssistedTeleop, 'assisted_teleop')
        self.assisted_teleop_goal_handle = None
        self.assisted_teleop_goal_pending = False
        self.param_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        
        # 自然言語指示トピックのサブスクライバー
        self.instruction_sub = self.create_subscription(
            String,
            '/llm_instruction',
            self.instruction_callback,
            10
        )
        self.battery_status_sub = self.create_subscription(
            String,
            '/sirius/battery_status',
            self.battery_status_callback,
            10
        )
        
        self.face_client = FaceClient(logger=self.get_logger())
        self.latest_battery_status = None
        self.latest_battery_status_time = 0.0
        self.surrounding_people_count = 0
        self.current_expression = "normal"
        self.current_humor_level = DEFAULT_HUMOR_LEVEL
        self.target_follow_status_last_spoken = {}
        self.last_spoken_text = ""
        self.marker_array_sub = self.create_subscription(
            MarkerArray,
            '/target_detector/target_markers',
            self.marker_array_callback,
            10
        )
        self.target_follow_status_sub = self.create_subscription(
            String,
            '/target_follower/status',
            self.target_follow_status_callback,
            10
        )

        self.lock = threading.Lock()
        
        # ゴール到達確認およびシーケンスキューの状態管理
        self.active_goal_x = None
        self.active_goal_y = None
        self.active_goal_yaw = None
        self.goal_reached_logged = True
        self.paused_goal_snapshot = None
        
        self.command_queue = []
        self.executing_command = False
        self.current_xy_tolerance = 0.50
        self.current_speed_setting = 0.90
        self.suppress_step_speech = False
        
        # Turn (旋回) コマンドのアーリーキャンセル用ターゲットyaw
        self.turn_target_yaw = None
        self.turn_remaining_angle = None
        self.last_yaw_robot = 0.0
        self.turn_arrival_triggered = False
        self.turn_goal_distance = 0.7
        self.turn_start_x = 0.0
        self.turn_start_y = 0.0
        self.turn_total_angle = 1.0
        
        # スタック検知用状態管理
        self.distance_remaining_history = []
        self.yaw_diff_history = []
        self.is_stuck = False
        self.command_start_time = None
        self.spin_goal_handle = None
        self.last_active_cmd_type = None
        
        # 直前アクションの実行結果パラメータ
        self.last_action_status = "none"
        self.last_action_type = "none"
        self.last_target_value = 0.0
        self.last_final_distance_error = 0.0
        self.last_final_yaw_error = 0.0
        self.last_action_start_x = 0.0
        self.last_action_start_y = 0.0
        self.last_action_start_yaw = 0.0
        
        self.assisted_drive_active = False
        self.assisted_drive_direction = 0.0
        self.assisted_drive_end_time = None
        self.assisted_drive_start_time = 0.0
        self.assisted_drive_speed = 0.0
        self.assisted_drive_distance = 0.0
        self.assisted_drive_start_x = None
        self.assisted_drive_start_y = None
        self.assisted_drive_distance_source = "unknown"
        self.assisted_drive_blocked_reported = False
        self.assisted_drive_last_route_log_time = 0.0
        self.assisted_drive_last_commanded_x = 0.0

        # モジュール類のインスタンス化
        self.llm_client = LlmClient(self)
        self.landmark_mgr = LandmarkManager(self)
        self.nav_ctrl = NavController(self)
        self.teleop_ctrl = TeleopHandler(self)
        self.cmd_executor = CommandExecutor(self)
        self.http_server = HttpInstructionServer(self)

        self.landmark_mgr.load_landmarks_for_current_map(force=True)

        self.get_logger().info(f'LLM Dynamic Goal Node initialized with State-Feedback loop.')
        self.get_logger().info(f'LM Studio URL: {self.lm_studio_url}')

        # タイマー登録
        self.assisted_drive_timer = self.create_timer(0.1, self.timer_assisted_drive_publisher)
        self.goal_monitor_timer = self.create_timer(1.0, self.monitor_goal_distance)
        self.dynamic_goal_timer = self.create_timer(0.1, self.timer_goal_publisher)
        self.landmark_reload_timer = self.create_timer(2.0, self.landmark_reload_timer_callback)
        self.landmark_marker_timer = self.create_timer(2.0, self.landmark_marker_timer_callback)

        # 対話型コマンドライン入力を別スレッドで開始
        self.running = True
        self.input_thread = threading.Thread(target=self.interactive_input_loop, daemon=True)
        self.input_thread.start()

        # HTTPサーバーを別スレッドで開始して外部からの指示を受け付ける
        self.http_server.start()

    # --- LandmarkManager Properties ---
    @property
    def landmarks(self):
        return self.landmark_mgr.landmarks

    @landmarks.setter
    def landmarks(self, value):
        self.landmark_mgr.landmarks = value

    @property
    def landmark_aliases(self):
        return self.landmark_mgr.landmark_aliases

    @landmark_aliases.setter
    def landmark_aliases(self, value):
        self.landmark_mgr.landmark_aliases = value

    @property
    def landmark_file_path(self):
        return self.landmark_mgr.landmark_file_path

    @landmark_file_path.setter
    def landmark_file_path(self, value):
        self.landmark_mgr.landmark_file_path = value

    @property
    def landmark_file_mtime(self):
        return self.landmark_mgr.landmark_file_mtime

    @landmark_file_mtime.setter
    def landmark_file_mtime(self, value):
        self.landmark_mgr.landmark_file_mtime = value

    @property
    def current_landmark_map_name(self):
        return self.landmark_mgr.current_landmark_map_name

    @current_landmark_map_name.setter
    def current_landmark_map_name(self, value):
        self.landmark_mgr.current_landmark_map_name = value

    @property
    def last_landmark_marker_count(self):
        return self.landmark_mgr.last_landmark_marker_count

    @last_landmark_marker_count.setter
    def last_landmark_marker_count(self, value):
        self.landmark_mgr.last_landmark_marker_count = value

    # --- LlmClient Properties ---
    @property
    def chat_history(self):
        return self.llm_client.chat_history

    @chat_history.setter
    def chat_history(self, value):
        self.llm_client.chat_history = value

    @property
    def history_max_turns(self):
        return self.llm_client.history_max_turns

    @history_max_turns.setter
    def history_max_turns(self, value):
        self.llm_client.history_max_turns = value

    # --- Wrapper/Delegator Methods for Compatibility ---
    def query_lm_studio(self, instruction):
        return self.llm_client.query_lm_studio(instruction)

    def load_landmarks_for_current_map(self, force=False):
        return self.landmark_mgr.load_landmarks_for_current_map(force)

    def get_landmark_names(self):
        return self.landmark_mgr.get_landmark_names()

    def publish_landmark_status(self):
        return self.landmark_mgr.publish_landmark_status()

    def handle_landmark_list_question(self, instruction):
        return self.landmark_mgr.handle_landmark_list_question(instruction)

    def find_landmark_in_instruction(self, instruction):
        return self.landmark_mgr.find_landmark_in_instruction(instruction)

    def handle_landmark_navigation_instruction(self, instruction):
        return self.landmark_mgr.handle_landmark_navigation_instruction(instruction)

    def cancel_navigation(self, clear_queue=True, preserve_current_goal=False):
        return self.nav_ctrl.cancel_navigation(clear_queue, preserve_current_goal)

    def publish_goal_pose(self, tx, ty, yaw_robot, r, theta):
        return self.nav_ctrl.publish_goal_pose(tx, ty, yaw_robot, r, theta)

    def publish_direct_map_goal(self, target_x, target_y, target_yaw):
        return self.nav_ctrl.publish_direct_map_goal(target_x, target_y, target_yaw)

    def publish_marker(self, pose_msg):
        return self.nav_ctrl.publish_marker(pose_msg)

    def delete_marker(self):
        return self.nav_ctrl.delete_marker()

    def publish_nav_control(self, command: str):
        return self.nav_ctrl.publish_nav_control(command)

    def ensure_assisted_teleop_goal(self):
        return self.nav_ctrl.ensure_assisted_teleop_goal()

    def set_controller_speed(self, speed_setting):
        return self.nav_ctrl.set_controller_speed(speed_setting)

    def set_node_parameters(self, node_name, params_dict):
        return self.nav_ctrl.set_node_parameters(node_name, params_dict)

    def send_spin_goal(self, relative_yaw):
        return self.nav_ctrl.send_spin_goal(relative_yaw)

    def spin_goal_response_callback(self, future):
        return self.nav_ctrl.spin_goal_response_callback(future)

    def spin_goal_result_callback(self, future):
        return self.nav_ctrl.spin_goal_result_callback(future)

    def handle_manual_teleop_instruction(self, instruction: str) -> bool:
        return self.teleop_ctrl.handle_manual_teleop_instruction(instruction)

    def publish_assisted_drive_twist(self, twist):
        return self.teleop_ctrl.publish_assisted_drive_twist(twist)

    def get_assisted_drive_position(self):
        return self.teleop_ctrl.get_assisted_drive_position()

    def start_assisted_drive(self, direction, distance, should_speak=True):
        return self.teleop_ctrl.start_assisted_drive(direction, distance, should_speak)

    def stop_assisted_drive(self):
        return self.teleop_ctrl.stop_assisted_drive()

    def execute_next_command(self):
        return self.cmd_executor.execute_next_command()

    def resume_navigation(self):
        return self.nav_ctrl.resume_navigation()

    def register_landmark(self, name):
        return self.landmark_mgr.register_current_pose_as_landmark(name)

    def normalize_landmark_key(self, text):
        return self.landmark_mgr.normalize_landmark_key(text)

    # --- Timer Callbacks delegating to modules ---
    def timer_assisted_drive_publisher(self):
        self.teleop_ctrl.timer_assisted_drive_publisher()

    def landmark_reload_timer_callback(self):
        self.landmark_mgr.load_landmarks_for_current_map()

    def landmark_marker_timer_callback(self):
        self.landmark_mgr.publish_landmark_markers()

    # --- ROS Subscriber and Service Callbacks ---
    def odom_callback(self, msg):
        """オドメトリから現在の速度を取得"""
        with self.lock:
            self.current_vel_x = msg.twist.twist.linear.x
            self.current_vel_theta = msg.twist.twist.angular.z
            self.current_odom_x = msg.pose.pose.position.x
            self.current_odom_y = msg.pose.pose.position.y
            self.has_odom_pose = True

    def costmap_callback(self, msg):
        """local_costmap を解析し、ロボットから見た前後左右の障害物までの最短距離を計算・更新する"""
        now = self.get_clock().now().nanoseconds / 1e9
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
            if cost >= 99:  # Inscribed / Lethal obstacle size
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

        with self.lock:
            self.obstacle_distances = {
                "front": front_dist,
                "left": left_dist,
                "right": right_dist,
                "back": back_dist
            }

    def marker_array_callback(self, msg):
        """周辺の人間トラッキングマーカーから人数をカウントする"""
        tracked_ids = set()
        for m in msg.markers:
            if m.ns == 'tracked_targets' and m.action == Marker.ADD:
                tracked_ids.add(m.id)
        with self.lock:
            self.surrounding_people_count = len(tracked_ids)

    def should_speak_target_follow_status(self, status_key: str, cooldown_sec: float) -> bool:
        now = time.time()
        last_time = self.target_follow_status_last_spoken.get(status_key, 0.0)
        if now - last_time < cooldown_sec:
            return False
        self.target_follow_status_last_spoken[status_key] = now
        return True

    def target_follow_status_callback(self, msg):
        """追従ノードからの状態通知を受けて、人に分かる形で報告する。"""
        if msg.data == "tracking_lost":
            self.get_logger().warning("[TargetFollow] tracking target lost")
            if not self.should_speak_target_follow_status("tracking_lost", 5.0):
                return
            self.send_sirius_speak("[sad]追従対象を見失ったのだ。いったん停止するのだ。")
        elif msg.data == "tracking_recovered":
            self.get_logger().info("[TargetFollow] tracking target recovered")
            if not self.should_speak_target_follow_status("tracking_recovered", 5.0):
                return
            self.send_sirius_speak("[happy]追従対象を見つけたのだ。追従を再開するのだ。")
        elif msg.data == "tracking_waiting":
            self.get_logger().warning("[TargetFollow] no tracking target available yet")
            if not self.should_speak_target_follow_status("tracking_waiting", 10.0):
                return
            self.send_sirius_speak("[sad]今は追従できる人を見つけられていないのだ。僕の前に来てから、もう一度言ってほしいのだ。")
        elif msg.data.startswith("tracking_soft_far"):
            self.get_logger().warning(f"[TargetFollow] tracking target is getting far: {msg.data}")
            if not self.should_speak_target_follow_status("tracking_soft_far", 20.0):
                return
            self.send_sirius_speak("[cry]あんまり僕から離れないでね、追いかけるのちょっと大変なのだ。")
        elif msg.data.startswith("tracking_far"):
            distance_text = ""
            try:
                _, raw_distance = msg.data.split(":", 1)
                distance_text = f"いま約{float(raw_distance):.1f}メートル離れているのだ。"
            except (ValueError, TypeError):
                pass
            self.get_logger().warning(f"[TargetFollow] tracking target is far: {msg.data}")
            if not self.should_speak_target_follow_status("tracking_far", 15.0):
                return
            self.send_sirius_speak(f"[surprised]{distance_text}それ以上離れたら見失うかもしれないのだ。")

    def battery_status_callback(self, msg):
        """BLE Gateway が配信する最新バッテリー情報を、顔gRPC失敗時の予備として保持する"""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if data.get("status") != "connected":
            return

        level = float(data.get("battery_level", -1.0))
        if level < 0.0:
            return

        with self.lock:
            self.latest_battery_status = data
            self.latest_battery_status_time = time.time()

    def instruction_callback(self, msg):
        """トピック経由で指示を受信したときのコールバック"""
        self.get_logger().info(f'Received instruction via topic: "{msg.data}"')
        threading.Thread(target=self.process_instruction, args=(msg.data,), daemon=True).start()

    def interactive_input_loop(self):
        """標準入力からインタラクティブに入力を受け付けるスレッド"""
        print(color_text("\n=== Sirius LLM Dynamic Goal Navigation ===", _MAGENTA))
        print("LM Studioを起動し、モデルがロードされていることを確認してください。")
        print("「ちょっと前に行って」「そこで右向いて」「停止」などの指示を入力してください。")
        print("「3m前に行って、右向いて」といった連続指示も実行可能です。")
        print("終了するには 'exit' または Ctrl+C を入力してください。\n")
        
        while self.running:
            try:
                print_command_prompt()
                line = sys.stdin.readline()
                if not line:
                    break
                instruction = line.strip()
                if not instruction:
                    continue
                if instruction.lower() in ['exit', 'quit']:
                    self.running = False
                    break
                
                threading.Thread(target=self.process_instruction, args=(instruction,), daemon=True).start()
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Input thread error: {e}")
                break

    def process_instruction(self, instruction):
        """指示文を解析し、適切なROS 2アクションを実行する"""
        instruction = normalize_instruction_text(instruction)

        if self.handle_follow_me_instruction(instruction):
            return

        if self.teleop_ctrl.handle_manual_teleop_instruction(instruction):
            return

        if self.landmark_mgr.handle_landmark_list_question(instruction):
            return

        if self.landmark_mgr.handle_landmark_navigation_instruction(instruction):
            return

        if self.landmark_mgr.handle_landmark_registration_instruction(instruction):
            return
        
        # 1. 停止・キャンセル指示の簡易キーワード判定（高速応答のため）
        cancel_keywords = ["キャンセル", "cancel", "中止", "取り消", "とりけし"]
        if any(kw in instruction.lower() for kw in cancel_keywords):
            self.get_logger().warning("Cancel keyword detected. Clearing active goal.")
            self.set_target_following(False, speak_on_failure=False)
            self.nav_ctrl.publish_nav_control("cancel")
            self.nav_ctrl.cancel_navigation(clear_queue=True, preserve_current_goal=False)
            self.send_sirius_speak("[sad]了解、目標を取り消したのだ。")
            return

        stop_keywords = ["止ま", "とまれ", "止め", "とめ", "ストップ", "停止", "stop", "とまって", "待機"]
        if any(kw in instruction.lower() for kw in stop_keywords):
            self.get_logger().warning("Stop keyword detected. Pausing active goal.")
            self.set_target_following(False, speak_on_failure=False)
            self.nav_ctrl.publish_nav_control("pause")
            self.nav_ctrl.cancel_navigation(preserve_current_goal=True)
            self.send_sirius_speak("[surprised]了解、止まるのだ！")
            return

        resume_keywords = ["再開", "再スタート", "続けて", "つづけて", "resume", "reopen", "再び", "動かして", "移動を再開", "続行"]
        if any(kw in instruction.lower() for kw in resume_keywords):
            self.get_logger().info("Resume keyword detected.")
            self.nav_ctrl.publish_nav_control("resume")
            if self.resume_navigation():
                self.send_sirius_speak("[happy]了解、再開するのだ！")
            else:
                self.send_sirius_speak("[happy]了解、再開命令を送ったのだ！")
            return

        # 2. LM Studio にリクエストを投げる
        self.get_logger().info(f"Querying LLM: '{instruction}'...")
        result = self.llm_client.query_lm_studio(instruction)
        
        if result is None:
            self.get_logger().error("Failed to parse command from LLM.")
            self.send_sirius_speak(DIALOGUE_TEMPLATES.get("parse_failure", "[sad]失敗したのだ。"))
            return

        if "commands" not in result:
            if "speed" in result:
                result = {"commands": [{"type": "speed", "value": result.get("speed")}], "cancel": False}
            elif "speak" in result or "response" in result:
                result = {"commands": [], "cancel": False, "speak": result.get("speak", result.get("response", ""))}
            
        is_cancel = result.get("cancel", False)
        if is_cancel:
            self.get_logger().warning("LLM interpreted instruction as STOP/CANCEL.")
            self.set_target_following(False, speak_on_failure=False)
            self.nav_ctrl.publish_nav_control("cancel")
            self.nav_ctrl.cancel_navigation(clear_queue=True, preserve_current_goal=False)
            self.send_sirius_speak("[sad]了解、目標を取り消したのだ。")
            return
            
        commands = result.get("commands", [])
        speak_text = result.get("speak", "")
        if speak_text:
            self.llm_client._append_dialogue_history(instruction, speak_text)
            self.send_sirius_speak(speak_text)
            
        if not commands:
            self.get_logger().warning("No commands generated from LLM.")
            if not result.get("fast_path", False) and not speak_text:
                self.send_sirius_speak(DIALOGUE_TEMPLATES.get("parse_failure", "[sad]失敗したのだ。"))
            return
            
        # Programmatic sanitization to resolve LLM confusion between 'face', 'turn', and 'spin'
        sanitized_commands = []
        for cmd in commands:
            c_type = cmd.get("type")
            c_value = cmd.get("value")

            if c_type == "goto" and isinstance(c_value, dict):
                if "x" in c_value and "y" in c_value:
                    try:
                        cmd = {"type": "goto", "value": [float(c_value["x"]), float(c_value["y"])]}
                        c_value = cmd["value"]
                    except (TypeError, ValueError):
                        self.get_logger().warning(f"Sanitizer: Invalid goto dict value from LLM: {c_value}")

            if c_type == "face" and isinstance(c_value, (int, float)):
                relative_words = ["右", "左", "migi", "hidari", "turn", "曲が"]
                absolute_words = ["北", "南", "東", "西", "kita", "minami", "higashi", "nishi", "map", "絶対"]
                has_relative = any(w in instruction.lower() for w in relative_words)
                has_absolute = any(w in instruction.lower() for w in absolute_words)
                is_radian_like = abs(c_value) <= 3.15 and not float(c_value).is_integer()
                
                if (has_relative and not has_absolute) or is_radian_like:
                    self.get_logger().warning(f"Sanitizer: Detected potential confusion. Converting command 'face' with value {c_value} to relative 'turn'.")
                    cmd = {"type": "turn", "value": c_value}
                    
            elif c_type == "turn" and isinstance(c_value, (int, float)):
                if abs(c_value) >= 15.0:
                    rad_value = math.radians(c_value)
                    self.get_logger().warning(f"Sanitizer: Detected 'turn' with large value {c_value}. Converting degrees to radians: {rad_value:.4f}.")
                    cmd = {"type": "turn", "value": rad_value}
                    
            elif c_type == "spin" and isinstance(c_value, (int, float)):
                if abs(c_value) <= 6.3 and not float(c_value).is_integer():
                    deg_value = math.degrees(c_value)
                    self.get_logger().warning(f"Sanitizer: Detected 'spin' with small float value {c_value}. Converting radians to degrees: {deg_value:.1f}.")
                    cmd = {"type": "spin", "value": deg_value}

            if cmd.get("type") == "turn" and isinstance(cmd.get("value"), (int, float)):
                has_left = any(word in instruction for word in ["左", "hidari", "ひだり", "left"])
                has_right = any(word in instruction for word in ["右", "migi", "みぎ", "right"])
                turn_val = float(cmd.get("value"))
                if has_left and not has_right and turn_val < 0.0:
                    self.get_logger().warning(
                        f"Sanitizer: instruction indicates LEFT but turn value was {turn_val:.4f}; flipping sign."
                    )
                    cmd = {"type": "turn", "value": abs(turn_val)}
                elif has_right and not has_left and turn_val > 0.0:
                    self.get_logger().warning(
                        f"Sanitizer: instruction indicates RIGHT but turn value was {turn_val:.4f}; flipping sign."
                    )
                    cmd = {"type": "turn", "value": -abs(turn_val)}
                    
            sanitized_commands.append(cmd)
        commands = sanitized_commands

        self.get_logger().info(f"Parsed {len(commands)} commands from instruction.")
        
        only_speed = all(c.get("type") == "speed" for c in commands)
        if only_speed:
            for c in commands:
                try:
                    speed_factor = float(c.get("value", 0.9))
                    self.nav_ctrl.set_controller_speed(speed_factor)
                    print(f"⚡ 走行中に速度パラメータを変更しました (factor: {speed_factor:.2f})")
                    self.send_sirius_speak(DIALOGUE_TEMPLATES["speed_change"].format(speed=speed_factor))
                except Exception as e:
                    self.get_logger().error(f"Failed to set speed: {e}")
            return

        sorted_commands = [c for c in commands if c.get("type") == "speed"] + [c for c in commands if c.get("type") != "speed"]
        
        if any(c.get("type") in ["forward", "backward", "turn", "spin", "face", "goto"] for c in sorted_commands):
            self.set_target_following(False, speak_on_failure=False)
            self.nav_ctrl.publish_nav_control("pause_silent")
        self.nav_ctrl.cancel_navigation(clear_queue=False)
        
        with self.lock:
            self.command_queue = sorted_commands
            self.executing_command = False
            
        self.cmd_executor.execute_next_command()

    # --- High-level Monitoring and Goal-checking Loops ---
    def timer_goal_publisher(self):
        """10Hzで実行され、旋回中であれば目標角度との差分を計算し、3度以下で自動キャンセル（到達判定）する"""
        with self.lock:
            executing = self.executing_command
            remaining = self.turn_remaining_angle
            last_yaw = self.last_yaw_robot
            
        if not executing or remaining is None:
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'sirius3/base_footprint',
                rclpy.time.Time()
            )
            
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw_robot = math.atan2(siny_cosp, cosy_cosp)
            
            delta_yaw = yaw_robot - last_yaw
            delta_yaw = (delta_yaw + math.pi) % (2 * math.pi) - math.pi
            
            new_remaining = remaining - delta_yaw
            
            if abs(new_remaining) < math.radians(3.0):
                self.get_logger().info(
                    f"✅ [Turn Arrived] Heading aligned: final diff={math.degrees(new_remaining):.1f}deg < 3.0deg. Stopping."
                )
                self.get_logger().info(
                    f"📍 [End Pose] X={trans.transform.translation.x:.3f}, Y={trans.transform.translation.y:.3f}, Yaw={math.degrees(yaw_robot):+.1f}deg"
                )
                with self.lock:
                    self.turn_remaining_angle = None
                    self.turn_target_yaw = None
                    self.goal_reached_logged = True
                    self.is_stuck = False
                    self.distance_remaining_history = []
                    self.yaw_diff_history = []
                    self.turn_arrival_triggered = True
                    self.last_action_status = "success"
                    self.last_final_yaw_error = float(math.degrees(new_remaining))
                    self.last_final_distance_error = 0.0
                
                self.cancel_navigation(clear_queue=False)
                return
            
            with self.lock:
                self.turn_remaining_angle = new_remaining
                self.last_yaw_robot = yaw_robot
                
        except Exception:
            pass

    def monitor_goal_distance(self):
        """現在の自己位置と目標位置の距離を比較し、到達やスタック状態を監視する"""
        stuck_msg_bool = Bool()
        with self.lock:
            stuck_msg_bool.data = self.is_stuck
        self.stuck_pub.publish(stuck_msg_bool)

        with self.lock:
            goal_x = self.active_goal_x
            goal_y = self.active_goal_y
            goal_yaw = self.active_goal_yaw
            logged = self.goal_reached_logged

        if goal_x is None or logged:
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'sirius3/base_footprint',
                rclpy.time.Time()
            )
        except Exception:
            return

        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        
        distance = math.sqrt((goal_x - tx)**2 + (goal_y - ty)**2)
        
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw_robot = math.atan2(siny_cosp, cosy_cosp)
        
        yaw_diff = abs(goal_yaw - yaw_robot)
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
        yaw_diff = abs(yaw_diff)

        should_cancel_due_to_stuck = False
        stuck_msg = ""
        
        elapsed_sec = 0.0
        with self.lock:
            turn_tgt = self.turn_target_yaw
            executing = self.executing_command
            vel_x = self.current_vel_x
            vel_theta = self.current_vel_theta
            if self.command_start_time is not None:
                elapsed_sec = (self.get_clock().now() - self.command_start_time).nanoseconds / 1e9
            
        if turn_tgt is not None:
            with self.lock:
                self.yaw_diff_history.append(yaw_diff)
                if len(self.yaw_diff_history) > 5:
                    self.yaw_diff_history.pop(0)
                history_len = len(self.yaw_diff_history)
                first_diff = self.yaw_diff_history[0] if history_len > 0 else 0
                last_diff = self.yaw_diff_history[-1] if history_len > 0 else 0
            
            if history_len == 5:
                angle_change = abs(first_diff - last_diff)
                if angle_change < math.radians(0.5) and abs(vel_theta) < 0.01 and elapsed_sec > 15.0:
                    should_cancel_due_to_stuck = True
                    stuck_msg = "🤖 [Stuck Detected during Turn] Robot angular movement is completely blocked."
        else:
            with self.lock:
                self.distance_remaining_history.append(distance)
                if len(self.distance_remaining_history) > 5:
                    self.distance_remaining_history.pop(0)
                history_len = len(self.distance_remaining_history)
                first_dist = self.distance_remaining_history[0] if history_len > 0 else 0
                last_dist = self.distance_remaining_history[-1] if history_len > 0 else 0
            
            if history_len == 5:
                dist_change = abs(first_dist - last_dist)
                if dist_change < 0.005 and abs(vel_x) < 0.005 and abs(vel_theta) < 0.005 and elapsed_sec > 15.0:
                    should_cancel_due_to_stuck = True
                    stuck_msg = "🤖 [Stuck Detected during Translation] Robot path is blocked or unable to reach the target."

        if should_cancel_due_to_stuck:
            self.get_logger().warning(stuck_msg)
            self.get_logger().warning("Automatically canceling active goal to prevent persistent blockage.")
            
            with self.lock:
                self.is_stuck = True
                self.last_action_status = "failed_stuck"
                self.last_final_distance_error = float(distance)
                self.last_final_yaw_error = float(math.degrees(yaw_diff))
                self.chat_history.append({
                    "role": "assistant", 
                    "content": "【System Feedback】Robot detected physical blockage/stuck state. Navigation has been automatically cancelled."
                })
            
            with self.lock:
                last_type = self.last_active_cmd_type
                obs_dists = getattr(self, 'obstacle_distances', {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0})
            
            speak_msg = DIALOGUE_TEMPLATES["stuck"]
            if last_type == "backward":
                dist = obs_dists.get("back", 999.0)
                if dist < 2.0:
                    speak_msg = f"[sad]後方 {dist:.1f}メートルに障害物があって、進めなくなっちゃったのだ！"
            elif last_type in ["forward", "goto"] or last_type is None:
                dist = obs_dists.get("front", 999.0)
                if dist < 2.0:
                    speak_msg = f"[sad]前方 {dist:.1f}メートルに障害物があって、進めなくなっちゃったのだ！"
            
            self.send_sirius_speak(speak_msg)
            self.cancel_navigation(preserve_current_goal=True)
            
            print(color_text(f"\n⚠️ {stuck_msg.split(']')[-1].strip()} 目標をキャンセルして停止します。", _YELLOW))
            print_command_prompt()
            return
        else:
            with self.lock:
                self.is_stuck = False

        is_arrived = False
        with self.lock:
            last_type = self.last_active_cmd_type
            xy_tolerance = self.current_xy_tolerance
        
        if (distance < xy_tolerance and yaw_diff < math.radians(30.0)) or (distance < xy_tolerance and last_type is None):
            is_arrived = True
            
        if is_arrived:
            with self.lock:
                self.goal_reached_logged = True
                self.is_stuck = False
                self.distance_remaining_history = []
                self.yaw_diff_history = []
                self.turn_target_yaw = None
                self.last_action_status = "success"
                self.last_final_distance_error = float(distance)
                self.last_final_yaw_error = float(math.degrees(yaw_diff))
            
            self.get_logger().info("🏆 [Goal Reached] Robot has arrived at the target waypoint.")
            self.get_logger().info(
                f"📍 [End Pose] X={tx:.3f}, Y={ty:.3f}, Yaw={math.degrees(yaw_robot):+.1f}deg (Remaining dist: {distance:.3f}m, angle: {math.degrees(yaw_diff):+.1f}deg)"
            )
            
            has_more = False
            with self.lock:
                if self.command_queue:
                    has_more = True
            
            if has_more:
                print("🎉 ウェイポイント到達。次のアクションを開始します。")
                self.execute_next_command()
            else:
                with self.lock:
                    self.executing_command = False
                    self.current_xy_tolerance = 0.50
                self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': 0.50})
                self.send_sirius_speak(DIALOGUE_TEMPLATES["arrival"])
                print(color_text("\n🎉 全ての目標地点に到着しました！次の指示をどうぞ。", _GREEN))
                print_command_prompt()

    # --- Interaction and Follow-me logic ---
    def handle_follow_me_instruction(self, instruction: str) -> bool:
        """「ついてきて」を、検出済みプライマリターゲットへの追従開始に割り当てる。"""
        normalized = instruction.strip().replace(" ", "").replace("　", "").lower()
        stop_follow_keywords = [
            "もういいよ", "もういい", "ついてこないで", "付いてこないで",
            "ついてくるのやめて", "付いてくるのやめて", "追従終了", "追従やめ",
            "追従停止", "followstop", "stopfollowing"
        ]
        if any(keyword.lower().replace(" ", "") in normalized for keyword in stop_follow_keywords):
            self.get_logger().info("Stop-follow keyword detected. Disabling target_follower.")
            self.set_target_following(False, speak_on_failure=False)
            self.cancel_navigation(clear_queue=True, preserve_current_goal=False)
            self.send_sirius_speak("[happy]了解、ついていくのを終わるのだ。")
            return True

        follow_keywords = [
            "ついてきて", "付いてきて", "ついて来て", "付いて来て",
            "ついてきな", "ついておいで", "ついて来な", "followme"
        ]
        if not any(keyword.lower().replace(" ", "") in normalized for keyword in follow_keywords):
            return False

        self.get_logger().info("Follow-me keyword detected. Enabling target_follower.")
        self.publish_nav_control("pause_silent")
        self.cancel_navigation(clear_queue=True, preserve_current_goal=False)
        if self.set_target_following(True):
            self.send_sirius_speak("[happy]了解、ロックしている人についていくのだ！")
        return True

    def set_target_following(self, enabled: bool, speak_on_failure: bool = True) -> bool:
        """target_follower を常駐ノードとして扱い、追従のON/OFFだけを切り替える。"""
        srv_name = "/target_follower/set_parameters"
        client = self.create_client(SetParameters, srv_name)

        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"Service {srv_name} not available. target_follower may not be running.")
            if enabled and speak_on_failure:
                self.send_sirius_speak("[sad]追従ノードが起動していないのだ。先に人検知と追従ノードを起動してほしいのだ。")
            return False

        req = SetParameters.Request()
        param = Parameter()
        param.name = "enable_following"
        p_val = ParameterValue()
        p_val.type = ParameterType.PARAMETER_BOOL
        p_val.bool_value = bool(enabled)
        param.value = p_val
        req.parameters.append(param)

        future = client.call_async(req)
        future.add_done_callback(
            lambda fut, state=enabled: self._target_following_param_callback(fut, state)
        )
        return True

    def _target_following_param_callback(self, future, enabled: bool):
        try:
            response = future.result()
            if response.results and not all(result.successful for result in response.results):
                reasons = ", ".join(result.reason for result in response.results if result.reason)
                self.get_logger().warning(f"[TargetFollow] failed to set enable_following={enabled}: {reasons}")
                return
            self.get_logger().info(f"[TargetFollow] enable_following set to {enabled}")
        except Exception as exc:
            self.get_logger().error(f"[TargetFollow] parameter update failed: {exc}")

    def print_prompt(self):
        print_command_prompt()

    def send_sirius_speak(self, text):
        """face_client 経由で音声送信する薄いラッパー"""
        with self.lock:
            humor_level = self.current_humor_level
            self.last_spoken_text = text
        styled_text = style_sirius_speak(text, humor_level)
        tag_match = re.match(r"^\[([a-zA-Z_]+)(?::[^\]]+)?\]", styled_text)
        if tag_match:
            expression = tag_match.group(1)
            if expression in ["normal", "happy", "angry", "sad", "surprised", "cat", "wink", "pien", "sleeping"]:
                with self.lock:
                    self.current_expression = expression
        self.face_client.send_speak(styled_text)
        self.get_logger().info(f"Sent Speak command: {styled_text}")

    def _normalize_expression_value(self, value):
        valid = ["normal", "happy", "angry", "sad", "surprised", "cat", "wink", "pien", "sleeping"]
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in valid:
                return lowered
            if lowered in ["笑顔", "にこにこ", "ニコニコ"]:
                return "happy"
            if lowered in ["ウインク", "ウィンク", "wink"]:
                return "wink"
        try:
            fval = float(value)
            if fval >= 0.75:
                return "happy"
            if fval >= 0.5:
                return "cat"
            if fval >= 0.25:
                return "normal"
            return "normal"
        except (TypeError, ValueError):
            return "normal"

    def get_battery_report_string(self):
        """face_client を優先し、失敗時は BLE Gateway の ROS キャッシュからバッテリー状態を返す"""
        def cached_status():
            with self.lock:
                cached = dict(self.latest_battery_status) if self.latest_battery_status else None
                cache_age = time.time() - self.latest_battery_status_time
            if not cached or cache_age > 30.0:
                return None
            level = float(cached.get("battery_level", -1.0))
            if level < 0.0:
                return None
            charging_val = 1.0 if cached.get("is_charging") else 2.0
            return level, charging_val

        status = self.face_client.get_battery_status()
        if status is None:
            status = cached_status()
            if status is None:
                return DIALOGUE_TEMPLATES.get("battery_fail", "[sad]バッテリー状態が確認できないのだ。")

        level, charging_val = status
        if level < 0.0:
            cached = cached_status()
            if cached is not None:
                level, charging_val = cached
            else:
                self.get_logger().warning(
                    "Face battery status returned invalid level and no fresh /sirius/battery_status cache is available."
                )
                return DIALOGUE_TEMPLATES.get("battery_error", "[sad]バッテリー残量データが不正なのだ。")

        if level < 0.0:
            return DIALOGUE_TEMPLATES.get("battery_error", "[sad]バッテリー残量データが不正なのだ。")

        charging_str = "未充電"
        if charging_val == 1.0:
            charging_str = "充電中"
        elif charging_val == 2.0:
            charging_str = "放電中（使用中）"

        template = DIALOGUE_TEMPLATES.get("battery_report", "[happy]現在のバッテリー残量は {level:.1f}パーセントなのだ！状態は {charging_str} なのだ。")
        return template.format(level=level, charging_str=charging_str)


def main(args=None):
    rclpy.init(args=args)
    node = LlmDynamicGoal()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
