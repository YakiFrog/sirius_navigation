#!/usr/bin/env python3
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

os.environ.setdefault("RCUTILS_COLORIZED_OUTPUT", "1")
os.environ.setdefault("RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{time}] [{name}]: {message}")

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool, String
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import Spin
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


class LlmDynamicGoal(Node):
    def __init__(self):
        super().__init__('llm_dynamic_goal')
        
        # パラメータ設定
        self.declare_parameter('lm_studio_url', 'http://localhost:1234/v1/chat/completions')
        self.declare_parameter('model_name', 'meta-llama-3-8b-instruct')
        
        self.lm_studio_url = self.get_parameter('lm_studio_url').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        
        # TFのリスナーを初期化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # パブリッシャーの定義
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/dynamic_goal_marker', 10)
        self.stop_pub = self.create_publisher(Bool, 'stop', 10)
        self.nav_control_pub = self.create_publisher(String, '/nav_control', 10)
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
        
        # アクションキャンセルのためのサービス
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.param_client = self.create_client(SetParameters, '/controller_server/set_parameters')



        
        # 自然言語指示トピックのサブスクライバー
        self.instruction_sub = self.create_subscription(
            String,
            '/llm_instruction',
            self.instruction_callback,
            10
        )
        
        self.face_client = FaceClient(logger=self.get_logger())
        self.surrounding_people_count = 0
        self.current_expression = "normal"
        self.current_humor_level = DEFAULT_HUMOR_LEVEL
        self.marker_array_sub = self.create_subscription(
            MarkerArray,
            '/target_detector/target_markers',
            self.marker_array_callback,
            10
        )

        self.lock = threading.Lock()
        self.get_logger().info(f'LLM Dynamic Goal Node initialized with State-Feedback loop.')
        self.get_logger().info(f'LM Studio URL: {self.lm_studio_url}')
        
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
        # None以外のとき: 現在実行中コマンドはturnで、このyaw角に近づいたらキャンセル
        self.turn_target_yaw = None
        self.turn_remaining_angle = None
        self.last_yaw_robot = 0.0
        self.turn_arrival_triggered = False
        self.turn_goal_distance = 0.7  # xy_goal_tolerance (0.5m) より大きい値として 0.7m に設定（壁との衝突を防ぐ）
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
        
        # 会話履歴管理 (マルチターン対話用)
        self.chat_history = []
        self.history_max_turns = 10  # 最大5往復分
        
        # 直前アクションの実行結果パラメータ
        self.last_action_status = "none"  # "success", "failed_stuck", "failed_cancelled", "none"
        self.last_action_type = "none"     # "forward", "backward", "turn", "spin", "goto", "none"
        self.last_target_value = 0.0
        self.last_final_distance_error = 0.0
        self.last_final_yaw_error = 0.0
        self.last_action_start_x = 0.0
        self.last_action_start_y = 0.0
        self.last_action_start_yaw = 0.0
        
        # 1Hzで自己位置と目標位置の距離を監視するタイマー
        self.goal_monitor_timer = self.create_timer(1.0, self.monitor_goal_distance)
        
        # 10Hz (0.1秒周期)で旋回中の角度監視を行うタイマー（preemptionを防ぐため再発行は行わない）
        self.dynamic_goal_timer = self.create_timer(0.1, self.timer_goal_publisher)

        # 対話型コマンドライン入力を別スレッドで開始
        self.running = True
        self.input_thread = threading.Thread(target=self.interactive_input_loop, daemon=True)
        self.input_thread.start()

        # HTTPサーバーを別スレッドで開始して外部（Docker等）からの指示を受け付ける
        self.start_http_instruction_server()

    def start_http_instruction_server(self):
        from http.server import BaseHTTPRequestHandler, HTTPServer
        import json
        
        class InstructionHTTPHandler(BaseHTTPRequestHandler):
            node = self
            
            def log_message(self, format, *args):
                pass # Suppress logs to stdout

            def do_POST(self):
                if self.path == '/instruction':
                    try:
                        content_length = int(self.headers['Content-Length'])
                        post_data = self.rfile.read(content_length)
                        data = json.loads(post_data.decode('utf-8'))
                        instruction = data.get('instruction', '')
                        if instruction:
                            self.node.get_logger().info(f'Received HTTP instruction: "{instruction}"')
                            threading.Thread(target=self.node.process_instruction, args=(instruction,), daemon=True).start()
                            self.send_response(200)
                            self.send_header('Content-Type', 'application/json')
                            self.end_headers()
                            self.wfile.write(json.dumps({"success": True}).encode('utf-8'))
                            return
                    except Exception as e:
                        self.node.get_logger().error(f"Error handling HTTP instruction: {e}")
                
                self.send_response(400)
                self.end_headers()

        def run_server():
            server_address = ('', 50060)
            try:
                httpd = HTTPServer(server_address, InstructionHTTPHandler)
                self.get_logger().info("=== Instruction HTTP Server running on port 50060 ===")
                httpd.serve_forever()
            except Exception as e:
                self.get_logger().error(f"Failed to start Instruction HTTP Server: {e}")

        threading.Thread(target=run_server, daemon=True).start()

    def odom_callback(self, msg):
        """オドメトリから現在の速度を取得"""
        with self.lock:
            self.current_vel_x = msg.twist.twist.linear.x
            self.current_vel_theta = msg.twist.twist.angular.z

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
        except Exception as e:
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
        # 全角英数字などを半角に正規化 (例: "１０ｍ" -> "10m", "０．５" -> "0.5")
        instruction = normalize_instruction_text(instruction)
        
        # 1. 停止・キャンセル指示の簡易キーワード判定（高速応答のため）
        cancel_keywords = ["キャンセル", "cancel", "中止", "取り消", "とりけし"]
        if any(kw in instruction.lower() for kw in cancel_keywords):
            self.get_logger().warning("Cancel keyword detected. Clearing active goal.")
            self.publish_nav_control("cancel")
            self.cancel_navigation(clear_queue=True, preserve_current_goal=False)
            self.send_sirius_speak("[sad]了解、目標を取り消したのだ。")
            return

        stop_keywords = ["止ま", "とまれ", "止め", "とめ", "ストップ", "停止", "stop", "とまって", "待機"]
        if any(kw in instruction.lower() for kw in stop_keywords):
            self.get_logger().warning("Stop keyword detected. Pausing active goal.")
            self.publish_nav_control("pause")
            self.cancel_navigation(preserve_current_goal=True)
            self.send_sirius_speak("[surprised]了解、止まるのだ！")
            return

        resume_keywords = ["再開", "再スタート", "続けて", "つづけて", "resume", "reopen", "再び", "動かして", "移動を再開", "続行"]
        if any(kw in instruction.lower() for kw in resume_keywords):
            self.get_logger().info("Resume keyword detected.")
            self.publish_nav_control("resume")
            if self.resume_navigation():
                self.send_sirius_speak("[happy]了解、再開するのだ！")
            else:
                # move_goal 側の pause を再開する用途では、こちらのスナップショットが無くても成立する
                self.send_sirius_speak("[happy]了解、再開命令を送ったのだ！")
            return

        # 2. LM Studio にリクエストを投げる
        self.get_logger().info(f"Querying LLM: '{instruction}'...")
        result = self.query_lm_studio(instruction)
        
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
            self.publish_nav_control("cancel")
            self.cancel_navigation(clear_queue=True, preserve_current_goal=False)
            self.send_sirius_speak("[sad]了解、目標を取り消したのだ。")
            return
            
        commands = result.get("commands", [])
        speak_text = result.get("speak", "")
        if speak_text:
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
                # Detect if 'face' is accidentally used with radian values or relative instructions
                relative_words = ["右", "左", "migi", "hidari", "turn", "曲が"]
                absolute_words = ["北", "南", "東", "西", "kita", "minami", "higashi", "nishi", "map", "絶対"]
                has_relative = any(w in instruction.lower() for w in relative_words)
                has_absolute = any(w in instruction.lower() for w in absolute_words)
                is_radian_like = abs(c_value) <= 3.15 and not float(c_value).is_integer()
                
                if (has_relative and not has_absolute) or is_radian_like:
                    self.get_logger().warning(f"Sanitizer: Detected potential confusion. Converting command 'face' with value {c_value} to relative 'turn'.")
                    cmd = {"type": "turn", "value": c_value}
                    
            elif c_type == "turn" and isinstance(c_value, (int, float)):
                # If 'turn' is specified with large values (assumed degrees), convert to radians
                if abs(c_value) >= 15.0:
                    rad_value = math.radians(c_value)
                    self.get_logger().warning(f"Sanitizer: Detected 'turn' with large value {c_value}. Converting degrees to radians: {rad_value:.4f}.")
                    cmd = {"type": "turn", "value": rad_value}
                    
            elif c_type == "spin" and isinstance(c_value, (int, float)):
                # If 'spin' (which expects degrees) is specified with small radian floats, convert to degrees
                if abs(c_value) <= 6.3 and not float(c_value).is_integer():
                    deg_value = math.degrees(c_value)
                    self.get_logger().warning(f"Sanitizer: Detected 'spin' with small float value {c_value}. Converting radians to degrees: {deg_value:.1f}.")
                    cmd = {"type": "spin", "value": deg_value}
                    
            sanitized_commands.append(cmd)
        commands = sanitized_commands

        self.get_logger().info(f"Parsed {len(commands)} commands from instruction.")
        
        # もし速度変更コマンドのみであれば、既存の移動キューをクリアせず、その場で速度変更を適用する
        only_speed = all(c.get("type") == "speed" for c in commands)
        if only_speed:
            for c in commands:
                try:
                    speed_factor = float(c.get("value", 0.9))
                    self.set_controller_speed(speed_factor)
                    print(f"⚡ 走行中に速度パラメータを変更しました (factor: {speed_factor:.2f})")
                    self.send_sirius_speak(DIALOGUE_TEMPLATES["speed_change"].format(speed=speed_factor))
                except Exception as e:
                    self.get_logger().error(f"Failed to set speed: {e}")
            return

        # speed コマンドがあれば、移動開始前に反映されるようにキューの最優先（先頭）に並び替える
        sorted_commands = [c for c in commands if c.get("type") == "speed"] + [c for c in commands if c.get("type") != "speed"]
        
        # 進行中の動作があれば、新しいコマンドを開始する前に一度キャンセルして停止させる（割り込み処理）
        self.cancel_navigation(clear_queue=False)
        
        with self.lock:
            # 既存のキューをクリアして新しい指示シーケンスを設定
            self.command_queue = sorted_commands
            self.executing_command = False
            
        self.execute_next_command()

    def execute_next_command(self):
        """キューから次のコマンドを取り出して実行する"""
        with self.lock:
            if not self.command_queue:
                self.executing_command = False
                self.current_xy_tolerance = 0.50
                self.suppress_step_speech = False
                self.get_logger().info("All commands in sequence finished.")
                self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': 0.50})
                return
            cmd = self.command_queue.pop(0)
            self.executing_command = True
            self.command_start_time = self.get_clock().now()
            
            should_speak = not self.suppress_step_speech
            if self.command_queue:
                self.suppress_step_speech = True
            
        cmd_type = cmd.get("type", "forward")
        value = cmd.get("value", 0.0)
        speak_override = cmd.get("speak")
        
        with self.lock:
            self.last_action_type = cmd_type
            if isinstance(value, list):
                self.last_target_value = value
            elif isinstance(value, str):
                # expression など文字列 value はそのまま保存
                self.last_target_value = value
            else:
                try:
                    self.last_target_value = float(value)
                except (TypeError, ValueError):
                    self.last_target_value = value
        
        self.get_logger().info(f"Executing next sequence command -> type: {cmd_type}, value: {value}")
        
        if cmd_type == "speed":
            try:
                speed_factor = float(value)
                self.set_controller_speed(speed_factor)
                print(f"⚡ 速度パラメータを変更しました (factor: {speed_factor:.2f})")
                if should_speak:
                    self.send_sirius_speak(DIALOGUE_TEMPLATES["speed_change"].format(speed=speed_factor))
            except Exception as e:
                self.get_logger().error(f"Failed to set speed: {e}")
            self.execute_next_command()
            return

        if cmd_type == "expression":
            exp_state = self._normalize_expression_value(value)
            ok = self.face_client.set_expression(exp_state)
            if ok:
                with self.lock:
                    self.current_expression = exp_state
                self.get_logger().info(f"[Expression] Set to '{exp_state}'")
                if should_speak:
                    self.send_sirius_speak(f"[{exp_state}]表情を変えたのだ！")
            else:
                self.get_logger().warning(f"[Expression] Failed to set '{exp_state}' (face server offline)")
            self.execute_next_command()
            return

        if cmd_type == "parameter":
            try:
                if isinstance(value, dict):
                    param_name = value.get("name")
                    param_amount = float(value.get("amount", 1.0))
                else:
                    param_name = None
                    param_amount = 1.0

                if param_name not in ["blushAmount", "sparkleAmount"]:
                    self.get_logger().warning(f"[Parameter] Unsupported parameter: {param_name}")
                else:
                    ok = self.face_client.update_parameters({param_name: param_amount})
                    if ok:
                        self.get_logger().info(f"[Parameter] Set {param_name}={param_amount}")
                        if should_speak:
                            if param_name == "blushAmount":
                                self.send_sirius_speak("[happy]照れを足したのだ！")
                            elif param_name == "sparkleAmount":
                                self.send_sirius_speak("[happy]キラキラを足したのだ！")
                    else:
                        self.get_logger().warning(f"[Parameter] Failed to set {param_name} (face server offline)")
            except Exception as e:
                self.get_logger().error(f"[Parameter] Failed to apply effect: {e}")
            self.execute_next_command()
            return

        if cmd_type == "humor":
            level = clamp_humor_level(value)
            with self.lock:
                self.current_humor_level = level
            self.get_logger().info(f"[Humor] Set humor level to {level:.2f}")
            if should_speak:
                if level <= 0.0:
                    self.send_sirius_speak("[normal]ユーモアレベルを0.0に設定しました。真面目な敬語モードで対応します。")
                elif level < 0.5:
                    self.send_sirius_speak(f"[normal]ユーモアレベルを{level:.1f}にしたのだ。落ち着いて対応するのだ。")
                else:
                    self.send_sirius_speak(f"[cat]ユーモアレベルを{level:.1f}にしたのだ！ちょっと生意気にいくのだ。")
            self.execute_next_command()
            return

        if cmd_type == "reset":
            ok = self.face_client.reset_face()
            if ok:
                with self.lock:
                    self.current_expression = "normal"
                self.get_logger().info("[Face] Reset face state")
                if should_speak:
                    self.send_sirius_speak("[normal]表情を戻したのだ！")
            else:
                self.get_logger().warning("[Face] Failed to reset face state (face server offline)")
            self.execute_next_command()
            return

        if cmd_type == "effect":
            effect_type = str(value or "shake")
            ok = self.face_client.trigger_effect(effect_type)
            if ok:
                self.get_logger().info(f"[Effect] Triggered '{effect_type}'")
                if should_speak:
                    self.send_sirius_speak("[happy]演出を出したのだ！")
            else:
                self.get_logger().warning(f"[Effect] Failed to trigger '{effect_type}' (face server offline)")
            self.execute_next_command()
            return

        if cmd_type == "look":
            look_map = {
                "center": (960.0, 540.0),
                "left": (0.0, 540.0),
                "right": (1920.0, 540.0),
                "up": (960.0, 0.0),
                "down": (960.0, 1080.0),
            }
            try:
                if isinstance(value, dict):
                    look_x = float(value.get("x", 960.0))
                    look_y = float(value.get("y", 540.0))
                    look_name = "custom"
                elif isinstance(value, (list, tuple)) and len(value) >= 2:
                    look_x = float(value[0])
                    look_y = float(value[1])
                    look_name = "custom"
                else:
                    look_name = str(value or "center").lower()
                    look_x, look_y = look_map.get(look_name, look_map["center"])

                ok = self.face_client.look_at(look_x, look_y)
                if ok:
                    self.get_logger().info(f"[Look] Set gaze '{look_name}' -> x={look_x:.1f}, y={look_y:.1f}")
                    if should_speak:
                        self.send_sirius_speak("[happy]目線を動かしたのだ！")
                else:
                    self.get_logger().warning(f"[Look] Failed to set gaze '{look_name}' (face server offline)")
            except Exception as e:
                self.get_logger().error(f"[Look] Failed to parse look command: {e}")
            self.execute_next_command()
            return

        # 1. 現在の位置 (TF) を取得 (gotoコマンドや回転計算に必要)
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'sirius3/base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            
            # クォータニオンからロボットのyaw角を抽出
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw_robot = math.atan2(siny_cosp, cosy_cosp)
            self.get_logger().info(f"📍 [Start Pose] X={tx:.3f}, Y={ty:.3f}, Yaw={math.degrees(yaw_robot):+.1f}deg")
            with self.lock:
                self.last_action_start_x = tx
                self.last_action_start_y = ty
                self.last_action_start_yaw = yaw_robot
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup TF (map -> sirius3/base_footprint) during execution: {e}")
            self.execute_next_command()
            return

        r = 0.0
        theta = 0.0
        
        if cmd_type in ["forward", "backward"]:
            with self.lock:
                active_x = self.active_goal_x
                active_y = self.active_goal_y
                active_yaw = self.active_goal_yaw
                logged = self.goal_reached_logged
                last_type = self.last_active_cmd_type
                
            if active_x is not None and not logged and last_type == cmd_type:
                base_x = active_x
                base_y = active_y
                base_yaw = active_yaw
                self.get_logger().info(f"Extending active {cmd_type} goal: X={base_x:.3f}, Y={base_y:.3f}, Yaw={math.degrees(base_yaw):+.1f}deg")
            else:
                base_x = tx
                base_y = ty
                base_yaw = yaw_robot
                
            with self.lock:
                self.last_active_cmd_type = cmd_type
        else:
            with self.lock:
                self.last_active_cmd_type = None
            base_x = tx
            base_y = ty
            base_yaw = yaw_robot
        
        # turn 以外のコマンドが実行される際は、旋回関連の監視パラメータを確実に初期化して誤作動を防ぐ
        if cmd_type not in ["turn", "spin", "face"]:
            with self.lock:
                self.turn_remaining_angle = None
                self.turn_target_yaw = None
                self.turn_arrival_triggered = False

        if cmd_type == "forward":
            # ウェイポイントは最小0.3mから置くように制限
            r = max(value, 0.3)
            theta = 0.0
            tolerance = max(r * 0.3, 0.15) if r < 1.0 else 0.50
            with self.lock:
                self.current_xy_tolerance = tolerance
            self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': tolerance})
            if should_speak:
                self.send_sirius_speak(DIALOGUE_TEMPLATES["forward_start"].format(distance=r))
            self.publish_goal_pose(base_x, base_y, base_yaw, r, theta)
            
        elif cmd_type == "backward":
            # 後退時: 目標位置は現在姿勢の後方へ置き、目標yawは現在yawのまま保持する。
            # yawまで180度反転させると、Nav2が「後ろを向いて到着」と解釈して旋回してしまう。
            # ウェイポイントは最小0.3mから置くように制限
            r = max(abs(value), 0.3)
            tolerance = max(r * 0.3, 0.15) if r < 1.0 else 0.50
            with self.lock:
                self.current_xy_tolerance = tolerance
            self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': tolerance})
            if should_speak:
                self.send_sirius_speak(DIALOGUE_TEMPLATES["backward_start"].format(distance=r))
            target_map_x = base_x - r * math.cos(base_yaw)
            target_map_y = base_y - r * math.sin(base_yaw)
            self.publish_direct_map_goal(target_map_x, target_map_y, base_yaw)
            
        elif cmd_type == "turn":
            # 旋回時: Nav2の標準ビヘイビアである /spin アクションを使用し、
            # 地図の干渉やダミーゴール座標の衝突を避け、確実かつ滑らかにその場旋回を実行する
            with self.lock:
                self.turn_remaining_angle = value
                self.last_yaw_robot = yaw_robot
                self.turn_target_yaw = None
                self.turn_arrival_triggered = False
            
            if should_speak:
                if speak_override:
                    turn_msg = speak_override
                else:
                    deg_val = round(math.degrees(abs(value)))
                    if abs(deg_val - 180) < 15:
                        turn_msg = "[wink]後ろを向くのだ！"
                    elif abs(deg_val - 90) < 15:
                        turn_msg = f"[wink]{'右' if value < 0 else '左'}に90度回るのだ！"
                    elif deg_val >= 300:
                        turn_msg = f"[wink]{'右' if value < 0 else '左'}にその場旋回するのだ！"
                    else:
                        turn_msg = f"[wink]{'右' if value < 0 else '左'}を向くのだ！"
                self.send_sirius_speak(turn_msg)
            self.send_spin_goal(value)
            
        elif cmd_type == "spin":
            # ======================================================
            # その場旋回: 指定角度(度数法)をラジアンの turn コマンドに変換して実行。
            # 分割せず、タイマー側で積算処理するため累積誤差が生じない。
            # ======================================================
            total_deg = float(value) if value != 0.0 else 360.0
            rad_value = math.radians(total_deg)
            self.get_logger().info(f"[Spin] Converting {total_deg:+.0f}deg to turn command ({rad_value:+.3f} rad).")
            # キューの先頭に turn コマンドを挿入して再実行
            direction = "右" if rad_value < 0 else "左"
            spin_speak = f"[wink]{direction}にその場旋回するのだ！"
            with self.lock:
                self.command_queue.insert(0, {"type": "turn", "value": rad_value, "speak": spin_speak})
            self.execute_next_command()
            return
            
        elif cmd_type == "face":
            # ======================================================
            # 絶対方角に向く: map座標系の絶対yaw角[度]に向けて旋回する。
            # value: 目標yaw角[度] (map座標系、0=X正方向、90=Y正方向)
            # ======================================================
            try:
                target_abs_deg = float(value)
                target_abs_yaw = math.radians(target_abs_deg)
                # 現在のyawとの差分を計算して turn コマンドに変換
                diff_yaw = target_abs_yaw - yaw_robot
                # -π〜π に正規化
                diff_yaw = (diff_yaw + math.pi) % (2 * math.pi) - math.pi
                
                if abs(diff_yaw) < math.radians(5.0):
                    self.get_logger().info("[Face] Already facing target direction. Skipping.")
                    self.execute_next_command()
                    return
                
                self.get_logger().info(
                    f"[Face] Current yaw={math.degrees(yaw_robot):+.1f}deg → "
                    f"Target={target_abs_deg:+.1f}deg, Δ={math.degrees(diff_yaw):+.1f}deg"
                )
                # turn コマンドとして処理
                with self.lock:
                    self.command_queue.insert(0, {"type": "turn", "value": diff_yaw})
                self.execute_next_command()
                return
            except Exception as e:
                self.get_logger().error(f"[Face] Failed to parse angle: {e}")
                self.execute_next_command()
                return
                
        elif cmd_type == "goto":
            # 絶対座標 (map上の座標 [x, y]) に向かう
            try:
                target_x = float(value[0])
                target_y = float(value[1])
                
                # ロボットから目的地に向かう角度を計算
                dx = target_x - tx
                dy = target_y - ty
                target_yaw = math.atan2(dy, dx)
                
                # 直接距離を計算
                r = math.sqrt(dx**2 + dy**2)
                tolerance = max(r * 0.3, 0.15) if r < 1.0 else 0.50
                with self.lock:
                    self.current_xy_tolerance = tolerance
                self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': tolerance})
                
                # 指示受け取り報告。原点は TTS で聞き取りやすいように専用表現にする。
                if abs(target_x) < 0.001 and abs(target_y) < 0.001:
                    self.send_sirius_speak("[happy]原点、Xゼロ、Yゼロに向かうのだ！")
                else:
                    self.send_sirius_speak(DIALOGUE_TEMPLATES["goto_start"].format(x=target_x, y=target_y))
                
                # 直接 map 座標系でゴールを発行する
                self.publish_direct_map_goal(target_x, target_y, target_yaw)
            except Exception as e:
                self.get_logger().error(f"Failed to parse coordinates for goto: {e}")
                self.execute_next_command()
                return
        else:
            self.get_logger().error(f"Unknown command type: {cmd_type}")
            self.execute_next_command()
            return

    def get_robot_state_context_string(self):
        """現在のロボットの物理的な実行状況を自然言語のテキストとして構築する"""
        with self.lock:
            goal_x = self.active_goal_x
            goal_y = self.active_goal_y
            goal_yaw = self.active_goal_yaw
            stuck = self.is_stuck
            vel_x = self.current_vel_x
            vel_theta = self.current_vel_theta
            queue_len = len(self.command_queue)
            executing = self.executing_command
            obs_dists = getattr(self, 'obstacle_distances', {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0})
            
        with self.lock:
            last_status = self.last_action_status
            last_type = self.last_action_type
            last_target = self.last_target_value
            last_dist_err = self.last_final_distance_error
            last_yaw_err = self.last_final_yaw_error
            humor_level = self.current_humor_level
            last_start_x = self.last_action_start_x
            last_start_y = self.last_action_start_y
            last_start_yaw = self.last_action_start_yaw

        # 現在位置とヘディング（角度）をTFから取得
        current_x, current_y, current_yaw_deg = 0.0, 0.0, 0.0
        current_yaw = 0.0
        try:
            trans = self.tf_buffer.lookup_transform('map', 'sirius3/base_footprint', rclpy.time.Time())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)
            current_yaw_deg = math.degrees(current_yaw)
        except Exception:
            pass

        # 直前のアクションにおける実際の移動距離・回転量を計算
        last_move_str = "N/A"
        if last_type in ["forward", "backward"]:
            dist_moved = math.sqrt((current_x - last_start_x)**2 + (current_y - last_start_y)**2)
            last_move_str = f"Moved linear distance of {dist_moved:.2f} meters (Start: X={last_start_x:.2f}, Y={last_start_y:.2f} -> End: X={current_x:.2f}, Y={current_y:.2f})"
        elif last_type in ["turn", "spin", "face"]:
            diff_rad = current_yaw - last_start_yaw
            # -pi から pi の範囲に正規化
            diff_rad = (diff_rad + math.pi) % (2 * math.pi) - math.pi
            diff_deg = math.degrees(diff_rad)
            last_move_str = f"Rotated angle of {diff_deg:+.1f} degrees (Start: {math.degrees(last_start_yaw):+.1f}deg -> End: {current_yaw_deg:+.1f}deg)"

        if goal_x is None:
            state_str = (
                "【Robot Current Hardware State Feedback】: Status=Idle. No active navigation goal. Robot is stationary.\n"
                f"- Current Robot Pose: X={current_x:.2f}, Y={current_y:.2f}, Yaw={current_yaw_deg:+.1f}deg\n"
                f"- Obstacle distances (meters): front={obs_dists.get('front', 999.0):.2f}, back={obs_dists.get('back', 999.0):.2f}, left={obs_dists.get('left', 999.0):.2f}, right={obs_dists.get('right', 999.0):.2f}\n"
                f"- Last Action Status: {last_status} (type: {last_type}, target: {last_target}, final distance error: {last_dist_err:.2f}m, final yaw error: {last_yaw_err:.1f}deg)\n"
                f"- Last Action Actual Execution Result: {last_move_str}"
            )
            return state_str
            
        # 現在位置と目標位置の差分（残差）を計算
        distance = -1.0
        try:
            distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        except Exception:
            pass
            
        stuck_str = "BLOCKED / STUCK (Robot is command-active but linear velocity is zero. An obstacle likely blocks the way.)" if stuck else "Moving normally towards target"
        vel_str = f"linear={vel_x:.2f}m/s, angular={vel_theta:.2f}rad/s"
        
        state_str = (
            f"【Robot Current Hardware State Feedback】\n"
            f"- Status: {'Executing Sequence' if executing else 'Idle/Ready'}\n"
            f"- Current Robot Pose: X={current_x:.2f}, Y={current_y:.2f}, Yaw={current_yaw_deg:+.1f}deg\n"
            f"- Target Waypoint: X={goal_x:.2f}, Y={goal_y:.2f}, Yaw={math.degrees(goal_yaw):+.1f}deg\n"
            f"- Distance remaining to target: {distance:.2f} meters\n"
            f"- Current velocities: {vel_str}\n"
            f"- Current Sirius humor level: {humor_level:.2f}\n"
            f"- Physical obstacles / blockage state: {stuck_str}\n"
            f"- Obstacle distances (meters): front={obs_dists.get('front', 999.0):.2f}, back={obs_dists.get('back', 999.0):.2f}, left={obs_dists.get('left', 999.0):.2f}, right={obs_dists.get('right', 999.0):.2f}\n"
            f"- Actions left in sequence queue: {queue_len} commands\n"
            f"- Last Action Status: {last_status} (type: {last_type}, target: {last_target}, final distance error: {last_dist_err:.2f}m, final yaw error: {last_yaw_err:.1f}deg)\n"
            f"- Last Action Actual Execution Result: {last_move_str}"
        )
        return state_str

    def _build_state_info(self):
        """parse_local_rules へ渡す現在の状態情報をまとめた辞書を構築する"""
        current_x, current_y = 0.0, 0.0
        try:
            trans = self.tf_buffer.lookup_transform('map', 'sirius3/base_footprint', rclpy.time.Time())
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
        except Exception:
            pass

        with self.lock:
            executing = self.executing_command
            queue_len = len(self.command_queue)
            stuck = self.is_stuck
            last_action_status = self.last_action_status
            last_action_type = self.last_action_type
            last_target_value = self.last_target_value
            last_action_start_x = self.last_action_start_x
            last_action_start_y = self.last_action_start_y
            last_action_start_yaw = self.last_action_start_yaw
            current_humor_level = self.current_humor_level
            # 最後のコマンド方向を chat_history から判定
            last_cmd_was_backward = False
            last_cmd_was_forward = False
            for msg in reversed(self.chat_history):
                if msg.get("role") == "assistant":
                    try:
                        import json as _json
                        hist_cmds = _json.loads(msg.get("content", "{}")).get("commands", [])
                        if hist_cmds:
                            last_t = hist_cmds[-1].get("type")
                            last_cmd_was_backward = (last_t == "backward")
                            last_cmd_was_forward = (last_t == "forward")
                            break
                    except Exception:
                        pass
            obs_dists = getattr(self, 'obstacle_distances', {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0})

        return {
            "current_x": current_x,
            "current_y": current_y,
            "executing": executing,
            "queue_len": queue_len,
            "stuck": stuck,
            "people_count": self.surrounding_people_count,
            "face_active": self.face_client.face_server_active,
            "current_expression": self.current_expression,
            "current_humor_level": current_humor_level,
            "current_speed_setting": self.current_speed_setting,
            "current_vel_x": self.current_vel_x,
            "current_vel_theta": self.current_vel_theta,
            "last_action_status": last_action_status,
            "last_action_type": last_action_type,
            "last_target_value": last_target_value,
            "last_action_start_x": last_action_start_x,
            "last_action_start_y": last_action_start_y,
            "last_action_start_yaw": last_action_start_yaw,
            "last_cmd_was_backward": last_cmd_was_backward,
            "last_cmd_was_forward": last_cmd_was_forward,
            "obstacle_distances": obs_dists,
        }

    def _local_parse_needs_llm_check(self, instruction, local_result, norm_inst):
        """ローカルパースが少し不安なときだけ LLM verifier に回す。"""
        if local_result.get("cancel", False):
            return False

        commands = local_result.get("commands", [])
        if not commands:
            return False

        cmd_types = [cmd.get("type") for cmd in commands if isinstance(cmd, dict)]
        if not cmd_types:
            return False

        uncertain_words = [
            "そのまま", "ちょっと", "少し", "すこし", "もう少し", "もうちょっと",
            "もっと", "さらに", "いきすぎ", "行き過ぎ", "進みすぎ", "下がりすぎ",
            "回りすぎ", "まわりすぎ", "逆", "反対", "違う", "ちがう", "そっちじゃ",
            "足りない", "たりない", "戻って", "もどって", "戻し", "やり直",
            "さっき", "前回", "同じ", "その方向", "そっち", "こっち",
        ]
        if any(word in norm_inst for word in uncertain_words):
            return True

        if len(commands) > 1:
            return True

        # 「-3,3移動して」のような裸座標は実用上便利だが、座標指定か相対移動かの誤読余地がある。
        if any(cmd_type == "goto" for cmd_type in cmd_types):
            explicit_coordinate_words = ["座標", "目標座標", "goto", "go to", "原点", "ホーム", "home"]
            if not any(word in norm_inst for word in explicit_coordinate_words):
                return True

        # 右/左/後ろだけの短い指示は、向く・移動・旋回の揺れが出やすい。
        if any(cmd_type in ["turn", "spin", "backward"] for cmd_type in cmd_types):
            directional_words = ["右", "左", "後ろ", "うしろ", "裏", "みぎ", "ひだり"]
            if any(word in norm_inst for word in directional_words) and len(norm_inst) <= 12:
                return True

        return False

    def _parse_llm_json_content(self, content):
        content = content.strip()
        if content.startswith("```"):
            lines = content.splitlines()
            if len(lines) >= 3:
                content = "\n".join(lines[1:-1]).strip()
        return json.loads(content), content

    def _validate_verifier_result(self, verifier_result, local_result):
        if not isinstance(verifier_result, dict):
            return None

        decision = verifier_result.get("decision", "accept")
        if decision == "accept":
            return local_result

        if decision == "ask":
            speak = verifier_result.get("speak") or "[normal]念のため確認したいのだ。どう動けばいいか、もう少し具体的に言ってほしいのだ。"
            return {"commands": [], "cancel": False, "fast_path": True, "speak": speak}

        if decision != "revise":
            return None

        allowed_types = {"forward", "backward", "turn", "spin", "face", "goto", "speed", "expression", "parameter", "reset", "effect", "look", "humor"}
        revised_commands = verifier_result.get("commands", [])
        if not isinstance(revised_commands, list):
            return None

        safe_commands = []
        for cmd in revised_commands:
            if not isinstance(cmd, dict):
                continue
            cmd_type = cmd.get("type")
            if cmd_type not in allowed_types:
                continue
            safe_commands.append(cmd)

        if not safe_commands and not verifier_result.get("speak"):
            return None

        result = {
            "commands": safe_commands,
            "cancel": bool(verifier_result.get("cancel", False)),
            "fast_path": True,
        }
        if verifier_result.get("speak"):
            result["speak"] = verifier_result.get("speak")
        return result

    def _verify_local_parse_with_llm(self, instruction, local_result, state_info):
        """ローカルパース結果を、短いLLM verifierで検算する。失敗時はNoneを返す。"""
        system_prompt = (
            "You are a safety verifier for a Japanese robot navigation parser. "
            "Check whether rule_parse correctly captures the user's intent. "
            "Reply raw JSON only, no markdown. "
            "Schema: {\"decision\":\"accept|revise|ask\", \"commands\":[], \"cancel\":false, \"speak\":\"optional Japanese\"}. "
            "Use accept if rule_parse is good enough. "
            "Use revise only when the rule_parse is clearly wrong. "
            "Use ask only when the instruction is genuinely ambiguous. "
            "If executing the command requires guessing missing direction, distance, destination, face expression, or effect, use ask and keep commands empty. "
            "Allowed command types: forward, backward, turn, spin, face, goto, speed, expression, parameter, reset, effect, look, humor. "
            "Important: backward means move backward while keeping the current yaw; do not convert it to a turn. "
            "Keep distances and angles conservative. Do not invent extra commands."
        )

        verifier_input = {
            "user_text": instruction,
            "rule_parse": local_result,
            "state": {
                "executing": state_info.get("executing", False),
                "last_action_status": state_info.get("last_action_status", ""),
                "last_action_type": state_info.get("last_action_type", ""),
                "last_target_value": state_info.get("last_target_value", 0.0),
                "current_humor_level": state_info.get("current_humor_level", DEFAULT_HUMOR_LEVEL),
                "obstacle_distances": state_info.get("obstacle_distances", {}),
            },
        }
        payload = {
            "model": self.model_name,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": json.dumps(verifier_input, ensure_ascii=False)},
            ],
            "temperature": 0.0,
            "max_tokens": 384,
            "top_p": 1.0,
        }

        try:
            req = urllib.request.Request(
                self.lm_studio_url,
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=5.0) as response:
                res_body = response.read().decode("utf-8")
                res_json = json.loads(res_body)
                content = res_json["choices"][0]["message"]["content"]
                verifier_json, cleaned_content = self._parse_llm_json_content(content)

            checked = self._validate_verifier_result(verifier_json, local_result)
            if checked is None:
                self.get_logger().warning(f"LLM verifier returned unusable result: {verifier_json}")
                return None

            self.get_logger().info(
                f"LLM verifier checked uncertain local parse: decision={verifier_json.get('decision', 'accept')} raw='{cleaned_content}'"
            )
            return checked
        except Exception as e:
            self.get_logger().warning(f"LLM verifier failed; using local parse as-is: {e}")
            return None

    def query_lm_studio(self, instruction):
        """ローカルルールを先に試み、マッチしなければLM StudioへLLMクエリを送る"""
        import re
        norm_inst = normalize_instruction_text(instruction).strip().replace(" ", "").replace("　", "").lower()

        def _extract_jsonish_speak(text):
            if not text:
                return None
            m = re.search(r'"speak"\s*:\s*"([^"]{1,400})', text)
            if m:
                return m.group(1)
            m = re.search(r'"response"\s*:\s*"([^"]{1,400})', text)
            if m:
                return m.group(1)
            return None

        # --- ローカルルールベース判定 ---
        state_info = self._build_state_info()
        local_result = parse_local_rules(
            instruction,
            state_info,
            battery_callback=self.get_battery_report_string
        )
        if local_result is not None:
            # 発話は process_instruction 側の send_sirius_speak に集約する。
            # ここで直接送ると同じ文が二重送信され、後段の割り込み発話で先発話を潰すことがある。
            if self._local_parse_needs_llm_check(instruction, local_result, norm_inst):
                self.get_logger().info(f"Local parse is uncertain; asking LLM verifier. parse={local_result}")
                checked_result = self._verify_local_parse_with_llm(instruction, local_result, state_info)
                if checked_result is not None:
                    return checked_result
            return local_result
        
        # ルールベースで一致しなかった複雑な指示は LLM (LM Studio) へ問い合わせる
        # -------------------------------------------------------------------
        system_prompt = (
            "You are a robotic navigator assistant. Output raw JSON only, no markdown, no reasoning, no extra text.\n"
            "Schema: {\"commands\": [ {\"type\": one_of(forward, backward, turn, spin, face, goto, speed, expression, parameter, reset, effect, look, humor), \"value\": any } ], \"cancel\": boolean, \"speak\": string?}\n"
            "If the user asks a question, explain briefly in Japanese in speak and keep commands empty.\n"
            "If the user requests movement, output the exact command sequence.\n"
            "If the user asks to change humor level, output a humor command with value 0.0 to 1.0.\n"
            "Character: you are Sirius, a cute outdoor autonomous robot. Use a short expression tag and speak like Sirius. "
            "Humor 0.0 means polite Japanese. Humor >=0.5 means lively Kansai-ben with playful confidence.\n"
            "Always prefix speak with exactly one expression tag from [normal], [happy], [angry], [sad], [surprised], [cat], [wink], [pien], [sleeping]. "
            "Use [normal] for clarification questions, [sad] for apologies/failures, [happy] for success/helpful answers, [wink] for jokes.\n"
            "If the request is missing required details or would require guessing intent, ask one concise clarification question in Japanese in speak and keep commands empty.\n"
            "Do not invent a direction, distance, destination, expression, or face effect that the user did not specify.\n"
            "Use the robot state context for current status, remaining distance, blockage, people count, battery, and last action.\n"
            "Always prefer a valid minimal JSON object. Do not wrap in code fences."
        )
        
        headers = {
            'Content-Type': 'application/json',
        }
        
        # 物理状態フィードバックの取得
        state_context = self.get_robot_state_context_string()
        
        # メッセージリストを構築（システムプロンプト + 過去の会話履歴 + リアルタイム状態 + 現在の指示）
        with self.lock:
            if len(self.chat_history) > 4:
                self.chat_history = self.chat_history[-4:]
            
            messages = [{"role": "system", "content": system_prompt}]
            messages.extend(self.chat_history)
            
            # 最新のユーザー指示の直前にロボットの物理状態フィードバックをコンテキストとして挿入
            messages.append({"role": "system", "content": state_context})
            messages.append({"role": "user", "content": instruction})
        
        payload = {
            "model": self.model_name,
            "messages": messages,
            "temperature": 0.0,
            "max_tokens": 2048,
            "top_p": 1.0
        }

        def _fallback_short_query(reason_hint=""):
            """長いプロンプトで失敗したときに、短い再問い合わせを行う"""
            fallback_prompt = (
                "You are a robotic navigator assistant. Reply with raw JSON only. "
                "If the user asks a general question, answer briefly in Japanese in speak. "
                "If details are missing or intent must be guessed, ask one concise clarification question in Japanese in speak and keep commands empty. "
                "Do not invent movement or face-control parameters. "
                "Do not reason. Do not use markdown."
            )
            fallback_messages = [
                {"role": "system", "content": fallback_prompt},
                {"role": "system", "content": state_context},
                {"role": "user", "content": instruction},
            ]
            fallback_payload = {
                "model": self.model_name,
                "messages": fallback_messages,
                "temperature": 0.0,
                "max_tokens": 256,
                "top_p": 1.0
            }
            try:
                req2 = urllib.request.Request(
                    self.lm_studio_url,
                    data=json.dumps(fallback_payload).encode('utf-8'),
                    headers=headers,
                    method='POST'
                )
                with urllib.request.urlopen(req2, timeout=15.0) as response2:
                    res_body2 = response2.read().decode('utf-8')
                    res_json2 = json.loads(res_body2)
                    content2 = res_json2['choices'][0]['message']['content'].strip()
                    if content2.startswith("```"):
                        lines2 = content2.splitlines()
                        if len(lines2) >= 3:
                            content2 = "\n".join(lines2[1:-1])
                    try:
                        parsed_json2 = json.loads(content2)
                    except Exception:
                        speak_hint2 = _extract_jsonish_speak(content2)
                        if speak_hint2:
                            self.get_logger().warning("Fallback JSON was broken, but speak text was recoverable.")
                            parsed_json2 = {"commands": [], "cancel": False, "speak": speak_hint2}
                        elif content2:
                            self.get_logger().warning("Fallback returned plain text; wrapping it as speak.")
                            parsed_json2 = {"commands": [], "cancel": False, "speak": content2[:400]}
                        else:
                            raise
                    self.get_logger().info(f"LLM Fallback Raw Output: '{content2}'")
                    with self.lock:
                        self.chat_history.append({"role": "user", "content": instruction})
                        self.chat_history.append({"role": "assistant", "content": content2})
                    return parsed_json2
            except Exception as e2:
                self.get_logger().error(f"Fallback LLM call also failed ({reason_hint}): {e2}")
                if 'res_body2' in locals():
                    self.get_logger().error(f"Fallback Raw Response: {res_body2}")
            return None
        
        try:
            req = urllib.request.Request(
                self.lm_studio_url,
                data=json.dumps(payload).encode('utf-8'),
                headers=headers,
                method='POST'
            )
            # タイムアウトを15秒に設定
            with urllib.request.urlopen(req, timeout=15.0) as response:
                res_body = response.read().decode('utf-8')
                res_json = json.loads(res_body)
                finish_reason = res_json['choices'][0].get('finish_reason')
                if finish_reason == "length":
                    self.get_logger().warning("LLM response was truncated by token limit; JSON may be incomplete.")
                content = res_json['choices'][0]['message']['content'].strip()
                if not content:
                    self.get_logger().warning("LLM returned empty content; retrying with a shorter prompt.")
                    return _fallback_short_query("empty_content")
                
                if content.startswith("```"):
                    lines = content.splitlines()
                    if len(lines) >= 3:
                        content = "\n".join(lines[1:-1])
                try:
                    parsed_json = json.loads(content)
                except Exception:
                    speak_hint = _extract_jsonish_speak(content)
                    if speak_hint:
                        self.get_logger().warning("LLM JSON was broken, but speak text was recoverable.")
                        parsed_json = {"commands": [], "cancel": False, "speak": speak_hint}
                    elif content:
                        self.get_logger().warning("LLM returned plain text; wrapping it as speak.")
                        parsed_json = {"commands": [], "cancel": False, "speak": content[:400]}
                    else:
                        raise
                self.get_logger().info(f"LLM Raw Output: '{content}'")
                
                # 正常にパースできたら履歴に追加
                with self.lock:
                    self.chat_history.append({"role": "user", "content": instruction})
                    self.chat_history.append({"role": "assistant", "content": content})
                    
                return parsed_json
        except urllib.error.URLError as e:
            self.get_logger().error(f"LM Studio API connection failed: {e}")
            self.get_logger().error("Make sure LM Studio is running on port 1234.")
            return _fallback_short_query("primary_url_error")
        except Exception as e:
            self.get_logger().error(f"Error calling LLM: {e}")
            if 'res_body' in locals():
                self.get_logger().error(f"Raw Response: {res_body}")
            return _fallback_short_query("primary_parse_failed")
        return None
        
    def publish_goal_pose(self, tx, ty, yaw_robot, r, theta):
        """相対目標値(r, theta)をmap絶対座標に変換してパブリッシュする内部メソッド"""
        target_map_x = tx + (r * math.cos(theta) * math.cos(yaw_robot) - r * math.sin(theta) * math.sin(yaw_robot))
        target_map_y = ty + (r * math.cos(theta) * math.sin(yaw_robot) + r * math.sin(theta) * math.cos(yaw_robot))
        
        target_map_yaw = yaw_robot + theta
        self.publish_direct_map_goal(target_map_x, target_map_y, target_map_yaw)

    def publish_direct_map_goal(self, target_x, target_y, target_yaw):
        """map座標系に直接目標ゴールをパブリッシュする"""
        target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi

        # PoseStamped メッセージの構築
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.position.x = target_x
        pose_msg.pose.position.y = target_y
        pose_msg.pose.position.z = 0.0
        
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(target_yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(target_yaw / 2.0)
            
        self.goal_pub.publish(pose_msg)
        self.publish_marker(pose_msg)
        
        # 目標監視用の変数をセット
        with self.lock:
            self.active_goal_x = target_x
            self.active_goal_y = target_y
            self.active_goal_yaw = target_yaw
            self.goal_reached_logged = False
            self.distance_remaining_history = []  # 監視開始時に履歴をリセット
            self.yaw_diff_history = []
            self.is_stuck = False
            
        self.get_logger().info(f"Published goal pose to /goal_pose: X={target_x:.2f}, Y={target_y:.2f}, Yaw={math.degrees(target_yaw):+.1f}deg")

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
            
            # クォータニオンからロボットのyaw角を抽出
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            yaw_robot = math.atan2(siny_cosp, cosy_cosp)
            
            # 前回からのロボットの微小回転量を算出（ラップアラウンド対策）
            delta_yaw = yaw_robot - last_yaw
            delta_yaw = (delta_yaw + math.pi) % (2 * math.pi) - math.pi
            
            # 残りの回転角度を更新
            new_remaining = remaining - delta_yaw
            
            # 1. 角度誤差が 3.0度 (約0.052rad) 以下になったら到達完了
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
                
                # Nav2のゴールをキャンセルして停止 (これより spin_goal_result_callback が呼ばれる)
                self.cancel_navigation(clear_queue=False)
                return
            
            # 内部の残り角度変数を最新に更新
            with self.lock:
                self.turn_remaining_angle = new_remaining
                self.last_yaw_robot = yaw_robot
                
        except Exception as e:
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
        
        # 距離の計算
        distance = math.sqrt((goal_x - tx)**2 + (goal_y - ty)**2)
        
        # クォータニオンからロボットのyaw角を抽出
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw_robot = math.atan2(siny_cosp, cosy_cosp)
        
        # 角度差の計算
        yaw_diff = abs(goal_yaw - yaw_robot)
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
        yaw_diff = abs(yaw_diff)

        # -------------------------------------------------------------
        # スタック検出ロジック (タイマー監視により更新)
        # -------------------------------------------------------------
        should_cancel_due_to_stuck = False
        stuck_msg = ""
        
        # コマンド開始からの経過時間を計算
        elapsed_sec = 0.0
        with self.lock:
            turn_tgt = self.turn_target_yaw
            executing = self.executing_command
            vel_x = self.current_vel_x
            vel_theta = self.current_vel_theta
            if self.command_start_time is not None:
                elapsed_sec = (self.get_clock().now() - self.command_start_time).nanoseconds / 1e9
            
        # -------------------------------------------------------------------
        # Turn アーリーキャンセルとスタック監視は 5Hz タイマー側で実施するため、
        # 1Hzの monitor_goal_distance 側では旋回中の直線距離スタック検知を行わない
        # -------------------------------------------------------------------
        if turn_tgt is not None:
            # 旋回中のスタック検知 (角度変化がまったくない場合のみ)
            with self.lock:
                self.yaw_diff_history.append(yaw_diff)
                if len(self.yaw_diff_history) > 5:
                    self.yaw_diff_history.pop(0)
                history_len = len(self.yaw_diff_history)
                first_diff = self.yaw_diff_history[0] if history_len > 0 else 0
                last_diff = self.yaw_diff_history[-1] if history_len > 0 else 0
            
            if history_len == 5:
                angle_change = abs(first_diff - last_diff)
                # 完全に動かない（0.5度未満の変化）かつ速度ゼロの場合のみスタックとみなす。かつ開始から15秒以上経過している場合
                if angle_change < math.radians(0.5) and abs(vel_theta) < 0.01 and elapsed_sec > 15.0:
                    should_cancel_due_to_stuck = True
                    stuck_msg = "🤖 [Stuck Detected during Turn] Robot angular movement is completely blocked."
        else:
            # 並進移動（前進・後退・goto）中のスタック検知
            with self.lock:
                self.distance_remaining_history.append(distance)
                if len(self.distance_remaining_history) > 5:
                    self.distance_remaining_history.pop(0)
                history_len = len(self.distance_remaining_history)
                first_dist = self.distance_remaining_history[0] if history_len > 0 else 0
                last_dist = self.distance_remaining_history[-1] if history_len > 0 else 0
            
            # 5秒間のデータがあり、並進距離の変化が極小で、実速度も極小の場合。かつ開始から15秒以上経過している場合
            if history_len == 5:
                dist_change = abs(first_dist - last_dist)
                # 閾値を0.005に下げて、短い距離で低速移動している場合の誤判定を防ぐ
                if dist_change < 0.005 and abs(vel_x) < 0.005 and abs(vel_theta) < 0.005 and elapsed_sec > 15.0:
                    should_cancel_due_to_stuck = True
                    stuck_msg = "🤖 [Stuck Detected during Translation] Robot path is blocked or unable to reach the target."

        if should_cancel_due_to_stuck:
            self.get_logger().warning(stuck_msg)
            self.get_logger().warning("Automatically canceling active goal to prevent persistent blockage.")
            
            # 会話履歴にスタック状態の物理フィードバックをシステムログとして挿入し、LLMに教える
            with self.lock:
                self.is_stuck = True
                self.last_action_status = "failed_stuck"
                self.last_final_distance_error = float(distance)
                self.last_final_yaw_error = float(math.degrees(yaw_diff))
                self.chat_history.append({
                    "role": "assistant", 
                    "content": "【System Feedback】Robot detected physical blockage/stuck state. Navigation has been automatically cancelled."
                })
            
            # Determine distance and details based on direction
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
            
            # 安全のためロックの外側でキャンセルを実行
            self.send_sirius_speak(speak_msg)
            self.cancel_navigation(preserve_current_goal=True)
            
            print(color_text(f"\n⚠️ {stuck_msg.split(']')[-1].strip()} 目標をキャンセルして停止します。", _YELLOW))
            print_command_prompt()
            return
        else:
            with self.lock:
                self.is_stuck = False
        # -------------------------------------------------------------

        # -------------------------------------------------------------------
        # Turn アーリーキャンセル: 5Hz timer側で行うため、ここは無効化
        # -------------------------------------------------------------------
        
        # 閾値判定 (距離 tolerance 以内 かつ 角度差30度以内、もしくは絶対座標指定 goto コマンドで距離 tolerance 以内の場合)
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
            
            # キューが残っているかチェックし、残っていれば次のコマンドを実行
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

    def set_controller_speed(self, speed_setting):
        """Configure controller_server and velocity_smoother based on navigation mode configs"""
        nav_modes = {
            'slow': {
                '/controller_server': {
                    'FollowPath.vx_max': 0.20,
                    'FollowPath.vx_min': -0.10,
                    'FollowPath.wz_max': 0.20,
                    'FollowPath.vx_std': 0.20,
                    'FollowPath.wz_std': 0.20,
                    'FollowPath.ax_max': 0.20,
                    'FollowPath.ax_min': -0.20,
                    'FollowPath.az_max': 0.50,
                },
                '/velocity_smoother': {
                    'max_velocity': [0.20, 0.0, 0.20],
                    'min_velocity': [-0.10, 0.0, -0.20],
                    'max_accel': [0.20, 0.0, 0.50],
                    'max_decel': [-0.20, 0.0, -0.50]
                }
            },
            'safe': {
                '/controller_server': {
                    'FollowPath.vx_max': 0.40,
                    'FollowPath.vx_min': -0.20,
                    'FollowPath.wz_max': 0.40,
                    'FollowPath.vx_std': 0.20,
                    'FollowPath.wz_std': 0.20,
                    'FollowPath.ax_max': 0.40,
                    'FollowPath.ax_min': -0.40,
                    'FollowPath.az_max': 1.00,
                },
                '/velocity_smoother': {
                    'max_velocity': [0.40, 0.0, 0.40],
                    'min_velocity': [-0.20, 0.0, -0.40],
                    'max_accel': [0.40, 0.0, 1.00],
                    'max_decel': [-0.40, 0.0, -1.00]
                }
            },
            'normal': {
                '/controller_server': {
                    'FollowPath.vx_max': 0.90,
                    'FollowPath.vx_min': -0.60,
                    'FollowPath.wz_max': 0.90,
                    'FollowPath.vx_std': 0.25,
                    'FollowPath.wz_std': 0.30,
                    'FollowPath.ax_max': 0.90,
                    'FollowPath.ax_min': -0.90,
                    'FollowPath.az_max': 1.50,
                },
                '/velocity_smoother': {
                    'max_velocity': [0.90, 0.0, 0.90],
                    'min_velocity': [-0.90, 0.0, -0.90],
                    'max_accel': [0.90, 0.0, 1.50],
                    'max_decel': [-0.90, 0.0, -1.50]
                }
            },
            'fast': {
                '/controller_server': {
                    'FollowPath.vx_max': 1.00,
                    'FollowPath.vx_min': -0.60,
                    'FollowPath.wz_max': 1.00,
                    'FollowPath.vx_std': 0.40,
                    'FollowPath.wz_std': 0.48,
                    'FollowPath.ax_max': 1.50,
                    'FollowPath.ax_min': -1.50,
                    'FollowPath.az_max': 2.20,
                },
                '/velocity_smoother': {
                    'max_velocity': [1.00, 0.0, 1.00],
                    'min_velocity': [-0.60, 0.0, -1.00],
                    'max_accel': [1.50, 0.0, 2.20],
                    'max_decel': [-1.50, 0.0, -2.20]
                }
            }
        }

        if isinstance(speed_setting, str):
            mode = speed_setting.lower()
        else:
            try:
                val = float(speed_setting)
                if val <= 0.25:
                    mode = 'slow'
                elif val <= 0.55:
                    mode = 'safe'
                elif val <= 0.95:
                    mode = 'normal'
                else:
                    mode = 'fast'
            except Exception:
                mode = 'normal'
                
        if mode not in nav_modes:
            mode = 'normal'
            
        self.get_logger().info(f"Applying navigation mode config: '{mode}'")
        with self.lock:
            self.current_speed_setting = {
                "slow": 0.20,
                "safe": 0.40,
                "normal": 0.90,
                "fast": 1.00,
            }.get(mode, 0.90)
        
        cfg = nav_modes[mode]
        for node_name, params in cfg.items():
            self.set_node_parameters(node_name, params)

    def set_node_parameters(self, node_name, params_dict):
        """Helper to call SetParameters service asynchronously on target node"""
        srv_name = f'{node_name}/set_parameters'
        client = self.create_client(SetParameters, srv_name)
        
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"Service {srv_name} not available!")
            return
            
        req = SetParameters.Request()
        for name, val in params_dict.items():
            param = Parameter()
            param.name = name
            
            p_val = ParameterValue()
            if isinstance(val, bool):
                p_val.type = ParameterType.PARAMETER_BOOL
                p_val.bool_value = val
            elif isinstance(val, int):
                p_val.type = ParameterType.PARAMETER_INTEGER
                p_val.integer_value = val
            elif isinstance(val, float):
                p_val.type = ParameterType.PARAMETER_DOUBLE
                p_val.double_value = val
            elif isinstance(val, list):
                if len(val) > 0 and isinstance(val[0], float):
                    p_val.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                    p_val.double_array_value = [float(v) for v in val]
            param.value = p_val
            req.parameters.append(param)
            
        client.call_async(req)


    def publish_marker(self, pose_msg):

        """目標位置にRViz可視化用のマーカーをパブリッシュする"""
        marker = Marker()
        marker.header = pose_msg.header
        marker.ns = 'llm_goal'
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        
        marker.scale.x = 0.6
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.9
        
        marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
        self.marker_pub.publish(marker)

    def delete_marker(self):
        """RViz上のマーカーを削除する"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'llm_goal'
        marker.id = 1
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def cancel_navigation(self, clear_queue=True, preserve_current_goal=False):
        """実行中のナビゲーションをキャンセル"""
        self.delete_marker()
        
        # Publish not stuck status instantly
        stuck_msg_bool = Bool()
        stuck_msg_bool.data = False
        self.stuck_pub.publish(stuck_msg_bool)
        
        # Spinアクションが動いていればキャンセル
        with self.lock:
            spin_handle = self.spin_goal_handle
            self.spin_goal_handle = None
            
        if spin_handle is not None:
            self.get_logger().info("Canceling active Spin action...")
            spin_handle.cancel_goal_async()
            
        with self.lock:
            if preserve_current_goal and self.active_goal_x is not None and self.active_goal_y is not None:
                self.paused_goal_snapshot = {
                    "active_goal_x": self.active_goal_x,
                    "active_goal_y": self.active_goal_y,
                    "active_goal_yaw": self.active_goal_yaw,
                    "last_action_type": self.last_action_type,
                    "last_target_value": self.last_target_value,
                    "turn_remaining_angle": self.turn_remaining_angle,
                    "turn_target_yaw": self.turn_target_yaw,
                    "current_xy_tolerance": self.current_xy_tolerance,
                }
            elif not preserve_current_goal:
                self.paused_goal_snapshot = None
            self.active_goal_x = None
            self.active_goal_y = None
            self.active_goal_yaw = None
            self.goal_reached_logged = True
            self.distance_remaining_history = []
            self.yaw_diff_history = []
            self.is_stuck = False
            self.turn_target_yaw = None
            if not self.turn_arrival_triggered:
                self.turn_arrival_triggered = False
            if self.last_action_status not in ["success", "failed_stuck"]:
                self.last_action_status = "failed_cancelled"
            
            if clear_queue:
                self.command_queue = []
                self.executing_command = False
                self.current_xy_tolerance = 0.50
        self.get_logger().info(
            "Navigation state updated: "
            f"{'pause' if preserve_current_goal else 'cancel'} "
            f"(clear_queue={clear_queue})"
        )
            
        if clear_queue:
            self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': 0.50})

        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Navigation cancel service not available.")
            return
        
        req = CancelGoal.Request()
        req.goal_info = GoalInfo()
        self.cancel_client.call_async(req)
        
        stop_msg = Bool()
        stop_msg.data = True
        self.stop_pub.publish(stop_msg)
        
        def reset_stop():
            msg = Bool()
            msg.data = False
            self.stop_pub.publish(msg)
        threading.Timer(1.0, reset_stop).start()

    def publish_nav_control(self, command: str):
        """move_goal.py へ明示的な制御コマンドを送る"""
        msg = String()
        msg.data = command
        self.nav_control_pub.publish(msg)

    def resume_navigation(self):
        """停止前の目標を再開する"""
        with self.lock:
            snapshot = self.paused_goal_snapshot

        if not snapshot:
            return False

        if snapshot.get("active_goal_x") is not None and snapshot.get("active_goal_y") is not None:
            self.publish_direct_map_goal(
                snapshot["active_goal_x"],
                snapshot["active_goal_y"],
                snapshot["active_goal_yaw"] if snapshot.get("active_goal_yaw") is not None else 0.0,
            )
            with self.lock:
                self.executing_command = True
                self.command_start_time = self.get_clock().now()
                self.last_active_cmd_type = snapshot.get("last_action_type", "goto")
                self.current_xy_tolerance = snapshot.get("current_xy_tolerance", 0.50)
                self.last_action_status = "none"
                self.distance_remaining_history = []
                self.yaw_diff_history = []
                self.is_stuck = False
                self.paused_goal_snapshot = None
            self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': self.current_xy_tolerance})
            return True

        return False

    def send_spin_goal(self, relative_yaw):
        if not self.spin_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Spin action server not available!")
            self.execute_next_command()
            return
            
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = float(relative_yaw)
        
        self.get_logger().info(f"Sending Spin action goal: target_yaw={math.degrees(relative_yaw):+.1f}deg")
        
        with self.lock:
            self.spin_goal_handle = None
            
        self.spin_send_goal_future = self.spin_client.send_goal_async(goal_msg)
        self.spin_send_goal_future.add_done_callback(self.spin_goal_response_callback)
        
    def spin_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Spin goal rejected by server")
                self.execute_next_command()
                return
                
            self.get_logger().info("Spin goal accepted by server")
            with self.lock:
                self.spin_goal_handle = goal_handle
                
            self.spin_result_future = goal_handle.get_result_async()
            self.spin_result_future.add_done_callback(self.spin_goal_result_callback)
        except Exception as e:
            self.get_logger().error(f"Error in spin goal response: {e}")
            self.execute_next_command()

    def spin_goal_result_callback(self, future):
        self.get_logger().info("Spin goal finished execution")
        
        success = False
        try:
            status = future.result().status
            if status == 4: # STATUS_SUCCEEDED in ROS 2
                success = True
        except Exception:
            pass

        with self.lock:
            if self.turn_arrival_triggered:
                success = True
                self.turn_arrival_triggered = False

        # 最終自己位置の取得と表示
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
            self.get_logger().info(
                f"📍 [End Pose] X={trans.transform.translation.x:.3f}, Y={trans.transform.translation.y:.3f}, Yaw={math.degrees(yaw_robot):+.1f}deg"
            )
        except Exception:
            pass

        with self.lock:
            self.spin_goal_handle = None
            self.executing_command = False
            self.current_xy_tolerance = 0.50
            has_more = bool(self.command_queue)
            obs_dists = getattr(self, 'obstacle_distances', {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0})
            nearest_obstacle = min(obs_dists.values()) if obs_dists else 999.0
            if success:
                self.last_action_status = "success"
                self.last_final_distance_error = 0.0
                self.last_final_yaw_error = 0.0
                self.is_stuck = False
            else:
                self.last_action_status = "failed_stuck" if nearest_obstacle < 0.8 else "failed"
                self.last_final_distance_error = 0.0
                self.last_final_yaw_error = abs(math.degrees(self.turn_remaining_angle)) if self.turn_remaining_angle is not None else 0.0
                self.is_stuck = nearest_obstacle < 0.8
                self.chat_history.append({
                    "role": "assistant",
                    "content": "【System Feedback】Spin/turn action failed. Nearby obstacle may have prevented safe rotation."
                })
            
        if has_more:
            self.execute_next_command()
        else:
            self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': 0.50})
            if success:
                self.send_sirius_speak(DIALOGUE_TEMPLATES["turn_success"])
            else:
                self.send_sirius_speak(DIALOGUE_TEMPLATES["turn_failure"])
            print(color_text("\n✅ 旋回完了！次の指示をどうぞ。", _GREEN))
            print_command_prompt()

    def send_sirius_speak(self, text):
        """face_client 経由で音声送信する薄いラッパー"""
        with self.lock:
            humor_level = self.current_humor_level
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
        """face_client から生情報を取得し、DIALOGUE_TEMPLATES を使ってバッテリー状態をフォーマットする"""
        status = self.face_client.get_battery_status()
        if status is None:
            return DIALOGUE_TEMPLATES.get("battery_fail", "[sad]バッテリー状態が確認できないのだ。")
        
        level, charging_val = status
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
