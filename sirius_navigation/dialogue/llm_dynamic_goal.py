#!/usr/bin/env python3
import sys
import math
import json
import urllib.request
import urllib.error
import threading
import unicodedata
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker
from rclpy.action import ActionClient
from nav2_msgs.action import Spin
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

try:
    from . import CHAT_KEYWORDS, DIALOGUE_TEMPLATES
except ImportError:
    try:
        from dialogue import CHAT_KEYWORDS, DIALOGUE_TEMPLATES
    except ImportError:
        from chat import CHAT_KEYWORDS
        from navigation import NAVIGATION_TEMPLATES
        from system import SYSTEM_TEMPLATES
        DIALOGUE_TEMPLATES = {}
        DIALOGUE_TEMPLATES.update(NAVIGATION_TEMPLATES)
        DIALOGUE_TEMPLATES.update(SYSTEM_TEMPLATES)


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

        # Janome トークナイザーの初期化
        try:
            from janome.tokenizer import Tokenizer
            self.tokenizer = Tokenizer()
            self.get_logger().info("Janome tokenizer successfully loaded for high-precision parsing.")
        except Exception as e:
            self.tokenizer = None
            self.get_logger().warning(f"Failed to load Janome: {e}. Falling back to simple substring matching.")

        
        # 自然言語指示トピックのサブスクライバー
        self.instruction_sub = self.create_subscription(
            String,
            '/llm_instruction',
            self.instruction_callback,
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
        
        self.command_queue = []
        self.executing_command = False
        self.current_xy_tolerance = 0.50
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

    def instruction_callback(self, msg):
        """トピック経由で指示を受信したときのコールバック"""
        self.get_logger().info(f'Received instruction via topic: "{msg.data}"')
        threading.Thread(target=self.process_instruction, args=(msg.data,), daemon=True).start()

    def interactive_input_loop(self):
        """標準入力からインタラクティブに入力を受け付けるスレッド"""
        print("\n=== Sirius LLM Dynamic Goal Navigation ===")
        print("LM Studioを起動し、モデルがロードされていることを確認してください。")
        print("「ちょっと前に行って」「そこで右向いて」「停止」などの指示を入力してください。")
        print("「3m前に行って、右向いて」といった連続指示も実行可能です。")
        print("終了するには 'exit' または Ctrl+C を入力してください。\n")
        
        while self.running:
            try:
                print("Command > ", end="", flush=True)
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
        instruction = unicodedata.normalize('NFKC', instruction)
        
        # 1. 停止・キャンセル指示の簡易キーワード判定（高速応答のため）
        stop_keywords = ["止まれ", "ストップ", "停止", "stop", "cancel", "キャンセル", "とまって", "待機"]
        if any(kw in instruction.lower() for kw in stop_keywords):
            self.get_logger().warning("Stop/Cancel keyword detected. Canceling active goal.")
            self.cancel_navigation()
            return

        # 2. LM Studio にリクエストを投げる
        self.get_logger().info(f"Querying LLM: '{instruction}'...")
        result = self.query_lm_studio(instruction)
        
        if result is None:
            self.get_logger().error("Failed to parse command from LLM.")
            self.send_sirius_speak(DIALOGUE_TEMPLATES.get("parse_failure", "[sad]失敗したのだ。"))
            return
            
        is_cancel = result.get("cancel", False)
        if is_cancel:
            self.get_logger().warning("LLM interpreted instruction as STOP/CANCEL.")
            self.cancel_navigation()
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
        
        with self.lock:
            self.last_action_type = cmd_type
            if isinstance(value, list):
                self.last_target_value = value
            else:
                self.last_target_value = float(value)
        
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
            # 後退時: keyboard_dynamic_goal.py に合わせ、theta=180度(pi)として後方に移動させる
            # これによりNav2が前方向のセンサを使って安全に目的地（後方）へアプローチできるようになる
            # ウェイポイントは最小0.3mから置くように制限
            r = max(abs(value), 0.3)
            theta = math.pi
            tolerance = max(r * 0.3, 0.15) if r < 1.0 else 0.50
            with self.lock:
                self.current_xy_tolerance = tolerance
            self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': tolerance})
            if should_speak:
                self.send_sirius_speak(DIALOGUE_TEMPLATES["backward_start"].format(distance=r))
            self.publish_goal_pose(base_x, base_y, base_yaw, r, theta)
            
        elif cmd_type == "turn":
            # 旋回時: Nav2の標準ビヘイビアである /spin アクションを使用し、
            # 地図の干渉やダミーゴール座標の衝突を避け、確実かつ滑らかにその場旋回を実行する
            with self.lock:
                self.turn_remaining_angle = value
                self.last_yaw_robot = yaw_robot
                self.turn_target_yaw = None
                self.turn_arrival_triggered = False
            
            if should_speak:
                deg_val = round(math.degrees(abs(value)))
                if abs(deg_val - 180) < 15:
                    turn_msg = "[wink]後ろを向くのだ！"
                elif abs(deg_val - 90) < 15:
                    turn_msg = f"[wink]{'右' if value < 0 else '左'}に90度回るのだ！"
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
            with self.lock:
                self.command_queue.insert(0, {"type": "turn", "value": rad_value})
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
                
                # 指示受け取り報告
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
            
        with self.lock:
            last_status = self.last_action_status
            last_type = self.last_action_type
            last_target = self.last_target_value
            last_dist_err = self.last_final_distance_error
            last_yaw_err = self.last_final_yaw_error
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
            f"- Physical obstacles / blockage state: {stuck_str}\n"
            f"- Actions left in sequence queue: {queue_len} commands\n"
            f"- Last Action Status: {last_status} (type: {last_type}, target: {last_target}, final distance error: {last_dist_err:.2f}m, final yaw error: {last_yaw_err:.1f}deg)\n"
            f"- Last Action Actual Execution Result: {last_move_str}"
        )
        return state_str

    def query_lm_studio(self, instruction):
        """LM StudioのOpenAI互換APIを呼び出し、会話履歴と現在の物理状況（状態フィードバック）を含めて指示を変換する"""
        import re
        norm_inst = instruction.strip().replace(" ", "").replace("　", "").lower()
        
        def normalize_kanji_numbers(s):
            s = s.replace("点", ".")
            comp_map = {
                "二十": "2", "三十": "3", "四十": "4", "五十": "5", "六十": "6", "七十": "7", "八十": "8", "九十": "9"
            }
            for k, v in comp_map.items():
                s = s.replace(k + "〇", v + "0")
                s = s.replace(k + "一", v + "1")
                s = s.replace(k + "二", v + "2")
                s = s.replace(k + "三", v + "3")
                s = s.replace(k + "四", v + "4")
                s = s.replace(k + "五", v + "5")
                s = s.replace(k + "六", v + "6")
                s = s.replace(k + "七", v + "7")
                s = s.replace(k + "八", v + "8")
                s = s.replace(k + "九", v + "9")
                s = s.replace(k, v + "0")
            
            teens_map = {
                "十一": "11", "十二": "12", "十三": "13", "十四": "14", "十五": "15",
                "十六": "16", "十七": "17", "十八": "18", "十九": "19", "十": "10"
            }
            for k, v in teens_map.items():
                s = s.replace(k, v)
                
            singles = {
                "〇": "0", "一": "1", "二": "2", "三": "3", "四": "4",
                "五": "5", "六": "6", "七": "7", "八": "8", "九": "9"
            }
            for k, v in singles.items():
                s = s.replace(k, v)
            return s

        def parse_part(part_raw):
            part_norm = normalize_kanji_numbers(part_raw.strip().lower())
            
            # 1. Goto coordinates
            coord_match = re.search(r"(?:goto|go\s*to|座標指定|座標|目標座標)?\s*\(?\s*(-?\d+(?:\.\d+)?)\s*[,，\s]\s*(-?\d+(?:\.\d+)?)\s*\)?", part_norm)
            if coord_match and ("座標" in part_norm or "goto" in part_norm or "," in part_norm):
                try:
                    x = float(coord_match.group(1))
                    y = float(coord_match.group(2))
                    return {"type": "goto", "value": [x, y]}
                except ValueError:
                    pass

            # 2. Speed
            speed_match = re.search(r"(?:速度|スピード|speed)\s*(\d+(?:\.\d+)?)", part_norm)
            if speed_match:
                return {"type": "speed", "value": float(speed_match.group(1))}
            speed_unit_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:m/s|の速度)", part_norm)
            if speed_unit_match:
                return {"type": "speed", "value": float(speed_unit_match.group(1))}

            # 3. Compass face directions
            face_map = {
                "北": 90.0, "kita": 90.0,
                "東": 0.0, "higashi": 0.0,
                "南": -90.0, "minami": -90.0,
                "西": 180.0, "nishi": 180.0,
                "前": 0.0, "正面": 0.0, "mae": 0.0, "shoumen": 0.0
            }
            for d_name, angle in face_map.items():
                if d_name in part_norm and ("向" in part_norm or "向き" in part_norm or "むい" in part_norm or "むく" in part_norm or "face" in part_norm):
                    return {"type": "face", "value": angle}

            # 4. Turn/Spin with numbers
            angle_match = re.search(r"(\d+(?:\.\d+)?)\s*(度|deg|°|rad|ラジアン|回転|旋回)", part_norm)
            if not angle_match:
                angle_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:に)?\s*(右|左|migi|hidari|旋回|回転|時計回り|反時計回り)", part_norm)
                
            if angle_match:
                val_str = angle_match.group(1)
                val = float(val_str)
                
                is_right = any(x in part_norm for x in ["右", "migi", "時計回り", "cw"])
                is_left = any(x in part_norm for x in ["左", "hidari", "反時計回り", "ccw"])
                direction_sign = -1.0 if is_right else 1.0
                
                is_spin = any(x in part_norm for x in ["旋回", "回転", "spin"])
                
                if "rad" in part_norm or "ラジアン" in part_norm:
                    rad_val = val * direction_sign
                    if is_spin:
                        return {"type": "spin", "value": math.degrees(rad_val)}
                    else:
                        return {"type": "turn", "value": rad_val}
                else:
                    unit_str = angle_match.group(2) if len(angle_match.groups()) > 1 else ""
                    if unit_str in ["回転", "旋回"]:
                        deg_val = val * 360.0 * direction_sign
                        return {"type": "spin", "value": deg_val}
                    
                    deg_val = val * direction_sign
                    if is_spin:
                        return {"type": "spin", "value": deg_val}
                    else:
                        return {"type": "turn", "value": math.radians(deg_val)}

            # 5. Forward/Backward with numbers
            dist_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|メートル|cm|センチ)?", part_norm)
            if dist_match:
                val = float(dist_match.group(1))
                if "cm" in part_norm or "センチ" in part_norm:
                    val = val / 100.0
                    
                is_backward = any(x in part_norm for x in ["下", "後退", "sagatt", "usirosag", "ushirosag", "back", "reverse", "さがって", "後ろ", "うしろ", "ushiro", "backward"])
                is_forward = any(x in part_norm for x in ["前", "進", "mae", "forward", "straight", "まえ", "すす", "行っ"])
                
                if is_backward:
                    return {"type": "backward", "value": val}
                elif is_forward:
                    return {"type": "forward", "value": val}
            return None

        def parse_no_number_part(part_raw):
            part_norm = part_raw.strip().lower()
            
            # Compass face directions
            face_map = {
                "北": 90.0, "kita": 90.0,
                "東": 0.0, "higashi": 0.0,
                "南": -90.0, "minami": -90.0,
                "西": 180.0, "nishi": 180.0,
                "前": 0.0, "正面": 0.0, "mae": 0.0, "shoumen": 0.0
            }
            for d_name, angle in face_map.items():
                if d_name in part_norm and ("向" in part_norm or "向き" in part_norm or "むい" in part_norm or "むく" in part_norm or "face" in part_norm):
                    return {"type": "face", "value": angle}

            is_right = any(pat in part_norm for pat in ["右", "migi", "みぎ", "みぎむ"])
            is_left = any(pat in part_norm for pat in ["左", "hidari", "ひだり", "ひだりむ"])
            is_back = any(pat in part_norm for pat in ["後ろ", "うしろ", "ushiro", "裏", "うら"])
            
            if (is_right or is_left or is_back) and not any(x in part_norm for x in ["前", "進", "下", "後退", "sagatt", "usirosag", "ushirosag", "back", "mae", "すす"]):
                if is_back:
                    val = 3.14159
                else:
                    val = -1.5708 if is_right else 1.5708
                    if any(x in part_norm for x in ["少し", "ちょっと", "微", "すこし"]):
                        val = -0.5236 if is_right else 0.5236
                return {"type": "turn", "value": val}
                
            if any(x in part_norm for x in ["旋回", "回転", "senkai", "spin"]):
                deg = 360.0
                if "右" in part_norm or "時計回り" in part_norm:
                    deg = -360.0
                return {"type": "spin", "value": deg}

            if any(x in part_norm for x in ["下", "後退", "sagatt", "usirosag", "ushirosag", "back", "reverse", "さがって", "後ろ", "うしろ", "ushiro"]):
                val = 1.0
                if any(x in part_norm for x in ["motto", "もっと", "大きく", "たくさん", "さらに"]):
                    val = 2.0
                return {"type": "backward", "value": val}

            if any(x in part_norm for x in ["前", "進", "mae", "forward", "straight", "まえ", "すす", "行っ"]):
                val = 1.5
                if any(x in part_norm for x in ["少し", "ちょっと", "すこし"]):
                    val = 1.0
                elif any(x in part_norm for x in ["もっと", "大きく", "たくさん", "さらに"]):
                    val = 2.5
                return {"type": "forward", "value": val}
            return None

        def parse_any_part(part_raw):
            cmd = parse_part(part_raw)
            if cmd is None:
                cmd = parse_no_number_part(part_raw)
            return cmd

        # 1. 停止・キャンセルの判定
        cancel_patterns = ["止ま", "ストップ", "停止", "stop", "cancel", "キャンセル", "とまって", "待機", "だめ", "無理", "おわり"]
        if any(pat in norm_inst for pat in cancel_patterns):
            return {"commands": [], "cancel": True}
        
        # 2. バッテリー情報の判定
        if any(x in norm_inst for x in ["バッテリー", "ばってりー", "電池", "でんち"]):
            battery_msg = self.get_sirius_battery_level()
            self.send_sirius_speak(battery_msg)
            return {"commands": [], "cancel": False, "fast_path": True}
            
        # 3. 軽い会話（パースバージョン）の高速判定
        for kw, reply in CHAT_KEYWORDS.items():
            if kw in norm_inst:
                self.send_sirius_speak(reply)
                return {"commands": [], "cancel": False, "fast_path": True}

        # 4. 状況・エラーの確認クエリの判定
        status_queries = ["状況", "状態", "ステータス", "どうなってる", "何してる", "どこにいる"]
        error_queries = ["なんで失敗", "何で失敗", "なぜ失敗", "なぜ止まった", "なんで止まった", "何で止まった", "失敗した理由", "止まった理由"]
        is_status_query = any(q in norm_inst for q in status_queries)
        is_error_query = any(q in norm_inst for q in error_queries)
        
        if is_status_query or is_error_query:
            current_x, current_y, current_yaw_deg = 0.0, 0.0, 0.0
            try:
                trans = self.tf_buffer.lookup_transform('map', 'sirius3/base_footprint', rclpy.time.Time())
                current_x = trans.transform.translation.x
                current_y = trans.transform.translation.y
                q = trans.transform.rotation
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                current_yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
            except Exception:
                pass
                
            speak_msg = ""
            if is_status_query:
                with self.lock:
                    executing = self.executing_command
                    queue_len = len(self.command_queue)
                    stuck = self.is_stuck
                if stuck:
                    speak_msg = "[sad]今は障害物に遮られて立ち往生している状態なのだ。進路が塞がれているみたいなのだ。"
                elif executing:
                    speak_msg = f"[happy]今は目標地点に向かって移動中なのだ！キューにはあと{queue_len}個のアクションが残っているのだ。"
                else:
                    speak_msg = f"[happy]今は停止中で、次の指示を待っている状態なのだ！現在位置は X={current_x:.2f}, Y={current_y:.2f} なのだ。"
            else:
                with self.lock:
                    last_status = self.last_action_status
                    last_type = self.last_action_type
                if last_status == "failed_stuck":
                    speak_msg = "[sad]さっきは進もうとしたんだけど、行く手が障害物で遮られちゃって、これ以上進めなくて停止したのだ。"
                elif last_status in ["failed_cancelled", "cancelled"]:
                    speak_msg = "[happy]さっきはユーザー指示で動作を途中でキャンセルしたのだ。"
                elif last_type == "turn" and last_status == "failed":
                    speak_msg = "[sad]さっきは旋回しようとしたんだけど、目標の角度まで回りきれずに途中で止まっちゃったのだ。"
                else:
                    speak_msg = "[happy]直前のアクションは正常に完了しているか、まだエラーは発生していないのだ！"
            
            self.send_sirius_speak(speak_msg)
            return {"commands": [], "cancel": False, "fast_path": True, "speak": speak_msg}

        # 5.是正・訂正・相対調整のルールベース判定
        correction_patterns_map = {
            "行き過ぎ": "overshot", "いきすぎ": "overshot", "回りすぎ": "overshot", "まわりすぎ": "overshot",
            "進みすぎ": "overshot", "すすみすぎ": "overshot", "下がりすぎ": "overshot", "さがりすぎ": "overshot",
            "動きすぎ": "overshot", "うごきすぎ": "overshot", "すぎだ": "overshot", "すぎよ": "overshot",
            "逆": "opposite", "ぎゃく": "opposite", "反対": "opposite", "はんたい": "opposite",
            "足りない": "more", "たりない": "more", "不足": "more", "ふそく": "more",
            "戻って": "goback", "もどって": "goback", "やり直し": "goback", "やりなおし": "goback"
        }
        correction_type = None
        for kw, c_t in correction_patterns_map.items():
            if kw in norm_inst:
                correction_type = c_t
                break
                
        if correction_type is not None:
            with self.lock:
                last_type = self.last_action_type
                last_target = self.last_target_value
                start_x = self.last_action_start_x
                start_y = self.last_action_start_y
                start_yaw = self.last_action_start_yaw
                
            cmd_list = []
            speak_msg = ""
            
            if correction_type == "overshot":
                if last_type == "forward":
                    val = max(0.3, last_target * 0.3 if isinstance(last_target, (int, float)) else 0.5)
                    cmd_list.append({"type": "backward", "value": val})
                    speak_msg = f"[sad]ごめんなさい、ちょっと進みすぎちゃったのだ！{val:.1f}メートル下がるのだ。"
                elif last_type == "backward":
                    val = max(0.3, last_target * 0.3 if isinstance(last_target, (int, float)) else 0.5)
                    cmd_list.append({"type": "forward", "value": val})
                    speak_msg = f"[sad]ごめんなさい、ちょっと下がりすぎちゃったのだ！{val:.1f}メートル進むのだ。"
                elif last_type in ["turn", "spin"]:
                    val = -last_target * 0.3 if isinstance(last_target, (int, float)) else -0.5
                    cmd_list.append({"type": last_type, "value": val})
                    dir_str = "右" if val < 0 else "左"
                    speak_msg = f"[sad]ごめんなさい、ちょっと回りすぎちゃったのだ！少し{dir_str}に戻るのだ。"
                    
            elif correction_type == "opposite":
                if last_type == "forward":
                    cmd_list.append({"type": "backward", "value": last_target if isinstance(last_target, (int, float)) else 1.0})
                    speak_msg = "[surprised]あ、逆向きだったのだ！反対側に進むのだ。"
                elif last_type == "backward":
                    cmd_list.append({"type": "forward", "value": last_target if isinstance(last_target, (int, float)) else 1.0})
                    speak_msg = "[surprised]あ、逆向きだったのだ！反対側に進むのだ。"
                elif last_type in ["turn", "spin"]:
                    val = -last_target if isinstance(last_target, (int, float)) else -1.57
                    cmd_list.append({"type": last_type, "value": val})
                    speak_msg = "[surprised]あ、逆向きだったのだ！反対側を向くのだ。"
                    
            elif correction_type == "more":
                if last_type in ["forward", "backward"]:
                    val = max(0.5, last_target * 0.5 if isinstance(last_target, (int, float)) else 1.0)
                    cmd_list.append({"type": last_type, "value": val})
                    speak_msg = f"[happy]もう少し動かすのだ！追加で {val:.1f}メートル移動するのだ。"
                elif last_type in ["turn", "spin"]:
                    val = last_target * 0.5 if isinstance(last_target, (int, float)) else 0.5
                    cmd_list.append({"type": last_type, "value": val})
                    speak_msg = "[happy]もう少し回るのだ！追加で旋回するのだ。"
                    
            elif correction_type == "goback":
                if last_type in ["forward", "backward", "goto"]:
                    cmd_list.append({"type": "goto", "value": [start_x, start_y]})
                    cmd_list.append({"type": "face", "value": math.degrees(start_yaw)})
                    speak_msg = "[happy]元いた場所に戻るのだ！"
                elif last_type in ["turn", "spin"]:
                    cmd_list.append({"type": "face", "value": math.degrees(start_yaw)})
                    speak_msg = "[happy]元の向きに戻るのだ！"
                    
            if cmd_list:
                self.send_sirius_speak(speak_msg)
                return {"commands": cmd_list, "cancel": False, "speak": speak_msg, "fast_path": True}

        # 6. 四角・正方形などの図形描画
        shape_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|メートル)?の(?:四角|正方形|スクエア)", norm_inst)
        if shape_match:
            size = float(shape_match.group(1))
            cmds = []
            for _ in range(4):
                cmds.append({"type": "forward", "value": size})
                cmds.append({"type": "turn", "value": -1.5708})
            return {"commands": cmds, "cancel": False, "speak": f"[happy]一辺{size}メートルの正方形を描くように移動するのだ！"}

        # 7. 繰り返し（ループ）
        loop_match = re.search(r"(\d+)\s*(?:回|回繰り返して|回繰り返し)", norm_inst)
        if loop_match:
            times = int(loop_match.group(1))
            base_part = re.sub(r"(?:のを)?\d+\s*(?:回|回繰り返して|回繰り返し).*", "", norm_inst).strip()
            base_cmds = []
            parts = [p.strip() for p in re.split(r"して|て|、|そして", base_part) if p.strip()]
            for part in parts:
                parsed_cmd = parse_any_part(part)
                if parsed_cmd:
                    base_cmds.append(parsed_cmd)
            if base_cmds:
                cmds = base_cmds * times
                return {"commands": cmds, "cancel": False, "speak": f"[happy]指示された動きを{times}回繰り返すのだ！"}

        # 8. 一般的な数値指定コマンドのパース（複数コマンドの連結に対応）
        # 例：「3m進んで右に90度曲がる」など
        has_number = any(char.isdigit() for char in norm_inst) or any(x in norm_inst for x in ["一", "二", "三", "四", "五", "六", "七", "八", "九", "十", "点", "度", "㍍", "メートル"])
        if has_number:
            parts = [p.strip() for p in re.split(r"して|て|、|そして", norm_inst) if p.strip()]
            parsed_cmds = []
            for part in parts:
                cmd = parse_any_part(part)
                if cmd:
                    parsed_cmds.append(cmd)
            if parsed_cmds:
                return {"commands": parsed_cmds, "cancel": False}

        # 9. 数値指定を含まない標準的な指示（簡易前進・後退・旋回・スピード変更）
        # (元々の1.5番目のロジックとほぼ同様)
        slow_patterns = ["ゆっくり", "遅く", "おそく", "おそい", "遅い", "スピード下げ", "スピード落と", "速度下げ", "yukkuri", "slow", "下げて", "スピードおと"]
        normal_patterns = ["ふつう", "普通", "通常", "normal"]
        fast_patterns = ["早く", "急いで", "スピード上げ", "速度上げ", "fast", "speedup", "上げて"]
        
        speed_val = None
        if any(pat in norm_inst for pat in slow_patterns):
            speed_val = 0.2
        elif any(pat in norm_inst for pat in normal_patterns):
            speed_val = 0.9
        elif any(pat in norm_inst for pat in fast_patterns):
            speed_val = 1.0

        cmd_list = []
        if speed_val is not None:
            cmd_list.append({"type": "speed", "value": speed_val})
            
        # Try to parse standard direction/orientation/movement commands without numbers using the helper
        no_num_cmd = parse_no_number_part(norm_inst)
        if no_num_cmd:
            cmd_list.append(no_num_cmd)
            return {"commands": cmd_list, "cancel": False}

        # Fallback for continuing/relative adjustments without direct directional keywords (e.g. "もっと", "すこし")
        has_dir = any(x in norm_inst for x in ["前", "進", "下", "後退", "sagatt", "usirosag", "ushirosag", "back", "mae", "右", "左", "migi", "hidari", "旋回", "回転", "senkai", "spin", "goto", "座標", "すす", "行っ", "さが", "下が", "さがっ", "まわ", "回っ", "むい", "後ろ", "うしろ", "ushiro", "裏", "うら"])
        if has_dir:
            last_cmd_was_backward = False
            last_cmd_was_forward = False
            with self.lock:
                if self.chat_history:
                    for msg in reversed(self.chat_history):
                        if msg.get("role") == "assistant":
                            try:
                                hist_json = json.loads(msg.get("content", "{}"))
                                hist_cmds = hist_json.get("commands", [])
                                if hist_cmds:
                                    last_type = hist_cmds[-1].get("type")
                                    if last_type == "backward":
                                        last_cmd_was_backward = True
                                        break
                                    elif last_type == "forward":
                                        last_cmd_was_forward = True
                                        break
                            except Exception:
                                pass
            
            if norm_inst in ["motto", "もっと", "もうすこし", "もう少し", "もうちょっと", "さらに", "すこし", "少し", "ちょっと"]:
                val = 1.0 if any(x in norm_inst for x in ["すこし", "少し", "ちょっと"]) else 2.0
                if last_cmd_was_backward:
                    cmd_list.append({"type": "backward", "value": max(1.0, val)})
                    return {"commands": cmd_list, "cancel": False}
                elif last_cmd_was_forward:
                    cmd_list.append({"type": "forward", "value": max(1.0, val)})
                    return {"commands": cmd_list, "cancel": False}
        else:
            if speed_val is not None:
                return {"commands": cmd_list, "cancel": False}

            
        # -------------------------------------------------------------------
        # ルールベースで一致しなかった複雑な指示は LLM (LM Studio) へ問い合わせる
        # -------------------------------------------------------------------
        system_prompt = (
            "You are a robotic navigator assistant. Your task is to translate natural language directions "
            "into a sequence of motion commands for the robot, or to answer user questions about your status, history, or errors.\n\n"
            "CRITICAL: Do not explain your reasoning. Do not output any reasoning steps, thoughts, or chain-of-thought content. "
            "You MUST output raw JSON and ONLY raw JSON directly and immediately. Do not include markdown code block syntax (like ```json).\n"
            "Output format:\n"
            "{\n"
            "  \"commands\": [\n"
            "    {\"type\": \"forward\"|\"backward\"|\"turn\"|\"spin\"|\"face\"|\"goto\"|\"speed\", \"value\": float | [float, float]}\n"
            "  ],\n"
            "  \"cancel\": boolean,\n"
            "  \"speak\": \"optional string in Japanese (if user asks a question, complaints about action, or asks why a failure/cancellation occurred, use this field to explain why using the physical state parameters)\"\n"
            "}\n\n"
            "【Commands Definition】\n"
            "- \"type\": \"forward\": Move straight forward. \"value\": distance in meters (positive float).\n"
            "  Default distance if unspecified: 1.5m. 'ちょっと'/'少し': 0.5m. '大きく'/'たくさん': 2.5m. Minimum distance is 0.3m.\n"
            "  Also maps to Romaji Japanese: 'mae'/'susunde'/'maeitte'/'susume'.\n"
            "- \"type\": \"backward\": Move straight backward. \"value\": distance in meters (positive float).\n"
            "  Default distance if unspecified: 1.0m. Minimum distance is 0.3m.\n"
            "  Also maps to Romaji Japanese: 'back'/'sagatte'/'usirosagatte'/'ushiro sagatte'.\n"
            "- \"type\": \"turn\": Rotate to face a relative angle. \"value\": angle in radians. "
            "POSITIVE=left(CCW), NEGATIVE=right(CW). Use for '右向いて'(-1.5708), '左向いて'(+1.5708), '少し右'(-0.5236).\n"
            "  Also maps to Romaji Japanese: 'migimuite'/'migi muite'(-1.5708), 'hidarimuite'/'hidari muite'(+1.5708).\n"
            "- \"type\": \"spin\": Rotate in place by specified degrees without translating. "
            "\"value\": total rotation angle in DEGREES. POSITIVE=left(CCW), NEGATIVE=right(CW), 0=full 360deg spin.\n"
            "  Use for: 'その場旋回'(0), '一回転'(360), '右に半回転'(-180), '時計回りに90度'(-90), 'sonobasenkai'(360).\n"
            "- \"type\": \"face\": Turn to face an absolute map direction. "
            "\"value\": target yaw angle in DEGREES in map frame (0=+X axis, 90=+Y axis, 180/-180=-X axis, -90=-Y axis).\n"
            "  Use when user says a compass direction like '北を向いて'(90), '南向き'(-90), '東向き'(0), '西向き'(180).\n"
            "- \"type\": \"goto\": Navigate to absolute map coordinates. \"value\": [x_coord, y_coord] (array of two floats).\n"
            "- \"type\": \"speed\": Change the robot movement speed. \"value\": speed factor or absolute speed in m/s "
            "(mapped to slow [0.20], safe [0.40], normal [0.90], fast [1.00]).\n\n"
            "【Conversational Context & Embodied Feedback Rules】\n"
            "You will receive feedback from both the user's conversation AND the robot's physical sensors "
            "(the system message labeled 【Robot Current Hardware State Feedback】).\n"
            "1. If the user complains about the last action (e.g. '全然進んでいない' / 'Not moving at all'), "
            "look at the robot's physical state. If velocity=0.0m/s with BLOCKED status, "
            "generate a corrective sequence (detour turn or reverse).\n"
            "2. If a command value was too small and robot arrived safely, amplify the value in the next command.\n"
            "3. CRITICAL ON CONTEXT DIRECTION: If the user says 'motto' (more) or 'mottosagatte' / 'motto sagatte' after a 'backward' command, "
            "you MUST output a 'backward' command (e.g. value: 1.0). Do not output 'forward' unless they explicitly ask to go forward (e.g. 'mae'/'susunde').\n"
            "4. LOOP UNROLLING: If the user asks to repeat an action sequence (e.g., 'X回繰り返して' / 'repeat X times'), you MUST fully unroll/expand the sequence into individual commands in the JSON list.\n"
            "5. SHAPES (e.g. Squares/四角/正方形): If the user asks to draw or move in a square of size X (e.g., 'Xm of square' / 'square of X meters'), unroll it into 8 commands: forward Xm, turn right (-1.5708), forward Xm, turn right, forward Xm, turn right, forward Xm, turn right.\n"
            "7. USER CORRECTION & RECALIBRATION: If the user complains about the last action (e.g. 'ちょっと行き過ぎ', '回りすぎ', '逆だよ', 'もっと右', '足りない', or complaining '後ろ向いてって言ってるのに1周した' / '右って言ったのに左に回った'), inspect the `Last Action Status` (type, target value) and the dialogue history. Generate a corrective/compensatory command. Crucially, if the last action was a rotation (turn, spin, or face) and the user says '行き過ぎ' (overshot) or '回りすぎ', you MUST output a turn in the OPPOSITE direction (e.g., if last action was a turn of 1.5708 and user says '行き過ぎ', output a turn of -0.5236). Do NOT output forward or backward linear translation commands if the last action was a rotation.\n"
            "8. CURRENT STATUS / GENERAL SITUATION QUESTIONS: If the user asks about the current status, situation, or what the robot is doing (e.g. '今の状況は？', 'どういう状態？', '今のステータスを教えて', 'どういう状況'), you MUST explain the current state in Japanese in the 'speak' field, using the details from 【Robot Current Hardware State Feedback】 (e.g., whether the robot is Idle/Ready or Executing, the current target coordinate, remaining distance, or obstacle blockage). Do not return any commands unless they explicitly command a new movement.\n"
            "9. UNRELATED / GENERAL CONVERSATION: If the user says something unrelated to robot control or navigation status (e.g., general chatter, joking, questions about weather, food, personal comments like '頭使って', '今日の天気は？'), do NOT generate any commands (set 'commands' to `[]`) and do NOT set cancel. Instead, respond naturally/politely in Japanese in the 'speak' field as a friendly navigator assistant.\n\n"
            "【Output Examples】\n"
            "- 'ちょっと前に行って' -> {\"commands\": [{\"type\": \"forward\", \"value\": 0.5}], \"cancel\": false}\n"
            "- 'ちょっと行き過ぎ' (when last action was backward 1.0) -> {\"commands\": [{\"type\": \"forward\", \"value\": 0.5}], \"cancel\": false, \"speak\": \"[sad]ごめんなさい、ちょっと下がりすぎちゃったのだ！少し前に戻るのだ。\"}\n"
            "- '行き過ぎ' (when last action was turn with value 1.5708) -> {\"commands\": [{\"type\": \"turn\", \"value\": -0.5236}], \"cancel\": false, \"speak\": \"[sad]ごめんなさい、ちょっと左に回りすぎちゃったのだ！少し右に戻るのだ。\"}\n"
            "- '逆だよ' (when last action was turn with value -1.5708) -> {\"commands\": [{\"type\": \"turn\", \"value\": 1.5708}], \"cancel\": false, \"speak\": \"[surprised]あ、逆向きだったのだ！反対側を向くのだ。\"}\n"
            "- '後ろ向いてって言ってるのに1周した' (when last action was spin 360) -> {\"commands\": [{\"type\": \"turn\", \"value\": 3.1415}], \"cancel\": false, \"speak\": \"[sad]ごめんなさい！一周するんじゃなくて、後ろを向くのだったのだ。半周（180度）回って後ろを向くのだ。\"}\n"
            "- '今の状況は？' (when Status is Idle/Ready) -> {\"commands\": [], \"cancel\": false, \"speak\": \"[happy]今は停止中で、次の指示を待っている状態なのだ！\"}\n"
            "- '今の状況を教えて' (when Status is Executing Sequence and remaining distance is 1.2 meters) -> {\"commands\": [], \"cancel\": false, \"speak\": \"[happy]今は目標地点に向かって移動中なのだ！目的地まではあと1.2メートルほどなのだ。\"}\n"
            "- 'どういう状況' (when obstacle blocks the way) -> {\"commands\": [], \"cancel\": false, \"speak\": \"[sad]今は障害物に阻まれて立ち往生している状態なのだ。進路が塞がれているみたいなのだ。\"}\n"
            "- '今日の天気は？' -> {\"commands\": [], \"cancel\": false, \"speak\": \"[sad]ごめんなさい、私はナビゲーション用のアシスタントなので、お天気の情報は持っていないのだ。お外が見たいのだ！\"}\n"
            "- 'システムの説明をして' -> {\"commands\": [], \"cancel\": false, \"speak\": \"[happy]ボクは音声やテキストの指示を理解して自律移動するナビゲーションシステムなのだ！「3m前に行って」や「右向いて」のように話しかけてほしいのだ！\"}\n"
            "- '何ができるの？' -> {\"commands\": [], \"cancel\": false, \"speak\": \"[happy]ボクは「前に進んで」や「右を向いて」といった移動の指示を理解して動くことができるのだ！今の状態やエラーの説明もできるのだ！\"}\n"
            "- '頭使って' -> {\"commands\": [], \"cancel\": false, \"speak\": \"[happy]うう、もっと頭を柔らかくして賢く考えられるように頑張るのだ！\"}\n"
            "- 'なんで失敗したの？' (when last action was turn with error) -> {\"commands\": [], \"cancel\": false, \"speak\": \"[sad]さっきは旋回しようとしたんだけど、目標の角度まで回りきれずに途中で止まっちゃったのだ。\"}\n"
            "- 'なんで止まった？' (when last action was forward and got stuck) -> {\"commands\": [], \"cancel\": false, \"speak\": \"[sad]進もうとしたんだけど、行く手が遮られちゃって、これ以上進めなかったのだ。\"}\n"
            "- '1.5m後退して' -> {\"commands\": [{\"type\": \"backward\", \"value\": 1.5}], \"cancel\": false}\n"
            "- '右向いて' -> {\"commands\": [{\"type\": \"turn\", \"value\": -1.5708}], \"cancel\": false}\n"
            "- 'usirosagatte' -> {\"commands\": [{\"type\": \"backward\", \"value\": 0.5}], \"cancel\": false}\n"
            "- 'motto' (when last command was backward) -> {\"commands\": [{\"type\": \"backward\", \"value\": 1.0}], \"cancel\": false}\n"
            "- 'mottosagatte' -> {\"commands\": [{\"type\": \"backward\", \"value\": 1.5}], \"cancel\": false}\n"
            "- 'mae itte' -> {\"commands\": [{\"type\": \"forward\", \"value\": 1.0}], \"cancel\": false}\n"
            "- 'migimuite' -> {\"commands\": [{\"type\": \"turn\", \"value\": -1.5708}], \"cancel\": false}\n"
            "- 'sonobasenkai' -> {\"commands\": [{\"type\": \"spin\", \"value\": 360}], \"cancel\": false}\n"
            "- '少し左に向いて' -> {\"commands\": [{\"type\": \"turn\", \"value\": 0.5236}], \"cancel\": false}\n"
            "- 'その場で旋回して' -> {\"commands\": [{\"type\": \"spin\", \"value\": 360}], \"cancel\": false}\n"
            "- '右に一回転' -> {\"commands\": [{\"type\": \"spin\", \"value\": -360}], \"cancel\": false}\n"
            "- '時計回りに半回転' -> {\"commands\": [{\"type\": \"spin\", \"value\": -180}], \"cancel\": false}\n"
            "- '北向きになって' -> {\"commands\": [{\"type\": \"face\", \"value\": 90}], \"cancel\": false}\n"
            "- '東を向いて' -> {\"commands\": [{\"type\": \"face\", \"value\": 0}], \"cancel\": false}\n"
            "- '座標(1.5, 2.0)に向かって' -> {\"commands\": [{\"type\": \"goto\", \"value\": [1.5, 2.0]}], \"cancel\": false}\n"
            "- '3m前に行って左に曲がって' -> {\"commands\": [{\"type\": \"forward\", \"value\": 3.0}, {\"type\": \"turn\", \"value\": 1.5708}], \"cancel\": false}\n"
            "- '旋回しながら前進して' -> {\"commands\": [{\"type\": \"spin\", \"value\": 360}, {\"type\": \"forward\", \"value\": 1.0}], \"cancel\": false}\n"
            "- 'ゆっくり3mすすんで' -> {\"commands\": [{\"type\": \"speed\", \"value\": 0.20}, {\"type\": \"forward\", \"value\": 3.0}], \"cancel\": false}\n"
            "- '0.50m/s of speed and go forward 3m' -> {\"commands\": [{\"type\": \"speed\", \"value\": 0.50}, {\"type\": \"forward\", \"value\": 3.0}], \"cancel\": false}\n"
            "- 'スピード上げて' -> {\"commands\": [{\"type\": \"speed\", \"value\": 1.00}], \"cancel\": false}\n"
            "- '速度を普通に戻して' -> {\"commands\": [{\"type\": \"speed\", \"value\": 0.90}], \"cancel\": false}\n"
            "- '4mの四角を描いて' -> {\"commands\": [{\"type\": \"forward\", \"value\": 4.0}, {\"type\": \"turn\", \"value\": -1.5708}, {\"type\": \"forward\", \"value\": 4.0}, {\"type\": \"turn\", \"value\": -1.5708}, {\"type\": \"forward\", \"value\": 4.0}, {\"type\": \"turn\", \"value\": -1.5708}, {\"type\": \"forward\", \"value\": 4.0}, {\"type\": \"turn\", \"value\": -1.5708}], \"cancel\": false}\n"
            "- '1m進んで右に曲がるのを4回繰り返して' -> {\"commands\": [{\"type\": \"forward\", \"value\": 1.0}, {\"type\": \"turn\", \"value\": -1.5708}, {\"type\": \"forward\", \"value\": 1.0}, {\"type\": \"turn\", \"value\": -1.5708}, {\"type\": \"forward\", \"value\": 1.0}, {\"type\": \"turn\", \"value\": -1.5708}, {\"type\": \"forward\", \"value\": 1.0}, {\"type\": \"turn\", \"value\": -1.5708}], \"cancel\": false}\n"
            "- '危ない、止まって！' -> {\"commands\": [], \"cancel\": true}"
        )
        
        headers = {
            'Content-Type': 'application/json',
        }
        
        # 物理状態フィードバックの取得
        state_context = self.get_robot_state_context_string()
        
        # メッセージリストを構築（システムプロンプト + 過去の会話履歴 + リアルタイム状態 + 現在の指示）
        with self.lock:
            if len(self.chat_history) > self.history_max_turns:
                self.chat_history = self.chat_history[-self.history_max_turns:]
            
            messages = [{"role": "system", "content": system_prompt}]
            messages.extend(self.chat_history)
            
            # 最新のユーザー指示の直前にロボットの物理状態フィードバックをコンテキストとして挿入
            messages.append({"role": "system", "content": state_context})
            messages.append({"role": "user", "content": instruction})
        
        payload = {
            "model": self.model_name,
            "messages": messages,
            "temperature": 0.0,
            "max_tokens": 1000
        }
        
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
                content = res_json['choices'][0]['message']['content'].strip()
                
                if content.startswith("```"):
                    lines = content.splitlines()
                    if len(lines) >= 3:
                        content = "\n".join(lines[1:-1])
                        
                parsed_json = json.loads(content)
                self.get_logger().info(f"LLM Raw Output: '{content}'")
                
                # 正常にパースできたら履歴に追加
                with self.lock:
                    self.chat_history.append({"role": "user", "content": instruction})
                    self.chat_history.append({"role": "assistant", "content": content})
                    
                return parsed_json
        except urllib.error.URLError as e:
            self.get_logger().error(f"LM Studio API connection failed: {e}")
            self.get_logger().error("Make sure LM Studio is running on port 1234.")
        except Exception as e:
            self.get_logger().error(f"Error calling LLM: {e}")
            if 'res_body' in locals():
                self.get_logger().error(f"Raw Response: {res_body}")
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
        """10Hzで実行され、旋回中であれば目標角度との差分を計算し、8度以下で自動キャンセル（到達判定）する"""
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
            
            # 安全のためロックの外側でキャンセルを実行
            self.send_sirius_speak(DIALOGUE_TEMPLATES["stuck"])
            self.cancel_navigation()
            
            print(f"\n⚠️ {stuck_msg.split(']')[-1].strip()} 目標をキャンセルして停止します。")
            print("Command > ", end="", flush=True)
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
                print("\n🎉 全ての目標地点に到着しました！次の指示をどうぞ。")
                print("Command > ", end="", flush=True)

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

    def cancel_navigation(self, clear_queue=True):
        """実行中のナビゲーションをキャンセル"""
        self.delete_marker()
        
        # Spinアクションが動いていればキャンセル
        with self.lock:
            spin_handle = self.spin_goal_handle
            self.spin_goal_handle = None
            
        if spin_handle is not None:
            self.get_logger().info("Canceling active Spin action...")
            spin_handle.cancel_goal_async()
            
        with self.lock:
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
            
        if has_more:
            self.execute_next_command()
        else:
            self.set_node_parameters('/controller_server', {'general_goal_checker.xy_goal_tolerance': 0.50})
            if success:
                self.send_sirius_speak(DIALOGUE_TEMPLATES["turn_success"])
            else:
                self.send_sirius_speak(DIALOGUE_TEMPLATES["turn_failure"])
            print("\n✅ 旋回完了！次の指示をどうぞ。")
            print("Command > ", end="", flush=True)

    def send_sirius_speak(self, text):
        """sirius_face_anim2 の gRPC サーバーに音声を送信して喋らせる"""
        try:
            import os
            import sys
            home_dir = os.path.expanduser("~")
            venv_packages = os.path.join(home_dir, "sirius_face_anim2/venv/lib/python3.12/site-packages")
            if venv_packages not in sys.path:
                sys.path.insert(0, venv_packages)
            stubs_path = os.path.join(home_dir, "sirius_face_anim2/scripts/stubs")
            if stubs_path not in sys.path:
                sys.path.insert(0, stubs_path)
            
            import grpc
            import face_control_pb2
            import face_control_pb2_grpc
            
            # 発話開始に伴い、思考フラグおよびパース制御（黄色の四隅）のフラグをリセットする
            try:
                with grpc.insecure_channel('localhost:50051') as face_channel:
                    face_stub = face_control_pb2_grpc.FaceServiceStub(face_channel)
                    face_stub.UpdateParameters(face_control_pb2.ParameterRequest(values={
                        "isThinking": 0.0,
                        "isParseControl": 0.0
                    }), timeout=1.0)
            except Exception as fe:
                self.get_logger().warn(f"Could not reset face indicator flags: {fe}")

            # タイムアウト付きでチャネルを作成
            with grpc.insecure_channel('localhost:50052') as channel:
                stub = face_control_pb2_grpc.PythonControlServiceStub(channel)
                req = face_control_pb2.SpeakRequest(text=text)
                stub.Speak(req, timeout=3.0)
                self.get_logger().info(f"Sent Speak command: {text}")
        except Exception as e:
            self.get_logger().error(f"Failed to send speak to Sirius face anim server: {e}")

    def get_sirius_battery_level(self):
        """FaceService (port 50051) から現在のバッテリー状況を取得して報告用テキストを作成する"""
        try:
            import os
            import sys
            home_dir = os.path.expanduser("~")
            venv_packages = os.path.join(home_dir, "sirius_face_anim2/venv/lib/python3.12/site-packages")
            if venv_packages not in sys.path:
                sys.path.insert(0, venv_packages)
            stubs_path = os.path.join(home_dir, "sirius_face_anim2/scripts/stubs")
            if stubs_path not in sys.path:
                sys.path.insert(0, stubs_path)
            
            import grpc
            import face_control_pb2
            import face_control_pb2_grpc
            from google.protobuf import empty_pb2
            
            # タイムアウト付きでチャネルを作成
            with grpc.insecure_channel('localhost:50051') as channel:
                stub = face_control_pb2_grpc.FaceServiceStub(channel)
                status = stub.GetStatus(empty_pb2.Empty(), timeout=3.0)
                level = status.current_parameters.get("batteryLevel", -1.0)
                charging_val = status.current_parameters.get("batteryCharging", 0.0)
                
                charging_str = "未充電"
                if charging_val == 1.0:
                    charging_str = "充電中"
                elif charging_val == 2.0:
                    charging_str = "放電中（使用中）"
                
                if level >= 0.0:
                    return DIALOGUE_TEMPLATES["battery_report"].format(level=level, charging_str=charging_str)
                else:
                    return DIALOGUE_TEMPLATES["battery_error"]
        except Exception as e:
            self.get_logger().error(f"Failed to query battery status: {e}")
            return DIALOGUE_TEMPLATES["battery_fail"]

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
