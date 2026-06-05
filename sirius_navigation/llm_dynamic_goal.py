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
        
        # Turn (旋回) コマンドのアーリーキャンセル用ターゲットyaw
        # None以外のとき: 現在実行中コマンドはturnで、このyaw角に近づいたらキャンセル
        self.turn_target_yaw = None
        self.turn_remaining_angle = None
        self.last_yaw_robot = 0.0
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
        
        # 会話履歴管理 (マルチターン対話用)
        self.chat_history = []
        self.history_max_turns = 10  # 最大5往復分
        
        # 1Hzで自己位置と目標位置の距離を監視するタイマー
        self.goal_monitor_timer = self.create_timer(1.0, self.monitor_goal_distance)
        
        # 10Hz (0.1秒周期)で旋回中の角度監視を行うタイマー（preemptionを防ぐため再発行は行わない）
        self.dynamic_goal_timer = self.create_timer(0.1, self.timer_goal_publisher)
        
        # 対話型コマンドライン入力を別スレッドで開始
        self.running = True
        self.input_thread = threading.Thread(target=self.interactive_input_loop, daemon=True)
        self.input_thread.start()

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
            return
            
        is_cancel = result.get("cancel", False)
        if is_cancel:
            self.get_logger().warning("LLM interpreted instruction as STOP/CANCEL.")
            self.cancel_navigation()
            return
            
        commands = result.get("commands", [])
        if not commands:
            self.get_logger().warning("No commands generated from LLM.")
            return
            
        self.get_logger().info(f"Parsed {len(commands)} commands from instruction.")
        
        with self.lock:
            # 既存のキューをクリアして新しい指示シーケンスを設定
            self.command_queue = commands
            self.executing_command = False
            
        self.execute_next_command()

    def execute_next_command(self):
        """キューから次のコマンドを取り出して実行する"""
        with self.lock:
            if not self.command_queue:
                self.executing_command = False
                self.get_logger().info("All commands in sequence finished.")
                return
            cmd = self.command_queue.pop(0)
            self.executing_command = True
            self.command_start_time = self.get_clock().now()
            
        cmd_type = cmd.get("type", "forward")
        value = cmd.get("value", 0.0)
        
        self.get_logger().info(f"Executing next sequence command -> type: {cmd_type}, value: {value}")
        
        if cmd_type == "speed":
            try:
                speed_factor = float(value)
                self.set_controller_speed(speed_factor)
                print(f"⚡ 速度パラメータを変更しました (factor: {speed_factor:.2f})")
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
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup TF (map -> sirius3/base_footprint) during execution: {e}")
            self.execute_next_command()
            return

        r = 0.0
        theta = 0.0
        
        if cmd_type == "forward":
            r = value
            theta = 0.0
            self.publish_goal_pose(tx, ty, yaw_robot, r, theta)
            
        elif cmd_type == "backward":
            # 後退時: keyboard_dynamic_goal.py に合わせ、theta=180度(pi)として後方に移動させる
            # これによりNav2が前方向のセンサを使って安全に目的地（後方）へアプローチできるようになる
            r = abs(value)
            theta = math.pi
            self.publish_goal_pose(tx, ty, yaw_robot, r, theta)
            
        elif cmd_type == "turn":
            # 旋回時: Nav2の標準ビヘイビアである /spin アクションを使用し、
            # 地図の干渉やダミーゴール座標の衝突を避け、確実かつ滑らかにその場旋回を実行する
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
            
        if goal_x is None:
            return "【Robot Current Hardware State Feedback】: Status=Idle. No active navigation goal. Robot is stationary."
            
        # 現在位置と目標位置の差分（残差）を計算
        try:
            trans = self.tf_buffer.lookup_transform('map', 'sirius3/base_footprint', rclpy.time.Time())
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            distance = math.sqrt((goal_x - tx)**2 + (goal_y - ty)**2)
        except Exception:
            distance = -1.0
            
        stuck_str = "BLOCKED / STUCK (Robot is command-active but linear velocity is zero. An obstacle likely blocks the way.)" if stuck else "Moving normally towards target"
        vel_str = f"linear={vel_x:.2f}m/s, angular={vel_theta:.2f}rad/s"
        
        state_str = (
            f"【Robot Current Hardware State Feedback】\n"
            f"- Status: {'Executing Sequence' if executing else 'Idle/Ready'}\n"
            f"- Target Waypoint: X={goal_x:.2f}, Y={goal_y:.2f}, Yaw={math.degrees(goal_yaw):+.1f}deg\n"
            f"- Distance remaining to target: {distance:.2f} meters\n"
            f"- Current velocities: {vel_str}\n"
            f"- Physical obstacles / blockage state: {stuck_str}\n"
            f"- Actions left in sequence queue: {queue_len} commands"
        )
        return state_str

    def query_lm_studio(self, instruction):
        """LM StudioのOpenAI互換APIを呼び出し、会話履歴と現在の物理状況（状態フィードバック）を含めて指示を変換する"""
        
        norm_inst = instruction.strip().replace(" ", "").replace("　", "").lower()
        
        # 1. 停止・キャンセルの判定
        cancel_patterns = ["止ま", "ストップ", "停止", "stop", "cancel", "キャンセル", "とまって", "待機", "だめ", "無理", "おわり"]
        if any(pat in norm_inst for pat in cancel_patterns):
            return {"commands": [], "cancel": True}
            
        # 1.5 スピード調整/旋回/前進/後退の判定 (数値指定を含まない標準指示の場合にのみ高速応答)
        has_number = any(char.isdigit() for char in norm_inst) or any(x in norm_inst for x in ["一", "二", "三", "四", "五", "六", "七", "八", "九", "十", "点", "度", "㍍", "メートル"])
        
        if not has_number:
            slow_patterns = ["ゆっくり", "遅く", "スピード下げ", "スピード落と", "速度下げ", "yukkuri", "slow", "下げて", "スピードおと"]
            normal_patterns = ["ふつう", "普通", "通常", "normal"]
            fast_patterns = ["早く", "急いで", "スピード上げ", "速度上げ", "fast", "speedup", "上げて"]
            
            has_dir = any(x in norm_inst for x in ["前", "進", "下", "後退", "sagatt", "usirosag", "ushirosag", "back", "mae", "右", "左", "migi", "hidari", "旋回", "回転", "senkai", "spin", "goto", "座標", "すす", "いっ", "行っ", "さが", "下が", "さがっ", "まわ", "回っ", "むい"])
            
            if not has_dir:
                if any(pat in norm_inst for pat in slow_patterns):
                    return {"commands": [{"type": "speed", "value": 0.2}], "cancel": False}
                if any(pat in norm_inst for pat in normal_patterns):
                    return {"commands": [{"type": "speed", "value": 0.9}], "cancel": False}
                if any(pat in norm_inst for pat in fast_patterns):
                    return {"commands": [{"type": "speed", "value": 1.0}], "cancel": False}
                
            # 2. 旋回・右左折の判定
            is_right = any(pat in norm_inst for pat in ["右", "migi", "みぎ", "みぎむ"])
            is_left = any(pat in norm_inst for pat in ["左", "hidari", "ひだり", "ひだりむ"])
            
            # 「右向いて」「左向いて」「みぎむいて」「ひだりむいて」「みぎいて」「ひだりいて」「右向けて」「右無知恵」（タイポ対応）
            if (is_right or is_left) and not any(x in norm_inst for x in ["前", "進", "下", "後退", "sagatt", "usirosag", "ushirosag", "back", "mae"]):
                val = -1.5708 if is_right else 1.5708
                if "少し" in norm_inst or "ちょっと" in norm_inst or "微" in norm_inst:
                    val = -0.5236 if is_right else 0.5236
                return {"commands": [{"type": "turn", "value": val}], "cancel": False}
                
            # その場旋回, 一回転, 360度
            if any(x in norm_inst for x in ["旋回", "回転", "senkai", "spin"]):
                deg = 360.0
                if "右" in norm_inst or "時計回り" in norm_inst:
                    deg = -360.0
                return {"commands": [{"type": "spin", "value": deg}], "cancel": False}
    
            # 3. 前進・後退の判定
            # 後退: 後ろ下がって, usirosagatte, motto, back
            last_cmd_was_backward = False
            with self.lock:
                if self.chat_history:
                    for msg in reversed(self.chat_history):
                        if msg.get("role") == "assistant":
                            try:
                                # アシスタントの返答内容からコマンドを推測
                                hist_json = json.loads(msg.get("content", "{}"))
                                hist_cmds = hist_json.get("commands", [])
                                if hist_cmds and hist_cmds[-1].get("type") == "backward":
                                    last_cmd_was_backward = True
                                    break
                            except Exception:
                                pass
            
            # 「もっと」「もうすこし」で直前が後退の場合、さらに後退させる
            if norm_inst in ["motto", "もっと", "もうすこし", "もうちょっと", "さらに"] and last_cmd_was_backward:
                return {"commands": [{"type": "backward", "value": 1.0}], "cancel": False}
    
            if any(x in norm_inst for x in ["下", "後退", "sagatt", "usirosag", "ushirosag", "back", "reverse", "さがって"]):
                val = 0.5
                if "motto" in norm_inst or "もっと" in norm_inst or "大きく" in norm_inst or "たくさん" in norm_inst:
                    val = 1.0
                return {"commands": [{"type": "backward", "value": val}], "cancel": False}
    
            # 前進: 前に行って, 進んで, mae, forward
            if any(x in norm_inst for x in ["前", "進", "mae", "forward", "straight", "まえ"]):
                val = 1.0
                if "少し" in norm_inst or "ちょっと" in norm_inst:
                    val = 0.5
                elif "もっと" in norm_inst or "大きく" in norm_inst or "たくさん" in norm_inst:
                    val = 2.0
                return {"commands": [{"type": "forward", "value": val}], "cancel": False}

            
        # -------------------------------------------------------------------
        # ルールベースで一致しなかった複雑な指示は LLM (LM Studio) へ問い合わせる
        # -------------------------------------------------------------------
        system_prompt = (
            "You are a robotic navigator assistant. Your task is to translate natural language directions "
            "into a sequence of motion commands for the robot.\n\n"
            "CRITICAL: Do not explain your reasoning. Do not output any reasoning steps, thoughts, or chain-of-thought content. "
            "You MUST output raw JSON and ONLY raw JSON directly and immediately. Do not include markdown code block syntax (like ```json).\n"
            "Output format:\n"
            "{\n"
            "  \"commands\": [\n"
            "    {\"type\": \"forward\"|\"backward\"|\"turn\"|\"spin\"|\"face\"|\"goto\"|\"speed\", \"value\": float | [float, float]}\n"
            "  ],\n"
            "  \"cancel\": boolean (true if user wants to stop, cancel, halt, or stand by, false otherwise)\n"
            "}\n\n"
            "【Commands Definition】\n"
            "- \"type\": \"forward\": Move straight forward. \"value\": distance in meters (positive float).\n"
            "  Default distance if unspecified: 1.0m. 'ちょっと'/'少し': 0.5m. '大きく'/'たくさん': 2.0m.\n"
            "  Also maps to Romaji Japanese: 'mae'/'susunde'/'maeitte'/'susume'.\n"
            "- \"type\": \"backward\": Move straight backward. \"value\": distance in meters (positive float).\n"
            "  Default distance if unspecified: 0.5m.\n"
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
            "5. SHAPES (e.g. Squares/四角/正方形): If the user asks to draw or move in a square of size X (e.g., 'Xmの四角を描いて' / 'square of X meters'), unroll it into 8 commands: forward Xm, turn right (-1.5708), forward Xm, turn right, forward Xm, turn right, forward Xm, turn right.\n\n"
            "【Output Examples】\n"
            "- 'ちょっと前に行って' -> {\"commands\": [{\"type\": \"forward\", \"value\": 0.5}], \"cancel\": false}\n"
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
            "- '0.50m/sの速度で3m前進んで' -> {\"commands\": [{\"type\": \"speed\", \"value\": 0.50}, {\"type\": \"forward\", \"value\": 3.0}], \"cancel\": false}\n"
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
            
            # 1. 角度誤差が 8.0度 (約0.14rad) 以下になったら到達完了
            if abs(new_remaining) < math.radians(8.0):
                self.get_logger().info(
                    f"✅ [Turn Arrived] Heading aligned: final diff={math.degrees(new_remaining):.1f}deg < 8.0deg. Stopping."
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
                    has_more = bool(self.command_queue)
                
                # Nav2のゴールをキャンセルして停止
                self.cancel_navigation(clear_queue=False)
                
                if has_more:
                    print("↪️  旋回完了。次のアクションを開始します。")
                    self.execute_next_command()
                else:
                    with self.lock:
                        self.executing_command = False
                    print("\n✅ 旋回完了！次の指示をどうぞ。")
                    print("Command > ", end="", flush=True)
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
                self.chat_history.append({
                    "role": "assistant", 
                    "content": "【System Feedback】Robot detected physical blockage/stuck state. Navigation has been automatically cancelled."
                })
            
            # 安全のためロックの外側でキャンセルを実行
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
        
        # 閾値判定 (距離0.35m以内 かつ 角度差20度以内)
        if distance < 0.35 and yaw_diff < math.radians(20.0):
            with self.lock:
                self.goal_reached_logged = True
                self.is_stuck = False
                self.distance_remaining_history = []
                self.yaw_diff_history = []
                self.turn_target_yaw = None
            
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
                self.executing_command = False
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
            
            if clear_queue:
                self.chat_history = []
                self.command_queue = []
                self.executing_command = False
            
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
            has_more = bool(self.command_queue)
            
        if has_more:
            self.execute_next_command()
        else:
            print("\n✅ 旋回完了！次の指示をどうぞ。")
            print("Command > ", end="", flush=True)

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
