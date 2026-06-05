#!/usr/bin/env python3
import sys
import math
import json
import urllib.request
import urllib.error
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

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
        
        # スタック検知用状態管理
        self.distance_remaining_history = []
        self.is_stuck = False
        
        # 会話履歴管理 (マルチターン対話用)
        self.chat_history = []
        self.history_max_turns = 10  # 最大5往復分
        
        # 1Hzで自己位置と目標位置の距離を監視するタイマー
        self.goal_monitor_timer = self.create_timer(1.0, self.monitor_goal_distance)
        
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
            
        cmd_type = cmd.get("type", "forward")
        value = cmd.get("value", 0.0)
        
        self.get_logger().info(f"Executing next sequence command -> type: {cmd_type}, value: {value}")
        
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
            r = -abs(value)
            theta = 0.0
            self.publish_goal_pose(tx, ty, yaw_robot, r, theta)
        elif cmd_type == "turn":
            # 旋回時: keyboard_dynamic_goal.py の 1.1m と 70deg に合わせる
            r = 1.1
            sign = 1.0 if value >= 0 else -1.0
            theta = sign * 1.2217
            self.publish_goal_pose(tx, ty, yaw_robot, r, theta)
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
        system_prompt = (
            "You are a robotic navigator assistant. Your task is to translate natural language directions "
            "into a sequence of relative target coordinates for the robot.\n\n"
            "CRITICAL: Do not explain your reasoning. Do not output any reasoning steps, thoughts, or chain-of-thought content. "
            "You MUST output raw JSON and ONLY raw JSON directly and immediately. Do not include markdown code block syntax (like ```json).\n"
            "Output format:\n"
            "{\n"
            "  \"commands\": [\n"
            "    {\"type\": \"forward\"|\"backward\"|\"turn\"|\"goto\", \"value\": float | [float, float]}\n"
            "  ],\n"
            "  \"cancel\": boolean (true if user wants to stop, cancel, halt, or stand by, false otherwise)\n"
            "}\n\n"
            "【Commands Definition】\n"
            "- \"type\": \"forward\": Move straight forward. \"value\": distance in meters (positive float).\n"
            "- \"type\": \"backward\": Move straight backward. \"value\": distance in meters (positive float).\n"
            "- \"type\": \"turn\": Rotate in place. \"value\": relative angle in radians. positive for left, negative for right.\n"
            "- \"type\": \"goto\": Navigate to absolute map coordinates. \"value\": [x_coord, y_coord] (array of two floats).\n\n"
            "【Conversational Context & Embodied Feedback Rules】\n"
            "You will receive feedback from both the user's conversation AND the robot's physical sensors (the system message labeled 【Robot Current Hardware State Feedback】).\n"
            "1. If the user complains about the last action (e.g. \"全然進んでいない\" / \"Not moving at all\", \"もっと進んで\"), "
            "look at the robot's physical state (e.g., if distance remaining is large and velocity is 0.0m/s with BLOCKED status), "
            "this implies an obstacle blocks the path. You should generate a corrective sequence like a detour turn or reversing:\n"
            "   - Example: \"全然進んでいない\" -> output a turn command to face a different angle, or reverse.\n"
            "2. If the robot was simply given a command that was too small (e.g. user says \"全然下がっていない\" after a backward 0.5m action and robot arrived safely), "
            "amplify the value in your next command (e.g., output \"type\": \"backward\", \"value\": 1.5).\n\n"
            "【Static Output Reference Examples】\n"
            "- \"ちょっと前に行って\" -> {\"commands\": [{\"type\": \"forward\", \"value\": 0.5}], \"cancel\": false}\n"
            "- \"そこで右向いて\" -> {\"commands\": [{\"type\": \"turn\", \"value\": -1.5708}], \"cancel\": false}\n"
            "- \"座標 (0.0, 0.0) に向かって\" -> {\"commands\": [{\"type\": \"goto\", \"value\": [0.0, 0.0]}], \"cancel\": false}\n"
            "- \"3m前に行って、左に曲がって\" -> {\"commands\": [{\"type\": \"forward\", \"value\": 3.0}, {\"type\": \"turn\", \"value\": 1.5708}], \"cancel\": false}\n"
            "- \"危ない、止まって！\" -> {\"commands\": [], \"cancel\": true}"
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
            self.is_stuck = False
            
        self.get_logger().info(f"Published goal pose to /goal_pose: X={target_x:.2f}, Y={target_y:.2f}, Yaw={math.degrees(target_yaw):+.1f}deg")

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
        with self.lock:
            self.distance_remaining_history.append(distance)
            if len(self.distance_remaining_history) > 5:
                self.distance_remaining_history.pop(0)
            
            # 5秒間のデータがあり、かつシーケンスが実行中の場合
            if len(self.distance_remaining_history) == 5 and self.executing_command:
                diff = abs(self.distance_remaining_history[0] - self.distance_remaining_history[-1])
                # 5秒間、距離の変化が極小で、オドメトリ速度も極小の場合にスタックと判定
                if diff < 0.05 and abs(self.current_vel_x) < 0.05 and abs(self.current_vel_theta) < 0.05:
                    if not self.is_stuck:
                        self.is_stuck = True
                        self.get_logger().warning("🤖 [Stuck Detected] Robot is blocked or unable to reach the target.")
                else:
                    self.is_stuck = False
            else:
                self.is_stuck = False
        # -------------------------------------------------------------

        # 閾値判定 (距離0.35m以内 かつ 角度差20度以内)
        if distance < 0.35 and yaw_diff < math.radians(20.0):
            with self.lock:
                self.goal_reached_logged = True
                self.is_stuck = False
                self.distance_remaining_history = []
            
            self.get_logger().info("🏆 [Goal Reached] Robot has arrived at the target waypoint.")
            
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

    def cancel_navigation(self):
        """実行中のナビゲーションをキャンセル"""
        self.delete_marker()
        with self.lock:
            self.active_goal_x = None
            self.active_goal_y = None
            self.active_goal_yaw = None
            self.goal_reached_logged = True
            self.chat_history = []
            self.command_queue = []
            self.executing_command = False
            self.distance_remaining_history = []
            self.is_stuck = False
            
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
