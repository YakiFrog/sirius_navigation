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
        
        # アクションキャンセルのためのサービス
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        
        # 自然言語指示トピックのサブスクライバー（他のノードや音声認識等からの入力を想定）
        self.instruction_sub = self.create_subscription(
            String,
            '/llm_instruction',
            self.instruction_callback,
            10
        )
        
        self.lock = threading.Lock()
        self.get_logger().info(f'LLM Dynamic Goal Node initialized.')
        self.get_logger().info(f'LM Studio URL: {self.lm_studio_url}')
        
        # ゴール到達確認のための状態管理
        self.active_goal_x = None
        self.active_goal_y = None
        self.active_goal_yaw = None
        self.goal_reached_logged = True
        
        # 1Hzで自己位置と目標位置の距離を監視するタイマー
        self.goal_monitor_timer = self.create_timer(1.0, self.monitor_goal_distance)
        
        # 対話型コマンドライン入力を別スレッドで開始
        self.running = True
        self.input_thread = threading.Thread(target=self.interactive_input_loop, daemon=True)
        self.input_thread.start()

    def instruction_callback(self, msg):
        """トピック経由で指示を受信したときのコールバック"""
        self.get_logger().info(f'Received instruction via topic: "{msg.data}"')
        # LLMへのリクエストは時間がかかるため、別スレッドで処理
        threading.Thread(target=self.process_instruction, args=(msg.data,), daemon=True).start()

    def interactive_input_loop(self):
        """標準入力からインタラクティブに入力を受け付けるスレッド"""
        print("\n=== Sirius LLM Dynamic Goal Navigation ===")
        print("LM Studioを起動し、モデルがロードされていることを確認してください。")
        print("「ちょっと前に行って」「そこで右向いて」「停止」などの指示を入力してください。")
        print("終了するには 'exit' または Ctrl+C を入力してください。\n")
        
        while self.running:
            try:
                # sys.stdinがブロックするため、安全に入力を取得
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
                
                # 指示の非同期処理
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
            
        r = result.get("r", 0.0)
        theta = result.get("theta", 0.0)
        is_cancel = result.get("cancel", False)
        
        if is_cancel:
            self.get_logger().warning("LLM interpreted instruction as STOP/CANCEL.")
            self.cancel_navigation()
            return
            
        # 旋回時の目標調整 (keyboard_dynamic_goal.py の 1.1m のその場旋回を参考にする)
        # LLMがその場回転 (r=0) で角度だけを返した場合、滑らかな走行のために r を 1.1 に自動補正
        if abs(r) < 0.1 and abs(theta) > 0.01:
            self.get_logger().info("Spin command detected: Auto-adjusting target distance to R=1.1m for smooth rotation.")
            r = 1.1
            # 旋回角度もスムーズな旋回のために約 70度 (1.22 rad) にクリップ/調整する (符号は維持)
            sign = 1.0 if theta > 0 else -1.0
            theta = sign * 1.2217  # 70 degrees in radians
            
        self.get_logger().info(f"LLM parsed targets -> r: {r:.2f} m, theta: {math.degrees(theta):+.1f} deg ({theta:.3f} rad)")
        self.publish_goal(r, theta)

    def query_lm_studio(self, instruction):
        """LM StudioのOpenAI互換APIを呼び出し、指示をR, thetaに変換する"""
        system_prompt = (
            "You are a robotic navigator assistant. Your task is to translate natural language directions "
            "into relative target coordinates for the robot. "
            "You MUST output raw JSON and ONLY raw JSON. Do not include markdown code block syntax (like ```json). "
            "Output format:\n"
            "{\n"
            "  \"r\": float (straight line distance in meters. positive for forward, negative for backward. For pure turning, set r to 1.1),\n"
            "  \"theta\": float (relative rotation angle in radians, positive for counter-clockwise / left, negative for clockwise / right),\n"
            "  \"cancel\": boolean (true if user wants to stop, cancel, halt, or stand by, false otherwise)\n"
            "}\n\n"
            "Examples:\n"
            "- \"ちょっと前に行って\" -> {\"r\": 0.5, \"theta\": 0.0, \"cancel\": false}\n"
            "- \"そこで右向いて\" -> {\"r\": 1.1, \"theta\": -1.2217, \"cancel\": false}\n"
            "- \"左向いて\" -> {\"r\": 1.1, \"theta\": 1.2217, \"cancel\": false}\n"
            "- \"少し左前に進んで\" -> {\"r\": 0.8, \"theta\": 0.5, \"cancel\": false}\n"
            "- \"危ない、止まって！\" -> {\"r\": 0.0, \"theta\": 0.0, \"cancel\": true}"
        )
        
        headers = {
            'Content-Type': 'application/json',
        }
        
        payload = {
            "model": self.model_name,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": instruction}
            ],
            "temperature": 0.0,
            "max_tokens": 100
        }
        
        try:
            req = urllib.request.Request(
                self.lm_studio_url,
                data=json.dumps(payload).encode('utf-8'),
                headers=headers,
                method='POST'
            )
            # タイムアウトを短めの5秒に設定
            with urllib.request.urlopen(req, timeout=5.0) as response:
                res_body = response.read().decode('utf-8')
                res_json = json.loads(res_body)
                content = res_json['choices'][0]['message']['content'].strip()
                
                # Markdownブロックのトリミング (```json ... ``` が含まれている場合の対処)
                if content.startswith("```"):
                    lines = content.splitlines()
                    if len(lines) >= 3:
                        content = "\n".join(lines[1:-1])
                        
                return json.loads(content)
        except urllib.error.URLError as e:
            self.get_logger().error(f"LM Studio API connection failed: {e}")
            self.get_logger().error("Make sure LM Studio is running on port 1234.")
        except Exception as e:
            self.get_logger().error(f"Error calling LLM: {e}")
            if 'res_body' in locals():
                self.get_logger().error(f"Raw Response: {res_body}")
        return None

    def publish_goal(self, r, theta):
        """相対距離r, 角度thetaをmap座標系の絶対目標として計算しパブリッシュする"""
        try:
            # 最新の変換情報を取得 (map -> base_footprint)
            trans = self.tf_buffer.lookup_transform(
                'map',
                'sirius3/base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup TF (map -> sirius3/base_footprint): {e}")
            return

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
        
        # ロボット座標系からmap座標系への回転と並進
        target_map_x = tx + (r * math.cos(theta) * math.cos(yaw_robot) - r * math.sin(theta) * math.sin(yaw_robot))
        target_map_y = ty + (r * math.cos(theta) * math.sin(yaw_robot) + r * math.sin(theta) * math.cos(yaw_robot))
        
        target_map_yaw = yaw_robot + theta

        # PoseStamped メッセージの構築
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.position.x = target_map_x
        pose_msg.pose.position.y = target_map_y
        pose_msg.pose.position.z = 0.0
        
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(target_map_yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(target_map_yaw / 2.0)
            
        self.goal_pub.publish(pose_msg)
        self.publish_marker(pose_msg)
        
        # 目標監視用の変数をセット
        with self.lock:
            self.active_goal_x = target_map_x
            self.active_goal_y = target_map_y
            self.active_goal_yaw = target_map_yaw
            self.goal_reached_logged = False
            
        self.get_logger().info(f"Published goal pose to /goal_pose: X={target_map_x:.2f}, Y={target_map_y:.2f}")

    def monitor_goal_distance(self):
        """現在の自己位置と目標位置の距離を比較し、到達したか監視する"""
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
        
        # 角度差の計算 (最小絶対値の角度差を求める)
        yaw_diff = abs(goal_yaw - yaw_robot)
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
        yaw_diff = abs(yaw_diff)

        # 閾値判定 (距離0.3m以内 かつ 角度差20度以内)
        if distance < 0.35 and yaw_diff < math.radians(20.0):
            with self.lock:
                self.goal_reached_logged = True
            
            self.get_logger().info("🏆 [Goal Reached] Robot has arrived at the target destination!")
            print("\n🎉 目標地点に到着しました！次の指示をどうぞ。")
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
        
        marker.scale.x = 0.6  # 矢印の長さ
        marker.scale.y = 0.15 # 矢印の太さ
        marker.scale.z = 0.15 # 矢印の高さ
        
        # 色設定 (青色、半透明)
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
            
        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Navigation cancel service not available.")
            return
        
        req = CancelGoal.Request()
        req.goal_info = GoalInfo()
        self.cancel_client.call_async(req)
        
        # 緊急停止トピックへ停止信号も投げる
        stop_msg = Bool()
        stop_msg.data = True
        self.stop_pub.publish(stop_msg)
        
        # 1秒後に緊急停止状態を自動解除して次の指示を受け付けられるようにする
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
