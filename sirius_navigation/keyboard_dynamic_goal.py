#!/usr/bin/env python3
import sys
import select
import termios
import tty
import threading
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo

# キー表記用のマッピング
KEY_NAMES = {
    '\x1b[A': '↑ (前方 0° に設定)',
    '\x1b[B': '↓ (後方 180° に設定)',
    '\x1b[D': '← (ゴール反時計回り回転 +15°)',
    '\x1b[C': '→ (ゴール時計回り回転 -15°)',
    'w': 'w (目標距離アップ +0.2m)',
    's': 's (目標距離ダウン -0.2m)',
    ' ': 'Space (ナビゲーション中断/キャンセル)',
    'k': 'k (ナビゲーション中断/キャンセル)',
    'g': 'g (現在の目標を再送/即時発行)',
    'e': 'e (緊急停止オン)',
    'q': 'q (緊急停止解除)'
}

class KeyboardDynamicGoal(Node):
    def __init__(self):
        super().__init__('keyboard_dynamic_goal')
        
        # /goal_pose パブリッシャー (Nav2の目標設定トピック, グローバル指定)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 緊急停止トピックのパブリッシャー
        self.stop_pub = self.create_publisher(Bool, 'stop', 10)
        
        # ナビゲーションキャンセルのためのアクションクライアント
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        
        # 目標設定の初期値
        self.r = 1.5                   # ロボットからの目標距離 (メートル)
        self.theta = 0.0               # ロボット正面に対する目標角度 (ラジアン, 反時計回りが正)
        
        self.r_min = 0.5
        self.r_max = 5.0
        self.r_step = 0.2
        self.theta_step = math.radians(15.0)  # 15度刻みで回転
        
        self.emergency_stop = False    # 緊急停止状態
        self.last_key_name = 'なし'     # 最後に認識された入力キー
        self.lock = threading.Lock()
        
        if not sys.stdin.isatty():
            self.get_logger().error('標準入力がTTY(端末)ではありません。キー操作ノードはインタラクティブな端末(Xtermなど)から直接実行するか、launchファイルで xterm プレフィックスを使用して起動してください。')
            sys.exit(1)

        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Sirius ダイナミックゴール・キーオペノードが起動しました')

    def getKey(self):
        if not sys.stdin.isatty():
            import time
            time.sleep(0.1)
            return ''
        tty.setraw(sys.stdin.fileno())
        # 入力待ち（タイムアウトを短めの 0.05秒にして応答性を向上）
        rlist, _, _ = select.select([sys.stdin.fileno()], [], [], 0.05)
        key_str = ''
        if rlist:
            import os
            b = os.read(sys.stdin.fileno(), 1)
            if not b:
                termios.tcsetattr(sys.stdin, termios.TCSANOW, self.settings)
                return ''
            
            # マルチバイト文字（全角文字など）の読み取り処理
            lead = b[0]
            num_bytes = 1
            if lead >= 0xf0:
                num_bytes = 4
            elif lead >= 0xe0:
                num_bytes = 3
            elif lead >= 0xc0:
                num_bytes = 2
                
            key = b
            if num_bytes > 1:
                rlist2, _, _ = select.select([sys.stdin.fileno()], [], [], 0.03)
                if rlist2:
                    b2 = os.read(sys.stdin.fileno(), num_bytes - 1)
                    key += b2
            
            # エスケープシーケンス（矢印キーなど）のハンドリング
            if key == b'\x1b':
                rlist2, _, _ = select.select([sys.stdin.fileno()], [], [], 0.03)
                if rlist2:
                    b2 = os.read(sys.stdin.fileno(), 2)
                    key += b2
                    
            key_str = key.decode('utf-8', errors='ignore')
            
        termios.tcsetattr(sys.stdin, termios.TCSANOW, self.settings)
        return key_str

    def refresh_display(self):
        """画面をスレッドセーフにリフレッシュする"""
        with self.lock:
            print("\033[H\033[J", end="", flush=True)
            # 改行コードを \r\n に置換して生端末モードでのズレを防ぐ
            status_msg = self.get_status_msg_unlocked().replace('\n', '\r\n')
            print(status_msg, flush=True)

    def get_status_msg_unlocked(self):
        # 角度を度数法に変換
        deg = math.degrees(self.theta)
        # -180 ~ 180度の範囲にクリップ
        while deg > 180.0: deg -= 360.0
        while deg <= -180.0: deg += 360.0

        # 方向ガイドテキスト
        dir_text = ""
        if -22.5 <= deg <= 22.5:
            dir_text = "前方"
        elif 22.5 < deg <= 67.5:
            dir_text = "左前方"
        elif 67.5 < deg <= 112.5:
            dir_text = "左方"
        elif 112.5 < deg <= 157.5:
            dir_text = "左後方"
        elif deg > 157.5 or deg <= -157.5:
            dir_text = "後方"
        elif -157.5 < deg <= -112.5:
            dir_text = "右後方"
        elif -112.5 < deg <= -67.5:
            dir_text = "右方"
        elif -67.5 < deg < -22.5:
            dir_text = "右前方"

        # 相対座標計算
        target_x = self.r * math.cos(self.theta)
        target_y = self.r * math.sin(self.theta)

        estop_msg = ""
        if self.emergency_stop:
            estop_msg = "\033[1;41;37m🚨  緊急停止オン (EキーでON / Qキーで解除) 🚨\033[0m\n"
        else:
            estop_msg = "\033[1;32m🟢  緊急停止オフ (Eキーで緊急停止)\033[0m\n"

        msg = f"""
=== Sirius 周囲360度ダイナミックゴール・キーオペ ===

緊急停止状態:
  {estop_msg}
入力フィードバック:
  最後に入力されたキー: \033[1;36m[{self.last_key_name}]\033[0m

現在のダイナミックゴール設定値 (ロボット中心座標系):
  目標距離 (R):  \033[1;32m{self.r:.2f} m\033[0m (可変範囲: {self.r_min}m 〜 {self.r_max}m)
  目標角度 (θ):  \033[1;32m{deg:+.1f} deg\033[0m ({dir_text}方向)
  相対座標値:   X = {target_x:+.2f} m, Y = {target_y:+.2f} m

操作キー:
  ↑ (矢印上)  : ゴールをロボットの真前方にセット (θ = 0°)
  ↓ (矢印下)  : ゴールをロボットの真後方にセット (θ = 180°)
  ← (矢印左)  : 反時計回りにゴールを回転 (+15°)
  → (矢印右)  : 時計回りにゴールを回転 (-15°)
  w / s       : ゴール距離 (R) を遠く/近く調整 (±{self.r_step:.1f}m)
  g           : 現在設定値で目標ゴールを再送 (即時発行)

停止・緊急停止キー:
  e           : 緊急停止 (オン)
  q           : 緊急停止 (解除/オフ)
  space / k   : 走行タスク中断 (Nav2アクティブゴール取り消し)

終了するには Ctrl+C を押してください
---------------------------------------------
"""
        return msg

    def publish_goal(self):
        """現在のR, thetaをロボットのローカルフレーム上の目標として発行"""
        if self.emergency_stop:
            self.get_logger().warning("緊急停止中のため、ゴール発行をスキップします。")
            return

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'sirius3/base_footprint'  # ロボットベースフレーム基準
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 極座標を直交座標(X, Y)に変換
        with self.lock:
            pose_msg.pose.position.x = self.r * math.cos(self.theta)
            pose_msg.pose.position.y = self.r * math.sin(self.theta)
            pose_msg.pose.position.z = 0.0
            
            # ゴールの向きを角度theta（外側向き）に設定
            pose_msg.pose.orientation.z = math.sin(self.theta / 2.0)
            pose_msg.pose.orientation.w = math.cos(self.theta / 2.0)
            
        self.goal_pub.publish(pose_msg)

    def cancel_navigation(self):
        """実行中のナビゲーションをキャンセル"""
        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("ナビゲーションキャンセルサービスが見つかりません。")
            return
        
        req = CancelGoal.Request()
        req.goal_info = GoalInfo()
        self.cancel_client.call_async(req)

    def run(self):
        self.refresh_display()

        try:
            while rclpy.ok():
                key = self.getKey()
                if not key:
                    continue
                
                key_lower = key.lower()
                key_name = KEY_NAMES.get(key_lower, KEY_NAMES.get(key, f"'{key}'"))
                
                with self.lock:
                    self.last_key_name = key_name

                # 全角入力を正規化
                key_norm = key
                if key == '　':
                    key_norm = ' '
                elif key == 'ｋ':
                    key_norm = 'k'
                elif key == 'ｅ':
                    key_norm = 'e'
                elif key == 'ｑ':
                    key_norm = 'q'
                elif key == 'ｗ':
                    key_norm = 'w'
                elif key == 'ｓ':
                    key_norm = 's'
                elif key == 'ｇ':
                    key_norm = 'g'

                key_lower = key_norm.lower()

                # 緊急停止（オン）
                if key_lower == 'e':
                    with self.lock:
                        self.emergency_stop = True
                    stop_msg = Bool()
                    stop_msg.data = True
                    self.stop_pub.publish(stop_msg)
                    self.cancel_navigation()
                    self.refresh_display()
                
                # 緊急停止解除（オフ）
                elif key_lower == 'q':
                    with self.lock:
                        self.emergency_stop = False
                    stop_msg = Bool()
                    stop_msg.data = False
                    self.stop_pub.publish(stop_msg)
                    self.refresh_display()

                # ゴール更新操作 (緊急停止中は受け付けない)
                elif not self.emergency_stop:
                    should_publish = False

                    if key == '\x1b[A':  # UP arrow
                        with self.lock:
                            self.theta = 0.0
                        should_publish = True
                    elif key == '\x1b[B':  # DOWN arrow
                        with self.lock:
                            self.theta = math.pi
                        should_publish = True
                    elif key == '\x1b[D':  # LEFT arrow
                        with self.lock:
                            self.theta += self.theta_step
                        should_publish = True
                    elif key == '\x1b[C':  # RIGHT arrow
                        with self.lock:
                            self.theta -= self.theta_step
                        should_publish = True
                    elif key_lower == 'w':  # 距離遠く
                        with self.lock:
                            self.r = min(self.r + self.r_step, self.r_max)
                        should_publish = True
                    elif key_lower == 's':  # 距離近く
                        with self.lock:
                            self.r = max(self.r - self.r_step, self.r_min)
                        should_publish = True
                    elif key_lower == 'g':  # 再送
                        should_publish = True
                    elif key_norm == ' ' or key_norm == 'k':  # 中断
                        self.cancel_navigation()
                        self.refresh_display()
                    elif key == '\x03':  # Ctrl+C
                        break
                    
                    if should_publish:
                        self.publish_goal()
                        self.refresh_display()
                else:
                    if key == '\x03':
                        break

        except Exception as e:
            self.get_logger().error(f'エラーが発生しました: {e}')

        finally:
            # 終了時にキャンセルと緊急停止解除
            self.cancel_navigation()
            
            stop_msg = Bool()
            stop_msg.data = False
            self.stop_pub.publish(stop_msg)
            
            termios.tcsetattr(sys.stdin, termios.TCSANOW, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardDynamicGoal()
    
    # ROS 2スピンを別スレッドで開始 (サービス呼び出しのため)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
