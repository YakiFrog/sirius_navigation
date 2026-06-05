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
from tf2_ros import Buffer, TransformListener

# キー表記用のマッピング
KEY_NAMES = {
    '\x1b[A': '↑ (進行方向設定 / 連続押しで距離・速度アップ)',
    '\x1b[B': '↓ (後退方向設定 / 連続押しで距離・速度アップ)',
    '\x1b[D': '← (ゴール反時計回り回転 +15°)',
    '\x1b[C': '→ (ゴール時計回り回転 -15°)',
    'w': 'w (目標距離アップ +0.2m)',
    's': 's (目標距離ダウン -0.2m)',
    ',': ', (直進距離のみ停止 - R=0.0m)',
    '、': '、 (直進距離のみ停止 - R=0.0m - 全角)',
    '，': '， (直進距離のみ停止 - R=0.0m - 全角)',
    '.': '. (旋回角度のみ停止 - θ=0.0°)',
    '。': '。 (旋回角度のみ停止 - θ=0.0° - 全角)',
    '．': '． (旋回角度のみ停止 - θ=0.0° - 全角)',
    ' ': 'Space (ナビゲーション中断/キャンセル)',
    'k': 'k (ナビゲーション中断/キャンセル)',
    'g': 'g (現在の目標を再送/即時発行)',
    'e': 'e (緊急停止オン)',
    'q': 'q (緊急停止解除)'
}

class KeyboardDynamicGoal(Node):
    def __init__(self):
        super().__init__('keyboard_dynamic_goal')
        
        # TFのリスナーを初期化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # /goal_pose パブリッシャー (Nav2の目標設定トピック, グローバル指定)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 緊急停止トピックのパブリッシャー
        self.stop_pub = self.create_publisher(Bool, 'stop', 10)
        
        # ナビゲーションキャンセルのためのアクションクライアント
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        
        # 目標設定の初期値
        self.r = 0.0                   # 初期値を0.0にしてその場回転を可能にする
        self.theta = 0.0               # ロボット正面に対する目標角度 (ラジアン, 反時計回りが正)
        
        self.r_min = 0.0
        self.r_max = 5.0
        self.r_step = 0.2
        self.theta_step = math.radians(15.0)  # 15度刻みで回転
        
        self.emergency_stop = False    # 緊急停止状態
        self.goal_active = False       # ゴールのアクティブ状態
        self.last_key_name = 'なし'     # 最後に認識された入力キー
        self.lock = threading.Lock()
        
        if not sys.stdin.isatty():
            self.get_logger().error('標準入力がTTY(端末)ではありません。キー操作ノードはインタラクティブな端末(Xtermなど)から直接実行するか、launchファイルで xterm プレフィックスを使用して起動してください。')
            sys.exit(1)

        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Sirius ダイナミックゴール・キーオペノードが起動しました')
        
        # タイマーの作成 (5Hz = 0.2秒ごとに最新の相対座標でゴールを更新/再発行)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        """ゴールがアクティブな場合、最新の自己位置から相対座標を再計算してゴールを発行し続ける"""
        with self.lock:
            active = self.goal_active
            estop = self.emergency_stop
            
        if active and not estop:
            self.publish_goal()

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

        active_status = ""
        if self.goal_active:
            active_status = "\033[1;32m🔥 追従走行中 (ロボット前方に追従中)\033[0m"
        else:
            active_status = "\033[1;33m⏸️  待機中 (キー入力で走行開始)\033[0m"

        msg = f"""
=== Sirius 周囲360度ダイナミックゴール・キーオペ ===

追従状態:
  {active_status}

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
  , (カンマ)  : 直進のみ停止 (R = 0.0m / その場旋回は継続)
  . (ピリオド): 旋回のみ停止 (θ = 0.0°)
  space / k   : 走行タスク中断 (Nav2アクティブゴール取り消し)

終了するには Ctrl+C を押してください
---------------------------------------------
"""
        return msg

    def publish_goal(self):
        """現在のR, thetaをロボットのローカルフレーム上の目標として発行 (map座標系に変換)"""
        if self.emergency_stop:
            self.get_logger().warning("緊急停止中のため、ゴール発行をスキップします。")
            return

        # 1. ロボット座標系での相対座標を計算
        with self.lock:
            x_local = self.r * math.cos(self.theta)
            y_local = self.r * math.sin(self.theta)
            theta_local = self.theta

        # 2. TFを使用してmapからsirius3/base_footprintへの変換を取得
        try:
            # 最新の変換情報を取得 (Time 0を指定)
            trans = self.tf_buffer.lookup_transform(
                'map',
                'sirius3/base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warning(f"TF (map -> sirius3/base_footprint) の取得に失敗したため、ゴール発行をスキップします: {e}")
            return

        # 3. 変換情報を使って、ロボット座標系上の目標点をmap座標系に変換
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
        
        # 回転と並進の適用
        target_map_x = tx + (x_local * math.cos(yaw_robot) - y_local * math.sin(yaw_robot))
        target_map_y = ty + (x_local * math.sin(yaw_robot) + y_local * math.cos(yaw_robot))
        
        # 目標の向き（ロボットの絶対姿勢yaw_robot + 相対角度theta_local）
        target_map_yaw = yaw_robot + theta_local

        # 4. PoseStamped メッセージの構築
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'  # Nav2が確実に受け入れられるmap座標系にする
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.position.x = target_map_x
        pose_msg.pose.position.y = target_map_y
        pose_msg.pose.position.z = 0.0
        
        # Yaw角から四元数への変換
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(target_map_yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(target_map_yaw / 2.0)
            
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
                elif key in (',', '、', '，'):
                    key_norm = ','
                elif key in ('.', '。', '．'):
                    key_norm = '.'

                key_lower = key_norm.lower()

                # 緊急停止（オン）
                if key_lower == 'e':
                    with self.lock:
                        self.emergency_stop = True
                        self.goal_active = False  # 緊急停止時はゴール無効化
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
                            # 進行方向が前以外（後ろを向いていた、またはその場回転中）だった場合は前にリセットして初期距離にする
                            if self.theta != 0.0 or not self.goal_active or self.r == 0.0:
                                self.theta = 0.0
                                self.r = 0.5  # 開始距離
                            else:
                                # すでに前進中なら距離（速度）を上げる
                                self.r = min(self.r + self.r_step, self.r_max)
                            self.goal_active = True
                        should_publish = True
                    elif key == '\x1b[B':  # DOWN arrow
                        with self.lock:
                            # 進行方向が後ろ以外だった場合は後ろにリセットして初期距離にする
                            if self.theta != math.pi or not self.goal_active or self.r == 0.0:
                                self.theta = math.pi
                                self.r = 0.5  # 開始距離
                            else:
                                # すでに後退中なら距離（速度）を上げる
                                self.r = min(self.r + self.r_step, self.r_max)
                            self.goal_active = True
                        should_publish = True
                    elif key == '\x1b[D':  # LEFT arrow
                        with self.lock:
                            # その場回転がスムーズに行えるよう、目標距離Rが1.0m未満の場合は自動的に1.0mに設定する
                            if self.r < 1.0:
                                self.r = 1.0
                            self.theta += self.theta_step
                            self.goal_active = True
                        should_publish = True
                    elif key == '\x1b[C':  # RIGHT arrow
                        with self.lock:
                            # その場回転がスムーズに行えるよう、目標距離Rが1.0m未満の場合は自動的に1.0mに設定する
                            if self.r < 1.0:
                                self.r = 1.0
                            self.theta -= self.theta_step
                            self.goal_active = True
                        should_publish = True
                    elif key_lower == 'w':  # 距離遠く
                        with self.lock:
                            self.r = min(self.r + self.r_step, self.r_max)
                            self.goal_active = True
                        should_publish = True
                    elif key_lower == 's':  # 距離近く
                        with self.lock:
                            self.r = max(self.r - self.r_step, self.r_min)
                            self.goal_active = True
                        should_publish = True
                    elif key_lower == 'g':  # 再送
                        with self.lock:
                            self.goal_active = True
                        should_publish = True
                    elif key_norm == ',':  # 直進停止 (R = 0.0m)
                        with self.lock:
                            self.r = 0.0
                            self.goal_active = True  # その場回転はできるように追従は維持
                        should_publish = True
                    elif key_norm == '.':  # 旋回停止 (theta = 0.0)
                        with self.lock:
                            self.theta = 0.0
                        should_publish = True
                    elif key_norm == ' ' or key_norm == 'k':  # 中断
                        with self.lock:
                            self.goal_active = False
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
