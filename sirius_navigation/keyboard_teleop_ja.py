#!/usr/bin/env python3
import sys
import select
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# キー表記用のマッピング (全角入力対応)
KEY_NAMES = {
    '\x1b[A': '↑ (直進速度アップ)',
    '\x1b[B': '↓ (直進速度ダウン/後退)',
    '\x1b[D': '← (左旋回速度アップ)',
    '\x1b[C': '→ (右旋回速度アップ/右旋回)',
    ',': ', (直進のみ停止)',
    '、': '、 (直進のみ停止 - 全角)',
    '，': '， (直進のみ停止 - 全角)',
    '.': '. (旋回のみ停止)',
    '。': '。 (旋回のみ停止 - 全角)',
    '．': '． (旋回のみ停止 - 全角)',
    ' ': 'Space (完全停止)',
    '　': '全角スペース (完全停止)',
    'k': 'k (完全停止)',
    'ｋ': 'ｋ (完全停止 - 全角)',
    'e': 'e (緊急停止オン)',
    'ｅ': 'ｅ (緊急停止オン - 全角)',
    'q': 'q (緊急停止解除)',
    'ｑ': 'ｑ (緊急停止解除 - 全角)',
    't': 't (アシストON/OFF切り替え)',
    'ｔ': 'ｔ (アシストON/OFF切り替え - 全角)'
}

class SiriusKeyboardTeleopJA(Node):
    def __init__(self):
        super().__init__('sirius_keyboard_teleop_ja')
        # アシスト機能有効時のパブリッシャー
        self.pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        # アシスト機能無効(ダイレクト操作)時のパブリッシャー (cmd_velへ直送り)
        self.pub_direct = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 緊急停止トピックのパブリッシャー (scnと共通)
        self.stop_pub = self.create_publisher(Bool, 'stop', 10)
        
        # 実際の速度を監視するために cmd_vel を購読
        self.sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmdVelCallback,
            10
        )
        
        # 速度設定とステップ幅
        self.target_linear_vel = 0.0   # m/s
        self.target_angular_vel = 0.0  # rad/s
        
        self.linear_vel_step = 0.1     # m/s
        self.linear_vel_max = 1.0      # m/s
        self.angular_vel_step = 0.05   # rad/s
        self.angular_vel_max = 0.9     # rad/s
        
        self.blocked = False           # 障害物による停止・減速フラグ
        self.emergency_stop = False    # 緊急停止状態フラグ
        self.assisted_mode = True      # アシスト操縦（障害物回避）モードのON/OFF
        self.last_key_name = 'なし'     # 最後に認識された入力キー
        self.lock = threading.Lock()
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Sirius 日本語キーボード操作ノード（scn互換）が起動しました')

    def cmdVelCallback(self, msg):
        with self.lock:
            # ダイレクト操作モードの時はブロック判定を行わない
            if not self.assisted_mode:
                if self.blocked:
                    self.blocked = False
                    print("\033[H\033[J", end="")
                    print(self.get_status_msg_unlocked())
                return

            is_blocked = False
            
            # 直進方向のブロッキング判定
            if abs(self.target_linear_vel) > 0.05:
                if self.target_linear_vel > 0:
                    if msg.linear.x < 0.05:
                        is_blocked = True
                elif self.target_linear_vel < 0:
                    if msg.linear.x > -0.05:
                        is_blocked = True
                        
            # 旋回方向のブロッキング判定
            if abs(self.target_angular_vel) > 0.05:
                if self.target_angular_vel > 0:
                    if msg.angular.z < 0.05:
                        is_blocked = True
                elif self.target_angular_vel < 0:
                    if msg.angular.z > -0.05:
                        is_blocked = True
            
            if is_blocked != self.blocked:
                self.blocked = is_blocked
                print("\033[H\033[J", end="")
                print(self.get_status_msg_unlocked())

    def refresh_display(self):
        """スレッドセーフに画面表示をクリアして更新する"""
        with self.lock:
            print("\033[H\033[J", end="")
            print(self.get_status_msg_unlocked())

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        # 入力待ち（タイムアウトを短めの 0.05秒にして応答性を向上）
        rlist, _, _ = select.select([sys.stdin.fileno()], [], [], 0.05)
        key_str = ''
        if rlist:
            import os
            # Pythonの内部バッファの影響を受けないよう、os.readで生FDから読み取る
            b = os.read(sys.stdin.fileno(), 1)
            
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
            
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key_str

    def get_status_msg_unlocked(self):
        """ロックが獲得されている状態で呼ばれる内部用メソッド"""
        warning_msg = ""
        if not self.assisted_mode:
            warning_msg = "\033[1;33m⚠️  [警告] アシスト機能無効 (障害物回避は作動しません)\033[0m\n"
        elif self.blocked:
            warning_msg = "\033[1;31m⚠️  [警告] 目の前または進路に障害物があるため動けません (制限中)\033[0m\n"
        else:
            warning_msg = "\033[1;32m✅  進路クリア (安全に走行可能です)\033[0m\n"

        estop_msg = ""
        if self.emergency_stop:
            estop_msg = "\033[1;41;37m🚨  緊急停止オン (EキーでON / Qキーで解除) 🚨\033[0m\n"
        else:
            estop_msg = "\033[1;32m🟢  緊急停止オフ (Eキーで緊急停止)\033[0m\n"

        assist_status = ""
        if self.assisted_mode:
            assist_status = "\033[1;32mON (安全アシスト有効)\033[0m"
        else:
            assist_status = "\033[1;33mOFF (ダイレクト操作)\033[0m"

        msg = f"""
=== Sirius アシスト付きキーボード操作ガイド (scn互換・加速式) ===

アシスト機能状態: {assist_status}  (Tキーで切り替え)
緊急停止状態:
  {estop_msg}
ステータス:
  {warning_msg}
入力フィードバック:
  最後に入力されたキー: \033[1;36m[{self.last_key_name}]\033[0m

操作キー:
  ↑ (矢印上)  : 直進速度を上げる (+{self.linear_vel_step:.2f} m/s)
  ↓ (矢印下)  : 直進速度を下げる (-{self.linear_vel_step:.2f} m/s)
  ← (矢印左)  : 左旋回速度を上げる (+{self.angular_vel_step:.2f} rad/s)
  → (矢印右)  : 右旋回速度を上げる (-{self.angular_vel_step:.2f} rad/s)

操作モード・緊急停止・停止キー (全角・日本語入力対応):
  t           : アシスト機能の有効/無効切り替え (障害物回避のバイパス)
  e           : 緊急停止 (オン)
  q           : 緊急停止 (解除/オフ)
  , (カンマ)  : 直進速度のみ停止 (0.0 m/s)
  . (ピリオド): 旋回速度のみ停止 (0.0 rad/s)
  space / k   : 完全停止

現在設定値:
  指示直進速度: {self.target_linear_vel:.2f} m/s (最大: {self.linear_vel_max:.2f})
  指示旋回速度: {self.target_angular_vel:.2f} rad/s (最大: {self.angular_vel_max:.2f})

終了するには Ctrl+C を押してください
---------------------------------------------
"""
        return msg

    def run(self):
        self.refresh_display()

        try:
            while rclpy.ok():
                key = self.getKey()
                if not key:
                    continue
                
                # 入力キー名称の取得
                key_name = KEY_NAMES.get(key, f"'{key}'")
                with self.lock:
                    self.last_key_name = key_name

                # 全角入力を半角に正規化して判定
                key_norm = key
                if key == '　':
                    key_norm = ' '
                elif key in ('、', '，'):
                    key_norm = ','
                elif key in ('。', '．'):
                    key_norm = '.'
                elif key == 'ｋ':
                    key_norm = 'k'
                elif key == 'ｅ':
                    key_norm = 'e'
                elif key == 'ｑ':
                    key_norm = 'q'
                elif key == 'ｔ':
                    key_norm = 't'

                key_lower = key_norm.lower()

                # アシスト機能のON/OFF切り替え
                if key_lower == 't':
                    with self.lock:
                        self.assisted_mode = not self.assisted_mode
                        # 切り替え時に古い指示を打ち消すため一度両方の系統に停止を送る
                        zero_twist = Twist()
                        self.pub.publish(zero_twist)
                        self.pub_direct.publish(zero_twist)
                        self.target_linear_vel = 0.0
                        self.target_angular_vel = 0.0
                    self.refresh_display()

                # 緊急停止（オン）
                elif key_lower == 'e':
                    with self.lock:
                        self.emergency_stop = True
                        self.target_linear_vel = 0.0
                        self.target_angular_vel = 0.0
                    stop_msg = Bool()
                    stop_msg.data = True
                    self.stop_pub.publish(stop_msg)
                    self.refresh_display()
                
                # 緊急停止解除（オフ）
                elif key_lower == 'q':
                    with self.lock:
                        self.emergency_stop = False
                    stop_msg = Bool()
                    stop_msg.data = False
                    self.stop_pub.publish(stop_msg)
                    self.refresh_display()

                # キー判定と処理 (緊急停止中は操作を受け付けない)
                elif not self.emergency_stop:
                    if key == '\x1b[A':  # UP arrow
                        with self.lock:
                            self.target_linear_vel = min(self.target_linear_vel + self.linear_vel_step, self.linear_vel_max)
                        self.refresh_display()
                    elif key == '\x1b[B':  # DOWN arrow
                        with self.lock:
                            self.target_linear_vel = max(self.target_linear_vel - self.linear_vel_step, -self.linear_vel_max)
                        self.refresh_display()
                    elif key == '\x1b[D':  # LEFT arrow
                        with self.lock:
                            self.target_angular_vel = min(self.target_angular_vel + self.angular_vel_step, self.angular_vel_max)
                        self.refresh_display()
                    elif key == '\x1b[C':  # RIGHT arrow
                        with self.lock:
                            self.target_angular_vel = max(self.target_angular_vel - self.angular_vel_step, -self.angular_vel_max)
                        self.refresh_display()
                    elif key_norm == ',':  # Stop linear only
                        with self.lock:
                            self.target_linear_vel = 0.0
                        self.refresh_display()
                    elif key_norm == '.':  # Stop angular only
                        with self.lock:
                            self.target_angular_vel = 0.0
                        self.refresh_display()
                    elif key_norm == ' ' or key_norm == 'k':  # Full stop
                        with self.lock:
                            self.target_linear_vel = 0.0
                            self.target_angular_vel = 0.0
                        self.refresh_display()
                    elif key == '\x03':  # Ctrl+C
                        break
                elif key == '\x03':  # Ctrl+C
                    break

                # Twist メッセージ送信 (緊急停止中は自動で 0.0 送信)
                twist = Twist()
                with self.lock:
                    twist.linear.x = self.target_linear_vel
                    twist.angular.z = self.target_angular_vel
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                
                # モードに応じて送信先トピックを切り替える
                if self.assisted_mode:
                    self.pub.publish(twist)
                else:
                    self.pub_direct.publish(twist)

        except Exception as e:
            self.get_logger().error(f'エラーが発生しました: {e}')

        finally:
            # 終了時に停止コマンドと緊急停止解除を送信して安全に終了
            twist = Twist()
            self.pub.publish(twist)
            self.pub_direct.publish(twist)
            
            stop_msg = Bool()
            stop_msg.data = False
            self.stop_pub.publish(stop_msg)
            
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = SiriusKeyboardTeleopJA()
    
    # サブスクリプション処理を非同期で行うため、別スレッドでスピンを開始
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
