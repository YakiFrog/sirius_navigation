#!/usr/bin/env python3
import sys
import select
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SiriusKeyboardTeleopJA(Node):
    def __init__(self):
        super().__init__('sirius_keyboard_teleop_ja')
        self.pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        
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
        self.linear_vel_max = 1.2      # m/s
        self.angular_vel_step = 0.15   # rad/s
        self.angular_vel_max = 1.8     # rad/s
        
        self.blocked = False           # 障害物による停止・減速フラグ
        self.lock = threading.Lock()
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Sirius 日本語キーボード操作ノード（scn互換）が起動しました')

    def cmdVelCallback(self, msg):
        with self.lock:
            # 指示速度に対して、実際の出力速度（cmd_vel）が著しく制限されているかを判定
            is_blocked = False
            
            # 直進方向のブロッキング判定
            if abs(self.target_linear_vel) > 0.05:
                if self.target_linear_vel > 0:
                    if msg.linear.x < 0.05:  # 前進指示に対して進んでいない
                        is_blocked = True
                elif self.target_linear_vel < 0:
                    if msg.linear.x > -0.05:  # 後退指示に対して進んでいない
                        is_blocked = True
                        
            # 旋回方向のブロッキング判定
            if abs(self.target_angular_vel) > 0.05:
                if self.target_angular_vel > 0:
                    if msg.angular.z < 0.05:  # 左旋回指示に対して旋回していない
                        is_blocked = True
                elif self.target_angular_vel < 0:
                    if msg.angular.z > -0.05:  # 右旋回指示に対して旋回していない
                        is_blocked = True
            
            if is_blocked != self.blocked:
                self.blocked = is_blocked
                # 画面表示を即時更新
                print("\033[H\033[J", end="")
                print(self.get_status_msg())

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        # select.select で入力待ち（タイムアウト 0.1秒）
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = ''
        if rlist:
            key = sys.stdin.read(1)
            # エスケープシーケンス（矢印キーなど）のハンドリング
            if key == '\x1b':
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def get_status_msg(self):
        warning_msg = ""
        if self.blocked:
            warning_msg = "\033[1;31m⚠️  [警告] 目の前または進路に障害物があるため動けません (制限中)\033[0m\n"
        else:
            warning_msg = "\033[1;32m✅  進路クリア (安全に走行可能です)\033[0m\n"

        msg = f"""
=== Sirius アシスト付きキーボード操作ガイド (scn互換) ===

ステータス:
  {warning_msg}
操作キー:
  ↑ (矢印上)  : 直進速度を上げる (+{self.linear_vel_step:.2f} m/s)
  ↓ (矢印下)  : 直進速度を下げる (-{self.linear_vel_step:.2f} m/s)
  ← (矢印左)  : 左旋回速度を上げる (+{self.angular_vel_step:.2f} rad/s)
  → (矢印右)  : 右旋回速度を上げる (-{self.angular_vel_step:.2f} rad/s)

停止キー:
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
        # 最初の画面表示
        print(self.get_status_msg())

        try:
            while rclpy.ok():
                key = self.getKey()
                
                # キー判定と処理
                if key == '\x1b[A':  # UP arrow
                    with self.lock:
                        self.target_linear_vel = min(self.target_linear_vel + self.linear_vel_step, self.linear_vel_max)
                    print("\033[H\033[J", end="")
                    print(self.get_status_msg())
                elif key == '\x1b[B':  # DOWN arrow
                    with self.lock:
                        self.target_linear_vel = max(self.target_linear_vel - self.linear_vel_step, -self.linear_vel_max)
                    print("\033[H\033[J", end="")
                    print(self.get_status_msg())
                elif key == '\x1b[D':  # LEFT arrow
                    with self.lock:
                        self.target_angular_vel = min(self.target_angular_vel + self.angular_vel_step, self.angular_vel_max)
                    print("\033[H\033[J", end="")
                    print(self.get_status_msg())
                elif key == '\x1b[C':  # RIGHT arrow
                    with self.lock:
                        self.target_angular_vel = max(self.target_angular_vel - self.angular_vel_step, -self.angular_vel_max)
                    print("\033[H\033[J", end="")
                    print(self.get_status_msg())
                elif key == ',':  # Stop linear only
                    with self.lock:
                        self.target_linear_vel = 0.0
                    print("\033[H\033[J", end="")
                    print(self.get_status_msg())
                elif key == '.':  # Stop angular only
                    with self.lock:
                        self.target_angular_vel = 0.0
                    print("\033[H\033[J", end="")
                    print(self.get_status_msg())
                elif key == ' ' or key == 'k':  # Full stop
                    with self.lock:
                        self.target_linear_vel = 0.0
                        self.target_angular_vel = 0.0
                    print("\033[H\033[J", end="")
                    print(self.get_status_msg())
                elif key == '\x03':  # Ctrl+C
                    break

                # Twist メッセージ送信
                twist = Twist()
                with self.lock:
                    twist.linear.x = self.target_linear_vel
                    twist.angular.z = self.target_angular_vel
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                self.pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f'エラーが発生しました: {e}')

        finally:
            # 終了時に停止コマンドを送信
            twist = Twist()
            self.pub.publish(twist)
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
