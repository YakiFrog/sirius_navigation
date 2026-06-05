#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 操作キーと速度比率のマッピング
moveBindings = {
    'i': (1.0, 0.0),    # 前進
    'o': (1.0, -1.0),   # 右前進
    'j': (0.0, 1.0),    # 左旋回
    'l': (0.0, -1.0),   # 右旋回
    'u': (1.0, 1.0),    # 左前進
    ',': (-1.0, 0.0),   # 後退
    '.': (-1.0, 1.0),   # 右後退
    'm': (-1.0, -1.0),  # 左後退
}

speedBindings = {
    'q': (1.1, 1.0),   # 直進速度+10%
    'z': (0.9, 1.0),   # 直進速度-10%
    'w': (1.0, 1.1),   # 旋回速度+10%
    'x': (1.0, 0.9),   # 旋回速度-10%
}

class SiriusKeyboardTeleopJA(Node):
    def __init__(self):
        super().__init__('sirius_keyboard_teleop_ja')
        self.pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.speed = 0.5  # m/s
        self.turn = 1.0   # rad/s
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Sirius 日本語キーボード操作ノードが起動しました')

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def get_status_msg(self):
        msg = f"""
=== Sirius アシスト付きキーボード操作ガイド ===

操作キー:
  u    i    o    (左前進 / 前進 / 右前進)
  j    k    l    (左旋回 / 停止 / 右旋回)
  m    ,    .    (左後退 / 後退 / 右後退)

速度調整:
  q/z : 直進速度を10%上げる/下げる
  w/x : 旋回速度を10%上げる/下げる
  space / k : 完全停止

現在の設定速度:
  直進速度: {self.speed:.2f} m/s
  旋回速度: {self.turn:.2f} rad/s

終了するには Ctrl+C を押してください
---------------------------------------------
"""
        return msg

    def run(self):
        x = 0.0
        th = 0.0
        status = 0
        
        # 最初の画面表示
        print(self.get_status_msg())

        try:
            while True:
                key = self.getKey()
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    th = moveBindings[key][1]
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    # 画面をクリアして再表示
                    print("\033[H\033[J", end="")
                    print(self.get_status_msg())
                elif key == ' ' or key == 'k':
                    x = 0.0
                    th = 0.0
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    # キー入力がない場合のタイムアウト（約0.1秒）
                    # 押し続けられているときは前回の値を維持し、離されたら緩やかに停止するかは
                    # teleop_twist_keyboardの挙動に準拠し、ここではそのまま値をキープ
                    pass

                twist = Twist()
                twist.linear.x = x * self.speed
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = th * self.turn
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
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
