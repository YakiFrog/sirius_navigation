#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class TwistMuxMonitor(Node):
    def __init__(self):
        super().__init__('twist_mux_monitor')
        
        # 最後にメッセージを受信した時刻を保持
        self.last_teleop_time = 0.0
        self.last_nav_time = 0.0
        self.is_stopped = False
        self.last_stop_time = 0.0
        
        # 現在の勝者
        self.current_winner = "IDLE"
        self.current_priority = 0
        
        # 購読設定
        self.create_subscription(Twist, 'cmd_vel_teleop', self.teleop_callback, 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self.nav_callback, 10)
        self.create_subscription(Bool, '/stop', self.stop_callback, 10)
        
        # Idle出力用のパブリッシャー
        self.idle_pub = self.create_publisher(Twist, 'cmd_vel_idle', 10)
        
        # 10Hzで状態チェック
        self.create_timer(0.1, self.check_status)
        
        self.get_logger().info('✅ TwistMux Monitor started')

    def teleop_callback(self, msg):
        self.last_teleop_time = self.get_clock().now().nanoseconds / 1e9

    def nav_callback(self, msg):
        self.last_nav_time = self.get_clock().now().nanoseconds / 1e9

    def stop_callback(self, msg):
        self.is_stopped = msg.data
        if self.is_stopped:
            self.last_stop_time = self.get_clock().now().nanoseconds / 1e9

    def check_status(self):
        now = self.get_clock().now().nanoseconds / 1e9
        timeout = 0.5  # yamlのtimeout設定に合わせる
        
        winner = "IDLE"
        priority = 0
        
        # 常にIDLE（速度0）を投げ続ける
        idle_msg = Twist()
        self.idle_pub.publish(idle_msg)

        # 優先順位が高い順にチェック
        # 1. STOP (255)
        if self.is_stopped and (now - self.last_stop_time < 1.0):
            winner = "STOP (LOCKED)"
            priority = 255
        # 2. TELEOP (100)
        elif (now - self.last_teleop_time < timeout):
            winner = "TELEOP (Manual)"
            priority = 100
        # 3. NAVIGATION (10)
        elif (now - self.last_nav_time < timeout):
            winner = "NAVIGATION (Auto)"
            priority = 10
            
        # 状態が変わった時だけログ出力
        if winner != self.current_winner:
            self.current_winner = winner
            self.current_priority = priority
            
            icon = "⚪"
            if priority == 255: icon = "🛑"
            elif priority == 100: icon = "🎮"
            elif priority == 10: icon = "🤖"
            
            self.get_logger().info(f'[TwistMux] {icon} Active Source: {winner} (Priority: {priority})')

def main(args=None):
    rclpy.init(args=args)
    node = TwistMuxMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
