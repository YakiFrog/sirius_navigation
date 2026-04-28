#!/usr/bin/env python3
"""
EKF Pose Initializer
RViz2の/initialposeをEKFのset_poseサービスに転送する
AMCLとEKFの初期位置を同期させる
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import SetPose


class EKFPoseInitializer(Node):
    def __init__(self):
        super().__init__('ekf_pose_initializer')
        
        # /initialposeトピックを購読（RViz2の2D Pose Estimate）
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )
        
        # EKFのset_poseサービスクライアント
        self.set_pose_client = self.create_client(
            SetPose,
            '/set_pose'
        )
        
        # サービスが利用可能になるまで待機（最大10秒）
        self.get_logger().info('EKF set_pose サービスを待機中...')
        retry_count = 0
        max_retries = 30
        
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            retry_count += 1
            if retry_count >= max_retries:
                self.get_logger().error('❌ EKF set_pose サービスが利用できません（タイムアウト）')
                self.get_logger().error('   EKFノードが起動しているか確認してください')
                break
            # 3秒ごとにログ出力（ログを減らす）
            if retry_count % 3 == 0:
                self.get_logger().info(f'待機中... ({retry_count}/{max_retries})')
        else:
            self.get_logger().info('✅ EKF Pose Initializer 起動完了')
            self.get_logger().info('   RViz2で "2D Pose Estimate" を設定できます')
    
    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """RViz2からの/initialposeを受信してEKFに転送"""
        self.get_logger().info(f'📍 Initial Pose受信: x={msg.pose.pose.position.x:.2f}, '
                               f'y={msg.pose.pose.position.y:.2f}')
        
        # SetPoseサービスリクエストを作成
        request = SetPose.Request()
        request.pose = msg
        
        # 非同期でサービスを呼び出し
        future = self.set_pose_client.call_async(request)
        future.add_done_callback(self.set_pose_callback)
    
    def set_pose_callback(self, future):
        """サービス呼び出しの結果を処理"""
        try:
            response = future.result()
            self.get_logger().info('✅ EKFの位置を初期化しました')
        except Exception as e:
            self.get_logger().error(f'❌ EKF位置初期化失敗: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = EKFPoseInitializer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
