#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener, LookupException
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
import math
import signal

class TargetFollower(Node):
    """
    Unityシミュレータ上のNPC（ターゲット）をROS 2 / Nav2を介して動的に追従するノード。
    ターゲットの位置情報を購読し、一定の距離（維持距離）を保つための目標位置を計算してNav2に送信します。
    """
    def __init__(self):
        super().__init__('target_follower')
        
        # ROS 2 パラメータの宣言と初期化
        self.declare_parameter('enable_following', False)     # 自律追従の有効化フラグ
        self.declare_parameter('follow_distance', 0.6)     # ターゲットとの維持目標距離（メートル）
        self.declare_parameter('min_update_distance', 0.15)   # ターゲットがこの距離以上動いたらゴールを再送信
        self.declare_parameter('control_rate', 2.0)           # 制御ループの実行頻度（Hz）
        self.declare_parameter('deadband', 0.1)               # 停止判定用の不感帯（維持距離±10cm）
        self.declare_parameter('robot_base_frame', 'sirius3/base_footprint') # ロボットのベースフレーム
        self.declare_parameter('tracking_soft_warning_distance', 4.0) # 軽い距離警告を出す距離（m）
        self.declare_parameter('tracking_warning_distance', 8.0) # 見失い警告を出す距離（m）
        
        # パラメータ値の取得
        self.enable_following = self.get_parameter('enable_following').value
        self.follow_distance = self.get_parameter('follow_distance').value
        self.min_update_distance = self.get_parameter('min_update_distance').value
        self.control_rate = self.get_parameter('control_rate').value
        self.deadband = self.get_parameter('deadband').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.tracking_soft_warning_distance = self.get_parameter('tracking_soft_warning_distance').value
        self.tracking_warning_distance = self.get_parameter('tracking_warning_distance').value
        
        # パラメータが動的に変更された際のコールバック関数を登録
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Nav2のNavigateToPoseアクションクライアントを作成
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # ロボットを即座に停止させるためのキャンセルサービス（直接アクションサーバーにキャンセル要求を投げる）
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        
        # ロボットの現在位置（自己位置）を取得するためのTFリスナーをセットアップ
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Unity上のNPCが配信するグラウンドトゥルース（真値）の位置トピックを購読
        self.target_sub = self.create_subscription(
            Odometry,
            '/npc/odom',
            self.target_callback,
            10
        )
        self.status_pub = self.create_publisher(String, '/target_follower/status', 10)
        
        # 各種状態変数の初期化
        self.target_pose = None             # 最新のターゲット座標
        self.last_target_time = None        # 最新のターゲット座標を受信した時刻
        self.last_sent_target_pose = None   # 最後にNav2にゴールを送信した時のターゲット座標
        self.goal_handle = None             # アクションのゴールハンドル（予備キャンセル用）
        self.goal_sending_in_progress = False # 多重送信防止フラグ
        self.is_goal_active = False         # アクション実行中フラグ
        self.has_seen_target_while_following = False
        self.tracking_lost_reported = False
        self.tracking_soft_far_reported = False
        self.tracking_far_reported = False
        self.tracking_waiting_reported = False

        
        # 制御ループタイマーの開始
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        
        self.get_logger().info(
            f"ターゲット追従ノードが起動しました。\n"
            f"  購読トピック: /npc/odom\n"
            f"  追従距離: {self.follow_distance}m\n"
            f"  最小更新移動距離: {self.min_update_distance}m\n"
            f"  不感帯 (デッドバンド): {self.deadband}m"
        )

    def parameter_callback(self, params):
        """
        ROS 2のパラメータが動的（BASHのコマンドなど）に変更されたときに呼び出されるコールバック。
        """
        for param in params:
            if param.name == 'enable_following':
                self.enable_following = param.value
                self.get_logger().info(f"パラメータ 'enable_following' が更新されました: {self.enable_following}")
                if self.enable_following:
                    self.has_seen_target_while_following = self.target_pose is not None
                    self.tracking_lost_reported = False
                    self.tracking_soft_far_reported = False
                    self.tracking_far_reported = False
                    self.tracking_waiting_reported = False
                # 追従が無効化された場合は、現在動いているロボットを即座に停止する
                if not self.enable_following:
                    self.has_seen_target_while_following = False
                    self.tracking_lost_reported = False
                    self.tracking_soft_far_reported = False
                    self.tracking_far_reported = False
                    self.tracking_waiting_reported = False
                    self.cancel_current_goal()
            elif param.name == 'follow_distance':
                self.follow_distance = param.value
                self.get_logger().info(f"パラメータ 'follow_distance' が更新されました: {self.follow_distance}m")
            elif param.name == 'min_update_distance':
                self.min_update_distance = param.value
                self.get_logger().info(f"パラメータ 'min_update_distance' が更新されました: {self.min_update_distance}m")
            elif param.name == 'deadband':
                self.deadband = param.value
                self.get_logger().info(f"パラメータ 'deadband' が更新されました: {self.deadband}m")
            elif param.name == 'control_rate':
                self.control_rate = param.value
                self.get_logger().info(f"パラメータ 'control_rate' が更新されました: {self.control_rate}Hz")
                # タイマー周期を動的に更新
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
            elif param.name == 'robot_base_frame':
                self.robot_base_frame = param.value
                self.get_logger().info(f"パラメータ 'robot_base_frame' が更新されました: {self.robot_base_frame}")
            elif param.name == 'tracking_soft_warning_distance':
                self.tracking_soft_warning_distance = param.value
                self.get_logger().info(f"パラメータ 'tracking_soft_warning_distance' が更新されました: {self.tracking_soft_warning_distance}m")
            elif param.name == 'tracking_warning_distance':
                self.tracking_warning_distance = param.value
                self.get_logger().info(f"パラメータ 'tracking_warning_distance' が更新されました: {self.tracking_warning_distance}m")
        return SetParametersResult(successful=True)

    def target_callback(self, msg: Odometry):
        """
        Unity上のNPCの位置トピック (/npc/odom) を受信したときに座標を更新するコールバック。
        """
        self.target_pose = msg.pose.pose
        self.last_target_time = self.get_clock().now()
        if self.enable_following:
            if self.tracking_lost_reported:
                self.publish_status("tracking_recovered")
                self.tracking_soft_far_reported = False
                self.tracking_far_reported = False
            self.has_seen_target_while_following = True
            self.tracking_lost_reported = False
            self.tracking_waiting_reported = False

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def cancel_current_goal(self, force=False):
        """
        現在Nav2で実行中の移動目標をすべてキャンセルし、ロボットを停止させる関数。
        """
        if not self.is_goal_active and not force:
            return
            
        self.get_logger().info("実行中のすべてのNavigateToPoseゴールをキャンセルしています...")
        self.last_sent_target_pose = None
        self.goal_handle = None
        self.is_goal_active = False
        
        # アクションのキャンセルサービスが立ち上がっているか確認
        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("キャンセルサービスが利用できません。アクションクライアントから個別キャンセルを試みます...")
            # サービスがダメなら、古い個別ゴールハンドルでのキャンセルを試みる（フォールバック）
            if self.goal_handle is not None:
                try:
                    self.goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().error(f"フォールバックキャンセルに失敗しました: {e}")
            return
            
        # キャンセルリクエストの送信（空のGoalInfoを渡すことで、該当アクションの全目標をキャンセル）
        req = CancelGoal.Request()
        req.goal_info = GoalInfo()
        self.cancel_client.call_async(req)

    def control_loop(self):
        """
        周期的に実行され、追従目標を計算・送信するメインの制御ループ。
        """
        # 追従が無効化されている場合は何もしない
        if not self.enable_following:
            return
            
        # ターゲットの位置情報がまだ届いていない場合は待機
        now = self.get_clock().now()
        if self.target_pose is None:
            if not self.tracking_waiting_reported:
                self.publish_status("tracking_waiting")
                self.tracking_waiting_reported = True
            self.get_logger().info("トピック /npc/odom からターゲット位置が届くのを待っています...", throttle_duration_sec=5.0)
            return

        if self.last_target_time is not None:
            elapsed = (now - self.last_target_time).nanoseconds / 1e9
            if elapsed > 5.0:  # 2秒→5秒に延長（一時的なトラッキングロストでキャンセルしない）
                self.get_logger().warning("ターゲットからのデータ更新が途絶えました。追従を一時停止します。", throttle_duration_sec=5.0)
                if self.has_seen_target_while_following and not self.tracking_lost_reported:
                    self.publish_status("tracking_lost")
                    self.tracking_lost_reported = True
                self.target_pose = None
                self.cancel_current_goal()
                return

        # 1. TFを使用してロボットの現在位置（map座標系上の base_footprint）を取得
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.robot_base_frame,
                now,
                rclpy.duration.Duration(seconds=0.1)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except LookupException as e:
            self.get_logger().warning(f"ロボットのTF座標を取得できませんでした: {e}", throttle_duration_sec=3.0)
            return
        except Exception as e:
            self.get_logger().warning(f"TFエラーが発生しました: {e}", throttle_duration_sec=3.0)
            return

        # ターゲット（NPC）の座標を取得
        target_x = self.target_pose.position.x
        target_y = self.target_pose.position.y

        # ロボットとターゲットとの間の直線距離を計算
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.sqrt(dx**2 + dy**2)

        # 目標距離とのズレを計算 (例: 現在距離 1.3m - 目標 1.2m = +0.1m)
        dist_diff = dist - self.follow_distance

        soft_warning_reset_distance = max(self.follow_distance + 0.5, self.tracking_soft_warning_distance - 0.75)
        warning_reset_distance = max(self.follow_distance + 0.5, self.tracking_warning_distance - 1.0)
        if dist >= self.tracking_soft_warning_distance and not self.tracking_soft_far_reported:
            self.publish_status(f"tracking_soft_far:{dist:.2f}")
            self.tracking_soft_far_reported = True
        elif dist < soft_warning_reset_distance:
            self.tracking_soft_far_reported = False

        if dist >= self.tracking_warning_distance and not self.tracking_far_reported:
            self.publish_status(f"tracking_far:{dist:.2f}")
            self.tracking_far_reported = True
        elif dist < warning_reset_distance:
            self.tracking_far_reported = False

        # 2. ターゲットが不感帯内（維持目標距離の前後 deadband）に入った場合は停止
        if abs(dist_diff) <= self.deadband:
            if self.is_goal_active:
                self.get_logger().info(
                    f"ターゲットに到達しました (距離: {dist:.2f}m / 目標: {self.follow_distance:.2f}m)。停止します。",
                    throttle_duration_sec=3.0
                )
                self.cancel_current_goal()
            return

        # 3. ターゲットがあまり動いていない場合は無駄なゴール更新をスキップ（通信量と計算負荷の削減）
        # ただし、ターゲットが「近づきすぎ」の状態（目標距離より手前）の場合は、後退を強制するためスキップしない
        if dist_diff >= -self.deadband and self.last_sent_target_pose is not None:
            tdx = target_x - self.last_sent_target_pose.position.x
            tdy = target_y - self.last_sent_target_pose.position.y
            moved_dist = math.sqrt(tdx**2 + tdy**2)
            if moved_dist < self.min_update_distance:
                # ターゲットの移動量が微小なため更新をスキップ
                return

        # 4. アクションサーバーが準備完了しているか確認
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warning("NavigateToPose アクションサーバーが起動していません。", throttle_duration_sec=5.0)
            return

        # 5. すでにゴール送信処理が走っている場合は重複送信を避ける
        if self.goal_sending_in_progress:
            return

        # ロボットからターゲットへの角度を計算
        theta = math.atan2(dy, dx)
        
        # 目標座標の計算: ターゲットの位置から「維持距離」分だけ手前の座標を求める
        # （ターゲットが近すぎる場合、この値はロボットの後方になり、後退（バック）がトリガーされます）
        goal_x = target_x - self.follow_distance * math.cos(theta)
        goal_y = target_y - self.follow_distance * math.sin(theta)

        # Nav2用のゴールメッセージを作成
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        
        # ロボットが常にターゲット（人間）の方向を向くようにゴール姿勢（ヨー角）を計算しクォータニオンに変換
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.get_logger().info(
            f"追従目標を送信します: ターゲットとの距離={dist:.2f}m -> 目標座標: ({goal_x:.2f}, {goal_y:.2f}) 角度={theta:.2f}rad"
        )
        
        # 送信した状態を記録
        self.last_sent_target_pose = self.target_pose
        self.goal_sending_in_progress = True
        
        # 非同期でゴールを送信
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        送信したゴールがサーバーに受理されたかどうかの結果を受け取るコールバック。
        """
        self.goal_sending_in_progress = False
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning("追従目標がNav2サーバーによって拒否されました。")
                self.is_goal_active = False
                return
            
            # アクティブなゴールハンドルを保存
            self.goal_handle = goal_handle
            self.is_goal_active = True
            
            # ゴールの実行結果を非同期で監視するコールバックを登録 (クロージャで goal_handle を渡す)
            result_future = self.goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda future_result, gh=goal_handle: self.goal_result_callback(future_result, gh)
            )
        except Exception as e:
            self.get_logger().error(f"ゴール応答コールバックでエラーが発生しました: {e}")

    def goal_result_callback(self, future, goal_handle):
        """
        ゴールの実行が完了（成功、中断、キャンセルなど）したときに呼び出されるコールバック。
        """
        # 現在のアクティブなゴールハンドルと一致する場合のみ状態を更新
        if self.goal_handle is not None and goal_handle == self.goal_handle:
            self.is_goal_active = False
            self.goal_handle = None
            
            status = future.result().status
            self.get_logger().info(f"アクティブなゴールが終了しました。ステータスコード: {status}")

def sigterm_handler(signum, frame):
    """
    pkillなどの強制終了シグナル (SIGTERM) を受け取った際に、
    KeyboardInterruptを発生させて安全な終了処理 (finallyブロック) を実行するためのハンドラ。
    """
    raise KeyboardInterrupt

def main(args=None):
    # システムシグナルハンドラの登録
    signal.signal(signal.SIGTERM, sigterm_handler)
    
    rclpy.init(args=args)
    node = TargetFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ノード終了時に、Nav2上のゴールをキャンセルしてロボットを停止状態にする
        node.cancel_current_goal(force=True)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
