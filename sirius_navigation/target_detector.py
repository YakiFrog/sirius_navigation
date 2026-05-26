#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import math
import numpy as np

class TargetDetector(Node):
    """
    LaserScan (/scan3) から人間（円柱状のターゲット）を検出し、
    カルマンフィルターで追従（トラッキング）して /npc/odom に配信するノード。
    起動時にロボット正面の人物の特徴（幅、点群密度）を抽出し、特徴ベースのコスト関数で追跡します。
    """
    def __init__(self):
        super().__init__('target_detector')

        # パラメータの宣言と初期値設定
        self.declare_parameter('scan_topic', '/scan3')
        self.declare_parameter('odom_topic', '/npc/odom')
        self.declare_parameter('min_range', 0.3)             # 検出最小距離（ロボット自身を弾く）
        self.declare_parameter('max_range', 3.5)             # 検出最大距離（3.5m以内）
        self.declare_parameter('fov_deg', 120.0)             # 前方視野角（左右60度、計120度）
        self.declare_parameter('cluster_tolerance', 0.25)    # クラスタリング時の点同士の最大距離（25cm）
        self.declare_parameter('min_cluster_size', 3)        # クラスタを構成する最小点数
        self.declare_parameter('max_cluster_size', 60)       # クラスタを構成する最大点数
        self.declare_parameter('min_human_width', 0.10)      # 人間（脚/胴体）の最小幅（10cm）
        self.declare_parameter('max_human_width', 0.60)      # 人間（脚/胴体）の最大幅（60cm）
        self.declare_parameter('gating_distance', 0.8)       # カルマンフィルターのデータ関連付けゲート距離（80cm）
        self.declare_parameter('max_lost_frames', 5)         # ターゲットを見失ったと判定する連続フレーム数（約0.5秒）
        self.declare_parameter('active_max_range', 5.0)      # 追従中の最大検出距離（5.0m）
        self.declare_parameter('active_fov_deg', 360.0)      # 追従中の視野角（360度全方位）

        # パラメータの取得
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.fov_deg = self.get_parameter('fov_deg').value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value
        self.min_human_width = self.get_parameter('min_human_width').value
        self.max_human_width = self.get_parameter('max_human_width').value
        self.gating_distance = self.get_parameter('gating_distance').value
        self.max_lost_frames = self.get_parameter('max_lost_frames').value
        self.active_max_range = self.get_parameter('active_max_range').value
        self.active_fov_deg = self.get_parameter('active_fov_deg').value

        # パブリッシャーとサブスクライバーのセットアップ
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        from visualization_msgs.msg import Marker
        self.marker_pub = self.create_publisher(Marker, '/target_detector/range_marker', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/target_detector/target_marker', 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        # TFリスナーのセットアップ（センサ座標系からmap座標系への変換用）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # カルマンフィルター状態変数の初期化
        # 状態ベクトル X = [x, y, vx, vy]^T (ロボットローカルのセンサ座標系基準)
        self.kf_state = np.zeros(4)
        self.kf_cov = np.eye(4) * 1.0

        # カルマンフィルターパラメータ
        # プロセスノイズ共分散 Q (小さくして運動モデルの滑らかさを重視)
        self.Q = np.diag([0.002, 0.002, 0.01, 0.01])
        # 観測行列 H
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])
        # 観測ノイズ共分散 R (大きくして点群重心の測定ノイズを滑らかに除去)
        self.R = np.diag([0.10, 0.10])

        # トラッキング状態および特徴量管理
        self.is_tracking = False
        self.lost_count = 0
        self.last_time = None
        
        # ロックオンしたターゲットの特徴量
        self.locked_width = 0.0             # ターゲットの物理幅（m）
        self.locked_points_factor = 0.0     # ターゲットの点群密度係数（点数 × 距離）

        # コスト計算用の重み付け
        self.w_dist = 1.0                   # 距離差の重み
        self.w_width = 2.0                  # 幅の差の重み
        self.w_points = 1.0                 # 点群密度差の重み
        self.max_gating_cost = 1.5          # このコスト値以下のクラスタのみ同一人物と判定

        self.get_logger().info(
            f"Target Detector Node (Feature-Based) Initialized.\n"
            f"  Subscribing to: {self.scan_topic}\n"
            f"  Publishing to: {self.odom_topic}\n"
            f"  Range: {self.min_range}m ~ {self.max_range}m\n"
            f"  FOV: {self.fov_deg} deg"
        )

    def scan_callback(self, msg: LaserScan):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            dt = 0.1  # 異常な時間差の場合は10Hz（0.1秒）を仮定

        # 1. 2Dレーザースキャンから極座標 -> デカルト座標(x, y)への変換 & 領域フィルタリング
        points = []
        angle = msg.angle_min
        
        # 追従状況に応じて走査範囲（距離・視野角）を動的に切り替える
        if self.is_tracking:
            max_r = self.active_max_range
            fov_r = self.active_fov_deg * math.pi / 180.0
        else:
            max_r = self.max_range
            fov_r = self.fov_deg * math.pi / 180.0

        for r in msg.ranges:
            # 距離フィルタ
            if self.min_range <= r <= max_r:
                # 視野角フィルタ (前方左右 fov_r/2 以内)
                if abs(angle) <= fov_r / 2.0:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    points.append((x, y))
            angle += msg.angle_increment

        # 2. 連続する点の距離による簡易クラスタリング
        clusters = []
        current_cluster = []

        for p in points:
            if not current_cluster:
                current_cluster.append(p)
            else:
                last_p = current_cluster[-1]
                dist = math.sqrt((p[0] - last_p[0])**2 + (p[1] - last_p[1])**2)
                if dist <= self.cluster_tolerance:
                    current_cluster.append(p)
                else:
                    clusters.append(current_cluster)
                    current_cluster = [p]
        if current_cluster:
            clusters.append(current_cluster)

        # 3. 各クラスタのサイズと幅による人間判定フィルタ
        detected_targets = []

        for cluster in clusters:
            # 点数フィルタ
            if self.min_cluster_size <= len(cluster) <= self.max_cluster_size:
                # 幅フィルタ (最初と最後の点の直線距離で近似)
                width = math.sqrt((cluster[0][0] - cluster[-1][0])**2 + (cluster[0][1] - cluster[-1][1])**2)
                if self.min_human_width <= width <= self.max_human_width:
                    # 重心(Centroid)の計算
                    cx = sum(p[0] for p in cluster) / len(cluster)
                    cy = sum(p[1] for p in cluster) / len(cluster)
                    dist = math.sqrt(cx**2 + cy**2)
                    detected_targets.append({
                        'centroid': (cx, cy),
                        'width': width,
                        'points_count': len(cluster),
                        'distance': dist,
                        'points_factor': len(cluster) * dist
                    })

        # 4. カルマンフィルターによる追跡更新
        # 状態遷移行列 F
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # 状態予測 (Prediction)
        pred_state = F @ self.kf_state
        pred_cov = F @ self.kf_cov @ F.T + self.Q

        measurement_updated = False
        measured_x = 0.0
        measured_y = 0.0

        if detected_targets:
            # 追跡中なら、複数特徴（位置・幅・点群密度）を考慮したコスト関数で同一判定を行う
            if self.is_tracking:
                best_target = None
                lowest_cost = float('inf')
                
                for target in detected_targets:
                    cx, cy = target['centroid']
                    # 1. 予測位置からの距離差
                    d_dist = math.sqrt((cx - pred_state[0])**2 + (cy - pred_state[1])**2)
                    if d_dist > self.gating_distance:
                        continue # ゲート外の極端に離れた物体はスキップ
                        
                    # 2. 幅の差
                    d_width = abs(target['width'] - self.locked_width)
                    
                    # 3. 点群密度係数の差 (比率)
                    if self.locked_points_factor > 0:
                        d_points = abs(target['points_factor'] - self.locked_points_factor) / self.locked_points_factor
                    else:
                        d_points = 0.0
                        
                    # 統合コスト計算 (距離 + 幅の差 + 点群密度の差)
                    cost = self.w_dist * d_dist + self.w_width * d_width + self.w_points * d_points
                    
                    if cost < lowest_cost:
                        lowest_cost = cost
                        best_target = target

                # ゲートしきい値判定 (コストが許容値以下なら観測値として採択)
                if best_target is not None and lowest_cost <= self.max_gating_cost:
                    measured_x, measured_y = best_target['centroid']
                    measurement_updated = True
                    
                    # ロックオンした特徴量の緩やかな適応的アップデート (人の体の向き変更への追従用)
                    self.locked_width = 0.95 * self.locked_width + 0.05 * best_target['width']
                    self.locked_points_factor = 0.95 * self.locked_points_factor + 0.05 * best_target['points_factor']
            else:
                # 未追跡時：ロボットの「正面」（X軸の0.5m〜2.5m以内、Y軸の左右±50cm以内）にいるターゲットでロックオン
                best_target = None
                closest_dist = float('inf')
                
                for target in detected_targets:
                    cx, cy = target['centroid']
                    # 正面判定 (ロボットから正面方向1.0m以内、左右50cm以内)
                    if 0.3 <= cx <= 1.0 and abs(cy) <= 0.5:
                        if target['distance'] < closest_dist:
                            closest_dist = target['distance']
                            best_target = target

                if best_target is not None:
                    cx, cy = best_target['centroid']
                    # カルマンフィルター状態の初期化 (初期速度は0)
                    self.kf_state = np.array([cx, cy, 0.0, 0.0])
                    self.kf_cov = np.eye(4) * 0.5
                    
                    # 初期特徴量のロックオン
                    self.locked_width = best_target['width']
                    self.locked_points_factor = best_target['points_factor']
                    
                    self.is_tracking = True
                    self.lost_count = 0
                    measurement_updated = True
                    measured_x, measured_y = cx, cy
                    
                    self.get_logger().info(
                        f"ターゲットを正面でロックオンしました！\n"
                        f"  位置: ({cx:.2f}, {cy:.2f}) 距離: {best_target['distance']:.2f}m\n"
                        f"  初期幅: {self.locked_width:.2f}m\n"
                        f"  初期点群係数: {self.locked_points_factor:.1f}"
                    )

        # 観測更新 (Update) or デッドレコニング (Prediction Only)
        if measurement_updated:
            # 残差 (Innovation)
            Z = np.array([measured_x, measured_y])
            Y = Z - self.H @ pred_state
            
            # カルマンゲイン K の計算
            S = self.H @ pred_cov @ self.H.T + self.R
            K = pred_cov @ self.H.T @ np.linalg.inv(S)
            
            # 状態と共分散の更新
            self.kf_state = pred_state + K @ Y
            self.kf_cov = (np.eye(4) - K @ self.H) @ pred_cov
            self.lost_count = 0
        else:
            if self.is_tracking:
                self.lost_count += 1
                if self.lost_count >= self.max_lost_frames:
                    # 一定時間検出できなければ追跡終了し、特徴量もクリア
                    self.is_tracking = False
                    self.locked_width = 0.0
                    self.locked_points_factor = 0.0
                    self.get_logger().warn("Target lost. Feature lock cleared.")
                else:
                    # 見失い中は等速モデルで予測（デッドレコニング）。ただし速度は徐々に減衰させる
                    self.kf_state = pred_state
                    self.kf_state[2] *= 0.9  # vx 減衰
                    self.kf_state[3] *= 0.9  # vy 減衰
                    self.kf_cov = pred_cov

        # 5. マップフレーム(map)への座標変換とOdometryメッセージの配信
        if self.is_tracking:
            # センサフレーム (LaserScanのフレーム) から map フレームへの変換を取得
            try:
                # センサのタイムスタンプに合わせたTF変換の取得
                trans = self.tf_buffer.lookup_transform(
                    'map',
                    msg.header.frame_id,
                    msg.header.stamp,
                    rclpy.duration.Duration(seconds=0.1)
                )
            except Exception:
                # 失敗した場合は最新のTFで試行する
                try:
                    trans = self.tf_buffer.lookup_transform(
                        'map',
                        msg.header.frame_id,
                        rclpy.time.Time(),
                        rclpy.duration.Duration(seconds=0.1)
                    )
                except Exception as e:
                    self.get_logger().warning(f"TF lookup failed: {e}", throttle_duration_sec=3.0)
                    return

            # クォータニオンから2D Yaw角への変換
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))

            # 並進移動量
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y

            # ターゲット位置をmap座標系へ変換 (2D回転 + 並進)
            local_x = self.kf_state[0]
            local_y = self.kf_state[1]
            map_x = local_x * math.cos(yaw) - local_y * math.sin(yaw) + tx
            map_y = local_x * math.sin(yaw) + local_y * math.cos(yaw) + ty

            # ターゲット速度をmap座標系へ変換 (回転のみ)
            local_vx = self.kf_state[2]
            local_vy = self.kf_state[3]
            map_vx = local_vx * math.cos(yaw) - local_vy * math.sin(yaw)
            map_vy = local_vx * math.sin(yaw) + local_vy * math.cos(yaw)

            # Odometry メッセージの作成と公開
            odom_msg = Odometry()
            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'npc/base_link'

            # Position
            odom_msg.pose.pose.position.x = map_x
            odom_msg.pose.pose.position.y = map_y
            odom_msg.pose.pose.position.z = 0.0

            # Orientation
            target_yaw = math.atan2(map_vy, map_vx) if (abs(map_vx) > 0.05 or abs(map_vy) > 0.05) else yaw
            odom_msg.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
            odom_msg.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

            # Velocity
            odom_msg.twist.twist.linear.x = map_vx
            odom_msg.twist.twist.linear.y = map_vy

            self.odom_pub.publish(odom_msg)
            # ターゲット位置のシリンダーマーカーをパブリッシュ
            self.publish_target_marker(map_x, map_y, now.to_msg())
        else:
            # ターゲットマーカーを削除
            self.delete_target_marker(now.to_msg())

        # 検出可能範囲（扇形/円）のマーカーをパブリッシュ
        self.publish_range_marker(msg.header.frame_id, now.to_msg())

    def publish_range_marker(self, frame_id, stamp):
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'detection_range'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        
        # 線の太さ
        marker.scale.x = 0.03
        
        if self.is_tracking:
            # 追従中: 青い全方位の円 (5.0m)
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 0.4
            
            radius = self.active_max_range
            steps = 36
            points = []
            for i in range(steps + 1):
                theta = i * (2.0 * math.pi / steps)
                p = Point()
                p.x = radius * math.cos(theta)
                p.y = radius * math.sin(theta)
                p.z = 0.0
                points.append(p)
            marker.points = points
        else:
            # 待機中: 緑の前方扇形 (1.0m, fov_deg)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            
            radius = 1.0  # ロックオンのしきい値(1.0m)に合わせる
            fov_rad = self.fov_deg * math.pi / 180.0
            steps = 20
            points = []
            
            p_center = Point()
            p_center.x = 0.0
            p_center.y = 0.0
            p_center.z = 0.0
            points.append(p_center)
            
            for i in range(steps + 1):
                theta = -fov_rad / 2.0 + i * (fov_rad / steps)
                p = Point()
                p.x = radius * math.cos(theta)
                p.y = radius * math.sin(theta)
                p.z = 0.0
                points.append(p)
                
            points.append(p_center)
            marker.points = points

        self.marker_pub.publish(marker)

    def publish_target_marker(self, map_x, map_y, stamp):
        from visualization_msgs.msg import Marker
        
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = 'tracked_target'
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = map_x
        marker.pose.position.y = map_y
        marker.pose.position.z = 0.5  # 半分浮かせる
        marker.pose.orientation.w = 1.0
        
        # シリンダーのサイズ (直径30cm, 高さ1m)
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 1.0
        
        # オレンジ色の半透明
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.6
        
        self.target_marker_pub.publish(marker)

    def delete_target_marker(self, stamp):
        from visualization_msgs.msg import Marker
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = 'tracked_target'
        marker.id = 1
        marker.action = Marker.DELETE
        self.target_marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
