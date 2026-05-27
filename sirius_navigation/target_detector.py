#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import math
import numpy as np

_PID_FILE = '/tmp/target_detector.pid'

class TargetDetector(Node):
    """
    LaserScan (/hokuyo_scan と /scan3) から人間（脚と胴体）を検出し、
    両者を map 座標系で空間的に関連付ける（フュージョン）ことで、
    高精度にターゲットを特定してカルマンフィルターで追跡し /npc/odom に配信するノード。
    """
    def __init__(self):
        super().__init__('target_detector')

        # パラメータの宣言と初期値設定
        self.declare_parameter('leg_scan_topic', '/hokuyo_scan')
        self.declare_parameter('torso_scan_topic', '/scan3')
        self.declare_parameter('association_threshold', 0.65)  # 脚と胴体のマッチング許容距離（65cm）
        self.declare_parameter('odom_topic', '/npc/odom')
        self.declare_parameter('robot_base_frame', 'sirius3/base_footprint')
        
        self.declare_parameter('min_range', 0.3)                # 検出最小距離（自身を弾く）
        self.declare_parameter('max_range', 5.0)                # 検出最大距離（3.5m以内）
        self.declare_parameter('fov_deg', 120.0)                # ロックオン用前方視野角（左右60度、計120度）
        
        # 脚検出用のクラスタリングパラメータ
        self.declare_parameter('leg_cluster_tolerance', 0.15)   # 点同士の最大距離（15cm）
        self.declare_parameter('min_leg_cluster_size', 2)       # 最小点数
        self.declare_parameter('max_leg_cluster_size', 60)      # 最大点数
        self.declare_parameter('min_leg_width', 0.06)           # 脚の最小幅（6cm）
        self.declare_parameter('max_leg_width', 0.80)           # 脚の最大幅（80cm）
        
        # 胴体検出用のクラスタリングパラメータ
        self.declare_parameter('torso_cluster_tolerance', 0.25) # 点同士の最大距離（25cm）
        self.declare_parameter('min_torso_cluster_size', 3)     # 最小点数
        self.declare_parameter('max_torso_cluster_size', 60)    # 最大点数
        self.declare_parameter('min_torso_width', 0.10)         # 胴体の最小幅（10cmに緩和）
        self.declare_parameter('max_torso_width', 0.85)         # 胴体の最大幅（85cmに緩和）

        self.declare_parameter('gating_distance', 0.6)          # カルマンフィルターの関連付けゲート距離（0.6mに縮小して静的オブジェクトへの吸い付きを防止）
        self.declare_parameter('max_lost_frames', 150)           # ロスト判定フレーム数（15秒相当）
        self.declare_parameter('active_max_range', 5.0)         # 追従中の最大検出距離（5.0m）
        self.declare_parameter('active_fov_deg', 270.0)         # 追従中の視野角（270度に広げて真横もカバー）
        self.declare_parameter('lockon_max_range', 1.0)         # ロックオン時の最大距離（m）起動時前方60cm想定
        self.declare_parameter('lockon_max_lateral', 0.8)       # ロックオン時の横方向最大距離（m）

        # パラメータの取得
        self.leg_scan_topic = self.get_parameter('leg_scan_topic').value
        self.torso_scan_topic = self.get_parameter('torso_scan_topic').value
        self.association_threshold = self.get_parameter('association_threshold').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.fov_deg = self.get_parameter('fov_deg').value

        self.leg_cluster_tolerance = self.get_parameter('leg_cluster_tolerance').value
        self.min_leg_cluster_size = self.get_parameter('min_leg_cluster_size').value
        self.max_leg_cluster_size = self.get_parameter('max_leg_cluster_size').value
        self.min_leg_width = self.get_parameter('min_leg_width').value
        self.max_leg_width = self.get_parameter('max_leg_width').value

        self.torso_cluster_tolerance = self.get_parameter('torso_cluster_tolerance').value
        self.min_torso_cluster_size = self.get_parameter('min_torso_cluster_size').value
        self.max_torso_cluster_size = self.get_parameter('max_torso_cluster_size').value
        self.min_torso_width = self.get_parameter('min_torso_width').value
        self.max_torso_width = self.get_parameter('max_torso_width').value

        self.gating_distance = self.get_parameter('gating_distance').value
        self.max_lost_frames = self.get_parameter('max_lost_frames').value
        self.active_max_range = self.get_parameter('active_max_range').value
        self.active_fov_deg = self.get_parameter('active_fov_deg').value
        self.lockon_max_range = self.get_parameter('lockon_max_range').value
        self.lockon_max_lateral = self.get_parameter('lockon_max_lateral').value

        # パブリッシャーとサブスクライバーのセットアップ
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        from visualization_msgs.msg import Marker
        self.marker_pub = self.create_publisher(Marker, '/target_detector/range_marker', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/target_detector/target_marker', 10)
        
        # 二つのスキャントピックを非同期的に購読
        self.leg_scan_sub = self.create_subscription(LaserScan, self.leg_scan_topic, self.leg_scan_callback, 10)
        self.torso_scan_sub = self.create_subscription(LaserScan, self.torso_scan_topic, self.torso_scan_callback, 10)

        # TFリスナーのセットアップ
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # カルマンフィルター状態変数の初期化
        # 状態ベクトル X = [x_map, y_map, vx_map, vy_map]^T (map座標系基準)
        self.kf_state = np.zeros(4)
        self.kf_cov = np.eye(4) * 1.0

        # カルマンフィルターパラメータ
        # プロセスノイズ共分散 Q
        self.Q = np.diag([0.005, 0.005, 0.02, 0.02])
        # 観測行列 H
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])
        # 観測ノイズ共分散 R
        self.R = np.diag([0.10, 0.10])

        # トラッキング状態および特徴量管理
        self.is_tracking = False
        self.lost_count = 0
        self.last_time = None
        
        # 最新の検出された脚候補リストを保持する
        self.latest_leg_candidates = []

        # ロックオンしたターゲットの特徴量
        self.locked_width = 0.0
        self.locked_points_factor = 0.0

        # コスト計算用の重み付け
        self.w_dist = 1.0    # 距離差の重み
        self.w_width = 0.5   # 幅差の重み（2.0→ノイズに弱わなかったため下げ）
        self.w_points = 0.3  # 点群係数差の重み
        self.max_gating_cost = 3.0  # コスト閾値（1.5→歩行中の変動を許容）

        # キャリブレーションバッファ（起動時に複数フレーム観測してからロックオン）
        self.calib_buffer = []          # 前方検出フレームを蓄積
        self.calib_target_count = 6     # 6フレーム安定検出でロックオン確定（約0.6秒）
        self.calib_miss_count = 0       # ターゲットが見えなかったフレーム数（リセット猶予）
        self.calib_miss_tolerance = 10   # この回数まではバッファをリセットしない

        self.get_logger().info(
            f"Target Detector Node (Leg-Torso Fusion) Initialized.\n"
            f"  Subscribing to leg scan: {self.leg_scan_topic}\n"
            f"  Subscribing to torso scan: {self.torso_scan_topic}\n"
            f"  Publishing to: {self.odom_topic}\n"
            f"  Range: {self.min_range}m ~ {self.max_range}m"
        )

    def detect_clusters(self, msg: LaserScan, min_w, max_w, tolerance, min_size, max_size):
        """2Dレーザースキャンから極座標 -> デカルト座標(x, y)への変換 & クラスタリングを行うヘルパー"""
        points = []
        angle = msg.angle_min
        
        # 追従状況に応じて走査範囲を動的に切り替える
        if self.is_tracking:
            max_r = self.active_max_range
            fov_r = self.active_fov_deg * math.pi / 180.0
        else:
            max_r = self.max_range
            fov_r = self.fov_deg * math.pi / 180.0

        for r in msg.ranges:
            if self.min_range <= r <= max_r:
                if abs(angle) <= fov_r / 2.0:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    points.append((x, y))
            angle += msg.angle_increment

        clusters = []
        current_cluster = []

        for p in points:
            if not current_cluster:
                current_cluster.append(p)
            else:
                last_p = current_cluster[-1]
                dist = math.sqrt((p[0] - last_p[0])**2 + (p[1] - last_p[1])**2)
                if dist <= tolerance:
                    current_cluster.append(p)
                else:
                    clusters.append(current_cluster)
                    current_cluster = [p]
        if current_cluster:
            clusters.append(current_cluster)

        detected = []
        for cluster in clusters:
            if min_size <= len(cluster) <= max_size:
                width = math.sqrt((cluster[0][0] - cluster[-1][0])**2 + (cluster[0][1] - cluster[-1][1])**2)
                if min_w <= width <= max_w:
                    cx = sum(p[0] for p in cluster) / len(cluster)
                    cy = sum(p[1] for p in cluster) / len(cluster)
                    dist = math.sqrt(cx**2 + cy**2)
                    detected.append({
                        'centroid_local': (cx, cy),
                        'width': width,
                        'points_count': len(cluster),
                        'distance': dist,
                        'points_factor': len(cluster) * dist
                    })
        return detected

    def transform_point(self, local_x, local_y, from_frame, to_frame, stamp):
        """任意のフレーム間での2D点 (local_x, local_y) の座標変換を行う"""
        try:
            trans = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                stamp
            )
        except Exception:
            # タイムスタンプ一致でのルックアップが失敗した場合、
            # map が絡む変換であれば、odom を中継した最新値(Time(0))同士での分割ルックアップにフォールバックする
            if (to_frame == 'map' and from_frame != 'map') or (from_frame == 'map' and to_frame != 'map'):
                odom_frame = 'sirius3/odom' if ('sirius3/' in from_frame or 'sirius3/' in to_frame) else 'odom'
                try:
                    if to_frame == 'map':
                        # 1. センサーフレームから odom への変換 (最新値)
                        trans_local = self.tf_buffer.lookup_transform(
                            odom_frame,
                            from_frame,
                            rclpy.time.Time()
                        )
                        # 2. odom から map への変換 (最新値)
                        trans_map = self.tf_buffer.lookup_transform(
                            'map',
                            odom_frame,
                            rclpy.time.Time()
                        )
                        
                        # センサーフレームから odom への変換適用
                        qx = trans_local.transform.rotation.x
                        qy = trans_local.transform.rotation.y
                        qz = trans_local.transform.rotation.z
                        qw = trans_local.transform.rotation.w
                        yaw_local = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
                        tx_local = local_x * math.cos(yaw_local) - local_y * math.sin(yaw_local) + trans_local.transform.translation.x
                        ty_local = local_x * math.sin(yaw_local) + local_y * math.cos(yaw_local) + trans_local.transform.translation.y
                        
                        # odom から map への変換適用
                        qx_m = trans_map.transform.rotation.x
                        qy_m = trans_map.transform.rotation.y
                        qz_m = trans_map.transform.rotation.z
                        qw_m = trans_map.transform.rotation.w
                        yaw_map = math.atan2(2.0 * (qw_m * qz_m + qx_m * qy_m), 1.0 - 2.0 * (qy_m**2 + qz_m**2))
                        tx_out = tx_local * math.cos(yaw_map) - ty_local * math.sin(yaw_map) + trans_map.transform.translation.x
                        ty_out = tx_local * math.sin(yaw_map) + ty_local * math.cos(yaw_map) + trans_map.transform.translation.y
                        
                        return (tx_out, ty_out)
                    else:
                        # 1. map から odom への変換 (最新値)
                        trans_map = self.tf_buffer.lookup_transform(
                            odom_frame,
                            'map',
                            rclpy.time.Time()
                        )
                        # 2. odom から センサーフレームへの変換 (最新値)
                        trans_local = self.tf_buffer.lookup_transform(
                            to_frame,
                            odom_frame,
                            rclpy.time.Time()
                        )
                        
                        # map から odom への変換適用
                        qx_m = trans_map.transform.rotation.x
                        qy_m = trans_map.transform.rotation.y
                        qz_m = trans_map.transform.rotation.z
                        qw_m = trans_map.transform.rotation.w
                        yaw_map = math.atan2(2.0 * (qw_m * qz_m + qx_m * qy_m), 1.0 - 2.0 * (qy_m**2 + qz_m**2))
                        tx_odom = local_x * math.cos(yaw_map) - local_y * math.sin(yaw_map) + trans_map.transform.translation.x
                        ty_odom = local_x * math.sin(yaw_map) + local_y * math.cos(yaw_map) + trans_map.transform.translation.y
                        
                        # odom から センサーフレームへの変換適用
                        qx = trans_local.transform.rotation.x
                        qy = trans_local.transform.rotation.y
                        qz = trans_local.transform.rotation.z
                        qw = trans_local.transform.rotation.w
                        yaw_local = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
                        tx_out = tx_odom * math.cos(yaw_local) - ty_odom * math.sin(yaw_local) + trans_local.transform.translation.x
                        ty_out = tx_odom * math.sin(yaw_local) + ty_odom * math.cos(yaw_local) + trans_local.transform.translation.y
                        
                        return (tx_out, ty_out)
                except Exception as e_split:
                    self.get_logger().warning(
                        f"Split TF lookup failed (map <-> odom <-> local) between {from_frame} and {to_frame}: {e_split}",
                        throttle_duration_sec=5.0
                    )
                    return None
            else:
                try:
                    trans = self.tf_buffer.lookup_transform(
                        to_frame,
                        from_frame,
                        rclpy.time.Time()
                    )
                except Exception as e:
                    self.get_logger().warning(
                        f"TF lookup failed from {from_frame} to {to_frame} using Time(0): {e}",
                        throttle_duration_sec=5.0
                    )
                    return None

        # クォータニオンから2D Yaw角への変換
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))

        # 並進移動量
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y

        # 回転 + 並進
        tx_out = local_x * math.cos(yaw) - local_y * math.sin(yaw) + tx
        ty_out = local_x * math.sin(yaw) + local_y * math.cos(yaw) + ty
        return (tx_out, ty_out)

    def transform_to_map(self, local_x, local_y, frame_id, stamp):
        """互換性のためのヘルパー：ローカルの座標を map 座標系へ変換する"""
        return self.transform_point(local_x, local_y, frame_id, 'map', stamp)

    def get_base_frame(self, sensor_frame_id):
        """センサーフレームIDからネームスペースを含んだベースフレームを取得するヘルパー"""
        if self.robot_base_frame and self.robot_base_frame != 'base_link':
            return self.robot_base_frame
        parts = sensor_frame_id.split('/')
        if len(parts) > 1:
            return parts[0] + '/base_link'
        return 'base_link'

    def transform_map_to_base_link(self, map_x, map_y, stamp, sensor_frame_id):
        """map座標系からbase_link座標系への逆変換を行う（正面ロックオン判定用）"""
        base_frame = self.get_base_frame(sensor_frame_id)
        return self.transform_point(map_x, map_y, 'map', base_frame, stamp)

    def leg_scan_callback(self, msg: LaserScan):
        """足元の 2D LiDAR (hokuyo_scan) のコールバック。検出した脚のロボットローカル座標（base_frame）での位置を保存する"""
        detected_legs = self.detect_clusters(
            msg,
            min_w=self.min_leg_width,
            max_w=self.max_leg_width,
            tolerance=self.leg_cluster_tolerance,
            min_size=self.min_leg_cluster_size,
            max_size=self.max_leg_cluster_size
        )

        now = self.get_clock().now()
        leg_candidates = []
        base_frame = self.get_base_frame(msg.header.frame_id)
        for leg in detected_legs:
            # 安定性の向上のため、ロボットローカル座標系 (base_frame) で脚の位置を保存
            base_pt = self.transform_point(leg['centroid_local'][0], leg['centroid_local'][1], msg.header.frame_id, base_frame, msg.header.stamp)
            if base_pt is not None:
                leg_candidates.append({
                    'centroid_base': base_pt,
                    'timestamp': now
                })
        self.latest_leg_candidates = leg_candidates

    def torso_scan_callback(self, msg: LaserScan):
        """
        胴体高さのスキャン (scan3) のコールバック。
        保存されている脚の候補と空間的にフュージョンした上で、カルマンフィルター追従のメインループを実行する。
        """
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            dt = 0.1

        # 0.3秒より古い脚候補はタイムアウトとして弾く
        valid_leg_candidates = [
            leg for leg in self.latest_leg_candidates
            if (now - leg['timestamp']).nanoseconds / 1e9 <= 0.3
        ]

        # 1. 胴体候補の検出
        detected_torsos = self.detect_clusters(
            msg,
            min_w=self.min_torso_width,
            max_w=self.max_torso_width,
            tolerance=self.torso_cluster_tolerance,
            min_size=self.min_torso_cluster_size,
            max_size=self.max_torso_cluster_size
        )

        # 2. 各胴体候補を base_frame および map 座標に変換し、かつ脚候補とフュージョン（関連付け）を行う
        detected_targets = []
        base_frame = self.get_base_frame(msg.header.frame_id)
        
        for torso in detected_torsos:
            # 胴体をロボットローカル座標（base_frame）に変換（脚とのフュージョン用）
            torso_base = self.transform_point(torso['centroid_local'][0], torso['centroid_local'][1], msg.header.frame_id, base_frame, msg.header.stamp)
            if torso_base is None:
                continue

            # 周辺に有効な脚候補があるかチェック (距離が association_threshold 以下)
            has_matching_leg = False
            for leg in valid_leg_candidates:
                dist_2d = math.sqrt((torso_base[0] - leg['centroid_base'][0])**2 + (torso_base[1] - leg['centroid_base'][1])**2)
                if dist_2d <= self.association_threshold:
                    has_matching_leg = True
                    break

            # ターゲット位置を map 座標系へ変換
            torso_map = self.transform_point(torso_base[0], torso_base[1], base_frame, 'map', msg.header.stamp)
            if torso_map is None:
                continue

            if has_matching_leg or self.is_tracking or True:
                detected_targets.append({
                    'centroid_map': torso_map,
                    'centroid_base': torso_base,  # ロックオン判定用（TF逆変換不要）
                    'width': torso['width'],
                    'points_count': torso['points_count'],
                    'distance': torso['distance'],
                    'points_factor': torso['points_factor'],
                    'has_matching_leg': has_matching_leg
                })

        # 3. カルマンフィルター状態予測ステップ (Prediction)
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        pred_state = F @ self.kf_state
        pred_cov = F @ self.kf_cov @ F.T + self.Q

        measurement_updated = False
        measured_x = 0.0
        measured_y = 0.0

        # 予測されたターゲット位置がセンサーの視野内にあるかチェック
        is_predicted_in_fov = True
        if self.is_tracking:
            pred_local = self.transform_map_to_base_link(pred_state[0], pred_state[1], msg.header.stamp, msg.header.frame_id)
            if pred_local is not None:
                pred_lx, pred_ly = pred_local
                pred_angle = math.atan2(pred_ly, pred_lx)
                # 2D LiDAR (scan3/hokuyo_scan) の物理的視野は [-pi/2, pi/2]
                # マージン 0.1 rad を付与して判定
                if pred_lx < -0.8 or abs(pred_angle) > (msg.angle_max + 0.1):
                    is_predicted_in_fov = False

        if detected_targets and is_predicted_in_fov:
            if self.is_tracking:
                best_target = None
                lowest_cost = float('inf')
                
                for target in detected_targets:
                    cx_map, cy_map = target['centroid_map']
                    # 予測位置からの距離差 (map座標系)
                    d_dist = math.sqrt((cx_map - pred_state[0])**2 + (cy_map - pred_state[1])**2)
                    if d_dist > self.gating_distance:
                        continue
                        
                    d_width = abs(target['width'] - self.locked_width)
                    if self.locked_points_factor > 0:
                        d_points = abs(target['points_factor'] - self.locked_points_factor) / self.locked_points_factor
                    else:
                        d_points = 0.0
                        
                    # 統合コスト計算
                    cost = self.w_dist * d_dist + self.w_width * d_width + self.w_points * d_points
                    if not target['has_matching_leg']:
                        cost += 1.2  # 脚の検出がない胴体候補には小さなペナルティを付与（側面に回り込んで脚が見えないときも追跡を継続するため）
                    
                    if cost < lowest_cost:
                        lowest_cost = cost
                        best_target = target

                # コスト閾値フィルター：最小コストが max_gating_cost 以下の場合のみ採用
                # （定義されていたが使われていなかったバグを修正）
                if best_target is not None and lowest_cost <= self.max_gating_cost:
                    measured_x, measured_y = best_target['centroid_map']
                    measurement_updated = True
                    
                    # 特徴量の適応的アップデート
                    self.locked_width = 0.95 * self.locked_width + 0.05 * best_target['width']
                    self.locked_points_factor = 0.95 * self.locked_points_factor + 0.05 * best_target['points_factor']
            else:
                # 未追跡時：キャリブレーションモード
                # centroid_base (ロボットローカル座標) を直接使用→ TF逆変換不要で安定
                best_target = None
                best_target_has_leg = False
                closest_dist = float('inf')

                for target in detected_targets:
                    local_x, local_y = target['centroid_base']  # 直接使用（TF不要）
                    # 距離条件：前方 0.3m〜lockon_max_range
                    # 横方向条件：固定 ±lockon_max_lateral
                    if (0.3 <= local_x <= self.lockon_max_range
                            and abs(local_y) <= self.lockon_max_lateral):
                        has_leg = target['has_matching_leg']
                        is_better = False
                        if best_target is None:
                            is_better = True
                        elif has_leg and not best_target_has_leg:
                            is_better = True
                        elif has_leg == best_target_has_leg and target['distance'] < closest_dist:
                            is_better = True
                        if is_better:
                            closest_dist = target['distance']
                            best_target = target
                            best_target_has_leg = has_leg

                if best_target is not None:
                    self.calib_miss_count = 0  # ターゲット発見でミスカウントをリセット
                    # キャリブレーションバッファに蓄積
                    self.calib_buffer.append(best_target)
                    remaining = self.calib_target_count - len(self.calib_buffer)
                    if len(self.calib_buffer) == 1:
                        self.get_logger().info(
                            f"キャリブレーション開始。前方 {closest_dist:.2f}m にターゲット検出。"
                            f"あと {remaining} フレームでロックオン..."
                        )

                    if len(self.calib_buffer) >= self.calib_target_count:
                        # 複数フレームの平均でKFを初期化
                        avg_x = sum(t['centroid_map'][0] for t in self.calib_buffer) / len(self.calib_buffer)
                        avg_y = sum(t['centroid_map'][1] for t in self.calib_buffer) / len(self.calib_buffer)
                        avg_width = sum(t['width'] for t in self.calib_buffer) / len(self.calib_buffer)
                        avg_pf = sum(t['points_factor'] for t in self.calib_buffer) / len(self.calib_buffer)
                        self.calib_buffer = []
                        self.calib_miss_count = 0

                        self.kf_state = np.array([avg_x, avg_y, 0.0, 0.0])
                        self.kf_cov = np.eye(4) * 0.05
                        self.locked_width = avg_width
                        self.locked_points_factor = avg_pf
                        self.is_tracking = True
                        self.lost_count = 0
                        measurement_updated = True
                        measured_x, measured_y = avg_x, avg_y

                        self.get_logger().info(
                            f"キャリブレーション完了！{self.calib_target_count}フレーム平均でロックオン！\n"
                            f"  位置(map): ({avg_x:.2f}, {avg_y:.2f})\n"
                            f"  平均幅: {avg_width:.3f}m  点群係数: {avg_pf:.1f}"
                        )
                else:
                    # ターゲットが見えない → 猶予カウンタで管理
                    self.calib_miss_count += 1
                    if self.calib_buffer and self.calib_miss_count > self.calib_miss_tolerance:
                        self.get_logger().info(
                            f"キャリブレーション中断（{self.calib_miss_count}フレーム連続で視野外）。"
                            f"バッファ({len(self.calib_buffer)}フレーム)をリセット。",
                            throttle_duration_sec=2.0
                        )
                        self.calib_buffer = []
                        self.calib_miss_count = 0

        elif not self.is_tracking and self.calib_buffer:
            # detected_targetsが空の場合も猶予カウンタで管理
            self.calib_miss_count += 1
            if self.calib_miss_count > self.calib_miss_tolerance:
                self.calib_buffer = []
                self.calib_miss_count = 0

        # 4. 観測更新 (Update) or デッドレコニング (Prediction Only)
        if measurement_updated:
            Z = np.array([measured_x, measured_y])
            Y = Z - self.H @ pred_state
            
            S = self.H @ pred_cov @ self.H.T + self.R
            K = pred_cov @ self.H.T @ np.linalg.inv(S)
            
            self.kf_state = pred_state + K @ Y
            self.kf_cov = (np.eye(4) - K @ self.H) @ pred_cov
            self.lost_count = 0
        else:
            if self.is_tracking:
                self.lost_count += 1
                # 視野外(後ろなど)にいるときは、旋回して再捕捉する時間を与えるため、ロスト判定の猶予を長く(5秒/50フレーム)する
                allowed_lost_frames = 50 if not is_predicted_in_fov else self.max_lost_frames
                if self.lost_count >= allowed_lost_frames:
                    self.is_tracking = False
                    self.locked_width = 0.0
                    self.locked_points_factor = 0.0
                    self.get_logger().warn("Target lost. Feature lock cleared.")
                else:
                    self.kf_state = pred_state
                    self.kf_state[2] *= 0.7  # vx 減衰（0.9→ドリフト防止のため加速）
                    self.kf_state[3] *= 0.7  # vy 減衰
                    self.kf_cov = pred_cov

        # 5. Odometryの配信
        if self.is_tracking:
            odom_msg = Odometry()
            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = 'map'
            odom_msg.child_frame_id = 'npc/base_link'

            # Position
            odom_msg.pose.pose.position.x = self.kf_state[0]
            odom_msg.pose.pose.position.y = self.kf_state[1]
            odom_msg.pose.pose.position.z = 0.0

            # Orientation
            map_vx = self.kf_state[2]
            map_vy = self.kf_state[3]
            
            # デフォルトはロボット自身の向きを取得して使用
            base_frame = self.get_base_frame(msg.header.frame_id)
            try:
                trans_base = self.tf_buffer.lookup_transform(
                    'map',
                    base_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1)
                )
                qx = trans_base.transform.rotation.x
                qy = trans_base.transform.rotation.y
                qz = trans_base.transform.rotation.z
                qw = trans_base.transform.rotation.w
                default_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
            except Exception:
                default_yaw = 0.0

            target_yaw = math.atan2(map_vy, map_vx) if (abs(map_vx) > 0.05 or abs(map_vy) > 0.05) else default_yaw
            odom_msg.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
            odom_msg.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

            # Velocity
            odom_msg.twist.twist.linear.x = map_vx
            odom_msg.twist.twist.linear.y = map_vy

            self.odom_pub.publish(odom_msg)
            
            # マーカーの配信
            self.publish_target_marker(self.kf_state[0], self.kf_state[1], now.to_msg())
        else:
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
        marker.scale.x = 0.03
        
        if self.is_tracking:
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
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            
            radius = 1.0
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
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 1.0
        
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
    # ===== 多重起動防止（PIDファイルロック） =====
    if os.path.exists(_PID_FILE):
        try:
            with open(_PID_FILE) as f:
                old_pid = int(f.read().strip())
            os.kill(old_pid, 0)  # プロセスが生きているか確認
            print(f'[ERROR] target_detector はすでに起動中です (PID={old_pid})。終了します。')
            return
        except (ProcessLookupError, ValueError, OSError):
            pass  # 古いPIDファイルなら無視
    with open(_PID_FILE, 'w') as f:
        f.write(str(os.getpid()))
    # ==========================================

    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass  # ExternalShutdownException などを包括
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass  # 二重呼び出しエラーを無視
        # PIDファイルを削除して次回起動を許可
        if os.path.exists(_PID_FILE):
            os.remove(_PID_FILE)

if __name__ == '__main__':
    main()
