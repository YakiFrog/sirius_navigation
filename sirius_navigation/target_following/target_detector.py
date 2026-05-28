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

class Track:
    """マルチターゲット追跡のための個別ターゲット管理クラス"""
    def __init__(self, track_id, state, cov, width, points_factor):
        self.id = track_id
        self.state = state  # np.array([x_map, y_map, vx_map, vy_map])
        self.cov = cov      # np.ndarray 4x4
        self.width = width
        self.points_factor = points_factor
        self.lost_count = 0
        self.age = 1

class TargetDetector(Node):
    """
    LaserScan (/hokuyo_scan と /scan3) から人間（脚と胴体）を検出し、
    複数人を個別のカルマンフィルターで同時にトラッキングし、
    追従用のプライマリターゲットのオドメトリを /npc/odom に配信するノード。
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
        self.declare_parameter('max_range', 5.0)                # 検出最大距離
        self.declare_parameter('fov_deg', 120.0)                # ロックオン用前方視野角（左右60度、計120度）
        
        # 脚検出用のクラスタリングパラメータ
        self.declare_parameter('leg_cluster_tolerance', 0.15)   # 点同士の最大距離（15cm）
        self.declare_parameter('min_leg_cluster_size', 2)       # 最小点数
        self.declare_parameter('max_leg_cluster_size', 60)      # 最大点数
        self.declare_parameter('min_leg_width', 0.08)           # 脚の最小幅（8cm）
        self.declare_parameter('max_leg_width', 0.25)           # 脚の最大幅（25cm）
        
        # 胴体検出用のクラスタリングパラメータ
        self.declare_parameter('torso_cluster_tolerance', 0.25) # 点同士の最大距離（25cm）
        self.declare_parameter('min_torso_cluster_size', 5)     # 最小点数（ノイズカットのため5点に引き上げ）
        self.declare_parameter('max_torso_cluster_size', 60)    # 最大点数
        self.declare_parameter('min_torso_width', 0.18)         # 胴体の最小幅（18cm）
        self.declare_parameter('max_torso_width', 0.55)         # 胴体の最大幅（55cm）

        self.declare_parameter('gating_distance', 0.6)          # カルマンフィルターの関連付けゲート距離（0.6m）
        self.declare_parameter('max_lost_frames', 150)          # ロスト判定フレーム数（15秒相当）
        self.declare_parameter('active_max_range', 5.0)         # 追従中の最大検出距離（5.0m）
        self.declare_parameter('active_fov_deg', 270.0)         # 追従中の視野角（270度）
        self.declare_parameter('lockon_max_range', 1.0)         # ロックオン時の最大距離（m）
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
        from visualization_msgs.msg import Marker, MarkerArray
        self.marker_pub = self.create_publisher(Marker, '/target_detector/range_marker', 10)
        self.target_marker_pub = self.create_publisher(MarkerArray, '/target_detector/target_markers', 10)
        
        # 二つのスキャントピックを非同期的に購読
        self.leg_scan_sub = self.create_subscription(LaserScan, self.leg_scan_topic, self.leg_scan_callback, 10)
        self.torso_scan_sub = self.create_subscription(LaserScan, self.torso_scan_topic, self.torso_scan_callback, 10)

        # TFリスナーのセットアップ
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # マルチターゲット管理用の状態変数
        self.tracks = []               # Trackオブジェクトのリスト
        self.next_track_id = 0         # 次に割り当てるTrack ID
        self.primary_track_id = None   # ロボットが追従するプライマリ・ターゲットのID
        
        # 各種カルマンフィルターのパラメータ
        # プロセスノイズ共分散 Q
        self.Q = np.diag([0.005, 0.005, 0.02, 0.02])
        # 観測行列 H
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])
        # 観測ノイズ共分散 R
        self.R = np.diag([0.10, 0.10])

        self.last_time = None
        self.latest_leg_candidates = []
        self.max_raw_markers = 0

        # コスト計算用の重み付け
        self.w_dist = 1.0    # 距離差の重み
        self.w_width = 0.5   # 幅差の重み
        self.w_points = 0.3  # 点群係数差の重み
        self.w_dir = 1.2     # 移動方向（速度ベクトル）の一致度の重み
        self.max_gating_cost = 3.5  # コスト閾値（方向ペナルティ考慮のため少し緩和）

        # キャリブレーションパラメータ
        self.calib_target_count = 6     # 6フレーム安定検出でロックオン
        self.calib_miss_tolerance = 10  # リセット猶予フレーム

        self.get_logger().info(
            f"Target Detector Node (Multi-Target Tracking) Initialized.\n"
            f"  Subscribing to leg scan: {self.leg_scan_topic}\n"
            f"  Subscribing to torso scan: {self.torso_scan_topic}\n"
            f"  Publishing odom to: {self.odom_topic}\n"
            f"  Publishing markers to: /target_detector/target_markers"
        )

    def detect_clusters(self, msg: LaserScan, min_w, max_w, tolerance, min_size, max_size):
        """2Dレーザースキャンから極座標 -> デカルト座標(x, y)への変換 & クラスタリングを行うヘルパー"""
        points = []
        angle = msg.angle_min
        
        # 追従状況（何らかの追跡対象があるか）に応じて走査範囲を動的に切り替える
        if self.tracks:
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
            # タイムスタンプ一致でのルックアップが失敗した場合のフォールバック
            if (to_frame == 'map' and from_frame != 'map') or (from_frame == 'map' and to_frame != 'map'):
                odom_frame = 'sirius3/odom' if ('sirius3/' in from_frame or 'sirius3/' in to_frame) else 'odom'
                try:
                    if to_frame == 'map':
                        trans_local = self.tf_buffer.lookup_transform(odom_frame, from_frame, rclpy.time.Time())
                        trans_map = self.tf_buffer.lookup_transform('map', odom_frame, rclpy.time.Time())
                        
                        qx = trans_local.transform.rotation.x
                        qy = trans_local.transform.rotation.y
                        qz = trans_local.transform.rotation.z
                        qw = trans_local.transform.rotation.w
                        yaw_local = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
                        tx_local = local_x * math.cos(yaw_local) - local_y * math.sin(yaw_local) + trans_local.transform.translation.x
                        ty_local = local_x * math.sin(yaw_local) + local_y * math.cos(yaw_local) + trans_local.transform.translation.y
                        
                        qx_m = trans_map.transform.rotation.x
                        qy_m = trans_map.transform.rotation.y
                        qz_m = trans_map.transform.rotation.z
                        qw_m = trans_map.transform.rotation.w
                        yaw_map = math.atan2(2.0 * (qw_m * qz_m + qx_m * qy_m), 1.0 - 2.0 * (qy_m**2 + qz_m**2))
                        tx_out = tx_local * math.cos(yaw_map) - ty_local * math.sin(yaw_map) + trans_map.transform.translation.x
                        ty_out = tx_local * math.sin(yaw_map) + ty_local * math.cos(yaw_map) + trans_map.transform.translation.y
                        return (tx_out, ty_out)
                    else:
                        trans_map = self.tf_buffer.lookup_transform(odom_frame, 'map', rclpy.time.Time())
                        trans_local = self.tf_buffer.lookup_transform(to_frame, odom_frame, rclpy.time.Time())
                        
                        qx_m = trans_map.transform.rotation.x
                        qy_m = trans_map.transform.rotation.y
                        qz_m = trans_map.transform.rotation.z
                        qw_m = trans_map.transform.rotation.w
                        yaw_map = math.atan2(2.0 * (qw_m * qz_m + qx_m * qy_m), 1.0 - 2.0 * (qy_m**2 + qz_m**2))
                        tx_odom = local_x * math.cos(yaw_map) - local_y * math.sin(yaw_map) + trans_map.transform.translation.x
                        ty_odom = local_x * math.sin(yaw_map) + local_y * math.cos(yaw_map) + trans_map.transform.translation.y
                        
                        qx = trans_local.transform.rotation.x
                        qy = trans_local.transform.rotation.y
                        qz = trans_local.transform.rotation.z
                        qw = trans_local.transform.rotation.w
                        yaw_local = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
                        tx_out = tx_odom * math.cos(yaw_local) - ty_odom * math.sin(yaw_local) + trans_local.transform.translation.x
                        ty_out = tx_odom * math.sin(yaw_local) + ty_odom * math.cos(yaw_local) + trans_local.transform.translation.y
                        return (tx_out, ty_out)
                except Exception as e_split:
                    return None
            else:
                try:
                    trans = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
                except Exception:
                    return None

        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))

        tx = trans.transform.translation.x
        ty = trans.transform.translation.y

        tx_out = local_x * math.cos(yaw) - local_y * math.sin(yaw) + tx
        ty_out = local_x * math.sin(yaw) + local_y * math.cos(yaw) + ty
        return (tx_out, ty_out)

    def transform_to_map(self, local_x, local_y, frame_id, stamp):
        return self.transform_point(local_x, local_y, frame_id, 'map', stamp)

    def get_base_frame(self, sensor_frame_id):
        if self.robot_base_frame and self.robot_base_frame != 'base_link':
            return self.robot_base_frame
        parts = sensor_frame_id.split('/')
        if len(parts) > 1:
            return parts[0] + '/base_link'
        return 'base_link'

    def transform_map_to_base_link(self, map_x, map_y, stamp, sensor_frame_id):
        base_frame = self.get_base_frame(sensor_frame_id)
        return self.transform_point(map_x, map_y, 'map', base_frame, stamp)

    def leg_scan_callback(self, msg: LaserScan):
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
            base_pt = self.transform_point(leg['centroid_local'][0], leg['centroid_local'][1], msg.header.frame_id, base_frame, msg.header.stamp)
            if base_pt is not None:
                leg_candidates.append({
                    'centroid_base': base_pt,
                    'timestamp': now
                })
        self.latest_leg_candidates = leg_candidates

    def torso_scan_callback(self, msg: LaserScan):
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

        # 2. 各胴体候補を base_frame および map 座標に変換し、かつ脚候補と関連付けを行う
        detected_targets = []
        base_frame = self.get_base_frame(msg.header.frame_id)
        
        for torso in detected_torsos:
            torso_base = self.transform_point(torso['centroid_local'][0], torso['centroid_local'][1], msg.header.frame_id, base_frame, msg.header.stamp)
            if torso_base is None:
                continue

            has_matching_leg = False
            for leg in valid_leg_candidates:
                dist_2d = math.sqrt((torso_base[0] - leg['centroid_base'][0])**2 + (torso_base[1] - leg['centroid_base'][1])**2)
                if dist_2d <= self.association_threshold:
                    has_matching_leg = True
                    break

            torso_map = self.transform_point(torso_base[0], torso_base[1], base_frame, 'map', msg.header.stamp)
            if torso_map is None:
                continue

            detected_targets.append({
                'centroid_map': torso_map,
                'centroid_base': torso_base,
                'width': torso['width'],
                'points_count': torso['points_count'],
                'distance': torso['distance'],
                'points_factor': torso['points_factor'],
                'has_matching_leg': has_matching_leg
            })

        # 3. 各アクティブトラックの予測 (Prediction)
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        for track in self.tracks:
            track.state = F @ track.state
            track.cov = F @ track.cov @ F.T + self.Q

        # 4. データ関連付け (scipy.optimize.linear_sum_assignment を用いたハンガリアン法)
        from scipy.optimize import linear_sum_assignment
        matched_detections = set()
        matched_tracks = set()
        
        num_tracks = len(self.tracks)
        num_detections = len(detected_targets)
        
        if num_tracks > 0 and num_detections > 0:
            cost_matrix = np.full((num_tracks, num_detections), 1e5)
            
            for t_idx, track in enumerate(self.tracks):
                is_predicted_in_fov = True
                pred_local = self.transform_map_to_base_link(track.state[0], track.state[1], msg.header.stamp, msg.header.frame_id)
                if pred_local is not None:
                    pred_lx, pred_ly = pred_local
                    pred_angle = math.atan2(pred_ly, pred_lx)
                    if pred_lx < -0.8 or abs(pred_angle) > (msg.angle_max + 0.1):
                        is_predicted_in_fov = False
                
                if not is_predicted_in_fov:
                    continue
                    
                for d_idx, det in enumerate(detected_targets):
                    cx_map, cy_map = det['centroid_map']
                    d_dist = math.sqrt((cx_map - track.state[0])**2 + (cy_map - track.state[1])**2)
                    
                    # ゲート距離チェック
                    if d_dist > self.gating_distance:
                        continue
                        
                    d_width = abs(det['width'] - track.width)
                    if track.points_factor > 0:
                        d_points = abs(det['points_factor'] - track.points_factor) / track.points_factor
                    else:
                        d_points = 0.0

                    # 進行方向（速度ベクトル）の一致度の計算
                    vx = track.state[2]
                    vy = track.state[3]
                    v_mag = math.sqrt(vx**2 + vy**2)
                    
                    prev_x = track.state[0] - vx * dt
                    prev_y = track.state[1] - vy * dt
                    disp_x = cx_map - prev_x
                    disp_y = cy_map - prev_y
                    disp_mag = math.sqrt(disp_x**2 + disp_y**2)
                    
                    dir_cost = 0.0
                    if v_mag > 0.15 and disp_mag > 0.05:
                        cos_theta = (vx * disp_x + vy * disp_y) / (v_mag * disp_mag)
                        cos_theta = max(-1.0, min(1.0, cos_theta))
                        # 反対方向（cos_theta < 0）に進むようなマッチングには強いペナルティを与える
                        dir_cost = self.w_dir * (1.0 - cos_theta)

                    cost = self.w_dist * d_dist + self.w_width * d_width + self.w_points * d_points + dir_cost
                    if not det['has_matching_leg']:
                        cost += 1.2
                        
                    if cost <= self.max_gating_cost:
                        cost_matrix[t_idx, d_idx] = cost
            
            # 最小コストとなる最適な組み合わせを選択
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            for r, c in zip(row_ind, col_ind):
                if cost_matrix[r, c] < 1e4:  # 無効な割当（1e5のまま）を除外
                    matched_tracks.add(r)
                    matched_detections.add(c)
                    
                    track = self.tracks[r]
                    det = detected_targets[c]
                    
                    Z = np.array([det['centroid_map'][0], det['centroid_map'][1]])
                    Y = Z - self.H @ track.state
                    S = self.H @ track.cov @ self.H.T + self.R
                    K = track.cov @ self.H.T @ np.linalg.inv(S)
                    
                    track.state = track.state + K @ Y
                    track.cov = (np.eye(4) - K @ self.H) @ track.cov
                    track.lost_count = 0
                    track.age += 1
                    
                    track.width = 0.95 * track.width + 0.05 * det['width']
                    track.points_factor = 0.95 * track.points_factor + 0.05 * det['points_factor']

        # 5. マッチしなかったトラックの更新（デッドレコニング）
        for t_idx, track in enumerate(self.tracks):
            if t_idx not in matched_tracks:
                track.lost_count += 1
                pred_local = self.transform_map_to_base_link(track.state[0], track.state[1], msg.header.stamp, msg.header.frame_id)
                is_predicted_in_fov = True
                if pred_local is not None:
                    pred_lx, pred_ly = pred_local
                    pred_angle = math.atan2(pred_ly, pred_lx)
                    if pred_lx < -0.8 or abs(pred_angle) > (msg.angle_max + 0.1):
                        is_predicted_in_fov = False
                
                allowed_lost_frames = 50 if not is_predicted_in_fov else self.max_lost_frames
                if track.lost_count >= allowed_lost_frames:
                    track.lost_count = 99999
                else:
                    track.state[2] *= 0.7
                    track.state[3] *= 0.7

        # ロストしたトラックの削除
        active_tracks = []
        for track in self.tracks:
            if track.lost_count < 99999:
                active_tracks.append(track)
            else:
                self.get_logger().warn(f"Track {track.id} lost. Removed.")
                if track.id == self.primary_track_id:
                    self.primary_track_id = None
        self.tracks = active_tracks

        # 6. 新規ターゲットの登録
        for d_idx, det in enumerate(detected_targets):
            if d_idx not in matched_detections:
                local_x, local_y = det['centroid_base']
                # ロックオンエリア内で最初に見つかった場合のみ、新規のTrack候補として登録する
                if (0.3 <= local_x <= self.lockon_max_range and abs(local_y) <= self.lockon_max_lateral):
                    # 【重複防止ガード】すでに半径0.5m以内に他の追従・追跡対象が存在する場合は、同一オブジェクトへの二重登録を防ぐため新規生成をスキップする
                    cx_map, cy_map = det['centroid_map']
                    too_close = False
                    for track in self.tracks:
                        dist = math.sqrt((cx_map - track.state[0])**2 + (cy_map - track.state[1])**2)
                        if dist < 0.5:
                            too_close = True
                            break
                    
                    if too_close:
                        continue
                        
                    new_track = Track(
                        track_id=self.next_track_id,
                        state=np.array([det['centroid_map'][0], det['centroid_map'][1], 0.0, 0.0]),
                        cov=np.eye(4) * 0.05,
                        width=det['width'],
                        points_factor=det['points_factor']
                    )
                    self.next_track_id += 1
                    self.tracks.append(new_track)
                    self.get_logger().info(f"New candidate track {new_track.id} spawned at ({new_track.state[0]:.2f}, {new_track.state[1]:.2f})")

        # 7. 追従対象（プライマリ・ターゲット）の選定・更新
        if self.primary_track_id is None and self.tracks:
            # 安定している（age >= calib_target_count）トラックのうち最も近いものをプライマリに選定
            best_track = None
            closest_dist = float('inf')
            
            base_frame = self.get_base_frame(msg.header.frame_id)
            try:
                trans_base = self.tf_buffer.lookup_transform(
                    'map',
                    base_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1)
                )
                rx = trans_base.transform.translation.x
                ry = trans_base.transform.translation.y
            except Exception:
                rx, ry = 0.0, 0.0

            for track in self.tracks:
                if track.age >= self.calib_target_count:
                    dist_to_robot = math.sqrt((track.state[0] - rx)**2 + (track.state[1] - ry)**2)
                    if dist_to_robot < closest_dist:
                        closest_dist = dist_to_robot
                        best_track = track
            
            if best_track is not None:
                self.primary_track_id = best_track.id
                self.get_logger().info(f"Target follow locked onto Track {self.primary_track_id}!")

        # 8. オドメトリとマーカーのパブリッシュ
        self.publish_all_markers(msg.header.stamp, msg.header.frame_id, detected_targets)
        
        if self.primary_track_id is not None:
            primary_track = next((t for t in self.tracks if t.id == self.primary_track_id), None)
            if primary_track is not None:
                self.publish_odom(primary_track, msg.header.stamp, msg.header.frame_id)

    def publish_all_markers(self, stamp, sensor_frame_id, detected_targets=None):
        from visualization_msgs.msg import Marker, MarkerArray
        marker_array = MarkerArray()
        
        # 全アクティブトラックを描画
        for track in self.tracks:
            # シリンダー（体積）
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'tracked_targets'
            marker.id = track.id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = track.state[0]
            marker.pose.position.y = track.state[1]
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 1.0
            
            # プライマリ（追従対象）はオレンジ、他は水色
            if track.id == self.primary_track_id:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 0.8
            else:
                marker.color.r = 0.0
                marker.color.g = 0.8
                marker.color.b = 0.8
                marker.color.a = 0.5
                
            marker_array.markers.append(marker)
            
            # テキスト（ID表示）
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = stamp
            text_marker.ns = 'target_ids'
            text_marker.id = track.id + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = track.state[0]
            text_marker.pose.position.y = track.state[1]
            text_marker.pose.position.z = 1.2
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.25
            
            if track.id == self.primary_track_id:
                text_marker.text = f"FOLLOW ID:{track.id}"
                text_marker.color.r = 1.0
                text_marker.color.g = 0.8
                text_marker.color.b = 0.0
            else:
                text_marker.text = f"ID:{track.id}"
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
            text_marker.color.a = 0.9
            marker_array.markers.append(text_marker)

        # 削除された古いIDのマーカーを消去
        for i in range(self.next_track_id):
            if not any(t.id == i for t in self.tracks):
                del_marker = Marker()
                del_marker.header.frame_id = 'map'
                del_marker.header.stamp = stamp
                del_marker.ns = 'tracked_targets'
                del_marker.id = i
                del_marker.action = Marker.DELETE
                marker_array.markers.append(del_marker)
                
                del_text = Marker()
                del_text.header.frame_id = 'map'
                del_text.header.stamp = stamp
                del_text.ns = 'target_ids'
                del_text.id = i + 10000
                del_text.action = Marker.DELETE
                marker_array.markers.append(del_text)

        # 生検出（未追従・追従中含むすべての検出候補）の描画
        raw_count = 0
        if detected_targets is not None:
            for idx, det in enumerate(detected_targets):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = stamp
                marker.ns = 'raw_detections'
                marker.id = idx
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                
                marker.pose.position.x = det['centroid_map'][0]
                marker.pose.position.y = det['centroid_map'][1]
                marker.pose.position.z = 0.01  # 地面すれすれに配置
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.01  # 極薄のシリンダー（円）にする
                
                # 確信度の違いで色分け
                if det.get('has_matching_leg', False):
                    # 高確信度（胴体＋脚を同時検出）: オレンジ
                    marker.color.r = 1.0
                    marker.color.g = 0.5
                    marker.color.b = 0.0
                    marker.color.a = 0.7
                else:
                    # 低確信度（胴体のみ、足元見えず）: 黄色
                    marker.color.r = 0.9
                    marker.color.g = 0.9
                    marker.color.b = 0.0
                    marker.color.a = 0.5
                
                marker_array.markers.append(marker)
                raw_count += 1

        # 不要になった古い生検出マーカーを消去
        for i in range(raw_count, self.max_raw_markers):
            del_marker = Marker()
            del_marker.header.frame_id = 'map'
            del_marker.header.stamp = stamp
            del_marker.ns = 'raw_detections'
            del_marker.id = i
            del_marker.action = Marker.DELETE
            marker_array.markers.append(del_marker)
        
        self.max_raw_markers = max(self.max_raw_markers, raw_count)

        self.target_marker_pub.publish(marker_array)

    def publish_odom(self, track: Track, stamp, sensor_frame_id):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'npc/base_link'

        # Position
        odom_msg.pose.pose.position.x = track.state[0]
        odom_msg.pose.pose.position.y = track.state[1]
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (角度)
        map_vx = track.state[2]
        map_vy = track.state[3]
        
        base_frame = self.get_base_frame(sensor_frame_id)
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

    def publish_range_marker(self, frame_id, stamp):
        # 検出範囲マーカー（必要に応じて動作）
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
        
        if self.tracks:
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

def main(args=None):
    if os.path.exists(_PID_FILE):
        try:
            with open(_PID_FILE) as f:
                old_pid = int(f.read().strip())
            os.kill(old_pid, 0)
            print(f'[ERROR] target_detector はすでに起動中です (PID={old_pid})。')
            return
        except (ProcessLookupError, ValueError, OSError):
            pass
    with open(_PID_FILE, 'w') as f:
        f.write(str(os.getpid()))

    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        if os.path.exists(_PID_FILE):
            os.remove(_PID_FILE)

if __name__ == '__main__':
    main()
