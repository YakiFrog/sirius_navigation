#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener, LookupException
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import yaml
import math
import time
import argparse
from dataclasses import dataclass
from typing import List
from math import sin, cos, pi
import os
from pathlib import Path
import subprocess

@dataclass
class Waypoint:
    number: int
    x: float
    y: float
    angle_radians: float
    rotate: float = 0.0
    stop: bool = False
    change_map: str = ""  # 地図変更用のフィールド（地図名を指定）
    
class Nav2GoalClient(Node):
    def __init__(self, count = 1):
        super().__init__('nav2_goal_client')
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.stop_publisher = self.create_publisher(Bool, '/stop', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'target_odom', 10)
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        
        while not self._action_client.wait_for_server(timeout_sec = 1.0):
            self.get_logger().info("Waiting for action server...")
        
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)
        
        # Allow overriding the waypoint file via env var SIRIUS_WAYPOINTS for flexibility
        file_path = os.environ.get(
            'SIRIUS_WAYPOINTS', "~/sirius_jazzy_ws/maps_waypoints/waypoints/waypoints.yaml"
        )
        self.waypoints = self.load_waypoints(file_path)
        self.count = count - 1
        self.loop_count = 0
        self.distance = float('inf')
        self.positions_list = []
        self.current_goal_index = None
        self.timer = self.create_timer(1.0, self.get_position)
        
    def load_waypoints(self, file_path: str) -> List[Waypoint]:
        # Expand ~ and resolve absolute path
        expanded = os.path.expanduser(file_path)
        path = Path(expanded).resolve()

        # If file not found, try typical workspace-relative location (cwd) as a fallback
        if not path.exists():
            fallback = Path.cwd() / 'maps_waypoints' / 'waypoints' / 'waypoints.yaml'
            if fallback.exists():
                path = fallback.resolve()
            else:
                # Provide a helpful error message with attempted locations
                tried = [str(path), str(fallback)]
                raise FileNotFoundError(
                    f"Waypoint file not found. Tried the following locations: {tried}.\n"
                    "Set the SIRIUS_WAYPOINTS environment variable or create the file at one of these paths."
                )

        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return [Waypoint(
            number = wp['number'],
            x = wp['x'],
            y = wp['y'],
            angle_radians = wp['angle_radians'],
            rotate = wp.get('rotate', 0.0),  # キーが存在しない場合は0.0を返す
            stop = wp.get('stop', False),  # キーが存在しない場合はFalseを返す
            change_map = wp.get('change_map', "")  # キーが存在しない場合は空文字を返す
        ) for wp in data['waypoints']]
        
    def euler_to_quaternion(self, yaw):
        """
        オイラー角（Yaw）からQuaternionに変換
        """
        return [
            0.0,  # x
            0.0,  # y
            sin(yaw / 2.0),  # z
            cos(yaw / 2.0)   # w
        ]
        
    def send_goal(self):
        if self.count >= len(self.waypoints):
            self.get_logger().info(f"All {len(self.waypoints)} waypoints completed.")
            return
        
        wp = self.waypoints[self.count]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp.x)
        goal_msg.pose.pose.position.y = float(wp.y)
        goal_msg.pose.pose.position.z = 0.0
        
        # 四元数の計算
        quat = self.euler_to_quaternion(float(wp.angle_radians))
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]
        
        # capture and store the current goal index for callbacks and logging
        self.current_goal_index = self.count
        self.get_logger().info(
            f"[WP:{wp.number}] {self.count + 1}/{len(self.waypoints)} -> pos=({float(wp.x):.2f},{float(wp.y):.2f}) yaw={float(wp.angle_radians):.2f}"
        )
        # Pass the goal index through a lambda so the callback can log the specific waypoint
        self._action_client.send_goal_async(goal_msg).add_done_callback(
            lambda fut, idx=self.current_goal_index: self.goal_response_callback(fut, idx)
        )
        # Also publish odometry for the target when sending a goal so the first waypoint odom is not missed
        try:
            self.send_odom()
        except Exception as e:
            self.get_logger().warning(f"Failed to publish odom for goal {wp.number}: {str(e)}")
        
    def send_odom(self):
        if self.count >= len(self.waypoints):
            self.get_logger().info(f"All {len(self.waypoints)} waypoints completed.")
            return
        
        wp = self.waypoints[self.count]
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'sirius3/odom'
        odom_msg.child_frame_id = 'target_frame'
        
        odom_msg.pose.pose.position.x = float(wp.x)
        odom_msg.pose.pose.position.y = float(wp.y)
        odom_msg.pose.pose.position.z = 0.0
        
        quat = self.euler_to_quaternion(float(wp.angle_radians))
        
        odom_msg.pose.pose.orientation = Quaternion(
            x = quat[0],
            y = quat[1],
            z = quat[2],
            w = quat[3]
        )
        
        self.odom_publisher.publish(odom_msg)
        self.get_logger().info(
            f"[WP:{wp.number}] odom pos=({odom_msg.pose.pose.position.x:.2f},{odom_msg.pose.pose.position.y:.2f})"
        )
        
    def goal_response_callback(self, future, goal_index = None):
        """Handle the action server's goal response. Uses a captured goal_index for accurate logging."""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().warning(f"Error getting goal response: {e}")
            return

        wp = None
        if goal_index is not None and goal_index < len(self.waypoints):
            wp = self.waypoints[goal_index]

        name = f"[WP:{wp.number}] " if wp else "[WP:?] "
        if goal_handle.accepted:
            self.get_logger().info(name + "Goal accepted by action server.")
            self.distance = float('inf')  # reset distance for new goal
        else:
            self.get_logger().warning(name + "Goal REJECTED by action server.")
        
    def publish_stop_command(self, should_stop: bool, goal_index: int = None):
        stop_msg = Bool()
        stop_msg.data = should_stop
        self.stop_publisher.publish(stop_msg)
        if should_stop == True:
            time.sleep(5)
            stop_msg.data = False
            self.stop_publisher.publish(stop_msg)
        wp = None
        if goal_index is not None and goal_index < len(self.waypoints):
            wp = self.waypoints[goal_index]
        prefix = f"[WP:{wp.number}] " if wp else "[WP:?] "
        if should_stop:
            self.get_logger().info(prefix + "STOP sent")
        else:
            self.get_logger().info(prefix + "RESUME sent")
    
    def publish_initial_pose(self, x: float, y: float, yaw: float, prefix: str = ""):
        """
        指定座標にイニシャルポーズを発行する
        
        Args:
            x: x座標 [m]
            y: y座標 [m]
            yaw: ヨー角 [rad]
            prefix: ログ用のプレフィックス
        """
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = "map"
        
        initial_pose_msg.pose.pose.position.x = x
        initial_pose_msg.pose.pose.position.y = y
        initial_pose_msg.pose.pose.position.z = 0.0
        
        # 四元数の計算
        quat = self.euler_to_quaternion(yaw)
        initial_pose_msg.pose.pose.orientation.x = quat[0]
        initial_pose_msg.pose.pose.orientation.y = quat[1]
        initial_pose_msg.pose.pose.orientation.z = quat[2]
        initial_pose_msg.pose.pose.orientation.w = quat[3]
        
        # 共分散を設定（初期位置の不確かさ）
        # 対角成分: [x, y, z, roll, pitch, yaw] の分散
        initial_pose_msg.pose.covariance[0] = 0.25   # x の分散 (0.5m)^2
        initial_pose_msg.pose.covariance[7] = 0.25   # y の分散 (0.5m)^2
        initial_pose_msg.pose.covariance[35] = 0.07  # yaw の分散 (~0.26rad ≈ 15度)^2
        
        self.initial_pose_publisher.publish(initial_pose_msg)
        self.get_logger().info(
            prefix + f"Initial pose published: pos=({x:.2f}, {y:.2f}) yaw={yaw:.2f}rad"
        )
    
    def change_map_command(self, map_name: str, goal_index: int = None) -> bool:
        """
        地図を切り替えるコマンドを実行し、対応するウェイポイントファイルも読み込む
        
        Returns:
            bool: 地図切り替えが成功した場合はTrue
        """
        wp = None
        if goal_index is not None and goal_index < len(self.waypoints):
            wp = self.waypoints[goal_index]
        prefix = f"[WP:{wp.number}] " if wp else "[WP:?] "
        
        if not map_name:
            return False
        
        self.get_logger().info(prefix + f"Changing map to: {map_name}")
        
        # シェルスクリプトを呼び出して地図を切り替え
        script_path = os.path.expanduser("~/sirius_jazzy_ws/bash/startup_bash/change_map_simple.sh")
        
        try:
            result = subprocess.run(
                ['bash', script_path, map_name],
                capture_output=True,
                text=True,
                timeout=60  # 60秒でタイムアウト
            )
            
            if result.returncode == 0:
                self.get_logger().info(prefix + f"Map changed successfully to: {map_name}")
                
                # 地図名と同じ名前のウェイポイントファイルを読み込む
                waypoints_path = f"~/sirius_jazzy_ws/maps_waypoints/waypoints/{map_name}.yaml"
                try:
                    new_waypoints = self.load_waypoints(waypoints_path)
                    self.waypoints = new_waypoints
                    self.count = 0  # ウェイポイントのインデックスをリセット
                    self.loop_count = 0
                    self.distance = float('inf')
                    self.get_logger().info(
                        prefix + f"Loaded new waypoints from {map_name}.yaml ({len(new_waypoints)} waypoints)"
                    )
                    
                    # 最初のウェイポイントの座標でイニシャルポーズを発行
                    if len(new_waypoints) > 0:
                        first_wp = new_waypoints[0]
                        # 少し待ってから発行（地図サーバーの準備完了を待つ）
                        time.sleep(1.0)
                        self.publish_initial_pose(
                            first_wp.x, 
                            first_wp.y, 
                            first_wp.angle_radians,
                            prefix + "[InitialPose] "
                        )
                    
                    return True
                except FileNotFoundError:
                    self.get_logger().warn(
                        prefix + f"Waypoints file not found: {waypoints_path}. Continuing with current waypoints."
                    )
                    return True
                except Exception as e:
                    self.get_logger().error(prefix + f"Failed to load waypoints: {str(e)}")
                    return True
            else:
                self.get_logger().error(prefix + f"Failed to change map: {result.stdout} {result.stderr}")
                return False
        except subprocess.TimeoutExpired:
            self.get_logger().error(prefix + f"Map change timed out for: {map_name}")
            return False
        except Exception as e:
            self.get_logger().error(prefix + f"Error changing map: {str(e)}")
            return False
            
    def get_position(self):
        try:
            # 最新のtransformを取得
            when = self.tfBuffer.get_latest_common_time('map', 'sirius3/base_footprint')
            transform = self.tfBuffer.lookup_transform(
                'map',
                'sirius3/base_footprint',
                when
            )
            translation = transform.transform.translation
            self.position = [translation.x, translation.y]

            # waypointsとpositions_listの長さをチェック
            if self.count < len(self.waypoints):
                current_wp = self.waypoints[self.count]
                x_goal = current_wp.x
                y_goal = current_wp.y
                x_distance = x_goal - self.position[0]
                y_distance = y_goal - self.position[1]
                self.distance = math.sqrt(x_distance**2 + y_distance**2)
                # Log the current distance to the active goal with formatting
                current_wp_info = f"[WP:{current_wp.number}] ({self.count + 1}/{len(self.waypoints)})"
                self.get_logger().info(f"{current_wp_info} dist={self.distance:.2f}m")
                
                # stopコマンドまたはchange_mapによって判定距離を変更
                if (hasattr(current_wp, 'stop') and current_wp.stop) or \
                   (hasattr(current_wp, 'change_map') and current_wp.change_map):
                    threshold_distance = 0.5  # stopがTrueまたはchange_mapが設定されている場合は0.5m
                else:
                    threshold_distance = 2.0  # それ以外の場合は1.5m

                if self.distance < threshold_distance:
                    self.get_logger().info(
                        f"{current_wp_info} reached (thr={threshold_distance:.2f}m) dist={self.distance:.2f}m"
                    )
                    
                    # stop属性の処理
                    if hasattr(current_wp, 'stop') and current_wp.stop is not None:
                        if current_wp.stop:  # stopがTrueの場合
                            self.publish_stop_command(True, self.count)  # 停止コマンドを送信
                        else:
                            self.publish_stop_command(False, self.count)  # 再開コマンドを送信
                    
                    # change_map属性の処理（地図切り替え + ウェイポイント切り替え）
                    map_changed = False
                    if hasattr(current_wp, 'change_map') and current_wp.change_map:
                        map_changed = self.change_map_command(current_wp.change_map, self.count)
                    
                    # 地図が切り替わった場合は count は change_map_command 内でリセット済み
                    # 切り替わらなかった場合のみ count を増やす
                    if not map_changed:
                        self.count += 1
                    
                    # Send the next goal (if any) and log it from send_goal
                    self.send_goal()
                
                # 定期的にゴールを再送信
                elif self.loop_count % 5 == 0:
                    self.get_logger().info(f"{current_wp_info} Resending...")
                    self.send_goal()
                
                self.loop_count += 1

        except LookupException:
            self.get_logger().warning("Transform lookup failed. Retrying...")
        except Exception as e:
            self.get_logger().warning(f"Transform error: {str(e)}")
            
        # タイマーの周期を2秒に変更
        self.timer.timer_period_ns = 2000000000  # 2秒
            
def main(args = None):
    parser = argparse.ArgumentParser(description='Nav2 waypoint follower with map switching support.')
    parser.add_argument('--count', type=int, default=1, 
                        help='Starting waypoint index (default: 1)')
    parser.add_argument('--waypoints', '-w', type=str, default=None,
                        help='Waypoints file path or name (e.g., "waypoints.yaml" or "atc_1F")')
    parsed_args = parser.parse_args()
    
    # ウェイポイントファイルのパスを決定
    # 優先順位: 1. コマンドライン引数 2. 環境変数 3. デフォルト
    if parsed_args.waypoints:
        waypoints_input = parsed_args.waypoints
        # .yaml が含まれていなければ追加
        if not waypoints_input.endswith('.yaml'):
            waypoints_input = waypoints_input + '.yaml'
        # 絶対パスでなければ waypoints ディレクトリからの相対パスとして扱う
        if not waypoints_input.startswith('/') and not waypoints_input.startswith('~'):
            waypoints_file = f"~/sirius_jazzy_ws/maps_waypoints/waypoints/{waypoints_input}"
        else:
            waypoints_file = waypoints_input
    else:
        waypoints_file = os.environ.get(
            'SIRIUS_WAYPOINTS', 
            "~/sirius_jazzy_ws/maps_waypoints/waypoints/waypoints.yaml"
        )
    
    # 環境変数に設定（Nav2GoalClientで使用）
    os.environ['SIRIUS_WAYPOINTS'] = os.path.expanduser(waypoints_file)
    
    rclpy.init(args=args)
    node = Nav2GoalClient(count=parsed_args.count)
    node.get_logger().info(
        f"Starting Nav2GoalClient: {len(node.waypoints)} waypoints from '{waypoints_file}', "
        f"starting from index {parsed_args.count}"
    )
    node.send_goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()