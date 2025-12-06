#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener 
import tf2_ros
import tf2_py as tf2
import yaml
import math
import os

file_path = os.path.expanduser("~/sirius_jazzy_ws/maps_waypoints/waypoints/raw_waypoints.yaml")

class GetPose(Node):
    def __init__(self):
        super().__init__('get_pose')
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.positions_list = []
        self.check_position = None  # 初回は現在位置で初期化するためNone
        self.position = []
        self.waypoint_number = 1
        self.last_written_number = 0
        self.is_first_position = True  # 初回フラグ
        # distance threshold as a ROS parameter, default 5.0 meters
        self.declare_parameter('distance_threshold', 4.0)
        self.distance_threshold = float(self.get_parameter('distance_threshold').value)

    def timer_callback(self):
        """Called periodically; checks tf and appends waypoint if distance threshold exceeded."""
        distance = 0.0
        translation = None
        rotation = None

        try:
            transform = self.tfBuffer.lookup_transform('map', 'sirius3/base_link', rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.position = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]
            
            # 初回は現在位置をcheck_positionに設定（ウェイポイントは記録しない）
            if self.is_first_position or self.check_position is None:
                self.check_position = [translation.x, translation.y]
                self.is_first_position = False
                self.get_logger().info(f'初期位置を設定: ({translation.x:.3f}, {translation.y:.3f})')
                return
            
            x_distance = self.check_position[0] - translation.x
            y_distance = self.check_position[1] - translation.y
            distance = math.sqrt(x_distance ** 2 + y_distance ** 2)

        except tf2.LookupException:
            self.get_logger().warning('Transform not found')
            return
        except tf2.ExtrapolationException:
            self.get_logger().warning('Extrapolation error')
            return
        except tf2.ConnectivityException:
            self.get_logger().warning("Could not find a connection between 'map' and 'sirius3/base_link'")
            return
        except Exception as e:
            self.get_logger().warning(f'Unexpected TF exception: {e}')
            return

        # Only process further if we have valid transform data and threshold exceeded
        if translation is not None and rotation is not None and distance > self.distance_threshold:
            yaw = math.atan2(2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                             1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))

            waypoint = {
                'x': float(translation.x),
                'y': float(translation.y),
                'angle_radians': float(yaw)
            }

            # Append to YAML immediately; this also sets waypoint['number']
            assigned_number = self._append_waypoint_to_file(waypoint)
            waypoint['number'] = assigned_number

            # Update position check to new point
            self.check_position[0] = translation.x
            self.check_position[1] = translation.y

            self.get_logger().info('Success!')
            # self.get_logger().info(f'Appended waypoint: {waypoint} (distance: {distance:.3f})')

    def finish_write(self):
        # 最後の位置も追加
        if self.position:
            yaw = math.atan2(2.0 * (self.position[6] * self.position[5] + self.position[3] * self.position[4]),
                           1.0 - 2.0 * (self.position[4] * self.position[4] + self.position[5] * self.position[5]))
            waypoint = {
                'number': self.waypoint_number,
                'x': float(self.position[0]),
                'y': float(self.position[1]),
                'angle_radians': float(yaw)
            }
            self.positions_list.append(waypoint)
        
        # Load existing YAML and append
        existing_data = None
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r', encoding='utf-8') as rf:
                    existing_data = yaml.safe_load(rf)
            except Exception:
                existing_data = None

        if not existing_data or 'waypoints' not in existing_data:
            existing_data = {
                'format_version': '1.0',
                'waypoints': []
            }

        # Determine base number to continue numbering
        try:
            last_num = int(existing_data['waypoints'][-1]['number']) if existing_data['waypoints'] else 0
        except Exception:
            last_num = 0
        # Append collected waypoints with incremented numbers if not already appended
        if self.positions_list:
            initial_count = len(existing_data['waypoints'])
            for i, wp in enumerate(self.positions_list):
                # Check duplication by comparing last waypoint coordinate(s)
                if existing_data['waypoints'] and i == 0:
                    last_wp = existing_data['waypoints'][-1]
                    if (abs(last_wp.get('x', 0) - wp.get('x', 0)) < 1e-6 and
                        abs(last_wp.get('y', 0) - wp.get('y', 0)) < 1e-6):
                        # skip duplicate
                        continue
                wp['number'] = last_num + i + 1
                existing_data['waypoints'].append(wp)
            # Log appended details
            appended_numbers = [wp['number'] for wp in existing_data['waypoints'][initial_count:]]
            if appended_numbers:
                self.get_logger().info(f'Appending {len(appended_numbers)} waypoint(s): numbers {appended_numbers}')

        # YAMLファイルに書き込む
        with open(file_path, 'w', encoding='utf-8') as f:
            yaml.dump(existing_data, f, default_flow_style=False)

        self.get_logger().info('Finish')

    def _append_waypoint_to_file(self, waypoint):
        """
        Read YAML and append the given waypoint, set its number and return the assigned number.
        """
        existing_data = None
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r', encoding='utf-8') as rf:
                    existing_data = yaml.safe_load(rf)
            except Exception:
                existing_data = None

        if not existing_data or 'waypoints' not in existing_data:
            existing_data = {
                'format_version': '1.0',
                'waypoints': []
            }

        try:
            last_num = int(existing_data['waypoints'][-1]['number']) if existing_data['waypoints'] else 0
        except Exception:
            last_num = 0

        assigned_number = last_num + 1
        waypoint['number'] = assigned_number
        existing_data['waypoints'].append(waypoint)
        with open(file_path, 'w', encoding='utf-8') as f:
            yaml.dump(existing_data, f, default_flow_style=False)

        self.last_written_number = assigned_number
        return assigned_number

def main(args=None):
    rclpy.init(args=args)
    node = GetPose()
    try:
        rclpy.spin(node)
    finally:
        # Always attempt to flush collected waypoints on shutdown
        try:
            node.finish_write()
        except Exception as e:
            node.get_logger().warning(f'finish_write failed: {e}')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
