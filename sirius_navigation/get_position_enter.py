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
import sys
import os

file_path = os.path.expanduser("~/sirius_jazzy_ws/maps_waypoints/waypoints/raw_waypoints.yaml")

class GetPose(Node):
    def __init__(self):
        super().__init__('get_pose')
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)
        self.position = []
        self.positions_list = []
        self.waypoint_number = 1
        self.timer = self.create_timer(1.0, self.get_position)

    def get_position(self):
        try:
            transform = self.tfBuffer.lookup_transform('map','sirius3/base_link',rclpy.time.Time())

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # クォータニオンからヨー角を計算
            yaw = math.atan2(2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                           1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))

            waypoint = {
                'number': self.waypoint_number,
                'x': float(translation.x),
                'y': float(translation.y),
                'angle_radians': float(yaw)
            }

            # Load existing YAML (safe) if present and prepare data structure
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

            # If there are existing waypoints, append and increment number
            if existing_data['waypoints']:
                try:
                    last_number = int(existing_data['waypoints'][-1].get('number', self.waypoint_number))
                except Exception:
                    last_number = self.waypoint_number
                waypoint['number'] = last_number + 1
                existing_data['waypoints'].append(waypoint)
            else:
                # No waypoints yet -> append new as first
                waypoint['number'] = self.waypoint_number
                existing_data['waypoints'].append(waypoint)

            # Write back YAML
            with open(file_path, 'w', encoding='utf-8') as f:
                yaml.dump(existing_data, f, default_flow_style=False)

            self.get_logger().info('Success!')

            sys.exit()

        except tf2.LookupException:
            self.get_logger().warning('Transform not found')

        except tf2.ExtrapolationException:
            self.get_logger().warning('Extrapolation error')
        except tf2.ConnectivityException:
            self.get_logger().warning("Could not find a connection between 'map' and 'sirius3/base_link'")
        

def main(args=None):
    rclpy.init(args=args)
    node = GetPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
