#!/usr/bin/env python3
"""
IMUとオドメトリのセンサフュージョン起動ファイル
robot_localizationパッケージを使用してEKF(拡張カルマンフィルタ)で融合
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パラメータファイルのパス（絶対パス）
    params_file = os.path.join(
        os.path.expanduser('~'),
        'sirius_jazzy_ws',
        'params',
        'ekf_fusion.yaml'
    )
    
    # EKFノード
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/odometry/filtered', '/odom/filtered')
        ]
    )
    
    return LaunchDescription([
        ekf_node
    ])