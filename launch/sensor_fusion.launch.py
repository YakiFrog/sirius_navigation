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
    
    # EKFノード（詳細ログとQoS互換性の設定）
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            params_file,
            {
                # QoS設定の追加パラメータ
                'debug': False,
                'debug_out_file': '/tmp/ekf_debug.txt'
            }
        ],
        remappings=[
            ('/odometry/filtered', '/odom/filtered')
        ],
        # ログレベルを詳細に設定
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        ekf_node
    ])