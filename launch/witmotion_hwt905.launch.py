#!/usr/bin/env python3
"""
Witmotion HWT905 IMUセンサー起動ファイル
sirius_navigationパッケージ用

使用例:
  # デフォルト設定で起動
  ros2 launch sirius_navigation witmotion_hwt905.launch.py

  # カスタム設定ファイルを指定
  ros2 launch sirius_navigation witmotion_hwt905.launch.py config_file:=/path/to/custom_config.yml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Expand home so os.path.exists works
    default_config = os.path.expanduser('~/sirius_jazzy_ws/params/wt905.yaml')

    if not os.path.exists(default_config):
        try:
            default_config = os.path.join(
                get_package_share_directory('witmotion_ros'),
                'config',
                'wt905.yml'
            )
        except Exception:
            default_config = ''

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Witmotion ROSノードの設定ファイルへのパス'
    )

    # ログで実際に使われる設定ファイルを出力する
    config_log = LogInfo(msg=['Using witmotion config: ', LaunchConfiguration('config_file')])

    witmotion_node = Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        name='witmotion',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        config_log,
        witmotion_node,
    ])
