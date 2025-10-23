#!/usr/bin/env python3
"""
Sirius Assisted Teleop with Nav2 Launchファイル

このlaunchファイルは以下を起動します:
1. Nav2スタック（behavior_serverを含む）
2. Assisted Teleopノード
3. Teleop Twist Keyboard（別ウィンドウ）

使用方法:
    # シミュレーションが既に起動している状態で:
    ros2 launch sirius_navigation assisted_teleop_with_nav2.launch.py

注意:
    - シミュレーション(sim.launch.py)が既に起動していることが前提
    - マップファイルのパスを適切に設定してください
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Descriptionを生成"""
    
    # パッケージディレクトリの取得
    sirius_nav_dir = get_package_share_directory('sirius_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # パラメータファイル
    params_file = os.path.join(sirius_nav_dir, 'config', 'nav2_params_assisted_teleop.yaml')
    
    # Launch引数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file_arg = LaunchConfiguration('params_file', default=params_file)
    
    # Launch引数の宣言
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use'
    )
    
    # Nav2 Bringup（localization + navigation）
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file_arg,
        }.items()
    )
    
    # Assisted Teleopノード（Nav2起動後に開始）
    assisted_teleop_node = Node(
        package='sirius_navigation',
        executable='assisted_teleop',
        name='sirius_assisted_teleop',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'time_allowance': 600},  # 10分
            {'set_initial_pose': True}
        ]
    )
    
    # Teleop Twist Keyboard（別ウィンドウ）
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )
    
    return LaunchDescription([
        # Launch引数
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        
        # 1. Nav2を起動
        nav2_bringup,
        
        # 2. 10秒後にAssisted Teleopを起動（Nav2の初期化を待つ）
        TimerAction(
            period=10.0,
            actions=[assisted_teleop_node]
        ),
        
        # 3. 12秒後にTeleopキーボードを起動
        TimerAction(
            period=12.0,
            actions=[teleop_keyboard_node]
        )
    ])
