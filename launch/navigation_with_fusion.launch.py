#!/usr/bin/env python3
"""
Nav2とセンサフュージョンを統合した起動ファイル
センサフュージョン(EKF) + Nav2を一括で起動
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():
    # センサフュージョンのlaunchファイル
    sensor_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sirius_navigation'),
                'launch',
                'sensor_fusion.launch.py'
            ])
        ])
    )
    
    # Nav2のlaunchファイル（3秒後に起動）
    nav2_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'params_file': os.path.join(
                        os.path.expanduser('~'),
                        'sirius_jazzy_ws',
                        'params',
                        'nav2_params.yaml'
                    ),
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        sensor_fusion_launch,
        nav2_launch
    ])
