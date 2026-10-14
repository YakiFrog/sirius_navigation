"""FAST-LIO2 + フレームブリッジ（camera_init/body -> sirius3/odom/base_footprint）。

FAST-LIO2は camera_init->body のTFと /Odometry を配信する。
アプリ/Nav2 が使う sirius3/odom -> sirius3/base_footprint に接続するため、
静的TFを2つ追加する:
  sirius3/odom -> camera_init   (恒等)
  body         -> sirius3/base_footprint (恒等)
※ LiDAR-IMU 外部校正(extrinsic_T/R)と robot TF の整合は Phase 0 で調整する。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sirius = get_package_share_directory('sirius_navigation')
    fast_lio_share = get_package_share_directory('fast_lio')

    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_height = LaunchConfiguration('imu_height')

    args = [
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(sirius, 'config')),
        DeclareLaunchArgument('config_file', default_value='fast_lio_sirius.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('imu_height', default_value='0.65',
                              description='base_footprintからIMUまでの高さ[m]'),
    ]

    bridge_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='lio_bridge_odom',
        arguments=['--frame-id', 'sirius3/odom', '--child-frame-id', 'camera_init'],
    )
    bridge_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='lio_bridge_base',
        arguments=['--frame-id', 'body', '--child-frame-id', 'sirius3/base_footprint',
                   '--z', PythonExpression(['-', imu_height])],
    )

    lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'config_path': config_path,
            'config_file': config_file,
            'use_sim_time': use_sim_time,
            'rviz': 'false',
        }.items(),
    )

    return LaunchDescription(args + [bridge_odom, bridge_base, lio])
