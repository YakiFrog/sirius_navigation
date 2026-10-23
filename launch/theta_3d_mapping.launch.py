"""ライブ3Dマッピング（RTAB-Map + FAST-LIO2オドメトリ）。

構成:
  - FAST-LIO2 : odom->base（別途 lio3d で起動済みのこと）
  - RTAB-Map  : /velodyne_points を外部オドメトリ(/Odometry)で取り込み、
                ループ閉じ込めしつつ /cloud_map と /rtabmap/grid_map を配信

保存は save3d（/cloud_map を指定）で行う。地図生成中は map->odom を配信するため、loc3dとは併用しない。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_topic = LaunchConfiguration('cloud_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    grid_range_max = LaunchConfiguration('grid_range_max')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/velodyne_points'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('grid_range_max', default_value='30.0'),
    ]

    rtabmap_node = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        parameters=[{
            'frame_id': 'sirius3/base_footprint',
            'subscribe_scan_cloud': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'approx_sync': True,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'wait_for_transform': 1.0,
            'publish_tf': True,
            'odom_frame_id': 'sirius3/odom',
            'Rtabmap/PublishTf': 'true',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/ProximityOdomGuess': 'true',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'Grid/FromDepth': 'false',
            # LiDAR-only registration: 0 is visual and requires camera features.
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',
            'Mem/MaxSize': '2000',
            'Rtabmap/DetectionRate': '2.0',
            'Grid/VoxelSize': '0.05',
            'Optimizer/Strategy': '1',
            'Grid/RangeMin': '0.8',
            'Grid/RangeMax': ParameterValue(grid_range_max, value_type=str),
        }],
        remappings=[
            ('scan_cloud', cloud_topic),
            ('odom', odom_topic),
            ('imu', '/sirius_3d_unused_imu'),
            ('map', '/rtabmap/grid_map'),
        ],
    )

    return LaunchDescription(args + [rtabmap_node])
