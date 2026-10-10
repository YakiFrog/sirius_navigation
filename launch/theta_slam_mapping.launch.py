"""ループ閉じ込めSLAM(SLAM Toolbox) + 色付き点群キーフレーム地図のオフライン用ランチ。

bag再生(--clock)と組み合わせて使う。役割:
  theta_cloud_to_scan_node : /velodyne_points_colored -> 2D /scan_from_cloud
  slam_toolbox             : 2D SLAM + ループ閉じ込め。map->odom を最適化（EKF車輪+IMUをodom priorに使用）
  theta_keyframe_map_node  : 色付き点群をodomでキーフレーム保存し、現在の map->odom で
                             全体を再描画。SIGINTでPLY保存（ループ閉じ込め反映）。

使い方:
  ros2 launch sirius_navigation theta_slam_mapping.launch.py save_path:=/path/colored_map_slam.ply
  別ターミナル: ros2 bag play <bag> --clock
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_topic = LaunchConfiguration('cloud_topic')
    save_path = LaunchConfiguration('save_path')
    voxel = LaunchConfiguration('voxel')
    slam_params = os.path.expanduser('~/sirius_jazzy_ws/params/theta_slam_3dmap_params.yaml')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='bag再生時は--clockと併せてtrue'),
        DeclareLaunchArgument('cloud_topic', default_value='/velodyne_points_colored'),
        DeclareLaunchArgument('save_path',
                              default_value=os.path.expanduser('~/colored_map_slam.ply')),
        DeclareLaunchArgument('voxel', default_value='0.05'),
    ]

    cloud_to_scan = Node(
        package='sirius_navigation', executable='theta_cloud_to_scan_node',
        name='theta_cloud_to_scan', output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'input_topic': cloud_topic,
            'output_topic': '/scan_from_cloud',
            'z_min': -0.5, 'z_max': 0.5,
            'range_min': 0.4, 'range_max': 50.0,
        }])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch',
                                  'online_async_launch.py'])]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': use_sim_time,
        }.items())

    keyframe = Node(
        package='sirius_navigation', executable='theta_keyframe_map_node',
        name='theta_keyframe_map', output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'cloud_topic': cloud_topic,
            'odom_frame': 'sirius3/odom',
            'map_frame': 'map',
            'output_topic': '/colored_map_3d',
            'voxel': ParameterValue(voxel, value_type=float),
            'save_path': save_path,
            'rebuild_period': 5.0,
        }])

    return LaunchDescription(args + [cloud_to_scan, slam, keyframe])
