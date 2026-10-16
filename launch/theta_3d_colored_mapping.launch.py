"""THETAで色付き3D点群マップを生成する（FAST-LIO2オドメトリ駆動）。

構成:
  theta_capture_node        : THETAデュアルフィッシュアイ配信
  theta_colorize_cloud_node : /velodyne_points をTHETA画像で色付け -> /velodyne_points_colored
  theta_keyframe_map_node   : 色付き点群をキーフレーム蓄積し、map->odom で再描画 -> PLY保存

前提:
  - lio3d（FAST-LIO2）が odom->base を配信していること
  - tf_real / rviz2real がロボットTF（base->lidar->theta）を配信していること
  - THETAが /dev/theta_capture で見えること（udev）
保存:
  - autosave_period 秒ごと + SIGINT時に save_path へPLY保存
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('sirius_navigation')

    save_path = LaunchConfiguration('save_path')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    voxel = LaunchConfiguration('voxel')
    autosave_period = LaunchConfiguration('autosave_period')
    use_capture = LaunchConfiguration('use_capture')
    rviz = LaunchConfiguration('rviz')

    args = [
        DeclareLaunchArgument('save_path',
                              default_value=os.path.expanduser('~/colored_map_fastlio.ply'),
                              description='色付き3Dマップの保存先(.ply)'),
        DeclareLaunchArgument('odom_frame', default_value='sirius3/odom'),
        DeclareLaunchArgument('map_frame', default_value='map',
                              description='空文字でSLAM補正なし(odomのまま)'),
        DeclareLaunchArgument('voxel', default_value='0.05'),
        DeclareLaunchArgument('autosave_period', default_value='30.0',
                              description='定期保存の間隔[s] (0で無効)'),
        DeclareLaunchArgument('use_capture', default_value='true',
                              description='theta_capture_node も起動する'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    capture = Node(
        package='sirius_navigation', executable='theta_capture_node', name='theta_capture',
        parameters=[{'device': '/dev/theta_capture', 'fourcc': 'YUYV', 'fps': 5.0,
                     'width': 1280, 'height': 720, 'publish_raw': True}],
        condition=IfCondition(use_capture),
        output='screen')

    colorize = Node(
        package='sirius_navigation', executable='theta_colorize_cloud_node',
        name='theta_colorize_cloud', output='screen',
        parameters=[{
            'calibration': os.path.join(share, 'config', 'theta_calibration_real.yaml'),
            'lidar_topic': '/velodyne_points',
            'output_topic': '/velodyne_points_colored',
            'image_transport': 'compressed',
            'use_tf': True,
            'camera_frame': 'sirius3/theta_link',
            'min_range': 0.5,
            'max_range': 60.0,
        }])

    with open(os.path.join(share, 'config', 'fast_lio_sirius.yaml')) as stream:
        mapping = yaml.safe_load(stream)['/**']['ros__parameters']['mapping']
    keyframe = Node(
        package='sirius_navigation', executable='theta_keyframe_map_node',
        name='theta_keyframe_map', output='screen',
        parameters=[{
            'cloud_topic': '/velodyne_points_colored',
            'pose_source': 'odom', 'odom_topic': '/Odometry', 'body_frame': 'body',
            'use_lidar_extrinsic': True, 'lidar_frame': 'sirius3/lidar_link',
            'lidar_translation': mapping['extrinsic_T'], 'lidar_rotation': mapping['extrinsic_R'],
            'geometry_topic': '/Laser_map', 'color_max_distance': 0.5,
            'odom_frame': odom_frame,
            'map_frame': map_frame,
            'output_topic': '/colored_map_3d',
            'voxel': ParameterValue(voxel, value_type=float),
            'min_range': 0.5,
            'max_range': 60.0,
            'rebuild_period': 5.0,
            'autosave_period': ParameterValue(autosave_period, value_type=float),
            'save_path': save_path,
        }])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_colored3d',
        arguments=['-d', os.path.join(share, 'rviz', 'theta_3d_map.rviz')],
        condition=IfCondition(rviz), output='screen')

    return LaunchDescription(args + [capture, colorize, keyframe, rviz_node])
