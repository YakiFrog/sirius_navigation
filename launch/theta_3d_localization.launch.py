"""3D地図への局所自己位置推定（small_gicp）を起動する。

前提:
  - FAST-LIO2 等が odom->base_footprint を配信していること
  - 事前3D地図（.npy/.ply/.pcd、mapフレーム）が用意されていること

AMCLとは排他で使用する（map->odom を二重配信しない）。
初期姿勢は initial_pose_* / initial_pose_file / RVizの /initialpose で与える。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('sirius_navigation')
    default_params = os.path.join(pkg, 'config', 'map_localizer.yaml')

    map_path = LaunchConfiguration('map_path')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    map_frame = LaunchConfiguration('map_frame')
    use_3dof = LaunchConfiguration('use_3dof')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    initial_pose_file = LaunchConfiguration('initial_pose_file')

    args = [
        DeclareLaunchArgument('map_path', default_value='',
                              description='事前3D地図 (.npy/.ply/.pcd, mapフレーム)'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/cloud_registered_body',
                              description='Deskewed FAST-LIO body-frame scan matched to its odometry TF'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='sirius3/odom'),
        DeclareLaunchArgument('base_frame', default_value='sirius3/base_footprint'),
        DeclareLaunchArgument('use_3dof', default_value='true'),
        DeclareLaunchArgument('publish_map', default_value='false',
                              description='PLY参照地図を /localization/map に配信'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_file', default_value='',
                              description='x,y,z,yaw を書いたYAML（任意）'),
    ]

    localizer = Node(
        package='sirius_navigation',
        executable='map_localizer_node',
        name='map_localizer',
        output='screen',
        parameters=[
            params_file,
            {
                'map_path': map_path,
                'cloud_topic': LaunchConfiguration('cloud_topic'),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'use_3dof': ParameterValue(use_3dof, value_type=bool),
                'initial_pose_x': ParameterValue(initial_pose_x, value_type=float),
                'initial_pose_y': ParameterValue(initial_pose_y, value_type=float),
                'initial_pose_z': ParameterValue(initial_pose_z, value_type=float),
                'initial_pose_yaw': ParameterValue(initial_pose_yaw, value_type=float),
                'initial_pose_file': initial_pose_file,
            },
        ],
    )

    reference_map = Node(
        package='sirius_navigation', executable='theta_ply_publisher',
        name='localization_map_publisher', output='screen',
        condition=IfCondition(LaunchConfiguration('publish_map')),
        parameters=[{'ply_file': map_path, 'frame_id': map_frame,
                     'topic': '/localization/map',
                     'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )
    return LaunchDescription(args + [localizer, reference_map])
