"""3D地図ベースの自己位置推定 + Nav2 自律移動セット。

構成:
  - map_localizer_node : FAST-LIO2オドメトリを初期値に small_gicp で map->odom を配信
  - map_server         : 2D投影地図を /map として配信（Nav2静的レイヤ用）
  - navigation_launch  : Nav2（AMCLなし。map_serverは本launchで起動）

AMCLは起動しない（map->odom は localizer が担当）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('sirius_navigation')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    default_localizer_params = os.path.join(pkg, 'config', 'map_localizer.yaml')

    map2d = LaunchConfiguration('map2d')
    map3d = LaunchConfiguration('map3d')
    nav2_params = LaunchConfiguration('nav2_params')
    localizer_params = LaunchConfiguration('localizer_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_3dof = LaunchConfiguration('use_3dof')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    initial_pose_file = LaunchConfiguration('initial_pose_file')

    args = [
        DeclareLaunchArgument('map2d', default_value='',
                              description='Nav2用2D投影地図 (.yaml)'),
        DeclareLaunchArgument('map3d', default_value='',
                              description='自己位置推定用3D地図 (.npy/.ply/.pcd)'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.expanduser('~/sirius_jazzy_ws/params/nav2_params.yaml')),
        DeclareLaunchArgument('localizer_params', default_value=default_localizer_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_3dof', default_value='true'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_file', default_value=''),
    ]

    localizer = Node(
        package='sirius_navigation',
        executable='map_localizer_node',
        name='map_localizer',
        output='screen',
        parameters=[
            localizer_params,
            {
                'map_path': map3d,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'use_3dof': ParameterValue(use_3dof, value_type=bool),
                'initial_pose_x': ParameterValue(initial_pose_x, value_type=float),
                'initial_pose_y': ParameterValue(initial_pose_y, value_type=float),
                'initial_pose_z': ParameterValue(initial_pose_z, value_type=float),
                'initial_pose_yaw': ParameterValue(initial_pose_yaw, value_type=float),
                'initial_pose_file': initial_pose_file,
            },
        ],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map2d,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    lifecycle_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_composition': 'False',
            'params_file': nav2_params,
        }.items(),
    )

    return LaunchDescription(
        args + [localizer, map_server, lifecycle_map, TimerAction(period=1.0, actions=[navigation])]
    )
