import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # sirius_description (Unity connection and Robot State Publisher)
    sirius_description_dir = get_package_share_directory('sirius_description')
    sirius_navigation_dir = get_package_share_directory('sirius_navigation')
    
    # Use the simulation-specific params file
    # Based on the file search, it's at jazzy_ws/params/nav2_params_sim.yaml
    # We should use the one in jazzy_ws/params directory as requested in implementation plan
    params_file = LaunchConfiguration('params_file')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join('/home/kotantu-nuc/sirius_jazzy_ws', 'params', 'nav2_params_sim.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )

    # 1. Unity Sim Bridge (UnityConnect, RobotStatePublisher, Rviz)
    unity_sim_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sirius_description_dir, 'launch', 'unity_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # 2. Nav2 Bringup
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
            'autostart': 'true',
            'map': os.path.join('/home/kotantu-nuc/sirius_jazzy_ws', 'maps_waypoints', 'map.yaml') # Placeholder map
        }.items()
    )

    return LaunchDescription([
        declare_params_file,
        unity_sim_bridge,
        nav2_bringup
    ])
