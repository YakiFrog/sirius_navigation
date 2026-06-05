#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package directories
    sirius_navigation_dir = get_package_share_directory('sirius_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths to default files
    default_map = os.path.expanduser('~/sirius_jazzy_ws/maps_waypoints/maps/0422map.yaml')
    default_params_file = os.path.expanduser('~/sirius_jazzy_ws/params/nav2_params_sim.yaml')
    
    # Declare arguments
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if True'
    )
    
    # Launch Configurations
    map_config = LaunchConfiguration('map')
    params_file_config = LaunchConfiguration('params_file')
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    
    # 1. Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_config,
            'map': map_config,
            'params_file': params_file_config,
            'autostart': 'True',
            'use_composition': 'False'
        }.items()
    )
    
    # 2. Assisted Teleop (using a TimerAction to delay startup until Nav2 is ready)
    assisted_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sirius_navigation_dir, 'launch', 'assisted_teleop.launch.py')
        )
    )
    
    # Delay Assisted Teleop startup by 10 seconds to allow behavior_server and controllers to initialize
    delayed_assisted_teleop = TimerAction(
        period=10.0,
        actions=[assisted_teleop]
    )
    
    return LaunchDescription([
        declare_map,
        declare_params_file,
        declare_use_sim_time,
        nav2_bringup,
        delayed_assisted_teleop
    ])
