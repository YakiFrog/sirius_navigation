"""Real VLP-16: raw points for LIO, leveled points for display and LaserScan."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('velodyne'), 'launch',
            'velodyne-all-nodes-VLP16-composed-launch.py')),
            launch_arguments={'scan_input_topic': '/velodyne_points_leveled'}.items()),
        Node(package='sirius_navigation', executable='leveled_lidar_cloud_node',
             name='leveled_lidar_cloud', output='screen'),
    ])
