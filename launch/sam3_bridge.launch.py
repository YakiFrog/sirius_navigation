import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sirius_navigation')
    params_file = os.path.join(pkg_share, 'config', 'sam3_bridge.yaml')

    sam3_bridge_node = Node(
        package='sirius_navigation',
        executable='sam3_ros_bridge',
        name='sam3_ros_bridge',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        sam3_bridge_node
    ])
