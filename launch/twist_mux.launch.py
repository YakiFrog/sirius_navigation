import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_sirius_nav = get_package_share_directory('sirius_navigation')
    
    # twist_mux config
    twist_mux_params = os.path.join(pkg_sirius_nav, 'config', 'twist_mux.yaml')
    
    # twist_mux node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params],
        remappings=[
            ('cmd_vel_out', 'cmd_vel')
        ]
    )
    
    # twist_mux monitor node
    twist_mux_monitor_node = Node(
        package='sirius_navigation',
        executable='twist_mux_monitor',
        name='twist_mux_monitor',
        output='screen'
    )
    
    return LaunchDescription([
        twist_mux_node,
        twist_mux_monitor_node
    ])
