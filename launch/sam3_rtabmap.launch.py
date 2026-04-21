import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Directories
    pkg_sirius_nav = get_package_share_directory('sirius_navigation')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # SAM3 ROS Bridge Node
    sam3_bridge_node = Node(
        package='sirius_navigation',
        executable='sam3_ros_bridge',
        name='sam3_ros_bridge',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # SLAM Toolbox Node (2D Mapping)
    slam_toolbox_params = os.path.join(pkg_sirius_nav, 'config', 'slam_toolbox_params.yaml')
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_toolbox_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # RTAB-Map Node (3D Colored Mapping)
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'sirius3/base_footprint',
            'subscribe_scan_cloud': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'approx_sync': True,
            'use_sim_time': use_sim_time,
            'wait_for_transform': 0.2,
            # RTAB-Map parameters
            'Rtabmap/PublishTf': 'false',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'Grid/FromDepth': 'true',
            'Reg/Strategy': '0', # 0=Visual, 1=ICP, 2=Both
            'Reg/Force3DoF': 'true', # Force 2D mapping for loop closures if preferred
        }],
        remappings=[
            ('scan_cloud', '/sam3/obstacles'),
            ('map', '/rtabmap/grid_map'),
        ],
        arguments=['--delete_db_on_start']
    )

    # RTAB-Map Viz (Optional, if you want a separate window)
    # rtabmap_viz_node = Node(
    #     package='rtabmap_viz',
    #     executable='rtabmapviz',
    #     name='rtabmapviz',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        sam3_bridge_node,
        slam_toolbox_node,
        rtabmap_node
    ])
