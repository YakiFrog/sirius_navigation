import os
import launch.conditions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_sirius_nav = get_package_share_directory('sirius_navigation')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    include_background = LaunchConfiguration('include_background')
    run_slam_toolbox = LaunchConfiguration('run_slam_toolbox')
    prompt = LaunchConfiguration('prompt')
    use_docker_backend = LaunchConfiguration('use_docker_backend')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo/Unity/Rosbag) clock if true'
    )
    
    declare_include_bg = DeclareLaunchArgument(
        'include_background',
        default_value='true',
        description='Include floor and background points in the colored 2D/3D map'
    )

    declare_run_slam = DeclareLaunchArgument(
        'run_slam_toolbox',
        default_value='true',
        description='Run Slam Toolbox node during offline mapping'
    )

    declare_prompt = DeclareLaunchArgument(
        'prompt',
        default_value='grass, tactile paving, roadway, sidewalk',
        description='SAM3 text prompts for semantic classes'
    )

    declare_use_docker = DeclareLaunchArgument(
        'use_docker_backend',
        default_value='true',
        description='Use GPU Docker container (sam3_zed_server on port 8080) for SAM3 inference'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for offline mapping monitoring if true'
    )
    use_rviz = LaunchConfiguration('rviz')

    # SlamToolbox Parameter Selection
    params_dir = os.path.join(os.path.expanduser('~'), 'sirius_jazzy_ws', 'params')
    sim_params = os.path.join(params_dir, 'mapper_params_online_async_sim.yaml')
    real_params = os.path.join(params_dir, 'mapper_params_online_async.yaml')
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=sim_params,
        description='Full path to the ROS2 parameters file to use for slam_toolbox'
    )

    # 1A. SAM3 Rosbag Player (Feeds rosbag frames to Docker backend on port 8080)
    sam3_player_node = Node(
        package='sirius_navigation',
        executable='sam3_rosbag_player',
        name='sam3_rosbag_player',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        condition=launch.conditions.IfCondition(use_docker_backend)
    )

    # 1B. SAM3 ROS Bridge (Receives 3D pointcloud from Docker backend via WebSocket)
    sam3_bridge_node = Node(
        package='sirius_navigation',
        executable='sam3_ros_bridge',
        name='sam3_ros_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_full_cloud': PythonExpression(["'", include_background, "' == 'true'"]),
        }],
        condition=launch.conditions.IfCondition(use_docker_backend)
    )

    # 1C. Standalone SAM3 Offline Node (Used when use_docker_backend:=false)
    sam3_offline_node = Node(
        package='sirius_navigation',
        executable='sam3_offline_node',
        name='sam3_offline_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'prompt': prompt,
            'threshold': 0.5,
            'downsample': 4,
            'max_depth_m': 15.0,
            'frame_id': 'sirius3/zed_camera_link',
        }],
        condition=launch.conditions.UnlessCondition(use_docker_backend)
    )

    # 2. SLAM Toolbox (Online Async SLAM with Lifecycle auto-start)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items(),
        condition=launch.conditions.IfCondition(run_slam_toolbox)
    )

    # 3. RTAB-Map Node (3D Spatial Mapping)
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
            'wait_for_transform': 0.5,
            'publish_tf': False,
            'odom_frame_id': 'sirius3/odom',
            'map_frame_id': 'map',
            'Rtabmap/PublishTf': 'false',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'Grid/FromDepth': 'true',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',
            'Mem/MaxSize': '3000',
            'Grid/VoxelSize': '0.05',
            'Optimizer/Strategy': '1',
            'Grid/RangeMax': '8.0',
            'Grid/RangeMin': '0.5',
            'Grid/NoiseFilteringRadius': '0.1',
            'Grid/NoiseFilteringMinNeighbors': '5',
            'Grid/CellSize': '0.05',
        }],
        remappings=[
            ('scan_cloud', PythonExpression(["'/sam3/full_cloud' if '", include_background, "' == 'true' else '/sam3/obstacles'"])),
            ('map', '/rtabmap/grid_map'),
        ],
        arguments=['--delete_db_on_start']
    )

    # 4. SAM3 Indexed 2D Map Node
    sam3_indexed_map_node = Node(
        package='sirius_navigation',
        executable='sam3_indexed_map_node',
        name='sam3_indexed_map_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'grid_resolution': 0.05,
            'map_frame': 'map',
            'semantic_cloud_topic': '/sam3/full_cloud_semantic',
        }]
    )

    # 5. Visualizer Node
    sam3_grid_visualizer_node = Node(
        package='sirius_navigation',
        executable='sam3_grid_visualizer',
        name='sam3_grid_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # 6. RViz2 Monitoring Node
    rviz_config_path = os.path.join(pkg_sirius_nav, 'rviz', 'sam3_offline_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_offline_mapping',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_include_bg,
        declare_run_slam,
        declare_prompt,
        declare_use_docker,
        declare_use_rviz,
        declare_params_file,
        sam3_player_node,
        sam3_bridge_node,
        sam3_offline_node,
        slam_toolbox_launch,
        rtabmap_node,
        sam3_indexed_map_node,
        sam3_grid_visualizer_node,
        rviz_node
    ])
