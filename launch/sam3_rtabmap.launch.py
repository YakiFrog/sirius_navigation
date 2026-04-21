import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Directories
    pkg_sirius_nav = get_package_share_directory('sirius_navigation')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    include_background = LaunchConfiguration('include_background')
    
    # SlamToolbox Parameter Selection
    params_dir = os.path.join(os.path.expanduser('~'), 'sirius_jazzy_ws', 'params')
    sim_params = os.path.join(params_dir, 'mapper_params_online_async_sim.yaml')
    real_params = os.path.join(params_dir, 'mapper_params_online_async.yaml')
    
    selected_params = PythonExpression([
        "'", sim_params, "' if '", use_sim_time, "' == 'true' else '", real_params, "'"
    ])
    
    slam_toolbox_params = LaunchConfiguration('slam_toolbox_params_file')
    
    # SAM3 ROS Bridge Node
    sam3_bridge_node = Node(
        package='sirius_navigation',
        executable='sam3_ros_bridge',
        name='sam3_ros_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_full_cloud': PythonExpression(["'", include_background, "' == 'true'"]),
        }]
    )

    # SLAM Toolbox Node (2D Mapping)
    # slam_toolbox_params = os.path.join(pkg_sirius_nav, 'config', 'slam_toolbox_params.yaml')
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
            'publish_tf': False,
            'odom_frame_id': 'map', # SlamToolboxの補正済み座標を基準にする(map)
            # RTAB-Map parameters
            'Rtabmap/PublishTf': 'false',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'Grid/FromDepth': 'true',
            'Reg/Strategy': '0', # 0=オドメトリ(SLAM Toolbox)を信頼, ICP補正なし
            'Reg/Force3DoF': 'true',
            # --- 負荷軽減のための設定 ---
            'Mem/MaxSize': '2000',           # メモリ内の最大ノード数。超えると古いノードをDBへ退避
            # --- 点群の間引き設定 ---
            'Grid/VoxelSize': '0.05',        # 2Dグリッドマップ作成時の間引き
            'Optimizer/Strategy': '1',       # 1=g2o (TOROの警告を回避するため)
            # --- ノイズ除去と精度向上のための設定 ---
            'Grid/RangeMax': '5.0',          # 5m以上先の不安定な点群は無視
            'Grid/RangeMin': '0.8',          # 0.8m以内の近すぎる点（ロボット自身など）を無視
            'Grid/NoiseFilteringRadius': '0.1', # 10cm以内に点がない孤立点を除去
            'Grid/NoiseFilteringMinNeighbors': '5', # 周囲に5点以上ない場合はノイズとして除去
            # --- 密度制限の強化 ---
            # 'scan_cloud_decimation': 2,      # 入力点群をあらかじめ1/2に間引く
            'Grid/CellSize': '0.05',         # 地図の解像度をVoxelSize(5cm)と同期
        }],
        remappings=[
            ('scan_cloud', PythonExpression(["'/sam3/full_cloud' if '", include_background, "' == 'true' else '/sam3/obstacles'"])),
            ('map', '/rtabmap/grid_map'),
        ],
        arguments=['--delete_db_on_start']
    )

    # Indexed Color Map Node (New from scratch)
    sam3_indexed_map_node = Node(
        package='sirius_navigation',
        executable='sam3_indexed_map_node',
        name='sam3_indexed_map_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'grid_resolution': 0.05,
        }]
    )

    # Real-time Grid Visualizer (New separate node)
    sam3_grid_visualizer_node = Node(
        package='sirius_navigation',
        executable='sam3_grid_visualizer',
        name='sam3_grid_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
        }]
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
        DeclareLaunchArgument('include_background', default_value='false'),
        DeclareLaunchArgument('slam_toolbox_params_file', default_value=selected_params),
        sam3_bridge_node,
        # slam_toolbox_node,  # 手動で起動するため、ここでは自動起動させない
        rtabmap_node,
        sam3_indexed_map_node,
        sam3_grid_visualizer_node
    ])
