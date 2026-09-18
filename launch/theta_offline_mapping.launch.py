"""THETA オフライン路面マッピング用ランチ（sirius_navigation）。

記録済みrosbagを再生しながら:
  theta_bev_node         : /theta/dual_fisheye/image_raw/compressed -> /theta/bev/image_raw
  theta_ground_cloud_node: /theta/bev/image_raw -> /theta/ground_cloud (z=0の地面点群)
  theta_indexed_map_node : /theta/ground_cloud -> /theta/colored_map_grid (代表色indexed地図)
  theta_sam3_perspective_node (sam3:=true): 透視投影SAM3 -> /theta/bev_semantic
  rtabmap                : /theta/ground_cloud -> /rtabmap/grid_map, /cloud_map

SAM3・ステレオ・深度は使用しない。姿勢TFはbag内の補正済みTFを使う。
校正は package share の config/theta_calibration.yaml を既定で使用。

デバッグ可視化: publish_debug:=true で透視ビュー/マスク/オーバーレイ/意味色点群を配信。
rviz_config:=<path> でRViz設定を差し替え（theta_debug.rviz で検出〜地図を同時表示）。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('sirius_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    sam3 = LaunchConfiguration('sam3')
    publish_raw = LaunchConfiguration('publish_raw')
    publish_debug = LaunchConfiguration('publish_debug')
    calibration = LaunchConfiguration('calibration')
    min_radius = LaunchConfiguration('min_radius')
    max_radius = LaunchConfiguration('max_radius')
    grid_range_max = LaunchConfiguration('grid_range_max')
    lidar_topic = LaunchConfiguration('lidar_topic')
    lidar_gate = LaunchConfiguration('lidar_gate')
    semantic_debug_topic = PythonExpression(
        ["'/theta/ground_cloud_semantic' if '", publish_debug, "' == 'true' else ''"])

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (rosbag) clock')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 to preview the map during replay')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(share, 'rviz', 'theta_offline.rviz'),
        description='RViz2 config（デバッグは theta_debug.rviz を指定）')
    declare_calibration = DeclareLaunchArgument(
        'calibration', default_value=os.path.join(share, 'config', 'theta_calibration.yaml'),
        description='校正YAML（実機は theta_calibration_real.yaml を指定）')
    declare_min_radius = DeclareLaunchArgument(
        'min_radius', default_value='1.2',
        description='地面点群に含める最小半径[m]（自機の足元を除外）')
    declare_max_radius = DeclareLaunchArgument(
        'max_radius', default_value='4.8',
        description='地面点群に含める最大半径[m]。狭い曲がり角では小さくすると食い合いを軽減')
    declare_grid_range_max = DeclareLaunchArgument(
        'grid_range_max', default_value='7.0',
        description='RTAB-Map Grid/RangeMax。max_radius に合わせて小さくすると安定')
    declare_lidar_topic = DeclareLaunchArgument(
        'lidar_topic', default_value='',
        description='Scan3(2D LiDAR)のトピック。指定するとフリースペースで地面点を制限可能')
    declare_lidar_gate = DeclareLaunchArgument(
        'lidar_gate', default_value='false',
        description='true で /scan3 のフリースペース内の地面点だけを残す（食い合い軽減）')
    declare_sam3 = DeclareLaunchArgument(
        'sam3', default_value='false',
        description='Run theta_sam3_perspective_node (needs sam3 docker server)')
    declare_publish_raw = DeclareLaunchArgument(
        'publish_raw', default_value='false',
        description='Republish decoded dual fisheye as raw Image (for RViz debug)')
    declare_publish_debug = DeclareLaunchArgument(
        'publish_debug', default_value='false',
        description='Publish SAM3 view/mask/overlay and semantic-colored cloud for RViz')

    theta_bev = Node(
        package='sirius_navigation', executable='theta_bev_node', name='theta_bev',
        parameters=[{'use_sim_time': use_sim_time, 'use_tf': True,
                     'publish_raw': ParameterValue(publish_raw, value_type=bool),
                     'calibration': calibration}],
        output='screen')

    theta_cloud = Node(
        package='sirius_navigation', executable='theta_ground_cloud_node', name='theta_ground_cloud',
        parameters=[{'use_sim_time': use_sim_time, 'semantic_debug_topic': semantic_debug_topic,
                     'semantic_cloud_topic': '/theta/ground_cloud_semantic',
                     'min_radius': ParameterValue(min_radius, value_type=float),
                     'max_radius': ParameterValue(max_radius, value_type=float),
                     'lidar_topic': lidar_topic,
                     'lidar_gate': ParameterValue(lidar_gate, value_type=bool),
                     'calibration': calibration}],
        output='screen')

    theta_indexed = Node(
        package='sirius_navigation', executable='theta_indexed_map_node', name='theta_indexed_map_node',
        parameters=[{'use_sim_time': use_sim_time,
                     'cloud_topic': '/theta/ground_cloud_semantic'}],
        output='screen')

    # SAM3セマンティック（任意）。dual-fisheyeを透視投影しSAM3でクラス分類→地面逆投影して
    # /theta/bev_semantic を出す（既存Dockerサーバの単一画像経路）。
    theta_sam3 = Node(
        package='sirius_navigation', executable='theta_sam3_perspective_node', name='theta_sam3_perspective_node',
        parameters=[{'use_sim_time': use_sim_time,
                     'publish_debug': ParameterValue(publish_debug, value_type=bool),
                     'min_radius': ParameterValue(min_radius, value_type=float),
                     'max_radius': ParameterValue(max_radius, value_type=float),
                     'calibration': calibration}],
        condition=IfCondition(sam3),
        output='screen')

    # RTAB-Map（ZED版と同じ設定。scan_cloudだけTHETA地面点群へ差し替え）
    rtabmap_node = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        parameters=[{
            'frame_id': 'sirius3/base_footprint',
            'subscribe_scan_cloud': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'approx_sync': True,
            'use_sim_time': use_sim_time,
            'wait_for_transform': 0.2,
            'publish_tf': False,
            'odom_frame_id': 'map',
            'Rtabmap/PublishTf': 'false',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'Grid/FromDepth': 'false',
            'Reg/Strategy': '0',
            'Reg/Force3DoF': 'true',
            'Mem/MaxSize': '2000',
            'Rtabmap/DetectionRate': '2.0',
            'Grid/VoxelSize': '0.05',
            'Optimizer/Strategy': '1',
            'Grid/RangeMax': ParameterValue(grid_range_max, value_type=str),
            'Grid/RangeMin': '0.8',
            'Grid/NoiseFilteringRadius': '0.1',
            'Grid/NoiseFilteringMinNeighbors': '5',
            'Grid/CellSize': '0.05',
        }],
        remappings=[('scan_cloud', '/theta/ground_cloud'), ('map', '/rtabmap/grid_map')],
        arguments=['--delete_db_on_start'])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_theta_offline',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_rviz_config,
        declare_calibration,
        declare_min_radius,
        declare_max_radius,
        declare_grid_range_max,
        declare_lidar_topic,
        declare_lidar_gate,
        declare_sam3,
        declare_publish_raw,
        declare_publish_debug,
        theta_bev,
        theta_cloud,
        theta_indexed,
        theta_sam3,
        rtabmap_node,
        rviz_node,
    ])
