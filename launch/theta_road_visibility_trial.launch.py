"""Paired road-color trials on recorded map TF; no second SLAM optimization.

Save /trial/{baseline,visible}/save to different prefixes. Unknown/occluded
ground is never filled from neighboring colors. SAM3 only classifies road types.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('sirius_navigation')
    cal = LaunchConfiguration('calibration')
    padding = LaunchConfiguration('obstacle_padding')
    actions = [DeclareLaunchArgument('calibration', default_value=os.path.join(
        share, 'config', 'theta_calibration_real.yaml'))]
    actions.append(DeclareLaunchArgument('obstacle_padding', default_value='0.0',
                                        description='遮蔽範囲を広げる物理半径[m]'))
    actions += [Node(package='sirius_navigation', executable='theta_bev_node',
                     name='trial_bev', parameters=[{'use_sim_time': True,
                     'calibration': cal, 'publish_raw': False}], output='screen')]
    actions += [Node(package='sirius_navigation', executable='theta_sam3_perspective_node',
                     name='trial_road_semantics', parameters=[{'use_sim_time': True,
                     'calibration': cal, 'infer_timeout_sec': 10.0}], output='screen')]
    for name, strict in [('baseline', False), ('visible', True)]:
        cloud = f'/trial/{name}/cloud'
        actions += [Node(package='sirius_navigation', executable='theta_ground_cloud_node',
            name=f'trial_{name}_ground', parameters=[{
                'use_sim_time': True, 'calibration': cal, 'stride': 4,
                'output_topic': f'/trial/{name}/immediate',
                'semantic_cloud_topic': cloud, 'semantic_wait_sec': 6.0,
                'max_pending': 128, 'lidar_topic': '/scan3',
                'lidar_gate': strict, 'lidar_strict_visibility': strict,
                'lidar_margin_m': 0.10,
                'lidar_obstacle_padding_m': ParameterValue(padding, value_type=float)}], output='screen')]
        actions += [Node(package='sirius_navigation', executable='theta_indexed_map_node',
            name=f'trial_{name}_map', parameters=[{
                'use_sim_time': True, 'cloud_topic': cloud,
                'save_topic': f'/trial/{name}/save',
                'grid_topic': f'/trial/{name}/grid',
                'struct_map_topic': '/trial/unused_structure',
                'fill_hole_m': 0.0}], output='screen')]
    return LaunchDescription(actions)
