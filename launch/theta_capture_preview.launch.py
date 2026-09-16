"""THETA 実機キャプチャのライブBEVプレビュー用ランチ（sirius_navigation）。

前提: `theta_capture`（theta_capture_node）を別ターミナルで起動済みであること。
  /theta/dual_fisheye/image_raw/compressed を配信している。

このランチが起動するもの:
  theta_bev_node : dual-fisheye -> /theta/bev/image_raw（平面BEV）
  rviz2          : theta_capture.rviz（raw dual-fisheye と BEV を並べて表示）

実機既定として config/theta_calibration_real.yaml を使用する。
カメラ姿勢はTF(sirius3/base_footprint<-sirius3/theta_link)を試し、無ければ
YAMLの camera_position へフォールバックする（use_tf:=false で常にYAML）。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('sirius_navigation')
    calibration = LaunchConfiguration('calibration')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_tf = LaunchConfiguration('use_tf')

    declare_calibration = DeclareLaunchArgument(
        'calibration', default_value=os.path.join(share, 'config', 'theta_calibration_real.yaml'),
        description='校正YAML（実機既定）')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='RViz2を起動して raw dual-fisheye と BEV を表示')
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(share, 'rviz', 'theta_capture.rviz'),
        description='RViz2設定（raw + BEV）')
    declare_use_tf = DeclareLaunchArgument(
        'use_tf', default_value='true',
        description='TFからカメラ姿勢を取得（無ければYAMLへフォールバック）')

    theta_bev = Node(
        package='sirius_navigation', executable='theta_bev_node', name='theta_bev',
        parameters=[{
            'use_sim_time': False,
            'use_tf': ParameterValue(use_tf, value_type=bool),
            'publish_raw': False,  # 生Imageは theta_capture_node が配信済み（二重配信を避ける）
            'calibration': calibration,
        }],
        output='screen')

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_theta_capture',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        output='screen')

    return LaunchDescription([
        declare_calibration,
        declare_rviz,
        declare_rviz_config,
        declare_use_tf,
        theta_bev,
        rviz_node,
    ])
