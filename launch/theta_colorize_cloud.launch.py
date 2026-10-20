"""VLP-16 3D点群を RICOH THETA で色付けるライブノードのランチ（sirius_navigation）。

起動するもの:
  theta_capture_node      : /theta/dual_fisheye/image_raw/compressed を配信（use_capture:=true のとき）
  theta_colorize_cloud_node: /velodyne_points + THETA -> /velodyne_points_colored
  rviz2                   : 色付き点群を表示（rviz:=true のとき）

前提:
  - VLP-16 が /velodyne_points(sensor_msgs/PointCloud2, frame=sirius3/lidar_link) を配信していること。
  - THETA->LiDAR の外部パラメータは TF(camera_frame<-lidar frame) を優先し、
    無ければ theta_x/y/z, theta_rpy_* にフォールバックする。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _colorize_node(context, *args, **kwargs):
    cfg = context.launch_configurations
    theta_xyz = [float(cfg['theta_x']), float(cfg['theta_y']), float(cfg['theta_z'])]
    theta_rpy = [float(cfg['theta_roll']), float(cfg['theta_pitch']), float(cfg['theta_yaw'])]
    return [Node(
        package='sirius_navigation', executable='theta_colorize_cloud_node',
        name='theta_colorize_cloud', output='screen',
        parameters=[{
            'calibration': cfg['calibration'],
            'colorization_config': cfg['colorization_config'],
            'fixed_frame': cfg['fixed_frame'],
            'lidar_topic': cfg['lidar_topic'],
            'output_topic': cfg['output_topic'],
            'image_transport': cfg['image_transport'],
            'use_tf': cfg['use_tf'].lower() == 'true',
            'camera_frame': cfg['camera_frame'],
            'theta_xyz': theta_xyz,
            'theta_rpy_degrees': theta_rpy,
            'min_range': float(cfg['min_range']),
            'max_range': float(cfg['max_range']),
        }])]


def generate_launch_description():
    share = get_package_share_directory('sirius_navigation')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz = LaunchConfiguration('rviz')
    use_capture = LaunchConfiguration('use_capture')

    args = [
        DeclareLaunchArgument('calibration',
                              default_value=os.path.join(share, 'config', 'theta_calibration_real.yaml'),
                              description='THETA校正YAML（実機既定）'),
        DeclareLaunchArgument('lidar_topic', default_value='/velodyne_points',
                              description='3D LiDAR点群トピック'),
        DeclareLaunchArgument('colorization_config', default_value=os.path.join(share, 'config', 'theta_colorization.yaml'),
                              description='ロボット共通の外部校正・遮蔽判定設定'),
        DeclareLaunchArgument('fixed_frame', default_value='sirius3/odom',
                              description='撮影時刻への運動補償に使う固定座標系'),
        DeclareLaunchArgument('output_topic', default_value='/velodyne_points_colored',
                              description='色付き点群の出力トピック'),
        DeclareLaunchArgument('image_transport', default_value='compressed',
                              description='compressed または raw'),
        DeclareLaunchArgument('use_tf', default_value='true',
                              description='TF(camera_frame<-lidar)を優先する'),
        DeclareLaunchArgument('camera_frame', default_value='sirius3/theta_link',
                              description='THETAカメラフレーム'),
        DeclareLaunchArgument('theta_x', default_value='0.0', description='THETA位置 x [m]（LiDAR基準）'),
        DeclareLaunchArgument('theta_y', default_value='0.0', description='THETA位置 y [m]（LiDAR基準）'),
        DeclareLaunchArgument('theta_z', default_value='0.285', description='THETA位置 z [m]（LiDAR基準）'),
        DeclareLaunchArgument('theta_roll', default_value='0.0', description='THETA姿勢 roll [deg]'),
        DeclareLaunchArgument('theta_pitch', default_value='0.0', description='THETA姿勢 pitch [deg]'),
        DeclareLaunchArgument('theta_yaw', default_value='0.0', description='THETA姿勢 yaw [deg]'),
        DeclareLaunchArgument('min_range', default_value='0.5', description='色付けする最小距離[m]'),
        DeclareLaunchArgument('max_range', default_value='80.0', description='色付けする最大距離[m]'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz2を起動'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(share, 'rviz', 'theta_debug.rviz'),
                              description='RViz2設定'),
        DeclareLaunchArgument('use_capture', default_value='false',
                              description='true で theta_capture_node も起動（実機）'),
    ]

    capture = Node(
        package='sirius_navigation', executable='theta_capture_node', name='theta_capture',
        parameters=[{'device': '/dev/theta_capture', 'fourcc': 'YUYV', 'fps': 5.0,
                     'width': 1280, 'height': 720, 'publish_raw': True}],
        condition=IfCondition(use_capture),
        output='screen')

    colorize = OpaqueFunction(function=_colorize_node)

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_theta_colorize',
        arguments=['-d', rviz_config], condition=IfCondition(rviz), output='screen')

    return LaunchDescription(args + [capture, colorize, rviz_node])
