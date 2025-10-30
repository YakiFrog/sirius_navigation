#!/usr/bin/env python3
"""
Witmotion HWT905 IMUローンチファイル
HWT905 AHRSセンサを起動し、EKFが期待するトピック名にリマップします。

座標系の変換:
- HWT905: X前、Y右、Z下（右手系だが重力が+Z）
- ROS2 REP-103: X前、Y左、Z上（右手系、重力が-Z）
- 変換: Y軸とZ軸を反転（Y' = -Y, Z' = -Z）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    HWT905 IMUセンサのローンチ設定を生成
    
    トピック構成:
    - HWT905デフォルト出力: /wit/imu
    - EKF期待入力: /imu
    - リマップ: /wit/imu -> /imu
    """
    
    # ========================================
    # Launch引数の定義
    # ========================================
    
    # シリアルポート
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='HWT905が接続されているシリアルポート（例: /dev/ttyUSB0, /dev/ttyACM0）'
    )
    
    # ボーレート
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='シリアル通信のボーレート（HWT905デフォルト: 115200）'
    )
    
    # 更新周波数
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='100.0',
        description='IMUデータの更新周波数[Hz]（0.2-200Hz、推奨: 50-100Hz）'
    )
    
    # IMUフレームID（HWT905の生データ用）
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link_raw',
        description='HWT905の生データフレームID（座標変換前）'
    )
    
    # ROS2標準フレームID
    ros_frame_id_arg = DeclareLaunchArgument(
        'ros_frame_id',
        default_value='imu_link',
        description='ROS2標準座標系のフレームID（座標変換後、EKFが使用）'
    )
    
    # IMUトピック名（リマップ前）
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu',
        description='リマップ後のIMUトピック名（EKFが購読するトピック）'
    )
    
    # 座標変換を有効化するか
    use_transform_arg = DeclareLaunchArgument(
        'use_transform',
        default_value='true',
        description='HWT905からROS2標準座標系への変換を有効化（true/false）'
    )
    
    # ログレベル
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ログレベル (debug, info, warn, error)'
    )
    
    # ========================================
    # Launch設定の取得
    # ========================================
    
    port = LaunchConfiguration('port')
    baud_rate = LaunchConfiguration('baud_rate')
    frequency = LaunchConfiguration('frequency')
    frame_id = LaunchConfiguration('frame_id')
    ros_frame_id = LaunchConfiguration('ros_frame_id')
    imu_topic = LaunchConfiguration('imu_topic')
    log_level = LaunchConfiguration('log_level')
    use_transform = LaunchConfiguration('use_transform')
    
    # ========================================
    # witmotion_rosの設定ファイルパス
    # ========================================
    
    witmotion_config = PathJoinSubstitution([
        FindPackageShare('witmotion_ros'),
        'config',
        'wt905.yml'
    ])
    
    # ========================================
    # Witmotion HWT905ノード
    # ========================================
    
    witmotion_node = Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        name='hwt905_imu',
        output='screen',
        parameters=[
            witmotion_config,
            {
                'port': port,
                'baud_rate': baud_rate,
                'polling_interval': 10,  # ms（100Hz更新）
                'imu_publisher.frame_id': ros_frame_id,  # ROS2標準フレーム使用
                'imu_publisher.topic_name': imu_topic,  # EKFが期待するトピック
                'imu_publisher.use_native_orientation': False,  # ROS2座標系を使用
            }
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )
    
    # ========================================
    # 静的TF配信（base_footprint → imu_link）
    # ========================================
    # IMUの取り付け位置をTFで配信
    # ロボットの中心からのオフセットを設定
    
    imu_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_publisher',
        arguments=[
            '0', '0', '0.1',  # x, y, z (ロボット中心から10cm上方)
            '0', '0', '0',    # roll, pitch, yaw（回転なし）
            'base_footprint', # 親フレーム
            ros_frame_id,     # 子フレーム（imu_link）
        ],
        output='screen',
    )
    
    # ========================================
    # 起動情報の表示
    # ========================================
    
    startup_info = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            'Witmotion HWT905 IMU起動\n',
            '=' * 60, '\n',
            'シリアルポート: ', port, '\n',
            'ボーレート: ', baud_rate, '\n',
            '更新周波数: ', frequency, ' Hz\n',
            'フレームID: ', frame_id, '\n',
            '出力トピック: ', imu_topic, '\n',
            'ログレベル: ', log_level, '\n',
            '=' * 60, '\n',
            '\n',
            '【HWT905仕様】\n',
            '  - 姿勢精度: 0.05° (静止時)\n',
            '  - 角速度範囲: ±2000°/s\n',
            '  - 加速度範囲: ±16g\n',
            '  - 出力周波数: 0.2-200Hz (可変)\n',
            '  - 内蔵Kalmanフィルタ搭載\n',
            '\n',
            '【座標系変換】\n',
            '  witmotion_rosパッケージのuse_native_orientation=falseにより、\n',
            '  HWT905の座標系からROS2標準座標系(REP-103)へ自動変換されます。\n',
            '  - HWT905: X前、Y右、Z下（右手系、重力+Z）\n',
            '  - ROS2:   X前、Y左、Z上（右手系、重力-Z）\n',
            '\n',
            '【トピック構成】\n',
            '  ', imu_topic, ' (sensor_msgs/msg/Imu)\n',
            '    ├─ header.frame_id: ', ros_frame_id, '\n',
            '    ├─ orientation (quaternion): ROS2標準座標系の姿勢\n',
            '    ├─ angular_velocity: 角速度 [rad/s]\n',
            '    └─ linear_acceleration: 線形加速度 [m/s²]\n',
            '\n',
            '【TFフレーム】\n',
            '  base_footprint → ', ros_frame_id, ' (IMU位置)\n',
            '\n',
            '【使用方法】\n',
            '  1. HWT905をUSBポートに接続\n',
            '  2. ポート権限を設定: sudo chmod 666 /dev/ttyUSB0\n',
            '  3. このローンチファイルを起動\n',
            '  4. EKFと組み合わせて使用\n',
            '\n',
            '【トラブルシューティング】\n',
            '  - デバイスが見つからない場合:\n',
            '    ls /dev/ttyUSB* または ls /dev/ttyACM*\n',
            '  - 権限エラーの場合:\n',
            '    sudo usermod -a -G dialout $USER (再ログイン必要)\n',
            '  - データが取得できない場合:\n',
            '    ボーレートを確認 (デフォルト: 115200)\n',
            '=' * 60, '\n',
        ]
    )
    
    # ========================================
    # LaunchDescriptionの構築
    # ========================================
    
    return LaunchDescription([
        # Launch引数
        port_arg,
        baud_rate_arg,
        frequency_arg,
        frame_id_arg,
        ros_frame_id_arg,
        imu_topic_arg,
        use_transform_arg,
        log_level_arg,
        
        # 起動情報
        startup_info,
        
        # ノード
        witmotion_node,
        imu_tf_publisher,
    ])
