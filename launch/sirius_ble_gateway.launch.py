from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_remote_server", default_value="true"),
        DeclareLaunchArgument("enable_battery_client", default_value="false"),
        DeclareLaunchArgument("battery_mac", default_value=""),
        DeclareLaunchArgument("battery_poll_interval", default_value="2.0"),
        DeclareLaunchArgument("battery_scan_before_connect", default_value="false"),
        DeclareLaunchArgument("advertise_name", default_value="SiriusBleBridge"),
        DeclareLaunchArgument("nav_http_target", default_value="http://localhost:50060/instruction"),
        DeclareLaunchArgument("face_speak_grpc_target", default_value="localhost:50052"),
        DeclareLaunchArgument("face_status_grpc_target", default_value="localhost:50051"),
        DeclareLaunchArgument("publish_face_battery_params", default_value="true"),
        Node(
            package="sirius_navigation",
            executable="sirius_ble_gateway",
            name="sirius_ble_gateway",
            output="screen",
            parameters=[{
                "enable_remote_server": LaunchConfiguration("enable_remote_server"),
                "enable_battery_client": LaunchConfiguration("enable_battery_client"),
                "battery_mac": LaunchConfiguration("battery_mac"),
                "battery_poll_interval": LaunchConfiguration("battery_poll_interval"),
                "battery_scan_before_connect": LaunchConfiguration("battery_scan_before_connect"),
                "advertise_name": LaunchConfiguration("advertise_name"),
                "nav_http_target": LaunchConfiguration("nav_http_target"),
                "face_speak_grpc_target": LaunchConfiguration("face_speak_grpc_target"),
                "face_status_grpc_target": LaunchConfiguration("face_status_grpc_target"),
                "publish_face_battery_params": LaunchConfiguration("publish_face_battery_params"),
            }],
        ),
    ])
