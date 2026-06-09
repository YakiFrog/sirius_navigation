from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_remote_server", default_value="true"),
        DeclareLaunchArgument("enable_battery_client", default_value="false"),
        DeclareLaunchArgument("enable_ear_led_client", default_value="true"),
        DeclareLaunchArgument("battery_mac", default_value=""),
        DeclareLaunchArgument("ear_led_left_mac", default_value="7C:2C:67:64:BD:3A"),
        DeclareLaunchArgument("ear_led_right_mac", default_value="7C:2C:67:64:A6:46"),
        DeclareLaunchArgument("ear_led_char_uuid", default_value="beb5483e-36e1-4688-b7f5-ea07361b26a8"),
        DeclareLaunchArgument("ear_led_period", default_value="0.5"),
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
                "enable_ear_led_client": LaunchConfiguration("enable_ear_led_client"),
                "battery_mac": LaunchConfiguration("battery_mac"),
                "ear_led_left_mac": LaunchConfiguration("ear_led_left_mac"),
                "ear_led_right_mac": LaunchConfiguration("ear_led_right_mac"),
                "ear_led_char_uuid": LaunchConfiguration("ear_led_char_uuid"),
                "ear_led_period": LaunchConfiguration("ear_led_period"),
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
