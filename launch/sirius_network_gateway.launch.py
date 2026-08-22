from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("listen_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("listen_port", default_value="8766"),
        DeclareLaunchArgument("pairing_ttl", default_value="300.0"),
        DeclareLaunchArgument("deadman_timeout", default_value="0.45"),
        Node(
            package="sirius_navigation",
            executable="sirius_network_gateway",
            name="sirius_network_gateway",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "listen_host": LaunchConfiguration("listen_host"),
                "listen_port": ParameterValue(
                    LaunchConfiguration("listen_port"),
                    value_type=int,
                ),
                "pairing_ttl": ParameterValue(
                    LaunchConfiguration("pairing_ttl"),
                    value_type=float,
                ),
                "deadman_timeout": ParameterValue(
                    LaunchConfiguration("deadman_timeout"),
                    value_type=float,
                ),
            }],
        ),
    ])
