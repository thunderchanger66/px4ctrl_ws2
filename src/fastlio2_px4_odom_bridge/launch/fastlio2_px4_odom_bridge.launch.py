import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("fastlio2_px4_odom_bridge")
    default_params = os.path.join(share, "config", "fastlio2_px4_odom_bridge.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        Node(
            package="fastlio2_px4_odom_bridge",
            executable="fastlio2_px4_odom_bridge_node",
            name="fastlio2_px4_odom_bridge",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
