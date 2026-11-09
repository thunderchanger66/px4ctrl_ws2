"""Publish and assess a figure eight after the takeoff launch is ready."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="/tmp/eight_runs"),
        DeclareLaunchArgument("period_sec", default_value="8.9"),
        Node(
            package="eight",
            executable="eight_node",
            name="eight",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "output_dir": LaunchConfiguration("output_dir"),
                "period_sec": LaunchConfiguration("period_sec"),
            }],
        ),
    ])
