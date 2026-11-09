"""Take off the bare X500 and keep px4ctrl streaming in Offboard."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    px4ctrl_share = get_package_share_directory("px4ctrl")
    return LaunchDescription([
        DeclareLaunchArgument("start_clock_bridge", default_value="true"),
        DeclareLaunchArgument("start_sitl_gcs", default_value="true"),
        DeclareLaunchArgument("px4ctrl_params", default_value=os.path.join(
            px4ctrl_share, "config", "px4ctrl_x500.yaml")),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="gz_clock_bridge", output="screen",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            condition=IfCondition(LaunchConfiguration("start_clock_bridge")),
        ),
        Node(
            package="eight", executable="sitl_gcs", name="eight_sitl_gcs",
            output="screen", condition=IfCondition(LaunchConfiguration("start_sitl_gcs")),
        ),
        Node(
            package="px4ctrl", executable="px4ctrl_node", name="px4ctrl",
            output="screen", parameters=[LaunchConfiguration("px4ctrl_params")],
        ),
        Node(
            package="eight", executable="takeoff_node", name="eight_takeoff",
            output="screen", parameters=[{"use_sim_time": True}],
        ),
    ])
