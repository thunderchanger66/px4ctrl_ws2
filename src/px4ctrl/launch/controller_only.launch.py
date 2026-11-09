"""Start only px4ctrl for a QGC-flown vehicle already airborne in PX4 SITL."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("px4ctrl")
    params = LaunchConfiguration("px4ctrl_params")
    start_agent = LaunchConfiguration("start_agent")
    start_clock_bridge = LaunchConfiguration("start_clock_bridge")
    agent_port = LaunchConfiguration("agent_port")
    return LaunchDescription([
        DeclareLaunchArgument(
            "px4ctrl_params",
            default_value=os.path.join(share, "config", "px4ctrl_x500.yaml"),
        ),
        DeclareLaunchArgument("start_agent", default_value="false"),
        DeclareLaunchArgument("start_clock_bridge", default_value="true"),
        DeclareLaunchArgument("agent_port", default_value="8888"),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", agent_port],
            name="micro_xrce_agent", output="screen",
            condition=IfCondition(start_agent),
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="gz_clock_bridge", output="screen",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            condition=IfCondition(start_clock_bridge),
        ),
        Node(
            package="px4ctrl", executable="px4ctrl_node", name="px4ctrl",
            output="screen", parameters=[params],
        ),
    ])
