"""Start EGO, its command adapter, odometry forwarding and px4ctrl.

PX4/Gazebo and FAST-LIO/PGO are intentionally external processes.  This launch
file only starts nodes that belong to this workspace, so it can be used after
the vehicle simulation and localization have been brought up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    px4ctrl_share = get_package_share_directory("px4ctrl")
    ego_share = get_package_share_directory("ego_planner")
    odom_bridge_share = get_package_share_directory("fastlio2_px4_odom_bridge")

    px4ctrl_params = LaunchConfiguration("px4ctrl_params")
    odom_bridge_params = LaunchConfiguration("odom_bridge_params")
    start_odom_bridge = LaunchConfiguration("start_odom_bridge")
    start_rviz = LaunchConfiguration("start_rviz")
    start_agent = LaunchConfiguration("start_agent")
    agent_port = LaunchConfiguration("agent_port")
    adapter_input_topic = LaunchConfiguration("adapter_input_topic")
    adapter_output_topic = LaunchConfiguration("adapter_output_topic")
    adapter_planner_frame = LaunchConfiguration("adapter_planner_frame")

    # single_uav_gazebo.launch.py contains only the EGO planner, trajectory
    # server and the two MID-360 frame transforms. It does not start Gazebo.
    ego_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ego_share, "launch", "single_uav_gazebo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "planning_frame": "lidar",
            "odom_topic": "/fastlio2/lio_odom",
            "cloud_topic": "/fastlio2/world_cloud",
            "depth_topic": "/depth_camera_bestef",
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ego_share, "launch", "rviz.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "px4ctrl_params",
            default_value=os.path.join(px4ctrl_share, "config", "px4ctrl_sitl.yaml"),
        ),
        DeclareLaunchArgument(
            "odom_bridge_params",
            default_value=os.path.join(
                odom_bridge_share, "config", "fastlio2_px4_odom_bridge.yaml"
            ),
        ),
        DeclareLaunchArgument("start_odom_bridge", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument(
            "start_agent",
            default_value="true",
            description="Start the Micro XRCE-DDS Agent required by PX4 ROS 2 topics",
        ),
        DeclareLaunchArgument("agent_port", default_value="8888"),
        DeclareLaunchArgument(
            "adapter_input_topic", default_value="/drone_0_planning/pos_cmd"),
        DeclareLaunchArgument(
            "adapter_output_topic", default_value="/px4ctrl/desired_state"),
        DeclareLaunchArgument("adapter_planner_frame", default_value="lidar"),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", agent_port],
            name="micro_xrce_agent",
            output="screen",
            condition=IfCondition(start_agent),
        ),
        ego_launch,
        rviz_launch,
        Node(
            package="fastlio2_px4_odom_bridge",
            executable="fastlio2_px4_odom_bridge_node",
            name="fastlio2_px4_odom_bridge",
            output="screen",
            parameters=[odom_bridge_params],
            condition=IfCondition(start_odom_bridge),
        ),
        Node(
            package="px4ctrl",
            executable="px4ctrl_node",
            name="px4ctrl",
            output="screen",
            parameters=[px4ctrl_params],
        ),
        Node(
            package="px4ctrl",
            executable="ego_position_command_adapter",
            name="ego_position_command_adapter",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "input_topic": adapter_input_topic,
                "output_topic": adapter_output_topic,
                "planner_frame": adapter_planner_frame,
            }],
        ),
    ])
