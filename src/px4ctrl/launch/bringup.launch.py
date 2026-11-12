"""Start the EGO-to-px4ctrl pipeline after PX4/Gazebo and FAST-LIO are running."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    px4ctrl_share = get_package_share_directory('px4ctrl')
    ego_share = get_package_share_directory('ego_planner')
    bridge_share = get_package_share_directory('fastlio2_px4_odom_bridge')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_agent = LaunchConfiguration('start_agent')
    start_odom_bridge = LaunchConfiguration('start_odom_bridge')
    start_rviz = LaunchConfiguration('start_rviz')
    ego_max_vel = LaunchConfiguration('ego_max_vel')
    ego_max_acc = LaunchConfiguration('ego_max_acc')
    ego_max_jerk = LaunchConfiguration('ego_max_jerk')
    ego_lambda_smooth = LaunchConfiguration('ego_lambda_smooth')
    ego_planning_horizon = LaunchConfiguration('ego_planning_horizon')
    ego_obstacle_inflation = LaunchConfiguration('ego_obstacle_inflation')
    ego_collision_clearance = LaunchConfiguration('ego_collision_clearance')

    ego = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ego_share, 'launch', 'single_uav_gazebo.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'planning_frame': 'lidar',
            'odom_topic': '/fastlio2/lio_odom',
            'cloud_topic': '/fastlio2/world_cloud',
            'max_vel': ego_max_vel,
            'max_acc': ego_max_acc,
            'max_jerk': ego_max_jerk,
            'lambda_smooth': ego_lambda_smooth,
            'planning_horizon': ego_planning_horizon,
            'obstacle_inflation': ego_obstacle_inflation,
            'collision_clearance': ego_collision_clearance,
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ego_share, 'launch', 'rviz.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_agent', default_value='true',
                              description='Set false if MicroXRCEAgent is already running'),
        DeclareLaunchArgument('agent_port', default_value='8888'),
        DeclareLaunchArgument('start_odom_bridge', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('ego_max_vel', default_value='1.0'),
        DeclareLaunchArgument('ego_max_acc', default_value='1.0'),
        DeclareLaunchArgument('ego_max_jerk', default_value='3.0'),
        DeclareLaunchArgument('ego_lambda_smooth', default_value='2.0'),
        DeclareLaunchArgument('ego_planning_horizon', default_value='7.5'),
        DeclareLaunchArgument('ego_obstacle_inflation', default_value='0.45'),
        DeclareLaunchArgument('ego_collision_clearance', default_value='0.65'),
        DeclareLaunchArgument(
            'px4ctrl_params',
            default_value=os.path.join(px4ctrl_share, 'config', 'px4ctrl_bringup.yaml')),
        DeclareLaunchArgument(
            'odom_bridge_params',
            default_value=os.path.join(bridge_share, 'config', 'fastlio2_px4_odom_bridge.yaml')),
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', LaunchConfiguration('agent_port')],
            name='micro_xrce_agent', output='screen', condition=IfCondition(start_agent)),
        ego,
        rviz,
        Node(
            package='fastlio2_px4_odom_bridge',
            executable='fastlio2_px4_odom_bridge_node',
            name='fastlio2_px4_odom_bridge', output='screen',
            parameters=[LaunchConfiguration('odom_bridge_params')],
            condition=IfCondition(start_odom_bridge)),
        Node(
            package='px4ctrl', executable='ego_position_command_adapter',
            name='ego_position_command_adapter', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'command_topic': '/drone_0_planning/pos_cmd',
                'odom_topic': '/fastlio2/lio_odom',
                'px4_local_position_topic': '/fmu/out/vehicle_local_position_v1',
                'desired_state_topic': '/px4ctrl/desired_state',
                'source_frame': 'lidar',
                'target_frame': 'px4_local_ned',
            }]),
        Node(
            package='px4ctrl', executable='px4ctrl_node', name='px4ctrl',
            output='screen', parameters=[LaunchConfiguration('px4ctrl_params')]),
    ])
