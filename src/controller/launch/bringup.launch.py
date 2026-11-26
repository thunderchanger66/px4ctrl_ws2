"""Bring up PX4Ctrl with the existing FAST-LIO2 and EGO pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    controller_share = get_package_share_directory('controller')
    ego_share = get_package_share_directory('ego_planner')
    bridge_share = get_package_share_directory('fastlio2_px4_odom_bridge')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_agent = LaunchConfiguration('start_agent')
    start_odom_bridge = LaunchConfiguration('start_odom_bridge')
    start_rviz = LaunchConfiguration('start_rviz')
    start_takeoff = LaunchConfiguration('start_takeoff')
    adapter_max_reference_acceleration = LaunchConfiguration('adapter_max_reference_acceleration')
    adapter_max_reference_jerk = LaunchConfiguration('adapter_max_reference_jerk')
    adapter_acceleration_preview = LaunchConfiguration('adapter_acceleration_preview')
    adapter_alignment_stationary_speed = LaunchConfiguration('adapter_alignment_stationary_speed')
    agent_port = LaunchConfiguration('agent_port')
    takeoff_altitude = LaunchConfiguration('takeoff_altitude')
    takeoff_timeout = LaunchConfiguration('takeoff_timeout')
    wait_for_stable_px4 = LaunchConfiguration('wait_for_stable_px4')
    controller_params = LaunchConfiguration('controller_params')
    odom_bridge_params = LaunchConfiguration('odom_bridge_params')

    ego = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ego_share, 'launch', 'single_uav_gazebo.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'planning_frame': 'lidar',
            'odom_topic': '/fastlio2/lio_odom',
            'cloud_topic': '/fastlio2/world_cloud',
            'wait_for_stable_px4': wait_for_stable_px4,
            'max_vel': LaunchConfiguration('ego_max_vel'),
            'max_acc': LaunchConfiguration('ego_max_acc'),
            'max_jerk': LaunchConfiguration('ego_max_jerk'),
            'lambda_smooth': LaunchConfiguration('ego_lambda_smooth'),
            'planning_horizon': LaunchConfiguration('ego_planning_horizon'),
            'obstacle_inflation': LaunchConfiguration('ego_obstacle_inflation'),
            'collision_clearance': LaunchConfiguration('ego_collision_clearance'),
            'min_flight_height': LaunchConfiguration('ego_min_flight_height'),
            'goal_tolerance': LaunchConfiguration('ego_goal_tolerance'),
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
        DeclareLaunchArgument('start_takeoff', default_value='true'),
        DeclareLaunchArgument('adapter_max_reference_acceleration', default_value='1.8'),
        DeclareLaunchArgument('adapter_max_reference_jerk', default_value='5.0'),
        DeclareLaunchArgument('adapter_acceleration_preview', default_value='0.15'),
        DeclareLaunchArgument('adapter_alignment_stationary_speed', default_value='0.20'),
        DeclareLaunchArgument('takeoff_altitude', default_value='0.5'),
        DeclareLaunchArgument('takeoff_timeout', default_value='45'),
        DeclareLaunchArgument('wait_for_stable_px4', default_value='true'),
        DeclareLaunchArgument('controller_params',
                              default_value=os.path.join(
                                  controller_share, 'config', 'px4ctrl_gra_mid360.yaml')),
        DeclareLaunchArgument('odom_bridge_params',
                              default_value=os.path.join(
                                  bridge_share, 'config', 'fastlio2_px4_odom_bridge.yaml')),
        DeclareLaunchArgument('ego_max_vel', default_value='2.0'),
        DeclareLaunchArgument('ego_max_acc', default_value='1.5'),
        DeclareLaunchArgument('ego_max_jerk', default_value='4.0'),
        DeclareLaunchArgument('ego_lambda_smooth', default_value='2.0'),
        DeclareLaunchArgument('ego_planning_horizon', default_value='7.5'),
        DeclareLaunchArgument('ego_obstacle_inflation', default_value='0.45'),
        DeclareLaunchArgument('ego_collision_clearance', default_value='0.65'),
        DeclareLaunchArgument('ego_min_flight_height', default_value='0.25'),
        DeclareLaunchArgument('ego_goal_tolerance', default_value='0.05'),
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', agent_port],
            name='micro_xrce_agent', output='screen', condition=IfCondition(start_agent)),
        ego,
        rviz,
        Node(
            package='fastlio2_px4_odom_bridge',
            executable='fastlio2_px4_odom_bridge_node',
            name='fastlio2_px4_odom_bridge', output='screen',
            parameters=[odom_bridge_params, {'use_sim_time': use_sim_time}],
            condition=IfCondition(start_odom_bridge)),
        Node(
            package='controller', executable='ego_position_command_adapter',
            name='ego_position_command_adapter', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'command_topic': '/drone_0_planning/pos_cmd',
                'odom_topic': '/fastlio2/lio_odom',
                'px4_local_position_topic': '/fmu/out/vehicle_local_position_v1',
                'desired_state_topic': '/controller/desired_state',
                'source_frame': 'lidar',
                'target_frame': 'px4_local_ned',
                'max_reference_acceleration_mps2': adapter_max_reference_acceleration,
                'max_reference_jerk_mps3': adapter_max_reference_jerk,
                'acceleration_preview_sec': adapter_acceleration_preview,
                'alignment_stationary_speed_mps': adapter_alignment_stationary_speed,
            }]),
        Node(
            package='controller', executable='px4ctrl_node', name='px4ctrl',
            output='screen', parameters=[controller_params, {'use_sim_time': use_sim_time}]),
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller', 'takeoff',
                 '--altitude', takeoff_altitude,
                 '--wait-timeout', takeoff_timeout,
                 '--use-sim-time', use_sim_time,
                 '--require-ev-fusion'],
            name='controller_takeoff', output='screen',
            condition=IfCondition(start_takeoff)),
    ])
