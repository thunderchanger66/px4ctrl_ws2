"""Plan in FAST-LIO's continuous local frame using Gazebo simulation time."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {
        'use_sim_time': 'true',
        'planning_frame': 'lidar',
        'drone_id': '0',
        'map_size_x': '80.0',
        'map_size_y': '80.0',
        'map_size_z': '4.0',
        'odom_topic': '/fastlio2/lio_odom',
        'cloud_topic': '/fastlio2/world_cloud',
        'depth_topic': '/depth_camera_bestef',
    }
    cfg = {name: LaunchConfiguration(name) for name in args}
    advanced = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'use_sim_time': cfg['use_sim_time'],
            'planning_frame': cfg['planning_frame'],
            'drone_id': cfg['drone_id'],
            'map_size_x_': cfg['map_size_x'],
            'map_size_y_': cfg['map_size_y'],
            'map_size_z_': cfg['map_size_z'],
            'odometry_topic': cfg['odom_topic'],
            'cloud_topic': cfg['cloud_topic'],
            'depth_topic': cfg['depth_topic'],
            # Keep enough stopping margin for the PX4 SITL position loop.
            'max_vel': '0.6',
            'max_acc': '0.8',
            'planning_horizon': '7.5',
            'obstacle_inflation': '0.45',
            'collision_clearance': '0.65',
            'use_distinctive_trajs': 'True',
            'flight_type': '1',
        }.items(),
    )
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in args.items()],
        advanced,
        # FAST-LIO publishes lidar -> body. The simulated IMU is the body frame,
        # mounted 0.08 m above base_link and rotated pi about its z axis.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='mid360_body_to_base_link',
            arguments=['--x', '0', '--y', '0', '--z', '-0.08',
                       '--yaw', '3.141592653589793', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'body', '--child-frame-id', 'base_link'],
            parameters=[{'use_sim_time': cfg['use_sim_time']}],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='mid360_body_to_livox_frame',
            arguments=['--frame-id', 'body', '--child-frame-id', 'livox_frame'],
            parameters=[{'use_sim_time': cfg['use_sim_time']}],
        ),
        Node(
            package='ego_planner', executable='traj_server',
            name=['drone_', cfg['drone_id'], '_traj_server'], output='screen',
            remappings=[
                ('position_cmd', ['drone_', cfg['drone_id'], '_planning/pos_cmd']),
                ('planning/bspline', ['drone_', cfg['drone_id'], '_planning/bspline']),
            ],
            parameters=[{'traj_server/time_forward': 1.0,
                         # The MID-360/IMU axes are pi radians from base_link.
                         'traj_server/initial_yaw': 3.141592653589793,
                         'use_sim_time': cfg['use_sim_time'],
                         'planning_frame': cfg['planning_frame']}],
        ),
    ])
