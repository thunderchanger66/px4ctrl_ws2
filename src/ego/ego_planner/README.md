# EGO Planner

`ego_planner` consumes odometry and the local occupancy map, plans a collision-aware B-spline, and publishes `quadrotor_msgs/PositionCommand` through `traj_server`.

## Current PX4Ctrl SITL path

`px4ctrl/bringup.launch.py` includes `single_uav_gazebo.launch.py`, which starts `ego_planner_node`, `traj_server`, and the MID-360 frame transforms. PX4/Gazebo and FAST-LIO/PGO remain separate processes.

The active interfaces are:

- Odometry: `/fastlio2/lio_odom`, frame `lidar`
- Obstacle cloud: `/fastlio2/world_cloud`, frame `lidar`
- Goal: `/goal_pose`, frame `lidar`
- Trajectory: `/drone_0_planning/bspline`
- Position command: `/drone_0_planning/pos_cmd`
- Inflated local map: `/drone_0_grid/grid_map/occupancy_inflate`

The PX4Ctrl bringup adapts position commands from `lidar` into PX4 local NED. EGO itself remains in the FAST-LIO local frame.

## Launches and optional capabilities

`advanced_param.launch.py` starts the planner node and exposes the map, dynamic limits, collision clearance, and optimizer parameters. `single_uav_gazebo.launch.py` supplies the current single-vehicle SITL defaults. `rviz.launch.py` loads `single_drone.rviz`.

The EGO message types and planner buffers retain swarm-trajectory interfaces. Dynamic-object prediction and `obj_generator` are optional interfaces and are not started by the current LiDAR-only single-UAV bringup. Keep them when those modes are needed.
