# FAST-LIO2 to PX4 EKF bridge

This package converts `/fastlio2/lio_odom` (`nav_msgs/msg/Odometry`) into
`/fmu/in/vehicle_visual_odometry` (`px4_msgs/msg/VehicleOdometry`). It only
publishes an external-odometry input. It does not publish Offboard setpoints
or change PX4 EKF parameters.

Build and run after sourcing the `px4_msgs` overlay:

```bash
cd /home/thunder/GRA_1/plan_ws
colcon build --symlink-install --packages-select fastlio2_px4_odom_bridge
source install/setup.bash
ros2 launch fastlio2_px4_odom_bridge fastlio2_px4_odom_bridge.launch.py
```

The existing PX4 `bringup.launch.py` starts this node by default alongside
`px4_bridge`. Disable it with `start_odom_bridge:=false`, or override its
configuration with `odom_bridge_params:=/path/to/file.yaml`.

PX4 parameter changes are separate from starting this ROS node. The repository's
Gazebo SITL launch injects its own parameter defaults when PX4 starts
(`EKF2_EV_CTRL=7`, `EKF2_GPS_CTRL=0`, `EKF2_HGT_REF=3`); inspect
`mid360_posix_sitl.launch.py` before changing them. Do not also persist a
conflicting value with `param save`. For a separate PX4 setup, configure and
verify external-vision fusion for that firmware version through QGroundControl
or the PX4 shell. For example, a position-only setup may use:

```text
param show EKF2_EV_CTRL
param show EKF2_EV_DELAY
param show EKF2_EV_NOISE_MD
```

The control bitmask meanings depend on PX4 version. In the PX4 revision paired
with this workspace, the SITL launch uses `7` for horizontal/vertical position
and velocity fusion, while `15` additionally enables yaw. Verify the parameter
metadata and estimator status for the exact firmware before enabling each
measurement. The `param show` commands above are inspection commands; they do
not change or save parameters.

Important assumptions and follow-up work:

* The default world rotation matches the existing planner bridge:
  `PX4_NED = [lidar_y, lidar_x, -lidar_z]`. Verify this with a small-axis test
  before enabling EKF fusion.
* In this Gazebo setup the MID-360 plugin rotates its IMU output back from the
  sensor's pi-yaw mounting so `/livox/imu` and FAST-LIO `body` axes align with
  X500 `base_link` (FLU: forward, left, up). PX4 uses FRD (forward, right,
  down), so `body_to_frd_row_major` is `diag(1,-1,-1)`, a 180 degree rotation
  about X. Real hardware must use its measured IMU/body-to-FRD rotation.
* The node rejects an odometry message unless `header.frame_id` is `lidar`.
  It does not use TF to silently transform data.
* The bridge uses `/fmu/out/vehicle_local_position_v1` to estimate the constant
  PX4/ROS clock offset. `VehicleOdometry.timestamp` is the current PX4 time and
  `timestamp_sample` preserves the mapped FAST-LIO sensor sample time. Keep the
  PX4 time source alive and verify the resulting delay and EV innovations for
  the target firmware.
* `transform_translation` is zero by default. This assumes the FAST-LIO local
  origin and PX4 local NED origin have been aligned. A nonzero calibrated
  translation is required when they are not aligned.
* Configure PX4 EKF2 external-vision fusion separately (`EKF2_EV_CTRL`, delay,
  sensor position, and height reference as appropriate for the PX4 version).
  Check `EstimatorStatus` flags `CS_EV_POS`, `CS_EV_VEL`, `CS_EV_HGT`, and
  `CS_EV_YAW`, plus EV innovations, to prove that fusion is active.
* Do not run this node and another visual-odometry publisher simultaneously.
  Duplicate publishers can make the EKF input intermittent.
