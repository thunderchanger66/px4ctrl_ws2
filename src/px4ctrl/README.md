# px4ctrl (ROS 2)

For integration into another ROS 2 project, see the Chinese
[migration and interface guide](MIGRATION_GUIDE.md).

`px4ctrl` closes the position and velocity loops outside PX4. It receives a
planner reference, uses PX4 `VehicleLocalPosition` as feedback, and publishes
`VehicleAttitudeSetpoint` (`q_d`, body-Z thrust and yaw-rate feedforward). PX4
continues to run its attitude and angular-rate loops. This controller does not
publish a three-axis rate setpoint; that would require `VehicleRatesSetpoint`
and bypass the PX4 attitude loop.

## Bare X500: automatic eight trajectory

Start the bare Gazebo X500 (not the gimbal/MID-360 model) and run
`MicroXRCEAgent udp4 -p 8888` in a separate terminal. The two `eight` launches
then perform position-Offboard takeoff, attitude handoff and trajectory publication separately.
The first launch keeps `px4ctrl` running until the vehicle has landed. QGC is
not required. See [eight/README.md](../eight/README.md) for the full sequence.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/zone/px4ctrl_ws-main
colcon build --packages-up-to eight --symlink-install --executor sequential
source install/setup.bash
ros2 launch eight takeoff.launch.py
# After this prints "Offboard ready", keep it running and use a new terminal:
ros2 launch eight eight.launch.py output_dir:=/tmp/eight_runs
```

The `eight` package publishes a two-lap trajectory and writes timestamped CSV
and JSON results to the given directory. The controller keeps its captured
position during the short Offboard handoff. No separate hover-point setup is
needed. Do not launch a second DDS agent or controller alongside these commands.
Confirm `/clock` advances with `ros2 topic echo /clock --once`. If another
Gazebo-to-ROS clock bridge already runs, use `start_clock_bridge:=false` to
avoid duplicate clock publishers. Pausing Gazebo pauses the control timer and
the eight trajectory together.

The bare-X500 config `config/px4ctrl_x500.yaml` uses PX4 local NED as both
reference and feedback (`planner_frame: px4_local_ned`, identity rotation,
no FAST-LIO origin alignment). Its initial `hover_thrust: 0.60` is based on
the PX4 bare-X500 airframe default and must be checked against actual stable
hover thrust in this simulation. A wrong value can cause vertical drift or
saturation during takeover. The existing `px4ctrl_sitl.yaml` has `0.864`
calibrated for an X500 **with gimbal and MID-360**; do not reuse it for the
bare model. Verify local position validity, PX4 status and thrust calibration
before treating an Offboard mode transition as a successful control test.
Airborne capture also requires a fresh PX4 `VehicleLandDetected` message with
`landed`, `maybe_landed`, and `ground_contact` all false, plus at least one
second of low-speed, valid local position. An armed vehicle still on the ground
will stay in `waiting_for_armed_stationary_airborne_state` and will not prestream
a ground-level hold target.

PX4's own failsafes remain active. For a SITL setup without RC stick input,
PX4 may trigger manual-control-loss failsafe in Offboard; check PX4 events
if `/px4ctrl/status` becomes `control_suspended_px4_failsafe`. For this
no-RC simulation only, PX4's `COM_RCL_EXCEPT` bit 2 (value `4`) exempts
Offboard from manual-control-loss failsafe. Do not change this parameter on a
vehicle with an expected RC safety link without reviewing that safety policy.

## Planner interface

Publish `px4ctrl/msg/DesiredState` on `/px4ctrl/desired_state`:

| Field | Convention |
|---|---|
| `header.stamp` | ROS time of this reference sample; `/clock` in SITL |
| `header.frame_id` | Must match `planner_frame` |
| `position`, `velocity`, `acceleration` | Meters, m/s, m/s^2 in the declared world frame |
| `yaw`, `yaw_rate` | Radians and rad/s about the world vertical axis |

The controller converts planner references to PX4 local NED using the
configured proper rotation and optional origin translation. With X500's
`px4_local_ned` frame, the values are already NED and no FAST-LIO is needed.
In airborne takeover, a valid, armed, stationary PX4 state is needed to
capture a hold point before the Offboard switch. Commands are accepted only
while fresh and with valid coordinates; stale commands return to hover.

The nominal control law is cascaded position P, velocity PI and acceleration
feedforward. It limits the requested acceleration, tilt and normalized thrust,
and prevents integral buildup during saturation. `hover_thrust`, gains and
limits are **airframe-specific**. `config/px4ctrl_x500.yaml` is a SITL starting
point rather than a hardware flight configuration.

`/px4ctrl/status` retains human-readable state strings. `/px4ctrl/debug`
publishes `ControlDebug` with local-NED errors, velocity reference, requested
and feasible accelerations, thrust, tilt, yaw rate, command age and PX4 health.
`saturation_flags` uses bit 0 for total acceleration limiting, bit 1 for the
vertical force envelope, bit 2 for horizontal tilt/force limiting, and bit 3
for final thrust clipping. A nonzero value means the command was constrained.

## Existing EGO workflow

The original `bringup.launch.py` still starts EGO, `traj_server`, the
FAST-LIO-to-PX4 odometry bridge, `px4ctrl`, and (by default) MicroXRCEAgent.
It now also starts `ego_position_command_adapter`, which converts EGO's
`/drone_0_planning/pos_cmd` (`PositionCommand`) to `DesiredState`. This adapter
requires the optional `quadrotor_msgs`, `ego_planner`, and
`fastlio2_px4_odom_bridge` packages; the controller-only path does not.
The existing `manual_goal` CLI and the X500 with gimbal/MID-360 SITL config
retain their original interface and automatic takeoff behavior:

```bash
ros2 launch px4ctrl bringup.launch.py start_rviz:=false
ros2 run px4ctrl manual_goal 4.0 -4.0 0.5 90 --wait-timeout 30
```

PX4/Gazebo and FAST-LIO/PGO remain external to this launch. Use
`start_agent:=false` if a DDS agent is already active. When loading a custom
`px4ctrl_params` file, set `adapter_input_topic`, `adapter_output_topic` and
`adapter_planner_frame` launch arguments to match its desired-state topic and
planner frame; the defaults preserve the existing EGO wiring.
