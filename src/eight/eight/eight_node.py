"""Publish an X500 figure eight and assess PX4 local-NED tracking."""

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck, VehicleLandDetected,
    VehicleLocalPosition, VehicleStatus,
)
from px4ctrl.msg import ControlDebug, DesiredState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .evaluation import summarize
from .trajectory import FigureEight


def _position_valid(msg: VehicleLocalPosition | None) -> bool:
    if msg is None or msg.timestamp == 0 or msg.dead_reckoning:
        return False
    if not (msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid):
        return False
    return all(math.isfinite(value) for value in (
        msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.heading,
    ))


class EightNode(Node):
    def __init__(self, phase: str = "trajectory") -> None:
        if phase not in ("takeoff", "trajectory"):
            raise ValueError("phase must be takeoff or trajectory")
        super().__init__("eight_takeoff" if phase == "takeoff" else "eight")
        self.phase = phase
        self.declare_parameter("command_topic", "/px4ctrl/desired_state")
        self.declare_parameter("local_position_topic", "/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v4")
        self.declare_parameter("land_detected_topic", "/fmu/out/vehicle_land_detected")
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
        self.declare_parameter("vehicle_command_ack_topic", "/fmu/out/vehicle_command_ack_v1")
        self.declare_parameter("offboard_mode_topic", "/fmu/in/offboard_control_mode")
        self.declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("takeover_permission_topic", "/eight/takeover_allowed")
        self.declare_parameter("controller_status_topic", "/px4ctrl/status")
        self.declare_parameter("control_debug_topic", "/px4ctrl/debug")
        self.declare_parameter("output_dir", "/tmp/eight_runs")
        self.declare_parameter("amplitude_x_m", 2.0)
        self.declare_parameter("amplitude_y_m", 1.0)
        self.declare_parameter("period_sec", 8.9)
        self.declare_parameter("cycles", 2)
        self.declare_parameter("ramp_sec", 3.0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("settle_timeout_sec", 10.0)
        self.declare_parameter("startup_timeout_sec", 45.0)
        self.declare_parameter("takeoff_timeout_sec", 30.0)
        self.declare_parameter("takeoff_altitude_m", 2.0)
        self.declare_parameter("land_detected_timeout_sec", 2.0)
        self.declare_parameter("takeover_timeout_sec", 20.0)
        self.declare_parameter("max_tracking_error_m", 1.5)

        self.amplitude_x = self.get_parameter("amplitude_x_m").value
        self.amplitude_y = self.get_parameter("amplitude_y_m").value
        self.period = self.get_parameter("period_sec").value
        self.cycles = self.get_parameter("cycles").value
        self.ramp = self.get_parameter("ramp_sec").value
        self.rate = self.get_parameter("publish_rate_hz").value
        self.settle_timeout = self.get_parameter("settle_timeout_sec").value
        self.output_dir = Path(self.get_parameter("output_dir").value).expanduser()
        self.startup_timeout = float(self.get_parameter("startup_timeout_sec").value)
        self.takeoff_timeout = float(self.get_parameter("takeoff_timeout_sec").value)
        self.takeoff_altitude = float(self.get_parameter("takeoff_altitude_m").value)
        self.land_detected_timeout = float(self.get_parameter("land_detected_timeout_sec").value)
        self.takeover_timeout = float(self.get_parameter("takeover_timeout_sec").value)
        self.max_tracking_error = float(self.get_parameter("max_tracking_error_m").value)
        if not math.isfinite(self.rate) or self.rate < 10 or self.rate > 200:
            raise ValueError("publish_rate_hz must be in [10, 200]")
        if not math.isfinite(self.settle_timeout) or self.settle_timeout <= 2:
            raise ValueError("settle_timeout_sec must be greater than 2")
        if not all(math.isfinite(v) and v > 0 for v in (
            self.startup_timeout, self.takeoff_timeout, self.takeover_timeout,
            self.takeoff_altitude,
            self.land_detected_timeout,
            self.max_tracking_error,
        )):
            raise ValueError("flight timeouts and maximum tracking error must be positive")
        FigureEight((0.0, 0.0, 0.0), self.amplitude_x, self.amplitude_y,
                    self.period, self.cycles, self.ramp)

        self.vehicle_status: VehicleStatus | None = None
        self.land_detected: VehicleLandDetected | None = None
        self.land_detected_time_ns: int | None = None
        self.vehicle_status_time_ns: int | None = None
        self.local_position: VehicleLocalPosition | None = None
        self.local_position_time_ns: int | None = None
        self.control_debug: ControlDebug | None = None
        self.control_debug_time_ns: int | None = None
        self.controller_status = ""
        self.controller_status_time_ns: int | None = None
        self.trajectory: FigureEight | None = None
        self.start_time_ns: int | None = None
        self.heading = 0.0
        self.rows: list[dict] = []
        self.tracking_rows: list[dict] = []
        self.last_recorded_timestamp: int | None = None
        self.reset_signature: tuple[int, int, int] | None = None
        self.settle_start_ns: int | None = None
        self.offboard_losses = 0
        self.failsafe_events = 0
        self.finalized = False
        self.aborted = False
        self.stage = "waiting_for_px4" if phase == "takeoff" else "waiting_for_controller"
        self.stage_start_ns = 0
        self.process_start_wall = time.monotonic()
        self.last_command_wall = 0.0
        self.last_diagnostic_wall = 0.0
        self.command_denial = ""
        self.large_error_since_ns: int | None = None
        self.owns_flight = False
        self.takeoff_target: tuple[float, float, float] | None = None
        self.takeoff_heading = 0.0
        self.takeoff_stable_since_ns: int | None = None
        self.takeover_permission_sent = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command_pub = self.create_publisher(
            DesiredState, self.get_parameter("command_topic").value, 20)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, self.get_parameter("vehicle_command_topic").value,
            sensor_qos)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, self.get_parameter("offboard_mode_topic").value,
            sensor_qos)
        self.position_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, self.get_parameter("trajectory_setpoint_topic").value,
            sensor_qos)
        self.takeover_permission_pub = self.create_publisher(
            Bool, self.get_parameter("takeover_permission_topic").value, 10)
        self.create_subscription(
            VehicleLocalPosition, self.get_parameter("local_position_topic").value,
            self._local_position_cb, sensor_qos)
        self.create_subscription(
            VehicleStatus, self.get_parameter("vehicle_status_topic").value,
            self._vehicle_status_cb, sensor_qos)
        self.create_subscription(
            VehicleLandDetected, self.get_parameter("land_detected_topic").value,
            self._land_detected_cb, sensor_qos)
        self.create_subscription(
            VehicleCommandAck, self.get_parameter("vehicle_command_ack_topic").value,
            self._command_ack_cb, sensor_qos)
        self.create_subscription(
            String, self.get_parameter("controller_status_topic").value,
            self._controller_status_cb, 10)
        self.create_subscription(
            ControlDebug, self.get_parameter("control_debug_topic").value,
            self._control_debug_cb, 10)
        self.create_timer(1.0 / self.rate, self._tick)
        if phase == "takeoff":
            self.get_logger().info("Waiting for PX4 health; will take off and hand over to px4ctrl Offboard")
        else:
            self.get_logger().info("Waiting for armed PX4 Offboard and px4ctrl before publishing the figure eight")

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    @staticmethod
    def _fresh(last_ns: int | None, now_ns: int, max_age_sec: float = 0.5) -> bool:
        return last_ns is not None and 0 <= now_ns - last_ns <= int(max_age_sec * 1e9)

    def _vehicle_status_cb(self, msg: VehicleStatus) -> None:
        if self.trajectory is not None and not self.aborted:
            if self.vehicle_status is not None:
                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
                        msg.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.offboard_losses += 1
            if msg.failsafe and (self.vehicle_status is None or not self.vehicle_status.failsafe):
                self.failsafe_events += 1
        self.vehicle_status = msg
        self.vehicle_status_time_ns = self._now_ns()

    def _local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.local_position = msg
        self.local_position_time_ns = self._now_ns()

    def _land_detected_cb(self, msg: VehicleLandDetected) -> None:
        self.land_detected = msg
        self.land_detected_time_ns = self._now_ns()

    def _command_ack_cb(self, msg: VehicleCommandAck) -> None:
        expected = {
            "offboard": VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            "arming": VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        }.get(self.stage)
        if msg.command == expected and msg.result in (
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED,
        ):
            self.command_denial = f"PX4 rejected command {msg.command}: result {msg.result}"

    def _controller_status_cb(self, msg: String) -> None:
        self.controller_status = msg.data
        self.controller_status_time_ns = self._now_ns()

    def _control_debug_cb(self, msg: ControlDebug) -> None:
        self.control_debug = msg
        self.control_debug_time_ns = self._now_ns()

    def _ready(self, now_ns: int) -> bool:
        status = self.vehicle_status
        position = self.local_position
        debug = self.control_debug
        if self.command_pub.get_subscription_count() == 0:
            return False
        if not self._fresh(self.vehicle_status_time_ns, now_ns, 1.5):
            return False
        if not all(self._fresh(t, now_ns) for t in (
            self.local_position_time_ns, self.controller_status_time_ns,
            self.control_debug_time_ns,
        )):
            return False
        if status is None or position is None or debug is None or not _position_valid(position):
            return False
        if status.arming_state != VehicleStatus.ARMING_STATE_ARMED or \
                status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or status.failsafe:
            return False
        if not (debug.local_valid and debug.status_fresh and debug.armed and debug.offboard):
            return False
        if self.controller_status != "hover_airborne_takeover_offboard_ready":
            return False
        return True

    def _stage(self, name: str, now_ns: int) -> None:
        self.stage = name
        self.stage_start_ns = now_ns
        self.last_command_wall = 0.0
        self.command_denial = ""
        self.get_logger().info(f"Flight sequence: {name}")

    def _diagnose_takeover(self) -> None:
        now = time.monotonic()
        if now - self.last_diagnostic_wall < 2.0:
            return
        self.last_diagnostic_wall = now
        status, position, land = self.vehicle_status, self.local_position, self.land_detected
        speed = math.sqrt(position.vx**2 + position.vy**2 + position.vz**2) if position else math.nan
        now_ns = self._now_ns()
        self.get_logger().info(
            f"Takeover wait: PX4 nav={status.nav_state if status else 'missing'}, "
            f"armed={status.arming_state if status else 'missing'}, "
            f"failsafe={status.failsafe if status else 'missing'}, "
            f"landed={land.landed if land else 'missing'}, "
            f"maybe_landed={land.maybe_landed if land else 'missing'}, "
            f"ground_contact={land.ground_contact if land else 'missing'}, "
            f"local_valid={_position_valid(position)}, "
            f"z_ned={position.z if position else math.nan:.2f} m, speed={speed:.2f} m/s, "
            f"permission={self.takeover_permission_sent}, "
            f"land_age_s={(now_ns - self.land_detected_time_ns) * 1e-9 if self.land_detected_time_ns else math.nan:.2f}, "
            f"px4ctrl={self.controller_status or 'missing'}, "
            f"controller_status_fresh={self._fresh(self.controller_status_time_ns, now_ns)}")

    def _send_command(self, command: int, param1: float = 0.0,
                      param2: float = 0.0) -> None:
        now = time.monotonic()
        if now - self.last_command_wall < 1.0:
            return
        msg = VehicleCommand()
        msg.timestamp = max(
            self.vehicle_status.timestamp if self.vehicle_status else 0,
            self.local_position.timestamp if self.local_position else 0,
        )
        if not msg.timestamp:
            return
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
        self.last_command_wall = now

    def _preflight_ready(self, now_ns: int) -> bool:
        status, position, land = self.vehicle_status, self.local_position, self.land_detected
        return (
            self._fresh(self.vehicle_status_time_ns, now_ns, 1.5)
            and self._fresh(self.local_position_time_ns, now_ns)
            and self._fresh(self.land_detected_time_ns, now_ns, self.land_detected_timeout)
            and status is not None and position is not None and land is not None
            and status.pre_flight_checks_pass and not status.failsafe
            and status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
            and _position_valid(position) and land.landed
            and self.vehicle_command_pub.get_subscription_count() > 0
            and self.mode_pub.get_subscription_count() > 0
            and self.position_setpoint_pub.get_subscription_count() > 0
        )

    def _publish_takeoff_setpoint(self, now_ns: int) -> None:
        if self.takeoff_target is None:
            return
        if not self._fresh(self.local_position_time_ns, now_ns) or \
                not _position_valid(self.local_position):
            self._abort("PX4 local position became invalid during takeoff")
            return
        timestamp = self.local_position.timestamp
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        self.mode_pub.publish(mode)
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = list(self.takeoff_target)
        setpoint.velocity = [math.nan] * 3
        setpoint.acceleration = [math.nan] * 3
        setpoint.jerk = [math.nan] * 3
        setpoint.yaw = self.takeoff_heading
        setpoint.yawspeed = 0.0
        self.position_setpoint_pub.publish(setpoint)
        permission = Bool()
        permission.data = self.stage == "airborne" and self._at_takeoff_target(now_ns)
        self.takeover_permission_sent = permission.data
        self.takeover_permission_pub.publish(permission)

    def _at_takeoff_target(self, now_ns: int) -> bool:
        position, land = self.local_position, self.land_detected
        if position is None or land is None or \
                not self._fresh(self.land_detected_time_ns, now_ns, self.land_detected_timeout) or \
                land.landed or land.maybe_landed or land.ground_contact:
            self.takeoff_stable_since_ns = None
            return False
        speed = math.sqrt(position.vx**2 + position.vy**2 + position.vz**2)
        if math.dist((position.x, position.y, position.z), self.takeoff_target) > 0.25 or speed > 0.20:
            self.takeoff_stable_since_ns = None
            return False
        if self.takeoff_stable_since_ns is None:
            self.takeoff_stable_since_ns = now_ns
        return now_ns - self.takeoff_stable_since_ns >= 500_000_000

    def _advance_startup(self, now_ns: int) -> None:
        status = self.vehicle_status
        if self.command_denial:
            self._abort(self.command_denial)
            return
        if self.stage != "waiting_for_px4":
            if not self._fresh(self.vehicle_status_time_ns, now_ns, 1.5) or status is None:
                self._abort("PX4 status became stale during startup")
                return
            if status.failsafe:
                self._abort("PX4 entered failsafe during startup")
                return
        if self.stage == "waiting_for_px4":
            if status and status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().error("Refusing to take over an already armed vehicle")
                self._stage("done", now_ns)
            elif self._preflight_ready(now_ns):
                position = self.local_position
                self.takeoff_target = (position.x, position.y, position.z - self.takeoff_altitude)
                self.takeoff_heading = position.heading
                self._stage("prestream", now_ns)
                self.get_logger().info(f"Offboard takeoff target NED {self.takeoff_target} m")
            elif time.monotonic() - self.process_start_wall > self.startup_timeout:
                self._abort("PX4 preflight not ready; check estimator, GCS heartbeat and DDS")
        elif self.stage == "prestream":
            if now_ns - self.stage_start_ns >= 1_500_000_000:
                self._stage("offboard", now_ns)
        elif self.stage == "offboard":
            if status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._stage("arming", now_ns)
            elif (now_ns - self.stage_start_ns) * 1e-9 > self.takeover_timeout:
                self._abort("PX4 did not enter position Offboard after prestream")
            else:
                self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif self.stage == "arming":
            if status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._abort("PX4 left position Offboard before arming")
                return
            if status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.owns_flight = True
                self._stage("airborne", now_ns)
            else:
                self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        elif self.stage == "airborne":
            if status.arming_state != VehicleStatus.ARMING_STATE_ARMED or \
                    status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._abort("PX4 left armed position Offboard during takeoff")
            elif self.controller_status.startswith("hover_airborne_takeover_") and self._fresh(
                self.controller_status_time_ns, now_ns
            ):
                self._stage("handoff", now_ns)
            elif (now_ns - self.stage_start_ns) * 1e-9 > self.takeoff_timeout:
                self._diagnose_takeover()
                self._abort("PX4 takeoff did not produce an airborne px4ctrl hold; see takeover diagnostics")
            else:
                self._diagnose_takeover()
        elif self.stage == "handoff":
            if (now_ns - self.stage_start_ns) * 1e-9 > self.takeover_timeout:
                self._abort("px4ctrl attitude handoff timed out")
                return
            if status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._abort("PX4 left Offboard during attitude handoff")
                return
            if self._ready(now_ns):
                self._stage("ready", now_ns)
                self.get_logger().info("Offboard ready; keep this launch running, then start eight.launch.py")
            else:
                self._diagnose_takeover()

        if self.stage == "arming" and (
            now_ns - self.stage_start_ns) * 1e-9 > self.takeoff_timeout:
            self._abort(f"timed out during {self.stage}")

    def _begin(self, now_ns: int) -> None:
        position = self.local_position
        self.trajectory = FigureEight(
            (position.x, position.y, position.z), self.amplitude_x,
            self.amplitude_y, self.period, self.cycles, self.ramp)
        self.start_time_ns = now_ns
        self.heading = position.heading
        self.reset_signature = (
            position.xy_reset_counter, position.z_reset_counter,
            position.heading_reset_counter)
        self.get_logger().info(
            f"Starting {self.cycles}-cycle figure eight at NED "
            f"({position.x:.2f}, {position.y:.2f}, {position.z:.2f}) m; "
            f"duration {self.trajectory.duration:.1f} s")

    def _publish(self, now_ns: int, elapsed: float) -> None:
        sample = self.trajectory.sample(elapsed)
        msg = DesiredState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "px4_local_ned"
        msg.position.x, msg.position.y, msg.position.z = sample.position
        msg.velocity.x, msg.velocity.y, msg.velocity.z = sample.velocity
        msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = sample.acceleration
        msg.yaw = self.heading
        msg.yaw_rate = 0.0
        self.command_pub.publish(msg)

    def _record(self, now_ns: int, elapsed: float) -> tuple[float, float] | None:
        position = self.local_position
        if position.timestamp == self.last_recorded_timestamp:
            return None
        self.last_recorded_timestamp = position.timestamp
        sample_time = max(0.0, (self.local_position_time_ns - self.start_time_ns) * 1e-9)
        sample = self.trajectory.sample(sample_time)
        error = math.dist(sample.position, (position.x, position.y, position.z))
        speed = math.sqrt(position.vx**2 + position.vy**2 + position.vz**2)
        debug = self.control_debug
        row = {
            "elapsed_s": round(elapsed, 6),
            "phase": "tracking" if sample_time <= self.trajectory.duration else "settling",
            "command_x_ned_m": sample.position[0],
            "command_y_ned_m": sample.position[1],
            "command_z_ned_m": sample.position[2],
            "command_vx_ned_mps": sample.velocity[0],
            "command_vy_ned_mps": sample.velocity[1],
            "command_vz_ned_mps": sample.velocity[2],
            "px4_x_ned_m": position.x,
            "px4_y_ned_m": position.y,
            "px4_z_ned_m": position.z,
            "px4_vx_ned_mps": position.vx,
            "px4_vy_ned_mps": position.vy,
            "px4_vz_ned_mps": position.vz,
            "position_error_m": error,
            "speed_mps": speed,
            "saturation_flags": debug.saturation_flags if debug else 0,
            "thrust": debug.thrust if debug else math.nan,
            "offboard": int(self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD),
            "failsafe": int(self.vehicle_status.failsafe),
        }
        self.rows.append(row)
        if row["phase"] == "tracking":
            self.tracking_rows.append(row)
        return error, speed

    def _abort(self, reason: str) -> None:
        if self.aborted or self.finalized:
            return
        self.aborted = True
        self.get_logger().error(f"{'Takeoff handoff' if self.phase == 'takeoff' else 'Figure-eight trial'} aborted: {reason}")
        if self.phase == "takeoff":
            if self.owns_flight and self.vehicle_status and \
                    self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED and \
                    not self.vehicle_status.failsafe:
                self._stage("landing", self._now_ns())
            else:
                self._stage("done", self._now_ns())
        else:
            self._finish(False, reason)

    def _finish(self, settled: bool, reason: str = "") -> None:
        if self.finalized:
            return
        self.finalized = True
        position = self.local_position
        if position is not None and self.trajectory is not None:
            final_error = math.dist(self.trajectory.origin, (position.x, position.y, position.z))
            final_speed = math.sqrt(position.vx**2 + position.vy**2 + position.vz**2)
        else:
            final_error = final_speed = math.nan
        result = summarize(
            self.tracking_rows, final_error, final_speed, settled,
            self.offboard_losses, self.failsafe_events, reason)
        result.update({
            "frame": "px4_local_ned",
            "amplitude_x_m": self.amplitude_x,
            "amplitude_y_m": self.amplitude_y,
            "period_sec": self.period,
            "cycles": self.cycles,
            "ramp_sec": self.ramp,
            "all_samples": len(self.rows),
        })
        self.output_dir.mkdir(parents=True, exist_ok=True)
        run_name = datetime.now(timezone.utc).strftime("eight_%Y%m%d_%H%M%S_%f")
        csv_path = self.output_dir / f"{run_name}.csv"
        json_path = self.output_dir / f"{run_name}.json"
        if self.rows:
            with csv_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=list(self.rows[0]))
                writer.writeheader()
                writer.writerows(self.rows)
        # JSON has no representation for NaN; missing measurements remain null.
        clean = {key: (None if isinstance(value, float) and not math.isfinite(value) else value)
                 for key, value in result.items()}
        with json_path.open("w", encoding="utf-8") as file:
            json.dump(clean, file, indent=2, allow_nan=False)
            file.write("\n")
        self.get_logger().info(
            f"Eight result: {'PASS' if result['passed'] else 'FAIL'}, "
            f"RMSE={result['tracking_rmse_m']:.3f} m, "
            f"P95={result['tracking_p95_m']:.3f} m, "
            f"max={result['tracking_max_m']:.3f} m; report={json_path}")
        if self.owns_flight and self.vehicle_status and \
                self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED and \
                not self.vehicle_status.failsafe:
            self._stage("landing", self._now_ns())
        else:
            self._stage("done", self._now_ns())

    def _tick(self) -> None:
        now_ns = self._now_ns()
        if now_ns <= 0 or self.stage == "done":
            return
        if self.stage == "landing":
            if self.vehicle_status and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self._stage("done", now_ns)
                return
            if self.vehicle_status and self.vehicle_status.failsafe:
                self._stage("done", now_ns)
                return
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            return
        if self.stage == "ready":
            if not self._fresh(self.vehicle_status_time_ns, now_ns, 1.5) or \
                    not self._fresh(self.local_position_time_ns, now_ns) or \
                    not _position_valid(self.local_position):
                self._abort("PX4 state became stale or invalid after handoff")
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self._stage("landing", now_ns)
            elif self.vehicle_status.failsafe or \
                    self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._abort("PX4 left Offboard or entered failsafe after handoff")
            elif not self._fresh(self.controller_status_time_ns, now_ns) or \
                    self.controller_status.startswith(("safety_fault", "control_suspended")):
                self._abort("px4ctrl stopped streaming after handoff")
            return
        if self.stage == "waiting_for_controller":
            if self._ready(now_ns):
                self._begin(now_ns)
                self.owns_flight = True
                self._stage("tracking", now_ns)
            elif time.monotonic() - self.process_start_wall > self.startup_timeout:
                self._abort("PX4/px4ctrl not ready for figure eight; start takeoff.launch.py first")
            return
        if self.stage != "tracking":
            self._advance_startup(now_ns)
            if self.phase == "takeoff" and self.stage in ("prestream", "offboard", "arming", "airborne"):
                self._publish_takeoff_setpoint(now_ns)
            return
        elapsed = max(0.0, (now_ns - self.start_time_ns) * 1e-9)
        status = self.vehicle_status
        if not self._fresh(self.vehicle_status_time_ns, now_ns, 1.5) or \
                not self._fresh(self.local_position_time_ns, now_ns) or \
                not self._fresh(self.control_debug_time_ns, now_ns) or \
                not _position_valid(self.local_position):
            self._abort("PX4 state became stale or invalid")
            return
        if not self.control_debug.local_valid or not self.control_debug.status_fresh:
            self._abort("px4ctrl input health failed")
            return
        if status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or \
                status.arming_state != VehicleStatus.ARMING_STATE_ARMED or status.failsafe:
            self._abort("PX4 left armed Offboard or entered failsafe")
            return
        if self.controller_status.startswith(("safety_fault", "control_suspended")):
            self._abort(f"px4ctrl stopped control: {self.controller_status}")
            return
        reset = (self.local_position.xy_reset_counter, self.local_position.z_reset_counter,
                 self.local_position.heading_reset_counter)
        if reset != self.reset_signature:
            self._abort("PX4 local frame reset during trial")
            return
        self._publish(now_ns, elapsed)
        if self.finalized:
            return
        measurement = self._record(now_ns, elapsed)
        if measurement is not None and measurement[0] > self.max_tracking_error:
            if self.large_error_since_ns is None:
                self.large_error_since_ns = now_ns
            elif now_ns - self.large_error_since_ns >= 500_000_000:
                self._abort(f"tracking error exceeded {self.max_tracking_error:.2f} m for 0.5 s")
                return
        else:
            self.large_error_since_ns = None
        if elapsed < self.trajectory.duration:
            return
        if measurement is not None:
            error, speed = measurement
            if error <= 0.25 and speed <= 0.20:
                if self.settle_start_ns is None:
                    self.settle_start_ns = now_ns
                elif now_ns - self.settle_start_ns >= 2_000_000_000:
                    self._finish(True)
            else:
                self.settle_start_ns = None
        if not self.finalized and elapsed >= self.trajectory.duration + self.settle_timeout:
            self._finish(False, "endpoint did not settle within timeout")


def main(args=None, phase: str = "trajectory") -> None:
    rclpy.init(args=args)
    node = EightNode(phase=phase)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.owns_flight and node.vehicle_status and \
                node.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            for _ in range(3):
                node.last_command_wall = 0.0
                node._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                rclpy.spin_once(node, timeout_sec=0.1)
        if phase == "trajectory" and not node.finalized:
            node._finish(False, "interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
