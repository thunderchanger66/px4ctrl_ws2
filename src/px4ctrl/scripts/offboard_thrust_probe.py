#!/usr/bin/env python3
"""Bounded SITL-only probe for PX4 attitude/thrust to Gazebo motor response."""

import argparse
import csv
import math
import time

import rclpy
from px4_msgs.msg import OffboardControlMode, VehicleAttitude, VehicleAttitudeSetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class ThrustProbe(Node):
    def __init__(self, args):
        super().__init__("offboard_thrust_probe")
        self.args = args
        self.px4_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", self.px4_qos
        )
        self.setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint_v1", self.px4_qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", self.px4_qos
        )
        self.status = None
        self.position = None
        self.attitude = None
        self.status_wall_time = None
        self.position_wall_time = None
        self.attitude_wall_time = None
        self.px4_timestamp = 0
        self.px4_stamp_wall = None
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v4", self.status_cb, self.px4_qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_cb,
            self.px4_qos,
        )
        self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self.attitude_cb, self.px4_qos
        )
        self.start_time = self.get_clock().now()
        self.last_command_time = 0.0
        self.armed_at = None
        self.started_at = None
        self.finished = False
        self.rows = []
        self.timer = self.create_timer(0.05, self.tick)

    def status_cb(self, msg):
        self.status = msg
        self.status_wall_time = time.monotonic()

    def position_cb(self, msg):
        self.position = msg
        self.px4_timestamp = max(self.px4_timestamp, msg.timestamp)
        self.px4_stamp_wall = time.monotonic()
        self.position_wall_time = self.px4_stamp_wall

    def attitude_cb(self, msg):
        if msg.timestamp:
            self.attitude = msg
            self.px4_timestamp = max(self.px4_timestamp, msg.timestamp)
            self.px4_stamp_wall = time.monotonic()
            self.attitude_wall_time = self.px4_stamp_wall

    def now_us(self):
        if self.px4_timestamp == 0:
            return 0
        elapsed = time.monotonic() - self.px4_stamp_wall if self.px4_stamp_wall else 0.0
        return self.px4_timestamp + int(elapsed * 1e6)

    def seconds_since(self, timestamp):
        return max(0.0, (self.get_clock().now() - timestamp).nanoseconds * 1e-9)

    def publish_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def position_healthy(self):
        p = self.position
        if not (
            p
            and self.attitude
            and p.timestamp
            and p.xy_valid
            and p.z_valid
            and p.v_xy_valid
            and p.v_z_valid
            and not p.dead_reckoning
            and self.status_wall_time is not None
            and time.monotonic() - self.status_wall_time < 0.5
            and self.position_wall_time is not None
            and time.monotonic() - self.position_wall_time < 0.5
            and self.attitude_wall_time is not None
            and time.monotonic() - self.attitude_wall_time < 0.5
        ):
            return False
        state = (p.x, p.y, p.z, p.vx, p.vy, p.vz)
        quaternion = self.attitude.q
        if not all(math.isfinite(float(value)) for value in state + tuple(quaternion)):
            return False
        norm = math.sqrt(sum(float(component) ** 2 for component in quaternion))
        return 0.5 <= norm <= 1.5

    def publish_setpoint(self, thrust):
        timestamp = self.now_us()
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.attitude = True
        self.mode_pub.publish(mode)
        sp = VehicleAttitudeSetpoint()
        sp.timestamp = timestamp
        q = self.attitude.q
        q_norm = math.sqrt(sum(float(component) ** 2 for component in q))
        if not math.isfinite(q_norm) or q_norm < 1e-6:
            return
        sp.q_d = [float(component) / q_norm for component in q]
        sp.thrust_body = [0.0, 0.0, -float(thrust)]
        sp.yaw_sp_move_rate = math.nan
        self.setpoint_pub.publish(sp)

    def tick(self):
        elapsed = self.seconds_since(self.start_time)
        if elapsed > self.args.timeout:
            self.finish("timeout_before_takeoff")
            return
        if not self.position_healthy() or self.status is None:
            return

        nav_offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if not self.finished:
            self.publish_setpoint(0.0 if not self.armed_at else self.current_thrust())

        if not nav_offboard and elapsed >= self.args.prestream_sec and elapsed - self.last_command_time > 1.0:
            self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.last_command_time = elapsed
        elif nav_offboard and not armed and elapsed - self.last_command_time > 1.0:
            if self.status.pre_flight_checks_pass:
                self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.last_command_time = elapsed

        if armed and nav_offboard and self.armed_at is None:
            self.armed_at = self.get_clock().now()
            self.get_logger().info("PX4 armed in Offboard; starting bounded thrust ramp")
        if self.armed_at is None:
            return

        age = self.seconds_since(self.armed_at)
        p = self.position
        row = {
            "elapsed_s": round(age, 3),
            "thrust": round(self.current_thrust(), 3),
            "x_ned_m": p.x,
            "y_ned_m": p.y,
            "z_ned_m": p.z,
            "vz_ned_mps": p.vz,
            "nav_state": self.status.nav_state,
            "arming_state": self.status.arming_state,
        }
        self.rows.append(row)
        if self.started_at is None:
            self.started_at = p.z
        climb = self.started_at - p.z
        self.get_logger().info(
            f"probe t={age:.1f}s thrust={row['thrust']:.2f} "
            f"altitude_change={climb:.2f}m vz_ned={p.vz:.2f}m/s"
        )
        if climb >= self.args.max_climb_m:
            self.finish("max_climb_reached")
        elif age >= self.args.ramp_duration:
            self.finish("ramp_complete")

    def current_thrust(self):
        age = self.seconds_since(self.armed_at) if self.armed_at else 0.0
        step = min(int(age / self.args.step_duration), len(self.args.thrust_steps) - 1)
        return self.args.thrust_steps[step]

    def finish(self, reason):
        if self.finished:
            return
        self.finished = True
        self.get_logger().warn(f"Probe ending: {reason}; requesting PX4 land")
        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.land_deadline = time.monotonic() + self.args.land_wait
        self.timer.cancel()
        self.land_timer = self.create_timer(0.1, self.land_tick)
        self.reason = reason

    def land_tick(self):
        if self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.write_csv()
            rclpy.shutdown()
        elif time.monotonic() >= self.land_deadline:
            self.get_logger().warn("Land wait limit reached; stopping probe publisher")
            self.write_csv()
            rclpy.shutdown()

    def write_csv(self):
        with open(self.args.csv, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "elapsed_s", "thrust", "x_ned_m", "y_ned_m", "z_ned_m",
                    "vz_ned_mps", "nav_state", "arming_state",
                ],
            )
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(f"Probe result={self.reason}; samples={len(self.rows)}; csv={self.args.csv}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--prestream-sec", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--step-duration", type=float, default=0.5)
    parser.add_argument("--ramp-duration", type=float, default=2.5)
    parser.add_argument("--max-climb-m", type=float, default=1.0)
    parser.add_argument("--land-wait", type=float, default=20.0)
    parser.add_argument("--thrust-steps", type=float, nargs="+", default=[0.55, 0.65, 0.75, 0.85, 0.90])
    args = parser.parse_args()
    if not args.thrust_steps or any(
        not math.isfinite(value) or not 0.0 < value <= 0.9
        for value in args.thrust_steps
    ):
        parser.error("thrust steps must be in (0, 0.9]")
    durations = (args.prestream_sec, args.timeout, args.step_duration,
                 args.ramp_duration, args.max_climb_m, args.land_wait)
    if any(not math.isfinite(value) or value <= 0.0 for value in durations):
        parser.error("prestream, timeout, step, ramp, climb and land durations must be finite and positive")
    rclpy.init()
    node = ThrustProbe(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish("interrupted")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
