#!/usr/bin/env python3
"""Record EGO position commands and PX4 local position for a bounded trial."""

import argparse
import csv
import math

import rclpy
from px4_msgs.msg import (
    FailsafeFlags,
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String


class TrackEval(Node):
    def __init__(self, args):
        super().__init__("px4ctrl_track_eval")
        self.args = args
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.rows = []
        self.command = None
        self.position = None
        self.status = None
        self.attitude = None
        self.attitude_setpoint = None
        self.failsafe = None
        self.land = None
        self.controller_status = ""
        self.planner_emergency_count = 0
        self.started = self.get_clock().now()
        self.create_subscription(
            PositionCommand, "/drone_0_planning/pos_cmd", self.command_cb, 20
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_cb,
            sensor_qos,
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v4", self.status_cb, sensor_qos
        )
        self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self.attitude_cb, sensor_qos
        )
        self.create_subscription(
            VehicleAttitudeSetpoint,
            "/fmu/in/vehicle_attitude_setpoint_v1",
            self.attitude_setpoint_cb,
            sensor_qos,
        )
        self.create_subscription(
            FailsafeFlags, "/fmu/out/failsafe_flags", self.failsafe_cb, sensor_qos
        )
        self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self.land_cb, sensor_qos
        )
        self.create_subscription(String, "/px4ctrl/status", self.controller_status_cb, 10)
        self.create_subscription(
            Empty, "/planning/emergency_stop", self.emergency_cb, 10
        )
        self.timer = self.create_timer(0.05, self.record)

    def command_cb(self, msg):
        if msg.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY:
            return
        self.command = msg

    def position_cb(self, msg):
        self.position = msg

    def status_cb(self, msg):
        self.status = msg

    def attitude_cb(self, msg):
        self.attitude = msg

    def attitude_setpoint_cb(self, msg):
        self.attitude_setpoint = msg

    def failsafe_cb(self, msg):
        self.failsafe = msg

    def land_cb(self, msg):
        self.land = msg

    def controller_status_cb(self, msg):
        self.controller_status = msg.data

    def emergency_cb(self, _msg):
        self.planner_emergency_count += 1

    def record(self):
        elapsed = max(0.0, (self.get_clock().now() - self.started).nanoseconds * 1e-9)
        if elapsed >= self.args.duration:
            self.write_csv()
            rclpy.shutdown()
            return
        c = self.command
        p = self.position
        s = self.status
        if c is None or p is None:
            return
        self.rows.append({
            "elapsed_s": elapsed,
            "cmd_x_lidar": c.position.x,
            "cmd_y_lidar": c.position.y,
            "cmd_z_lidar": c.position.z,
            "cmd_vx_lidar": c.velocity.x,
            "cmd_vy_lidar": c.velocity.y,
            "cmd_vz_lidar": c.velocity.z,
            "cmd_ax_lidar": c.acceleration.x,
            "cmd_ay_lidar": c.acceleration.y,
            "cmd_az_lidar": c.acceleration.z,
            "cmd_yaw_lidar": c.yaw,
            "cmd_yaw_dot_lidar": c.yaw_dot,
            "px4_x_ned": p.x,
            "px4_y_ned": p.y,
            "px4_z_ned": p.z,
            "px4_vx_ned": p.vx,
            "px4_vy_ned": p.vy,
            "px4_vz_ned": p.vz,
            "px4_ax_ned": p.ax,
            "px4_ay_ned": p.ay,
            "px4_az_ned": p.az,
            "px4_heading_ned": p.heading,
            "xy_reset_counter": p.xy_reset_counter,
            "z_reset_counter": p.z_reset_counter,
            "vxy_reset_counter": p.vxy_reset_counter,
            "vz_reset_counter": p.vz_reset_counter,
            "heading_reset_counter": p.heading_reset_counter,
            "xy_valid": int(p.xy_valid),
            "z_valid": int(p.z_valid),
            "dead_reckoning": int(p.dead_reckoning),
            "nav_state": s.nav_state if s else -1,
            "arming_state": s.arming_state if s else -1,
            "failsafe": int(s.failsafe) if s else -1,
            "offboard_signal_lost": int(self.failsafe.offboard_control_signal_lost)
            if self.failsafe else -1,
            "landed": int(self.land.landed) if self.land else -1,
            "maybe_landed": int(self.land.maybe_landed) if self.land else -1,
            "att_q_w": self.attitude.q[0] if self.attitude else math.nan,
            "att_q_x": self.attitude.q[1] if self.attitude else math.nan,
            "att_q_y": self.attitude.q[2] if self.attitude else math.nan,
            "att_q_z": self.attitude.q[3] if self.attitude else math.nan,
            "sp_q_w": self.attitude_setpoint.q_d[0] if self.attitude_setpoint else math.nan,
            "sp_q_x": self.attitude_setpoint.q_d[1] if self.attitude_setpoint else math.nan,
            "sp_q_y": self.attitude_setpoint.q_d[2] if self.attitude_setpoint else math.nan,
            "sp_q_z": self.attitude_setpoint.q_d[3] if self.attitude_setpoint else math.nan,
            "thrust_x": self.attitude_setpoint.thrust_body[0]
            if self.attitude_setpoint else math.nan,
            "thrust_y": self.attitude_setpoint.thrust_body[1]
            if self.attitude_setpoint else math.nan,
            "thrust_z": self.attitude_setpoint.thrust_body[2]
            if self.attitude_setpoint else math.nan,
            "yaw_sp_move_rate": self.attitude_setpoint.yaw_sp_move_rate
            if self.attitude_setpoint else math.nan,
            "px4ctrl_status": self.controller_status,
            "planner_emergency_count": self.planner_emergency_count,
        })

    def write_csv(self):
        fields = [
            "elapsed_s", "cmd_x_lidar", "cmd_y_lidar", "cmd_z_lidar",
            "cmd_vx_lidar", "cmd_vy_lidar", "cmd_vz_lidar", "cmd_yaw_lidar",
            "cmd_ax_lidar", "cmd_ay_lidar", "cmd_az_lidar", "cmd_yaw_dot_lidar",
            "px4_x_ned", "px4_y_ned", "px4_z_ned", "px4_vx_ned",
            "px4_vy_ned", "px4_vz_ned", "px4_ax_ned", "px4_ay_ned", "px4_az_ned",
            "px4_heading_ned", "xy_reset_counter", "z_reset_counter",
            "vxy_reset_counter", "vz_reset_counter", "heading_reset_counter",
            "xy_valid", "z_valid",
            "dead_reckoning", "nav_state", "arming_state", "failsafe",
            "offboard_signal_lost", "landed", "maybe_landed",
            "att_q_w", "att_q_x", "att_q_y", "att_q_z",
            "sp_q_w", "sp_q_x", "sp_q_y", "sp_q_z",
            "thrust_x", "thrust_y", "thrust_z", "yaw_sp_move_rate", "px4ctrl_status",
            "planner_emergency_count",
        ]
        with open(self.args.csv, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(f"saved {len(self.rows)} samples to {self.args.csv}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("duration must be positive")
    rclpy.init()
    node = TrackEval(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.write_csv()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
