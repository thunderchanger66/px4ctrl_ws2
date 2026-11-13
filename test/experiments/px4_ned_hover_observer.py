#!/usr/bin/env python3
"""Measure PX4 local-NED trajectory tracking after a settled hover takeoff."""

import argparse
import csv
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from px4ctrl.msg import DesiredState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


FIELDS = [
    "t_from_trajectory_start_s", "px4_x_n", "px4_y_e", "px4_z_d",
    "px4_vx_n", "px4_vy_e", "px4_vz_d", "ref_x_n", "ref_y_e", "ref_z_d",
    "ref_vx_n", "ref_vy_e", "ref_vz_d", "error_n", "error_e", "error_d",
    "tracking_error_3d_m", "tracking_error_xy_m", "actual_speed_mps",
    "reference_speed_mps", "armed", "offboard", "failsafe",
]


def norm3(x, y, z):
    return math.sqrt(x*x + y*y + z*z)


class PX4NedObserver(Node):
    def __init__(self, args):
        super().__init__("px4_ned_hover_benchmark", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.args = args
        self.goal = np.array([args.goal_x, args.goal_y, args.goal_z], dtype=float)
        self.px4 = None
        self.px4_received = None
        self.reference = None
        self.reference_received = None
        self.vehicle_failsafe = False
        self.hover_position = None
        self.hover_since = None
        self.ready_wall_time = None
        self.trajectory_start = None
        self.arrival_since = None
        self.arrival_time = None
        self.invalid_since = None
        self.invalid_sample_ticks = 0
        self.skip_reasons = {"local_invalid_or_stale": 0, "vehicle_status_stale": 0,
                             "not_armed": 0, "not_offboard": 0, "failsafe": 0}
        self.terminal_reference = None
        self.rows = []
        self.result = "waiting_for_hover"
        self.started_wall = time.monotonic()
        self.csv_path = Path(args.csv)
        self.json_path = self.csv_path.with_suffix(".json")
        self.ready_path = Path(args.ready_file)
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=FIELDS)
        self.writer.writeheader()

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1", self.position_cb, qos)
        self.create_subscription(VehicleStatus,
            "/fmu/out/vehicle_status_v4", self.status_cb, qos)
        self.create_subscription(DesiredState,
            "/px4ctrl/desired_state", self.reference_cb, 20)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info("Waiting for valid PX4 NED hover and goal")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def position_cb(self, msg):
        self.px4 = msg
        self.px4_received = self.now()

    def status_cb(self, msg):
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.vehicle_failsafe = msg.failsafe
        self.status_received = self.now()

    def reference_cb(self, msg):
        if msg.header.frame_id != "px4_local_ned":
            return
        values = (msg.position.x, msg.position.y, msg.position.z,
                  msg.velocity.x, msg.velocity.y, msg.velocity.z)
        if not all(math.isfinite(v) for v in values):
            return
        self.reference = msg
        self.reference_received = self.now()

    def goal_cb(self, msg):
        p = msg.pose.position
        if norm3(p.x-self.goal[0], p.y-self.goal[1], p.z-self.goal[2]) <= 0.05:
            self.get_logger().info("Matching planner goal observed")

    def local_valid(self, now):
        p = self.px4
        return (p is not None and self.px4_received is not None and
                now-self.px4_received <= self.args.max_data_age and
                p.timestamp > 0 and p.xy_valid and p.z_valid and
                p.v_xy_valid and p.v_z_valid and not p.dead_reckoning and
                all(math.isfinite(v) for v in (p.x, p.y, p.z, p.vx, p.vy, p.vz)))

    def controller_ready(self, now):
        status_fresh = getattr(self, "status_received", None) is not None and \
            now-self.status_received <= self.args.max_status_age
        return (status_fresh and self.armed and self.offboard and
                not self.vehicle_failsafe)

    def tick(self):
        now = self.now()
        wall_elapsed = time.monotonic() - self.started_wall
        if wall_elapsed > self.args.startup_timeout and self.trajectory_start is None:
            self.finish("startup_timeout")
            return
        if self.trajectory_start is not None and now-self.trajectory_start > self.args.timeout:
            self.finish("trajectory_timeout")
            return
        if not self.local_valid(now) or not self.controller_ready(now):
            if self.trajectory_start is not None:
                self.invalid_sample_ticks += 1
                if not self.local_valid(now):
                    self.skip_reasons["local_invalid_or_stale"] += 1
                if getattr(self, "status_received", None) is None or \
                        now-self.status_received > self.args.max_status_age:
                    self.skip_reasons["vehicle_status_stale"] += 1
                if not getattr(self, "armed", False):
                    self.skip_reasons["not_armed"] += 1
                if not getattr(self, "offboard", False):
                    self.skip_reasons["not_offboard"] += 1
                if getattr(self, "vehicle_failsafe", False):
                    self.skip_reasons["failsafe"] += 1
                if self.invalid_since is None:
                    self.invalid_since = now
                elif now-self.invalid_since >= self.args.invalid_state_timeout:
                    self.finish("invalid_px4_state_during_trajectory")
            return
        self.invalid_since = None

        p = self.px4
        speed = norm3(p.vx, p.vy, p.vz)
        if self.hover_position is None:
            if speed <= self.args.hover_speed:
                if self.hover_since is None:
                    self.hover_since = now
                if now-self.hover_since >= self.args.hover_dwell:
                    self.hover_position = np.array([p.x, p.y, p.z], dtype=float)
                    self.ready_wall_time = time.monotonic()
                    self.ready_path.write_text("hover_ready\n", encoding="utf-8")
                    self.get_logger().info(
                        "Hover baseline captured in PX4 local NED: " +
                        json.dumps(self.hover_position.tolist()))
            else:
                self.hover_since = None
            return

        ref = self.reference
        if ref is None or now-self.reference_received > self.args.max_data_age:
            return
        ref_pos = np.array([ref.position.x, ref.position.y, ref.position.z], dtype=float)
        ref_vel = np.array([ref.velocity.x, ref.velocity.y, ref.velocity.z], dtype=float)
        if self.trajectory_start is None:
            displacement = np.linalg.norm(ref_pos-self.hover_position)
            if displacement >= self.args.start_displacement or \
                    np.linalg.norm(ref_vel) >= self.args.start_reference_speed:
                self.trajectory_start = now
                self.get_logger().info(
                    f"NED tracking window started from hover; reference displacement="
                    f"{displacement:.3f} m, speed={np.linalg.norm(ref_vel):.3f} m/s")
            return

        px = np.array([p.x, p.y, p.z], dtype=float)
        vel = np.array([p.vx, p.vy, p.vz], dtype=float)
        error = ref_pos-px
        ref_speed = float(np.linalg.norm(ref_vel))
        elapsed = now-self.trajectory_start
        row = {
            "t_from_trajectory_start_s": elapsed,
            "px4_x_n": p.x, "px4_y_e": p.y, "px4_z_d": p.z,
            "px4_vx_n": p.vx, "px4_vy_e": p.vy, "px4_vz_d": p.vz,
            "ref_x_n": ref_pos[0], "ref_y_e": ref_pos[1], "ref_z_d": ref_pos[2],
            "ref_vx_n": ref_vel[0], "ref_vy_e": ref_vel[1], "ref_vz_d": ref_vel[2],
            "error_n": error[0], "error_e": error[1], "error_d": error[2],
            "tracking_error_3d_m": float(np.linalg.norm(error)),
            "tracking_error_xy_m": float(np.linalg.norm(error[:2])),
            "actual_speed_mps": speed, "reference_speed_mps": ref_speed,
            "armed": self.armed, "offboard": self.offboard,
            "failsafe": self.vehicle_failsafe,
        }
        self.rows.append(row)
        self.writer.writerow(row)
        self.csv_file.flush()

        # Arrival requires the NED reference to stop moving and PX4 to settle at it.
        ref_stable = (ref_speed <= self.args.terminal_reference_speed and
                      self.terminal_reference is not None and
                      np.linalg.norm(ref_pos-self.terminal_reference) <=
                      self.args.terminal_reference_drift)
        if not ref_stable:
            self.terminal_reference = ref_pos.copy()
        arrived = (ref_stable and np.linalg.norm(error) <= self.args.arrival_radius and
                   speed <= self.args.arrival_speed)
        if arrived:
            if self.arrival_since is None:
                self.arrival_since = now
            elif now-self.arrival_since >= self.args.settle_duration:
                self.arrival_time = now-self.trajectory_start
                self.finish("arrived")
        else:
            self.arrival_since = None

    def summary(self):
        e = np.asarray([r["tracking_error_3d_m"] for r in self.rows], dtype=float)
        xy = np.asarray([r["tracking_error_xy_m"] for r in self.rows], dtype=float)
        final = self.rows[-1] if self.rows else None
        return {
            "result": self.result,
            "trial_valid": self.result == "arrived" and len(self.rows) > 0,
            "metric_frame": "px4_local_ned",
            "metric_definition": "3D Euclidean norm of PX4 local-NED DesiredState position minus PX4 VehicleLocalPosition; sampled at 20 Hz during active reference motion through settled arrival, requiring valid fresh PX4 local state and armed/offboard non-failsafe status.",
            "hover_start_ned_m": self.hover_position.tolist() if self.hover_position is not None else None,
            "goal_lidar_m": self.goal.tolist(),
            "trajectory_start_trigger": "PX4 NED DesiredState moved from the captured hover position by the displacement threshold or its speed exceeded the threshold",
            "arrival_time_s": self.arrival_time,
            "samples": len(self.rows),
            "tracking_duration_s": final["t_from_trajectory_start_s"] if final else None,
            "sample_rate_hz": (len(self.rows)/final["t_from_trajectory_start_s"]
                               if final and final["t_from_trajectory_start_s"] > 0 else None),
            "px4_ned_tracking_error_rmse_m": float(np.sqrt(np.mean(e**2))) if e.size else None,
            "px4_ned_tracking_error_p95_m": float(np.percentile(e, 95)) if e.size else None,
            "px4_ned_tracking_error_max_m": float(np.max(e)) if e.size else None,
            "px4_ned_tracking_error_xy_rmse_m": float(np.sqrt(np.mean(xy**2))) if xy.size else None,
            "px4_ned_final_tracking_error_m": float(np.linalg.norm([
                final["error_n"], final["error_e"], final["error_d"]])) if final else None,
            "px4_ned_goal_error_final_m": (float(np.linalg.norm([
                final["error_n"], final["error_e"], final["error_d"]]))
                if final and self.result == "arrived" else None),
            "terminal_reference_ned_m": [final["ref_x_n"], final["ref_y_e"], final["ref_z_d"]] if final else None,
            "arrival_radius_m": self.args.arrival_radius,
            "arrival_speed_mps": self.args.arrival_speed,
            "terminal_reference_speed_mps": self.args.terminal_reference_speed,
            "settle_duration_s": self.args.settle_duration,
            "failsafe_samples": sum(bool(row["failsafe"]) for row in self.rows),
            "invalid_state_sample_ticks_skipped": self.invalid_sample_ticks,
            "invalid_state_skip_reasons": self.skip_reasons,
        }

    def finish(self, result):
        if self.result in ("arrived", "trajectory_timeout", "startup_timeout",
                           "invalid_px4_state_during_trajectory"):
            return
        self.result = result
        summary = self.summary()
        self.json_path.write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n",
                                  encoding="utf-8")
        self.get_logger().info("PX4_NED_RESULT " + json.dumps(summary, allow_nan=False))
        self.csv_file.close()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--ready-file", required=True)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--goal-z", type=float, required=True)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--startup-timeout", type=float, default=300.0)
    parser.add_argument("--max-data-age", type=float, default=0.30)
    parser.add_argument("--max-status-age", type=float, default=1.50)
    parser.add_argument("--invalid-state-timeout", type=float, default=1.0)
    parser.add_argument("--hover-speed", type=float, default=0.15)
    parser.add_argument("--hover-dwell", type=float, default=1.5)
    parser.add_argument("--start-displacement", type=float, default=0.10)
    parser.add_argument("--start-reference-speed", type=float, default=0.15)
    parser.add_argument("--terminal-reference-speed", type=float, default=0.08)
    parser.add_argument("--terminal-reference-drift", type=float, default=0.03)
    parser.add_argument("--arrival-radius", type=float, default=0.20)
    parser.add_argument("--arrival-speed", type=float, default=0.20)
    parser.add_argument("--settle-duration", type=float, default=1.0)
    args = parser.parse_args()
    rclpy.init()
    node = PX4NedObserver(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node.csv_file.closed:
            node.csv_file.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    from rclpy.executors import ExternalShutdownException
    main()
