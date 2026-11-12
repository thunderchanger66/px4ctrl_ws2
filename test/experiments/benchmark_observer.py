#!/usr/bin/env python3
"""Record one EGO/PX4 SITL run and summarize tracking and map clearance."""

import argparse
import csv
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
from px4ctrl.msg import ControlDebug, DesiredState
from quadrotor_msgs.msg import PositionCommand
from rcl_interfaces.msg import Log
from traj_utils.msg import Bspline
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


FIELDS = [
    "t_sim", "goal_x", "goal_y", "goal_z",
    "odom_x", "odom_y", "odom_z", "odom_vx", "odom_vy", "odom_vz",
    "cmd_x", "cmd_y", "cmd_z", "cmd_vx", "cmd_vy", "cmd_vz",
    "cmd_ax", "cmd_ay", "cmd_az", "trajectory_flag",
    "track_error_3d", "track_error_xy", "goal_error_3d", "speed_mps",
    "px4_x", "px4_y", "px4_z", "px4_vx", "px4_vy", "px4_vz",
    "debug_error_3d", "debug_accel_requested", "debug_accel_feasible",
    "thrust", "tilt_deg", "yaw_rate", "saturation_flags", "local_valid",
    "armed", "offboard", "failsafe", "inflated_clearance_m", "map_age_sec",
    "px4_status", "adapter_status",
]


def rotate(q, v):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-8 or not math.isfinite(n):
        return np.full(3, np.nan)
    x, y, z, w = x/n, y/n, z/n, w/n
    qv = np.array([x, y, z])
    vec = np.asarray(v, dtype=float)
    return vec + 2.0 * np.cross(qv, np.cross(qv, vec) + w * vec)


def magnitude(values):
    arr = np.asarray(values, dtype=float)
    return float(np.linalg.norm(arr)) if np.isfinite(arr).all() else math.nan


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


class BenchmarkObserver(Node):
    def __init__(self, args):
        super().__init__("px4ctrl_benchmark_observer", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.args = args
        self.goal = np.array([args.goal_x, args.goal_y, args.goal_z], dtype=float)
        self.goal_start = None
        self.goal_start_source = None
        self.goal_pose_matches = None
        self.odom = None
        self.odom_velocity = None
        self.command = None
        self.command_count = 0
        self.bspline_count = 0
        self.last_trajectory_id = -1
        self.events = {
            "ego_odom_timeout": 0,
            "ego_odom_out_of_map": 0,
            "ego_goal_rejected": 0,
            "ego_replan_failed": 0,
            "ego_trajectory_ended_away_from_goal": 0,
            "ego_emergency": 0,
            "px4_reference_blended": 0,
            "px4_acceleration_projected": 0,
        }
        self.px4 = None
        self.debug = None
        self.px4_status = ""
        self.adapter_status = ""
        self.cloud_points = None
        self.cloud_time = None
        self.last_stable_start = None
        self.arrival_time = None
        self.vehicle_status = None
        self.px4_timestamp = 0
        self.rows = []
        self.out_csv = Path(args.csv)
        self.out_json = self.out_csv.with_suffix(".json")
        self.out_csv.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.out_csv.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()

        px4_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.create_subscription(Odometry, "/fastlio2/lio_odom", self.odom_cb, 20)
        self.create_subscription(
            PositionCommand, "/drone_0_planning/pos_cmd", self.command_cb, 20)
        self.create_subscription(
            Bspline, "/drone_0_planning/bspline", self.bspline_cb, 20)
        self.create_subscription(DesiredState, "/px4ctrl/desired_state", self.desired_cb, 20)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.px4_cb, px4_qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4",
                                 self.vehicle_status_cb, px4_qos)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude",
                                 self.attitude_cb, px4_qos)
        self.create_subscription(ControlDebug, "/px4ctrl/debug", self.debug_cb, 20)
        self.create_subscription(String, "/px4ctrl/status", self.status_cb, 10)
        self.create_subscription(
            String, "/px4ctrl/reference_adapter_status", self.adapter_cb, 10)
        self.create_subscription(Log, "/rosout", self.log_cb, 100)
        self.create_subscription(
            PointCloud2, "/drone_0_grid/grid_map/occupancy_inflate", self.map_cb, 2)
        self.create_timer(0.05, self.tick)
        self.wall_start = time.monotonic()
        self.get_logger().info(f"Writing benchmark samples to {self.out_csv}")

    def goal_cb(self, msg):
        incoming = np.array([msg.pose.position.x, msg.pose.position.y,
                             msg.pose.position.z], dtype=float)
        self.goal_pose_matches = bool(np.linalg.norm(incoming - self.goal) <= 0.05)
        if not self.goal_pose_matches:
            self.get_logger().warning(
                f"Observed goal {incoming.tolist()} differs from configured target {self.goal.tolist()}")
            return
        if self.goal_start is None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.goal_start = stamp if stamp > 0 else self.get_clock().now().nanoseconds * 1e-9
            self.goal_start_source = "goal_pose"
            self.get_logger().info(f"Observed goal timestamp for {self.goal.tolist()} in {msg.header.frame_id}")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        t = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        self.odom = np.array([p.x, p.y, p.z], dtype=float)
        self.odom_velocity = rotate((q.x, q.y, q.z, q.w), (t.x, t.y, t.z))

    def command_cb(self, msg):
        self.command = msg
        self.command_count += 1
        if self.goal_start is None:
            self.goal_start = self.get_clock().now().nanoseconds * 1e-9
            self.goal_start_source = "first_position_command_fallback"
        if self.command_count == 1:
            self.get_logger().info("Received first EGO position command")

    def bspline_cb(self, msg):
        self.bspline_count += 1
        self.last_trajectory_id = int(msg.traj_id)
        if self.goal_start is None:
            stamp = msg.start_time.sec + msg.start_time.nanosec * 1e-9
            self.goal_start = stamp if stamp > 0 else self.get_clock().now().nanoseconds * 1e-9
            self.goal_start_source = "first_bspline_fallback"
        if self.bspline_count == 1:
            self.get_logger().info(f"Received first EGO B-spline id={self.last_trajectory_id}")

    def desired_cb(self, msg):
        del msg

    def px4_cb(self, msg):
        self.px4 = msg
        self.px4_timestamp = max(self.px4_timestamp, int(msg.timestamp))

    def vehicle_status_cb(self, msg):
        self.vehicle_status = msg
        self.px4_timestamp = max(self.px4_timestamp, int(msg.timestamp))

    def attitude_cb(self, msg):
        self.px4_timestamp = max(self.px4_timestamp, int(msg.timestamp))

    def debug_cb(self, msg):
        self.debug = msg

    def status_cb(self, msg):
        self.px4_status = msg.data

    def adapter_cb(self, msg):
        self.adapter_status = msg.data

    def log_cb(self, msg):
        if self.goal_start is None:
            return
        text = msg.msg.lower()
        logger = msg.name.lower()
        checks = (
            ("ego_odom_timeout", "odometry timeout" in text and "ego" in logger),
            ("ego_odom_out_of_map", "rejecting odometry" in text and "outside map" in text),
            ("ego_goal_rejected", "rejecting unreachable waypoint" in text or
             "rejecting goal" in text),
            ("ego_replan_failed", "unable to validate replanned trajectory" in text or
             "unable to validate a return trajectory" in text or
             "failed to generate the first trajectory" in text),
            ("ego_trajectory_ended_away_from_goal", "trajectory finished" in text and
             "from goal" in text),
            ("ego_emergency", "emergency" in text and "ego" in logger),
            ("px4_reference_blended", "blending discontinuous reference" in text),
            ("px4_acceleration_projected", "acceleration projected" in text),
        )
        for name, matched in checks:
            if matched:
                self.events[name] += 1

    def map_cb(self, msg):
        if self.odom is None or msg.width == 0:
            return
        try:
            xyz = np.asarray(point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True), dtype=float)
            if xyz.ndim == 2 and xyz.shape[0] > 0:
                distance = np.linalg.norm(xyz - self.odom[None, :], axis=1)
                self.cloud_points = float(np.min(distance))
                self.cloud_time = self.get_clock().now().nanoseconds * 1e-9
        except (ValueError, TypeError, RuntimeError) as exc:
            self.get_logger().warning(f"Could not decode inflated map cloud: {exc}")

    def tick(self):
        if self.odom is None or self.goal_start is None:
            if time.monotonic() - self.wall_start > self.args.startup_timeout:
                self.get_logger().error("No FAST-LIO odometry or EGO trajectory received before startup timeout")
                self.finish("startup_failed")
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.goal_start
        cmd = self.command
        px4 = self.px4
        debug = self.debug
        track_err = self.command.position if cmd else None
        if track_err:
            error = self.odom - np.array([track_err.x, track_err.y, track_err.z])
            track_error = magnitude(error)
            track_error_xy = magnitude(error[:2])
        else:
            track_error = track_error_xy = math.nan
        goal_error = magnitude(self.goal - self.odom)
        speed = magnitude(self.odom_velocity) if self.odom_velocity is not None else math.nan
        debug_error = magnitude((debug.position_error.x, debug.position_error.y,
                                 debug.position_error.z)) if debug else math.nan
        requested_accel = magnitude((cmd.acceleration.x, cmd.acceleration.y,
                                     cmd.acceleration.z)) if cmd else math.nan
        feasible_accel = magnitude((debug.acceleration_feasible.x,
                                    debug.acceleration_feasible.y,
                                    debug.acceleration_feasible.z)) if debug else math.nan
        map_age = now - self.cloud_time if self.cloud_time is not None else math.nan

        row = {
            "t_sim": elapsed,
            "goal_x": self.goal[0], "goal_y": self.goal[1], "goal_z": self.goal[2],
            "odom_x": self.odom[0], "odom_y": self.odom[1], "odom_z": self.odom[2],
            "odom_vx": self.odom_velocity[0], "odom_vy": self.odom_velocity[1],
            "odom_vz": self.odom_velocity[2],
            "cmd_x": cmd.position.x if cmd else math.nan,
            "cmd_y": cmd.position.y if cmd else math.nan,
            "cmd_z": cmd.position.z if cmd else math.nan,
            "cmd_vx": cmd.velocity.x if cmd else math.nan,
            "cmd_vy": cmd.velocity.y if cmd else math.nan,
            "cmd_vz": cmd.velocity.z if cmd else math.nan,
            "cmd_ax": cmd.acceleration.x if cmd else math.nan,
            "cmd_ay": cmd.acceleration.y if cmd else math.nan,
            "cmd_az": cmd.acceleration.z if cmd else math.nan,
            "trajectory_flag": cmd.trajectory_flag if cmd else -1,
            "track_error_3d": track_error, "track_error_xy": track_error_xy,
            "goal_error_3d": goal_error, "speed_mps": speed,
            "px4_x": px4.x if px4 else math.nan,
            "px4_y": px4.y if px4 else math.nan,
            "px4_z": px4.z if px4 else math.nan,
            "px4_vx": px4.vx if px4 else math.nan,
            "px4_vy": px4.vy if px4 else math.nan,
            "px4_vz": px4.vz if px4 else math.nan,
            "debug_error_3d": debug_error,
            "debug_accel_requested": magnitude((debug.acceleration_requested.x,
                                                  debug.acceleration_requested.y,
                                                  debug.acceleration_requested.z)) if debug else math.nan,
            "debug_accel_feasible": feasible_accel,
            "thrust": debug.thrust if debug else math.nan,
            "tilt_deg": debug.tilt_deg if debug else math.nan,
            "yaw_rate": debug.yaw_rate if debug else math.nan,
            "saturation_flags": debug.saturation_flags if debug else -1,
            "local_valid": debug.local_valid if debug else False,
            "armed": debug.armed if debug else False,
            "offboard": debug.offboard if debug else False,
            "failsafe": debug.failsafe if debug else False,
            "inflated_clearance_m": self.cloud_points if self.cloud_points is not None else math.nan,
            "map_age_sec": map_age,
            "px4_status": self.px4_status, "adapter_status": self.adapter_status,
        }
        self.rows.append(row)
        self.writer.writerow({key: finite_or_none(value) if isinstance(value, (float, np.floating)) else value
                              for key, value in row.items()})
        self.file.flush()

        arrived = (goal_error <= self.args.arrival_radius and
                   speed <= self.args.arrival_speed)
        if arrived:
            if self.last_stable_start is None:
                self.last_stable_start = now
            elif now - self.last_stable_start >= self.args.settle_duration:
                self.arrival_time = now - self.goal_start
                self.finish("arrived")
                return
        else:
            self.last_stable_start = None

        if elapsed >= self.args.timeout:
            self.finish("timeout")

    def summarize(self, result):
        def values(field):
            return np.asarray([r[field] for r in self.rows if math.isfinite(float(r[field]))], dtype=float)
        error = values("track_error_3d")
        error_xy = values("track_error_xy")
        goal_error = values("goal_error_3d")
        speed = values("speed_mps")
        clear = values("inflated_clearance_m")
        tilt = values("tilt_deg")
        accel_cmd = np.asarray([magnitude((r["cmd_ax"], r["cmd_ay"], r["cmd_az"]))
                                for r in self.rows if all(math.isfinite(float(r[k]))
                                for k in ("cmd_ax", "cmd_ay", "cmd_az"))])
        saturation = np.asarray([r["saturation_flags"] for r in self.rows
                                 if r["saturation_flags"] >= 0], dtype=float)
        map_ages = values("map_age_sec")
        summary = {
            "result": result,
            "trial_valid": bool(result == "arrived" and self.command_count > 0 and
                                 self.bspline_count > 0 and clear.size > 0),
            "goal": self.goal.tolist() if self.goal is not None else None,
            "goal_start_source": self.goal_start_source,
            "goal_pose_topic_matched_config": self.goal_pose_matches,
            "arrival_time_s": self.arrival_time,
            "samples": len(self.rows),
            "ego_bspline_messages": self.bspline_count,
            "ego_position_command_messages": self.command_count,
            "last_trajectory_id": self.last_trajectory_id,
            "event_counts": self.events,
            "track_error_3d_rmse_m": finite_or_none(np.sqrt(np.mean(error**2))) if error.size else None,
            "track_error_3d_p95_m": finite_or_none(np.percentile(error, 95)) if error.size else None,
            "track_error_3d_max_m": finite_or_none(np.max(error)) if error.size else None,
            "track_error_xy_rmse_m": finite_or_none(np.sqrt(np.mean(error_xy**2))) if error_xy.size else None,
            "goal_error_final_m": finite_or_none(goal_error[-1]) if goal_error.size else None,
            "speed_peak_mps": finite_or_none(np.max(speed)) if speed.size else None,
            "speed_p95_mps": finite_or_none(np.percentile(speed, 95)) if speed.size else None,
            "command_accel_peak_mps2": finite_or_none(np.max(accel_cmd)) if accel_cmd.size else None,
            "tilt_peak_deg": finite_or_none(np.max(tilt)) if tilt.size else None,
            "inflated_map_clearance_min_m": finite_or_none(np.min(clear)) if clear.size else None,
            "inflated_map_clearance_p05_m": finite_or_none(np.percentile(clear, 5)) if clear.size else None,
            "map_age_p95_s": finite_or_none(np.percentile(map_ages, 95)) if map_ages.size else None,
            "saturation_any_fraction": finite_or_none(np.mean(saturation != 0)) if saturation.size else None,
            "failsafe_samples": int(sum(bool(r["failsafe"]) for r in self.rows)),
            "adapter_status_final": self.adapter_status,
            "px4_status_final": self.px4_status,
            "arrival_radius_m": self.args.arrival_radius,
            "arrival_speed_mps": self.args.arrival_speed,
            "settle_duration_s": self.args.settle_duration,
            "notes": "Inflated-map clearance is nearest published occupied-inflated voxel center, not a Gazebo collision sensor measurement.",
        }
        self.out_json.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        return summary

    def finish(self, result):
        if getattr(self, "finished", False):
            return
        self.finished = True
        summary = self.summarize(result)
        self.get_logger().info("BENCHMARK_RESULT " + json.dumps(summary, allow_nan=False))
        self.file.close()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--goal-z", type=float, required=True)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--startup-timeout", type=float, default=600.0)
    parser.add_argument("--arrival-radius", type=float, default=0.20)
    parser.add_argument("--arrival-speed", type=float, default=0.20)
    parser.add_argument("--settle-duration", type=float, default=1.0)
    args = parser.parse_args()
    rclpy.init()
    node = BenchmarkObserver(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish("interrupted")
    finally:
        if not getattr(node, "finished", False):
            node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
