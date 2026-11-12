#!/usr/bin/env python3
"""Align EGO's planner frame to PX4 local NED and convert trajectory samples."""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from px4ctrl.msg import DesiredState
from quadrotor_msgs.msg import PositionCommand
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def quaternion_matrix(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-6:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class EgoPositionCommandAdapter(Node):
    def __init__(self):
        super().__init__("ego_position_command_adapter")
        self.planner_frame = self.declare_parameter("planner_frame", "lidar").value
        self.reference_frame = self.declare_parameter("reference_frame", "px4_local_ned").value
        self.odom_topic = self.declare_parameter("odom_topic", "/fastlio2/lio_odom").value
        self.local_position_topic = self.declare_parameter(
            "local_position_topic", "/fmu/out/vehicle_local_position_v1"
        ).value
        self.input_topic = self.declare_parameter(
            "input_topic", "/drone_0_planning/pos_cmd"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/px4ctrl/desired_state"
        ).value
        self.stationary_speed = float(self.declare_parameter(
            "alignment_stationary_speed_mps", 0.20
        ).value)
        self.state_timeout = float(self.declare_parameter("state_timeout_sec", 0.5).value)
        rotation = self.declare_parameter("transform_rotation_row_major", [
            0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, -1.0,
        ]).value
        if len(rotation) != 9:
            raise ValueError("transform_rotation_row_major must contain 9 values")
        self.rotation = np.asarray(rotation, dtype=float).reshape((3, 3))
        if (not np.isfinite(self.rotation).all()
                or not np.allclose(self.rotation @ self.rotation.T, np.eye(3), atol=1e-4)
                or abs(np.linalg.det(self.rotation) - 1.0) > 1e-4):
            raise ValueError("transform_rotation_row_major must be a proper rotation")

        self.odom = None
        self.odom_received_at = None
        self.local_position = None
        self.local_received_at = None
        self.translation = None
        self.last_reset_counters = None
        self.heading_offset = 0.0
        self.last_alignment_log = self.get_clock().now()

        self.publisher = self.create_publisher(DesiredState, self.output_topic, 20)
        self.create_subscription(PositionCommand, self.input_topic, self.on_command, 20)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        px4_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            VehicleLocalPosition, self.local_position_topic, self.on_local_position, px4_qos
        )

    def on_odom(self, msg):
        if msg.header.frame_id != self.planner_frame:
            return
        q = msg.pose.pose.orientation
        body_to_world = quaternion_matrix((q.x, q.y, q.z, q.w))
        if body_to_world is None:
            return
        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                      msg.pose.pose.position.z], dtype=float)
        v_body = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                           msg.twist.twist.linear.z], dtype=float)
        if not np.isfinite(p).all() or not np.isfinite(v_body).all():
            return
        self.odom = (p, body_to_world @ v_body)
        self.odom_received_at = self.get_clock().now()
        self.try_align()

    def on_local_position(self, msg):
        self.local_position = msg
        self.local_received_at = self.get_clock().now()
        counters = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        if self.last_reset_counters is not None and self.translation is not None:
            old_xy, old_z, old_heading = self.last_reset_counters
            if msg.xy_reset_counter != old_xy:
                self.translation[:2] += np.asarray(msg.delta_xy, dtype=float)
            if msg.z_reset_counter != old_z:
                self.translation[2] += float(msg.delta_z)
            if msg.heading_reset_counter != old_heading:
                self.heading_offset += float(msg.delta_heading)
        self.last_reset_counters = counters
        self.try_align()

    def local_position_valid(self):
        if self.local_position is None or self.local_received_at is None:
            return False
        if (self.get_clock().now() - self.local_received_at).nanoseconds > int(self.state_timeout * 1e9):
            return False
        p = self.local_position
        return (p.timestamp > 0 and p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid
                and not p.dead_reckoning)

    def try_align(self):
        if self.translation is not None or self.odom is None or not self.local_position_valid():
            return
        if (self.get_clock().now() - self.odom_received_at).nanoseconds > int(self.state_timeout * 1e9):
            return
        odom_position, odom_velocity = self.odom
        px4_velocity = np.array([self.local_position.vx, self.local_position.vy,
                                 self.local_position.vz], dtype=float)
        px4_position = np.array([self.local_position.x, self.local_position.y,
                                 self.local_position.z], dtype=float)
        if not np.isfinite(px4_position).all() or not np.isfinite(px4_velocity).all():
            return
        if (np.linalg.norm(odom_velocity) > self.stationary_speed
                or np.linalg.norm(px4_velocity) > self.stationary_speed):
            return
        self.translation = px4_position - self.rotation @ odom_position
        self.get_logger().info(
            "Aligned EGO frame to PX4 local NED at rest; translation="
            f"[{self.translation[0]:.2f}, {self.translation[1]:.2f}, {self.translation[2]:.2f}] m"
        )

    def on_command(self, command):
        if (command.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY
                or command.header.frame_id != self.planner_frame):
            return
        self.try_align()
        if self.translation is None or not self.local_position_valid():
            now = self.get_clock().now()
            if (now - self.last_alignment_log).nanoseconds > 5_000_000_000:
                self.get_logger().warning("Waiting for stationary FAST-LIO/PX4 frame alignment")
                self.last_alignment_log = now
            return

        p = np.array([command.position.x, command.position.y, command.position.z])
        v = np.array([command.velocity.x, command.velocity.y, command.velocity.z])
        a = np.array([command.acceleration.x, command.acceleration.y, command.acceleration.z])
        values = np.concatenate((p, v, a, [command.yaw, command.yaw_dot]))
        if not np.isfinite(values).all():
            return

        yaw = command.yaw
        heading = self.rotation @ np.array([math.cos(yaw), math.sin(yaw), 0.0])
        heading_next = self.rotation @ np.array([
            math.cos(yaw + 1e-4), math.sin(yaw + 1e-4), 0.0
        ])
        yaw_ned = math.atan2(heading[1], heading[0]) + self.heading_offset
        yaw_next = math.atan2(heading_next[1], heading_next[0]) + self.heading_offset
        yaw_rate = math.remainder(yaw_next - yaw_ned, 2.0 * math.pi) / 1e-4 * command.yaw_dot

        desired = DesiredState()
        desired.header = command.header
        desired.header.frame_id = self.reference_frame
        mapped_position = self.rotation @ p + self.translation
        mapped_velocity = self.rotation @ v
        mapped_acceleration = self.rotation @ a
        (desired.position.x, desired.position.y, desired.position.z) = mapped_position.tolist()
        (desired.velocity.x, desired.velocity.y, desired.velocity.z) = mapped_velocity.tolist()
        (desired.acceleration.x, desired.acceleration.y, desired.acceleration.z) = mapped_acceleration.tolist()
        desired.yaw = yaw_ned
        desired.yaw_rate = yaw_rate
        self.publisher.publish(desired)


def main():
    rclpy.init()
    node = EgoPositionCommandAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
