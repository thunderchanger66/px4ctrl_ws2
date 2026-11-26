#!/usr/bin/env python3
"""Convert EGO's lidar-frame PositionCommand into PX4 local-NED DesiredState."""

import math

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from controller.msg import DesiredState


class EgoPositionCommandAdapter(Node):
    def __init__(self):
        super().__init__('ego_position_command_adapter')
        self.declare_parameter('command_topic', '/drone_0_planning/pos_cmd')
        self.declare_parameter('odom_topic', '/fastlio2/lio_odom')
        self.declare_parameter('px4_local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('desired_state_topic', '/controller/desired_state')
        self.declare_parameter('status_topic', '/controller/reference_adapter_status')
        self.declare_parameter('source_frame', 'lidar')
        self.declare_parameter('target_frame', 'px4_local_ned')
        self.declare_parameter('alignment_stationary_speed_mps', 0.20)
        self.declare_parameter('state_timeout_sec', 0.50)
        self.declare_parameter('max_reference_acceleration_mps2', 3.0)
        self.declare_parameter('max_reference_jerk_mps3', 30.0)
        self.declare_parameter('acceleration_preview_sec', 0.0)

        self.rotation = ((0.0, 1.0, 0.0),
                         (1.0, 0.0, 0.0),
                         (0.0, 0.0, -1.0))
        self.odom_position = None
        self.odom_velocity = None
        self.odom_received_at = None
        self.px4_position = None
        self.px4_velocity = None
        self.px4_received_at = None
        self.translation = None
        self.alignment_diag_counter = 0
        self.last_reset_counters = None
        self.last_px4_stamp = 0
        self.last_command_stamp_ns = 0
        self.last_reference_acceleration = None
        self.last_reference_acceleration_stamp_ns = None
        self.last_raw_reference_acceleration = None
        self.last_raw_reference_acceleration_stamp_ns = None
        self.filtered_reference_jerk = None

        self.publisher = self.create_publisher(
            DesiredState, self.get_parameter('desired_state_topic').value, 20)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 20)
        px4_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            VehicleLocalPosition,
            self.get_parameter('px4_local_position_topic').value,
            self.local_position_callback, px4_qos)
        self.create_subscription(
            PositionCommand,
            self.get_parameter('command_topic').value,
            self.command_callback, 20)
        self.get_logger().info(
            'Waiting for FAST-LIO odometry and valid PX4 local position to align lidar origin')

    def transform_vector(self, vector):
        # Explicit Python floats are required by generated ROS message setters.
        values = tuple(float(value) for value in (vector.x, vector.y, vector.z))
        return tuple(float(sum(self.rotation[row][col] * values[col] for col in range(3)))
                     for row in range(3))

    def odom_callback(self, msg):
        expected = self.get_parameter('source_frame').value
        if msg.header.frame_id != expected:
            self.publish_status(f'frame_mismatch:{msg.header.frame_id}')
            return
        p = msg.pose.pose.position
        position = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        rotated_velocity = self.rotate_body_velocity((v.x, v.y, v.z), (q.x, q.y, q.z, q.w))
        if (rotated_velocity is None or
                not all(math.isfinite(value) for value in position + rotated_velocity)):
            return
        self.odom_position = position
        self.odom_velocity = rotated_velocity
        self.odom_received_at = self.get_clock().now()
        self.try_align()

    def local_position_callback(self, msg):
        if msg.timestamp <= self.last_px4_stamp:
            return
        self.last_px4_stamp = msg.timestamp
        counters = (msg.xy_reset_counter, msg.z_reset_counter)
        if self.translation is not None and self.last_reset_counters is not None:
            old_xy, old_z = self.last_reset_counters
            xy = tuple(float(value) for value in msg.delta_xy) \
                if msg.xy_reset_counter != old_xy else (0.0, 0.0)
            delta = (*xy, float(msg.delta_z) if msg.z_reset_counter != old_z else 0.0)
            if all(math.isfinite(value) for value in delta):
                self.translation = tuple(float(a) + float(b)
                                         for a, b in zip(self.translation, delta))
            else:
                self.translation = None
                self.px4_position = None
                self.px4_received_at = None
                self.get_logger().warning('Invalid PX4 origin reset; waiting to realign')
        self.last_reset_counters = counters
        if not (msg.timestamp and msg.xy_valid and msg.z_valid and msg.v_xy_valid and
                msg.v_z_valid and not msg.dead_reckoning and
                all(math.isfinite(v) for v in (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz))):
            self.px4_received_at = None
            return
        self.px4_position = tuple(float(v) for v in (msg.x, msg.y, msg.z))
        self.px4_velocity = tuple(float(v) for v in (msg.vx, msg.vy, msg.vz))
        self.px4_received_at = self.get_clock().now()
        self.try_align()

    @staticmethod
    def rotate_body_velocity(velocity, quaternion):
        x, y, z, w = quaternion
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not math.isfinite(norm) or norm < 1e-6:
            return None
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        vx, vy, vz = velocity
        # Quaternion rotation, expanded as R(q) * v.
        return (
            (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy + 2 * (x * z + y * w) * vz,
            2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - x * w) * vz,
            2 * (x * z - y * w) * vx + 2 * (y * z + x * w) * vy + (1 - 2 * (x * x + y * y)) * vz,
        )

    def try_align(self):
        if self.translation is not None:
            return
        if (self.odom_position is None or self.px4_position is None or
                self.odom_velocity is None or self.px4_velocity is None or
                self.odom_received_at is None or self.px4_received_at is None):
            self.alignment_diag_counter += 1
            if self.alignment_diag_counter % 200 == 1:
                self.get_logger().warning(
                    'Origin alignment inputs missing: '
                    f'FAST-LIO position={self.odom_position is not None}, '
                    f'PX4 valid local position={self.px4_position is not None}')
            return
        now = self.get_clock().now()
        timeout = self.get_parameter('state_timeout_sec').value
        odom_age = (now - self.odom_received_at).nanoseconds * 1e-9
        px4_age = (now - self.px4_received_at).nanoseconds * 1e-9
        if odom_age > timeout or px4_age > timeout:
            self.alignment_diag_counter += 1
            if self.alignment_diag_counter % 200 == 1:
                self.get_logger().warning(
                    f'Origin alignment inputs stale: FAST-LIO age={odom_age:.3f}s, '
                    f'PX4 age={px4_age:.3f}s')
            return
        stationary = self.get_parameter('alignment_stationary_speed_mps').value
        odom_speed = math.sqrt(sum(v * v for v in self.odom_velocity))
        px4_speed = math.sqrt(sum(v * v for v in self.px4_velocity))
        if odom_speed > stationary or px4_speed > stationary:
            self.alignment_diag_counter += 1
            if self.alignment_diag_counter % 200 == 1:
                self.get_logger().warning(
                    f'Origin alignment not stationary: FAST-LIO {odom_speed:.3f} m/s, '
                    f'PX4 {px4_speed:.3f} m/s (limit {stationary:.3f})')
            return
        raw_ned = self.transform_vector_xyz(self.odom_position)
        self.translation = tuple(float(self.px4_position[i]) - float(raw_ned[i])
                                 for i in range(3))
        self.alignment_diag_counter = 0
        self.get_logger().info(
            'Latched lidar-to-PX4 origin translation NED '
            f'[{self.translation[0]:.3f}, {self.translation[1]:.3f}, '
            f'{self.translation[2]:.3f}] m')
        self.publish_status('aligned_waiting_for_ego_command')

    def transform_vector_xyz(self, values):
        return tuple(sum(self.rotation[row][col] * values[col] for col in range(3))
                     for row in range(3))

    def command_callback(self, msg):
        if msg.header.frame_id != self.get_parameter('source_frame').value:
            self.publish_status(f'command_frame_mismatch:{msg.header.frame_id}')
            return
        if msg.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY:
            return
        if self.translation is None:
            self.publish_status('waiting_for_origin_alignment')
            return
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        if stamp_ns <= self.last_command_stamp_ns:
            return

        position = self.transform_vector(msg.position)
        velocity = self.transform_vector(msg.velocity)
        acceleration = self.transform_vector(msg.acceleration)
        acceleration_limit = float(self.get_parameter(
            'max_reference_acceleration_mps2').value)
        jerk_limit = float(self.get_parameter('max_reference_jerk_mps3').value)
        preview_sec = float(self.get_parameter('acceleration_preview_sec').value)
        if not (math.isfinite(acceleration_limit) and acceleration_limit > 0.0 and
                math.isfinite(jerk_limit) and jerk_limit > 0.0 and
                math.isfinite(preview_sec) and 0.0 <= preview_sec <= 0.5):
            self.publish_status('invalid_reference_acceleration_limits_or_preview')
            return

        # Optional acceleration preview approximates a short look-ahead from
        # the bounded derivative of the EGO acceleration command. It is off by
        # default and clamped by the same reference jerk/acceleration limits.
        raw_acceleration = tuple(float(value) for value in acceleration)
        if (preview_sec > 0.0 and self.last_raw_reference_acceleration is not None and
                self.last_raw_reference_acceleration_stamp_ns is not None):
            dt = (stamp_ns - self.last_raw_reference_acceleration_stamp_ns) * 1e-9
            if 0.005 <= dt <= 0.2:
                jerk = tuple((raw_acceleration[i] - self.last_raw_reference_acceleration[i]) / dt
                             for i in range(3))
                jerk_norm = math.sqrt(sum(value * value for value in jerk))
                if jerk_norm > jerk_limit and jerk_norm > 1e-9:
                    jerk = tuple(value * jerk_limit / jerk_norm for value in jerk)
                if self.filtered_reference_jerk is None:
                    self.filtered_reference_jerk = jerk
                else:
                    self.filtered_reference_jerk = tuple(
                        0.5 * old + 0.5 * new
                        for old, new in zip(self.filtered_reference_jerk, jerk))
                acceleration = tuple(
                    raw_acceleration[i] + preview_sec * self.filtered_reference_jerk[i]
                    for i in range(3))
        self.last_raw_reference_acceleration = raw_acceleration
        self.last_raw_reference_acceleration_stamp_ns = stamp_ns
        acceleration_norm = math.sqrt(sum(value * value for value in acceleration))
        if acceleration_norm > acceleration_limit:
            scale = acceleration_limit / acceleration_norm
            acceleration = tuple(float(value * scale) for value in acceleration)
        if self.last_reference_acceleration is not None and \
                self.last_reference_acceleration_stamp_ns is not None:
            dt = (stamp_ns - self.last_reference_acceleration_stamp_ns) * 1e-9
            if 0.0 < dt <= 0.2:
                delta = tuple(acceleration[i] - self.last_reference_acceleration[i]
                              for i in range(3))
                delta_norm = math.sqrt(sum(value * value for value in delta))
                max_delta = jerk_limit * dt
                if delta_norm > max_delta and delta_norm > 1e-9:
                    scale = max_delta / delta_norm
                    acceleration = tuple(float(self.last_reference_acceleration[i] +
                                                delta[i] * scale) for i in range(3))
        self.last_reference_acceleration = tuple(float(value) for value in acceleration)
        self.last_reference_acceleration_stamp_ns = stamp_ns
        position = tuple(float(position[i] + self.translation[i]) for i in range(3))
        values = position + velocity + acceleration + (msg.yaw, msg.yaw_dot)
        if not all(math.isfinite(value) for value in values):
            return

        # EGO yaw is CCW about lidar +Z. The configured proper rotation maps
        # it to PX4 NED heading (clockwise from North): yaw_NED = pi/2 - yaw_lidar.
        # The controller owns PX4 heading-reset compensation for every reference source.
        yaw = math.atan2(math.cos(msg.yaw), math.sin(msg.yaw))
        yaw_rate = -msg.yaw_dot
        out = DesiredState()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.get_parameter('target_frame').value
        out.position.x, out.position.y, out.position.z = (float(v) for v in position)
        out.velocity.x, out.velocity.y, out.velocity.z = (float(v) for v in velocity)
        out.acceleration.x, out.acceleration.y, out.acceleration.z = (
            float(v) for v in acceleration)
        out.yaw = float(yaw)
        out.yaw_rate = float(yaw_rate)
        self.publisher.publish(out)
        self.last_command_stamp_ns = stamp_ns
        self.publish_status('publishing_px4_local_ned_reference')

    def publish_status(self, value):
        msg = String()
        msg.data = value
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EgoPositionCommandAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
