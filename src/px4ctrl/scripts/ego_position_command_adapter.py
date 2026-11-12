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

from px4ctrl.msg import DesiredState


class EgoPositionCommandAdapter(Node):
    def __init__(self):
        super().__init__('ego_position_command_adapter')
        self.declare_parameter('command_topic', '/drone_0_planning/pos_cmd')
        self.declare_parameter('odom_topic', '/fastlio2/lio_odom')
        self.declare_parameter('px4_local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('desired_state_topic', '/px4ctrl/desired_state')
        self.declare_parameter('status_topic', '/px4ctrl/reference_adapter_status')
        self.declare_parameter('source_frame', 'lidar')
        self.declare_parameter('target_frame', 'px4_local_ned')
        self.declare_parameter('alignment_stationary_speed_mps', 0.20)
        self.declare_parameter('state_timeout_sec', 0.50)

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
        self.heading_offset = 0.0
        self.last_reset_counters = None
        self.last_command_stamp_ns = 0

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
        values = (vector.x, vector.y, vector.z)
        return tuple(sum(self.rotation[row][col] * values[col] for col in range(3))
                     for row in range(3))

    def odom_callback(self, msg):
        expected = self.get_parameter('source_frame').value
        if msg.header.frame_id != expected:
            self.publish_status(f'frame_mismatch:{msg.header.frame_id}')
            return
        p = msg.pose.pose.position
        self.odom_position = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        rotated_velocity = self.rotate_body_velocity((v.x, v.y, v.z), (q.x, q.y, q.z, q.w))
        if rotated_velocity is None:
            return
        self.odom_velocity = rotated_velocity
        self.odom_received_at = self.get_clock().now()
        self.try_align()

    def local_position_callback(self, msg):
        counters = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        if self.translation is not None and self.last_reset_counters is not None:
            old_xy, old_z, old_heading = self.last_reset_counters
            if msg.xy_reset_counter != old_xy:
                self.translation = (self.translation[0] + msg.delta_xy[0],
                                    self.translation[1] + msg.delta_xy[1],
                                    self.translation[2])
            if msg.z_reset_counter != old_z:
                self.translation = (self.translation[0], self.translation[1],
                                    self.translation[2] + msg.delta_z)
            if msg.heading_reset_counter != old_heading:
                self.heading_offset += msg.delta_heading
        self.last_reset_counters = counters
        if not (msg.timestamp and msg.xy_valid and msg.z_valid and msg.v_xy_valid and
                msg.v_z_valid and not msg.dead_reckoning and
                math.isfinite(msg.x) and math.isfinite(msg.y) and math.isfinite(msg.z)):
            return
        self.px4_position = (msg.x, msg.y, msg.z)
        self.px4_velocity = (msg.vx, msg.vy, msg.vz)
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
        if (self.translation is not None or self.odom_position is None or
                self.px4_position is None or self.odom_velocity is None or
                self.px4_velocity is None or self.odom_received_at is None or
                self.px4_received_at is None):
            return
        now = self.get_clock().now()
        timeout = self.get_parameter('state_timeout_sec').value
        if ((now - self.odom_received_at).nanoseconds > timeout * 1e9 or
                (now - self.px4_received_at).nanoseconds > timeout * 1e9):
            return
        stationary = self.get_parameter('alignment_stationary_speed_mps').value
        if (math.sqrt(sum(v * v for v in self.odom_velocity)) > stationary or
                math.sqrt(sum(v * v for v in self.px4_velocity)) > stationary):
            return
        raw_ned = self.transform_vector_xyz(self.odom_position)
        self.translation = tuple(self.px4_position[i] - raw_ned[i] for i in range(3))
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
        self.last_command_stamp_ns = stamp_ns

        position = self.transform_vector(msg.position)
        velocity = self.transform_vector(msg.velocity)
        acceleration = self.transform_vector(msg.acceleration)
        position = tuple(position[i] + self.translation[i] for i in range(3))
        values = position + velocity + acceleration + (msg.yaw, msg.yaw_dot)
        if not all(math.isfinite(value) for value in values):
            return

        # EGO yaw is CCW about lidar +Z. The configured proper rotation maps
        # it to PX4 NED heading (clockwise from North): yaw_NED = pi/2 - yaw_lidar.
        yaw = math.atan2(math.cos(msg.yaw), math.sin(msg.yaw)) + self.heading_offset
        yaw_rate = -msg.yaw_dot
        out = DesiredState()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.get_parameter('target_frame').value
        out.position.x, out.position.y, out.position.z = position
        out.velocity.x, out.velocity.y, out.velocity.z = velocity
        out.acceleration.x, out.acceleration.y, out.acceleration.z = acceleration
        out.yaw = yaw
        out.yaw_rate = yaw_rate
        self.publisher.publish(out)
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
