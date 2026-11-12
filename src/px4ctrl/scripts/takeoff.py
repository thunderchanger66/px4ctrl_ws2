#!/usr/bin/env python3
"""Take off and settle in PX4 local NED before starting planner navigation."""

import argparse
import math
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from px4ctrl.msg import DesiredState


class Takeoff(Node):
    def __init__(self, altitude, velocity_tolerance):
        super().__init__('px4ctrl_takeoff', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.altitude = altitude
        self.velocity_tolerance = velocity_tolerance
        self.local_position = None
        self.takeoff_reference = None
        self.status = 'waiting_for_controller'
        self.stable_since = None
        self.publisher = self.create_publisher(DesiredState, '/px4ctrl/desired_state', 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String, '/px4ctrl/status', self.status_callback, 10)
        self.timer = self.create_timer(0.05, self.publish_reference)

    def position_callback(self, msg):
        values = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.heading)
        if (msg.timestamp and msg.xy_valid and msg.z_valid and msg.v_xy_valid and
                msg.v_z_valid and not msg.dead_reckoning and
                all(math.isfinite(value) for value in values)):
            self.local_position = msg
            if self.takeoff_reference is None:
                self.takeoff_reference = (msg.x, msg.y, msg.z - self.altitude, msg.heading)

    def status_callback(self, msg):
        self.status = msg.data

    def publish_reference(self):
        p = self.local_position
        reference = self.takeoff_reference
        if p is None or reference is None:
            return
        msg = DesiredState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'px4_local_ned'
        msg.position.x, msg.position.y, msg.position.z, msg.yaw = reference
        self.publisher.publish(msg)

        altitude_reached = p.z <= reference[2] + 0.08
        speed = math.sqrt(p.vx * p.vx + p.vy * p.vy + p.vz * p.vz)
        controller_tracking = self.status == 'state_tracking' or self.status.startswith('tracking_')
        if altitude_reached and speed <= self.velocity_tolerance and controller_tracking:
            if self.stable_since is None:
                self.stable_since = time.monotonic()
        else:
            self.stable_since = None


def main(args=None):
    parser = argparse.ArgumentParser(description='Take off before sending an EGO planner goal')
    parser.add_argument('--altitude', type=float, default=0.5,
                        help='climb distance in metres (default: 0.5)')
    parser.add_argument('--velocity-tolerance', type=float, default=0.15,
                        help='settling speed limit in m/s (default: 0.15)')
    parser.add_argument('--wait-timeout', type=float, default=45.0,
                        help='wall-clock timeout in seconds (default: 45)')
    options = parser.parse_args(args)
    if (not all(math.isfinite(value) for value in
                (options.altitude, options.velocity_tolerance, options.wait_timeout)) or
            options.altitude <= 0.0 or options.velocity_tolerance <= 0.0 or
            options.wait_timeout <= 0.0):
        parser.error('altitude, velocity tolerance, and timeout must be finite and positive')

    rclpy.init()
    node = Takeoff(options.altitude, options.velocity_tolerance)
    deadline = time.monotonic() + options.wait_timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.stable_since is not None and time.monotonic() - node.stable_since >= 0.8:
                print(f'PX4Ctrl takeoff complete: climbed {options.altitude:g} m and settled.')
                return 0
        raise RuntimeError(
            f'takeoff did not settle before timeout; last px4ctrl status: {node.status}')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f'takeoff: {exc}')
        raise SystemExit(1)
