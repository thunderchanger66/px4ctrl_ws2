#!/usr/bin/env python3
"""Request PX4 SITL landing and wait for landed + disarmed confirmation."""

import argparse
import time

import rclpy
from px4_msgs.msg import (
    VehicleAttitude, VehicleCommand, VehicleLandDetected, VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class LandRequester(Node):
    def __init__(self):
        super().__init__("px4ctrl_benchmark_lander", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.timestamp = 0
        self.status = None
        self.local = None
        self.land_detected = None
        self.stable_ground_since = None
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4",
                                 self.on_status, qos)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude",
                                 self.on_attitude, qos)
        self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected",
                                 self.on_land, qos)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                 self.on_local, qos)
        self.publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.last_send = 0.0
        self.create_timer(0.2, self.tick)

    def on_status(self, msg):
        self.status = msg
        self.timestamp = max(self.timestamp, int(msg.timestamp))

    def on_attitude(self, msg):
        self.timestamp = max(self.timestamp, int(msg.timestamp))

    def on_land(self, msg):
        self.land_detected = msg

    def on_local(self, msg):
        self.local = msg
        self.timestamp = max(self.timestamp, int(msg.timestamp))

    def tick(self):
        if self.timestamp == 0:
            return
        armed = self.status and self.status.arming_state != VehicleStatus.ARMING_STATE_DISARMED
        if armed and self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            if time.monotonic() - self.last_send >= 1.0:
                self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.last_send = time.monotonic()
        local = self.local
        near_ground_and_still = bool(
            local and local.timestamp > 0 and
            (local.dist_bottom_valid or (self.land_detected and
                                         self.land_detected.close_to_ground_or_skipped_check)) and
            0.0 <= local.dist_bottom <= 0.10 and abs(local.vz) <= 0.05 and
            math_hypot(local.vx, local.vy) <= 0.10 and
            local.xy_valid and local.z_valid and local.v_xy_valid and local.v_z_valid
        )
        if near_ground_and_still:
            if self.stable_ground_since is None:
                self.stable_ground_since = time.monotonic()
        else:
            self.stable_ground_since = None
        ground_confirmed = (self.stable_ground_since is not None and
                            time.monotonic() - self.stable_ground_since >= 3.0)
        landed = self.land_detected is not None and self.land_detected.landed
        if landed or ground_confirmed:
            if armed and ground_confirmed and not landed:
                self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                                     param1=0.0, param2=0.0)
                return
            if not armed:
                self.get_logger().info(
                    "PX4 is disarmed; land flag or near-ground/stationary condition confirmed")
                rclpy.shutdown()

    def publish_command(self, command_id, param1=0.0, param2=0.0):
        command = VehicleCommand()
        command.timestamp = self.timestamp
        command.command = command_id
        command.param1 = param1
        command.param2 = param2
        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 1
        command.from_external = True
        self.publisher.publish(command)


def math_hypot(x, y):
    return (x * x + y * y) ** 0.5


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()
    rclpy.init()
    node = LandRequester()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if rclpy.ok():
            raise TimeoutError("PX4 did not confirm landed and disarmed before timeout")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
