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
        self.confirmed = False
        self.status = None
        self.local = None
        self.land_detected = None
        self.status_received = 0.0
        self.land_received = 0.0
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
        self.last_disarm_send = 0.0
        self.last_diagnostic = 0.0
        self.create_timer(0.2, self.tick)

    def on_status(self, msg):
        self.status = msg
        self.status_received = time.monotonic()
        self.timestamp = max(self.timestamp, int(msg.timestamp))

    def on_attitude(self, msg):
        self.timestamp = max(self.timestamp, int(msg.timestamp))

    def on_land(self, msg):
        self.land_detected = msg
        self.land_received = time.monotonic()

    def on_local(self, msg):
        self.local = msg
        self.timestamp = max(self.timestamp, int(msg.timestamp))

    def tick(self):
        if self.timestamp == 0:
            return
        now_wall = time.monotonic()
        status_fresh = self.status is not None and now_wall - self.status_received < 3.0
        if not status_fresh:
            return
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        disarmed = self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
        if armed and self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            if now_wall - self.last_send >= 1.0:
                self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.last_send = now_wall

        local = self.local
        land_fresh = self.land_detected is not None and now_wall - self.land_received < 3.0
        landed = land_fresh and self.land_detected.landed
        land_close = land_fresh and self.land_detected.close_to_ground_or_skipped_check
        if landed and disarmed:
            self.get_logger().info("PX4 reported landed and disarmed")
            self.confirmed = True
            return
        if landed and armed and now_wall - self.last_disarm_send >= 1.0:
            self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
            self.last_disarm_send = now_wall

        if now_wall - self.last_diagnostic >= 5.0:
            if local is None:
                local_diag = "local_position=missing"
            else:
                local_diag = (
                    f"dist_bottom={local.dist_bottom:.3f}m "
                    f"dist_valid={local.dist_bottom_valid} "
                    f"velocity=({local.vx:.3f},{local.vy:.3f},{local.vz:.3f})m/s "
                    f"local_valid={local.xy_valid and local.z_valid and local.v_xy_valid and local.v_z_valid}")
            self.get_logger().info(
                f"landing status armed={armed} nav_state="
                f"{self.status.nav_state if self.status else 'missing'} "
                f"landed={landed} close_to_ground={land_close}; {local_diag}")
            self.last_diagnostic = now_wall

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



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()
    rclpy.init()
    node = LandRequester()
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.confirmed and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.confirmed:
            local = node.local
            land = node.land_detected
            status = node.status
            raise TimeoutError(
                "PX4 did not confirm landed and disarmed before timeout; "
                f"armed={bool(status and status.arming_state != VehicleStatus.ARMING_STATE_DISARMED)}, "
                f"nav_state={status.nav_state if status else 'missing'}, "
                f"landed={bool(land and land.landed)}, "
                f"close_to_ground={bool(land and land.close_to_ground_or_skipped_check)}, "
                f"dist_bottom={local.dist_bottom if local else 'missing'}, "
                f"dist_bottom_valid={local.dist_bottom_valid if local else 'missing'}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
