#!/usr/bin/env python3
"""Send one EGO goal after the planner and controller interfaces are ready."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Empty, Float64, String


def parse_args():
    parser = argparse.ArgumentParser(description="Send a goal in the EGO planning frame")
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("yaw_deg", nargs="?", type=float)
    parser.add_argument("--auto-yaw", action="store_true")
    parser.add_argument("--frame", default="lidar")
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--odom-topic", default="/fastlio2/lio_odom")
    parser.add_argument("--command-topic", default="/drone_0_planning/pos_cmd")
    parser.add_argument("--manual-yaw-topic", default="/px4ctrl/manual_yaw")
    parser.add_argument("--clear-manual-yaw-topic", default="/px4ctrl/clear_manual_yaw")
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument("--publish-count", type=int, default=1)
    args = parser.parse_args()
    if args.auto_yaw == (args.yaw_deg is not None):
        parser.error("provide yaw_deg, or use --auto-yaw")
    values = [args.x, args.y, args.z]
    if args.yaw_deg is not None:
        values.append(args.yaw_deg)
    if not all(math.isfinite(value) for value in values):
        parser.error("coordinates and yaw must be finite")
    if not math.isfinite(args.wait_timeout) or args.wait_timeout <= 0.0 or args.publish_count < 1:
        parser.error("wait-timeout must be finite and positive; publish-count must be at least one")
    if any(not value.strip() for value in (
        args.frame, args.goal_topic, args.odom_topic, args.command_topic,
        args.manual_yaw_topic, args.clear_manual_yaw_topic,
    )):
        parser.error("frame and topic names must not be empty")
    return args


class GoalSender(Node):
    def __init__(self, args):
        super().__init__("px4ctrl_manual_goal")
        self.args = args
        self.goal_pub = self.create_publisher(PoseStamped, args.goal_topic, 10)
        self.command_pub = self.create_publisher(PositionCommand, args.command_topic, 10)
        self.yaw_pub = self.create_publisher(Float64, args.manual_yaw_topic, 10)
        self.clear_yaw_pub = self.create_publisher(Empty, args.clear_manual_yaw_topic, 10)
        self.controller_status = ""
        self.create_subscription(
            String, "/px4ctrl/status", self.controller_status_callback, 10
        )
        self.px4_status = None
        self.local_position = None
        self.local_position_time = None
        self.reset_signature = None
        self.reset_stable_since = None
        px4_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.px4_status_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.local_position_callback,
            px4_qos,
        )
        self.latest_odom = None
        self.latest_odom_time = None
        self.odom_samples = 0
        self.create_subscription(Odometry, args.odom_topic, self.odom_callback, 10)

    def controller_status_callback(self, msg):
        self.controller_status = msg.data

    def px4_status_callback(self, msg):
        self.px4_status = msg

    def local_position_callback(self, msg):
        signature = (
            msg.xy_reset_counter,
            msg.z_reset_counter,
            msg.heading_reset_counter,
        )
        now = time.monotonic()
        if signature != self.reset_signature:
            self.reset_signature = signature
            self.reset_stable_since = now
        self.local_position = msg
        self.local_position_time = now

    def odom_callback(self, msg):
        if msg.header.frame_id != self.args.frame:
            self.odom_samples = 0
            return
        values = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.odom_samples = 0
            return
        self.odom_samples += 1
        self.latest_odom = msg
        self.latest_odom_time = time.monotonic()

    def wait_for_odometry(self):
        deadline = time.monotonic() + self.args.wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self.odom_samples >= 3
                and self.latest_odom_time is not None
                and time.monotonic() - self.latest_odom_time < 0.5
            ):
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(
            f"no fresh odometry in frame '{self.args.frame}' on {self.args.odom_topic}"
        )

    def wait_for_controller_ready(self):
        deadline = time.monotonic() + self.args.wait_timeout
        command = PositionCommand()
        command.header.frame_id = self.args.frame
        command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        odom = self.latest_odom
        command.position.x = odom.pose.pose.position.x
        command.position.y = odom.pose.pose.position.y
        command.position.z = odom.pose.pose.position.z
        command.yaw = 0.0
        command.yaw_dot = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            status = self.controller_status
            local_position_ready = (
                self.local_position is not None
                and self.local_position_time is not None
                and time.monotonic() - self.local_position_time < 0.5
                and self.local_position.xy_valid
                and self.local_position.z_valid
                and self.local_position.v_xy_valid
                and self.local_position.v_z_valid
                and not self.local_position.dead_reckoning
            )
            px4_flight_ready = (
                self.px4_status is not None
                and self.px4_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and self.px4_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            )
            estimator_settled = (
                local_position_ready
                and self.reset_stable_since is not None
                and time.monotonic() - self.reset_stable_since >= 2.0
                and math.sqrt(
                    self.local_position.vx ** 2
                    + self.local_position.vy ** 2
                    + self.local_position.vz ** 2
                ) <= 0.20
            )
            odometry_settled = (
                self.latest_odom is not None
                and self.latest_odom_time is not None
                and time.monotonic() - self.latest_odom_time < 0.5
                and math.sqrt(
                    self.latest_odom.twist.twist.linear.x ** 2
                    + self.latest_odom.twist.twist.linear.y ** 2
                    + self.latest_odom.twist.twist.linear.z ** 2
                ) <= 0.20
            )
            if (
                status.startswith(("hover_", "tracking_"))
                and estimator_settled
                and odometry_settled
                and px4_flight_ready
            ):
                print(f"PX4Ctrl ready: {status}")
                return
            if status.startswith("safety_fault"):
                raise RuntimeError(f"PX4Ctrl is faulted: {status}")
            if not status.startswith(("takeoff_", "hover_", "tracking_")):
                if self.command_pub.get_subscription_count() > 0:
                    for _ in range(5):
                        odom = self.latest_odom
                        if odom is None or self.latest_odom_time is None or \
                                time.monotonic() - self.latest_odom_time >= 0.5:
                            break
                        command.header.stamp = self.get_clock().now().to_msg()
                        command.position.x = odom.pose.pose.position.x
                        command.position.y = odom.pose.pose.position.y
                        command.position.z = odom.pose.pose.position.z
                        self.command_pub.publish(command)
                        rclpy.spin_once(self, timeout_sec=0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(
            f"PX4Ctrl did not reach hover/tracking before timeout; last status: "
            f"{self.controller_status or 'no status'}"
        )

    def wait_for_goal_subscriber(self):
        deadline = time.monotonic() + self.args.wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.goal_pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"no subscriber discovered for {self.args.goal_topic}")

    def wait_for_control_subscriber(self, publisher, topic):
        deadline = time.monotonic() + self.args.wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"no subscriber discovered for {topic}")

    def send(self):
        if self.args.auto_yaw:
            self.wait_for_control_subscriber(
                self.clear_yaw_pub, self.args.clear_manual_yaw_topic
            )
            heading = "EGO path heading"
        else:
            self.wait_for_control_subscriber(self.yaw_pub, self.args.manual_yaw_topic)
            yaw = Float64()
            yaw.data = math.radians(self.args.yaw_deg)
            heading = f"{self.args.yaw_deg:g} deg"

        goal = PoseStamped()
        goal.header.frame_id = self.args.frame
        goal.pose.position.x = self.args.x
        goal.pose.position.y = self.args.y
        goal.pose.position.z = self.args.z
        yaw_rad = 0.0 if self.args.auto_yaw else math.radians(self.args.yaw_deg)
        goal.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.orientation.w = math.cos(yaw_rad / 2.0)
        for _ in range(self.args.publish_count):
            if self.args.auto_yaw:
                self.clear_yaw_pub.publish(Empty())
            else:
                self.yaw_pub.publish(yaw)
            goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(goal)
            rclpy.spin_once(self, timeout_sec=0.1)
        print(
            f"Sent goal ({self.args.x:g}, {self.args.y:g}, {self.args.z:g}) m "
            f"in {self.args.frame}; heading: {heading}."
        )


def main():
    args = parse_args()
    rclpy.init()
    node = GoalSender(args)
    try:
        node.wait_for_odometry()
        node.wait_for_goal_subscriber()
        node.wait_for_control_subscriber(
            node.command_pub, args.command_topic
        )
        node.wait_for_controller_ready()
        node.send()
    except (RuntimeError, KeyboardInterrupt) as exc:
        if str(exc):
            print(f"manual_goal: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
