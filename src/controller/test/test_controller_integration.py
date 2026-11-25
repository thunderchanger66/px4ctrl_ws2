"""Exercise the real controller process with isolated ROS 2 topic traffic."""

import math
import os
from pathlib import Path
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from px4_msgs.msg import (
    OffboardControlMode, VehicleAttitudeSetpoint, VehicleLandDetected,
    VehicleLocalPosition, VehicleStatus,
)
from controller.msg import ControlDebug, DesiredState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


@pytest.fixture
def ros_graph():
    old_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    rclpy.init()
    node = Node("px4ctrl_test_driver")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        yield node, executor
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        if old_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = old_domain


def _start(executable, params):
    binary = Path(get_package_prefix("controller")) / "lib" / "controller" / executable
    assert binary.exists(), binary
    command = [str(binary), "--ros-args"]
    for name, value in params.items():
        command.extend(("-p", f"{name}:={value}"))
    process = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                               stderr=subprocess.PIPE, text=True)
    return process


def _stop(process):
    process.terminate()
    try:
        _, errors = process.communicate(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        _, errors = process.communicate(timeout=3)
    assert process.returncode in (-15, 0), errors


def _wait(executor, condition, publisher=None, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if publisher is not None:
            publisher()
        executor.spin_once(timeout_sec=0.04)
        if condition():
            return
    raise AssertionError("ROS condition did not become true before timeout")


SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class Harness:
    """A controller process plus a fake PX4 that it can talk to.

    The controller is configured for `airborne_takeover`, so it never arms,
    never switches the mode and never publishes a setpoint until a takeover
    permission arrives and the fake PX4 reports an airborne vehicle.

    No odometry topic is published here on purpose: the controller must reach
    the airborne hold using PX4 feedback alone.
    """

    def __init__(self, node, executor, suffix):
        self.node = node
        self.executor = executor
        root = f"/px4ctrl_test_{suffix}"
        self.topics = {
            "desired_state_topic": f"{root}/desired",
            "px4_status_topic": f"{root}/vehicle_status",
            "px4_local_position_topic": f"{root}/local_position",
            "px4_land_detected_topic": f"{root}/land_detected",
            "px4_attitude_topic": f"{root}/attitude",
            "offboard_control_mode_topic": f"{root}/offboard_mode",
            "attitude_setpoint_topic": f"{root}/attitude_setpoint",
            "vehicle_command_topic": f"{root}/vehicle_command",
            "status_topic": f"{root}/status",
            "debug_topic": f"{root}/debug",
            "takeover_permission_topic": f"{root}/takeover_allowed",
        }
        self.params = {
            **self.topics,
            "use_sim_time": "false",
            "entry_mode": "airborne_takeover",
            "require_takeover_permission": "true",
            "reference_frame": "px4_local_ned",
            "start_offboard_on_first_command": "false",
            "arm_on_first_command": "false",
            "auto_takeoff_altitude_m": "0.0",
            "minimum_takeoff_altitude_m": "0.0",
            "minimum_tracking_altitude_m": "0.0",
            "prestream_seconds": "0.15",
            "hover_settle_sec": "0.1",
            "publish_rate_hz": "50.0",
            "hover_thrust": "0.6",
        }
        self.statuses = []
        self.modes = []
        self.setpoints = []
        self.diagnostics = []
        self.flight = {"armed": False, "offboard": False, "landed": True}
        self._last_land = {"time": 0.0, "landed": None}

        self.status_pub = node.create_publisher(
            VehicleStatus, self.topics["px4_status_topic"], SENSOR_QOS)
        self.local_pub = node.create_publisher(
            VehicleLocalPosition, self.topics["px4_local_position_topic"], SENSOR_QOS)
        self.land_pub = node.create_publisher(
            VehicleLandDetected, self.topics["px4_land_detected_topic"], SENSOR_QOS)
        self.command_pub = node.create_publisher(
            DesiredState, self.topics["desired_state_topic"], 20)
        self.permission_pub = node.create_publisher(
            Bool, self.topics["takeover_permission_topic"], 10)
        node.create_subscription(String, self.topics["status_topic"],
                                 lambda msg: self.statuses.append(msg.data), 10)
        node.create_subscription(OffboardControlMode, self.topics["offboard_control_mode_topic"],
                                 lambda msg: self.modes.append(msg), SENSOR_QOS)
        node.create_subscription(VehicleAttitudeSetpoint, self.topics["attitude_setpoint_topic"],
                                 lambda msg: self.setpoints.append(msg), SENSOR_QOS)
        node.create_subscription(ControlDebug, self.topics["debug_topic"],
                                 lambda msg: self.diagnostics.append(msg), 10)

    def publish_state(self):
        stamp = time.monotonic_ns() // 1000
        status = VehicleStatus()
        status.timestamp = stamp
        status.arming_state = (VehicleStatus.ARMING_STATE_ARMED if self.flight["armed"]
                               else VehicleStatus.ARMING_STATE_DISARMED)
        status.nav_state = (VehicleStatus.NAVIGATION_STATE_OFFBOARD if self.flight["offboard"]
                            else VehicleStatus.NAVIGATION_STATE_POSCTL)
        self.status_pub.publish(status)
        position = VehicleLocalPosition()
        position.timestamp = stamp
        position.xy_valid = position.z_valid = True
        position.v_xy_valid = position.v_z_valid = True
        position.x, position.y, position.z = 1.0, -2.0, -2.0
        position.heading = 0.0
        self.local_pub.publish(position)
        now = time.monotonic()
        last_land = self._last_land
        if now - last_land["time"] >= 1.0 or last_land["landed"] != self.flight["landed"]:
            land = VehicleLandDetected()
            land.timestamp = stamp
            land.landed = self.flight["landed"]
            land.ground_contact = self.flight["landed"]
            land.maybe_landed = self.flight["landed"]
            self.land_pub.publish(land)
            last_land.update(time=now, landed=self.flight["landed"])

    def publish_permission(self):
        self.publish_state()
        self.permission_pub.publish(Bool(data=True))

    def publish_command(self, frame_id="px4_local_ned", x=1.2, y=-2.0, z=-2.0):
        self.publish_state()
        desired = DesiredState()
        desired.header.stamp = self.node.get_clock().now().to_msg()
        desired.header.frame_id = frame_id
        desired.position.x, desired.position.y, desired.position.z = x, y, z
        self.command_pub.publish(desired)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.publish_state()
            self.executor.spin_once(timeout_sec=0.04)

    def states(self):
        return {item.state for item in self.diagnostics}

    def take_off(self):
        """Bring the fake PX4 and the controller to the airborne hold state."""
        _wait(self.executor,
              lambda: self.status_pub.get_subscription_count() > 0 and
              self.local_pub.get_subscription_count() > 0, self.publish_state)
        assert not self.setpoints, "disarmed state must not produce attitude setpoints"

        self.flight["armed"] = True
        self.spin(0.3)
        assert not self.setpoints, "armed but landed state must not capture an airborne hold"

        self.flight["landed"] = False
        self.flight["offboard"] = True
        self.spin(1.2)
        assert not self.setpoints, "controller must not compete with position takeoff before permission"
        assert not self.modes, "controller must not replace the position Offboard mode before permission"

        _wait(self.executor,
              lambda: "hover_airborne_takeover_offboard_ready" in self.statuses and
              len(self.setpoints) >= 3, self.publish_permission)
        assert math.isclose(-self.setpoints[-1].thrust_body[2], 0.6, abs_tol=0.02)
        assert self.modes and self.modes[-1].attitude


def test_airborne_takeover_and_offboard_loss(ros_graph):
    node, executor = ros_graph
    harness = Harness(node, executor, uuid.uuid4().hex[:8])
    process = _start("px4ctrl_node", harness.params)
    try:
        harness.take_off()

        harness.publish_command("px4_local_ned")
        _wait(executor, lambda: any(
            item.state == "tracking" and item.position_error.x > 0.05
            for item in harness.diagnostics), harness.publish_command)
        assert harness.setpoints[-1].timestamp > 0

        harness.flight["offboard"] = False
        _wait(executor, lambda: "control_suspended_offboard_lost" in harness.statuses,
              harness.publish_state)
        harness.setpoints.clear()
        harness.spin(0.3)
        assert not harness.setpoints, "controller must stop publishing after Offboard loss"
    finally:
        _stop(process)


def test_reference_frame_contract(ros_graph):
    """Only a reference declared in the configured NED frame may be applied."""
    node, executor = ros_graph
    harness = Harness(node, executor, uuid.uuid4().hex[:8])
    process = _start("px4ctrl_node", harness.params)
    try:
        harness.take_off()
        assert "tracking" not in harness.states()

        # A reference in any other frame must be rejected outright.
        for _ in range(10):
            harness.publish_command("lidar")
            executor.spin_once(timeout_sec=0.04)
        assert "tracking" not in harness.states(), \
            "a reference outside the configured NED frame must not be tracked"

        # The same command in the configured frame must be accepted.
        _wait(executor, lambda: "tracking" in harness.states(),
              lambda: harness.publish_command("px4_local_ned"))
    finally:
        _stop(process)
