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
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode, VehicleAttitudeSetpoint, VehicleLandDetected,
    VehicleLocalPosition, VehicleStatus,
)
from px4ctrl.msg import ControlDebug, DesiredState
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
    binary = Path(get_package_prefix("px4ctrl")) / "lib" / "px4ctrl" / executable
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


def test_airborne_takeover_and_offboard_loss(ros_graph):
    node, executor = ros_graph
    suffix = uuid.uuid4().hex[:8]
    root = f"/px4ctrl_test_{suffix}"
    topics = {
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
    params = {
        **topics,
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
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    statuses, modes, setpoints, diagnostics = [], [], [], []
    status_pub = node.create_publisher(VehicleStatus, topics["px4_status_topic"], sensor_qos)
    local_pub = node.create_publisher(VehicleLocalPosition, topics["px4_local_position_topic"], sensor_qos)
    land_pub = node.create_publisher(VehicleLandDetected, topics["px4_land_detected_topic"], sensor_qos)
    command_pub = node.create_publisher(DesiredState, topics["desired_state_topic"], 20)
    permission_pub = node.create_publisher(Bool, topics["takeover_permission_topic"], 10)
    node.create_subscription(String, topics["status_topic"],
                             lambda msg: statuses.append(msg.data), 10)
    node.create_subscription(OffboardControlMode, topics["offboard_control_mode_topic"],
                             lambda msg: modes.append(msg), sensor_qos)
    node.create_subscription(VehicleAttitudeSetpoint, topics["attitude_setpoint_topic"],
                             lambda msg: setpoints.append(msg), sensor_qos)
    node.create_subscription(ControlDebug, topics["debug_topic"],
                             lambda msg: diagnostics.append(msg), 10)
    flight = {"armed": False, "offboard": False, "landed": True}
    last_land = {"time": 0.0, "landed": None}

    def publish_state():
        stamp = time.monotonic_ns() // 1000
        status = VehicleStatus()
        status.timestamp = stamp
        status.arming_state = (VehicleStatus.ARMING_STATE_ARMED if flight["armed"]
                               else VehicleStatus.ARMING_STATE_DISARMED)
        status.nav_state = (VehicleStatus.NAVIGATION_STATE_OFFBOARD if flight["offboard"]
                            else VehicleStatus.NAVIGATION_STATE_POSCTL)
        status_pub.publish(status)
        position = VehicleLocalPosition()
        position.timestamp = stamp
        position.xy_valid = position.z_valid = True
        position.v_xy_valid = position.v_z_valid = True
        position.x, position.y, position.z = 1.0, -2.0, -2.0
        position.heading = 0.0
        local_pub.publish(position)
        now = time.monotonic()
        if now - last_land["time"] >= 1.0 or last_land["landed"] != flight["landed"]:
            land = VehicleLandDetected()
            land.timestamp = stamp
            land.landed = flight["landed"]
            land.ground_contact = flight["landed"]
            land.maybe_landed = flight["landed"]
            land_pub.publish(land)
            last_land.update(time=now, landed=flight["landed"])

    process = _start("px4ctrl_node", params)
    try:
        _wait(executor, lambda: status_pub.get_subscription_count() > 0 and
              local_pub.get_subscription_count() > 0, publish_state)
        assert not setpoints, "disarmed state must not produce attitude setpoints"

        flight["armed"] = True
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            publish_state()
            executor.spin_once(timeout_sec=0.04)
        assert not setpoints, "armed but landed state must not capture an airborne hold"

        flight["landed"] = False
        flight["offboard"] = True
        deadline = time.monotonic() + 1.2
        while time.monotonic() < deadline:
            publish_state()
            executor.spin_once(timeout_sec=0.04)
        assert not setpoints, "controller must not compete with position takeoff before permission"
        assert not modes, "controller must not replace the position Offboard mode before permission"

        def publish_permission():
            publish_state()
            permission_pub.publish(Bool(data=True))

        _wait(executor, lambda: "hover_airborne_takeover_offboard_ready" in statuses and
              len(setpoints) >= 3, publish_permission)
        assert math.isclose(-setpoints[-1].thrust_body[2], 0.6, abs_tol=0.02)
        assert modes and modes[-1].attitude

        def publish_command():
            publish_state()
            desired = DesiredState()
            desired.header.stamp = node.get_clock().now().to_msg()
            desired.header.frame_id = "px4_local_ned"
            desired.position.x, desired.position.y, desired.position.z = 1.2, -2.0, -2.0
            command_pub.publish(desired)

        _wait(executor, lambda: any(
            item.state == "tracking" and item.position_error.x > 0.05
            for item in diagnostics), publish_command)
        assert setpoints[-1].timestamp > 0

        flight["offboard"] = False
        _wait(executor, lambda: "control_suspended_offboard_lost" in statuses,
              publish_state)
        setpoints.clear()
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            publish_state()
            executor.spin_once(timeout_sec=0.04)
        assert not setpoints, "controller must stop publishing after Offboard loss"
    finally:
        _stop(process)


def test_ego_adapter_filters_and_translates(ros_graph):
    quadrotor_msgs = pytest.importorskip("quadrotor_msgs.msg")
    PositionCommand = quadrotor_msgs.PositionCommand
    node, executor = ros_graph
    suffix = uuid.uuid4().hex[:8]
    input_topic = f"/px4ctrl_adapter_test_{suffix}/input"
    output_topic = f"/px4ctrl_adapter_test_{suffix}/output"
    odom_topic = f"/px4ctrl_adapter_test_{suffix}/odom"
    local_topic = f"/px4ctrl_adapter_test_{suffix}/local_position"
    source = node.create_publisher(PositionCommand, input_topic, 20)
    odom_source = node.create_publisher(Odometry, odom_topic, 20)
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    local_source = node.create_publisher(VehicleLocalPosition, local_topic, sensor_qos)
    received = []
    node.create_subscription(DesiredState, output_topic,
                             lambda msg: received.append(msg), 20)
    process = _start("ego_position_command_adapter", {
        "input_topic": input_topic,
        "output_topic": output_topic,
        "planner_frame": "lidar",
        "odom_topic": odom_topic,
        "local_position_topic": local_topic,
    })
    try:
        def publish_alignment_state():
            odom = Odometry()
            odom.header.frame_id = "lidar"
            odom.pose.pose.position.x = 10.0
            odom.pose.pose.position.y = 20.0
            odom.pose.pose.orientation.w = 1.0
            odom_source.publish(odom)
            local = VehicleLocalPosition()
            local.timestamp = time.monotonic_ns() // 1000
            local.xy_valid = local.z_valid = True
            local.v_xy_valid = local.v_z_valid = True
            local.x, local.y, local.z = 2.0, 3.0, -1.0
            local_source.publish(local)

        _wait(executor, lambda: source.get_subscription_count() > 0 and
              odom_source.get_subscription_count() > 0 and
              local_source.get_subscription_count() > 0, publish_alignment_state)
        command = PositionCommand()
        command.header.frame_id = "wrong"
        command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        command.position.x = 3.0
        source.publish(command)
        for _ in range(4):
            executor.spin_once(timeout_sec=0.05)
        assert not received

        command.header.frame_id = "lidar"
        command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_EMPTY
        source.publish(command)
        executor.spin_once(timeout_sec=0.1)
        assert not received

        command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        command.velocity.y = -0.4
        command.acceleration.z = 0.2
        command.yaw, command.yaw_dot = 0.5, 0.1
        def publish_inputs():
            publish_alignment_state()
            source.publish(command)

        _wait(executor, lambda: bool(received), publish_inputs)
        actual = received[-1]
        assert actual.header.frame_id == "px4_local_ned"
        assert math.isclose(actual.position.x, -18.0, abs_tol=1e-6)
        assert math.isclose(actual.position.y, -4.0, abs_tol=1e-6)
        assert math.isclose(actual.position.z, -1.0, abs_tol=1e-6)
        assert math.isclose(actual.velocity.x, -0.4, abs_tol=1e-6)
        assert math.isclose(actual.acceleration.z, -0.2, abs_tol=1e-6)
        assert math.isclose(actual.yaw, math.pi / 2.0 - 0.5, abs_tol=1e-6)
        assert math.isclose(actual.yaw_rate, -0.1, abs_tol=1e-6)
    finally:
        _stop(process)
