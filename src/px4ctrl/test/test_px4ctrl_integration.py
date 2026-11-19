"""Exercise the takeoff and adapter boundaries without starting PX4."""

import importlib.util
import math
from pathlib import Path

from nav_msgs.msg import Odometry
from px4_msgs.msg import EstimatorStatusFlags, VehicleLocalPosition
import pytest
from quadrotor_msgs.msg import PositionCommand
import rclpy


def load_script(name):
    path = Path(__file__).resolve().parents[1] / 'scripts' / f'{name}.py'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def ros(monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_DOMAIN_ID', '91')
    rclpy.init()
    yield
    rclpy.try_shutdown()


def local_position():
    return VehicleLocalPosition(timestamp=1, xy_valid=True, z_valid=True,
                                v_xy_valid=True, v_z_valid=True, z=0.0)


def test_takeoff_waits_for_external_vision_and_valid_local_state(ros):
    node = load_script('takeoff').Takeoff(0.5, 0.15, True, 0.0, False)
    try:
        received = []
        node.publisher.publish = received.append
        p = local_position()
        node.position_callback(p)
        node.publish_reference()
        assert not received

        node.estimator_status_callback(EstimatorStatusFlags(
            cs_ev_pos=True, cs_ev_hgt=True, cs_ev_vel=True))
        node.publish_reference()
        assert len(received) == 1
        assert received[0].position.z == pytest.approx(-0.5)

        p.z_valid = False
        node.position_callback(p)
        node.publish_reference()
        assert len(received) == 1
    finally:
        node.destroy_node()


def test_adapter_aligns_only_finite_state_and_does_not_duplicate_yaw_reset(ros):
    node = load_script('ego_position_command_adapter').EgoPositionCommandAdapter()
    try:
        received = []
        node.publisher.publish = received.append
        odom = Odometry()
        odom.header.frame_id = 'lidar'
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.x = math.nan
        node.local_position_callback(local_position())
        node.odom_callback(odom)
        assert node.translation is None

        odom.pose.pose.position.x = 0.0
        node.odom_callback(odom)
        assert node.translation == (0.0, 0.0, 0.0)

        p = local_position()
        p.timestamp = 2
        p.xy_reset_counter = 1
        p.delta_xy = [0.25, -0.50]
        p.heading_reset_counter = 1
        p.delta_heading = 0.3
        node.local_position_callback(p)
        node.local_position_callback(p)  # duplicate PX4 sample

        cmd = PositionCommand(trajectory_flag=PositionCommand.TRAJECTORY_STATUS_READY)
        cmd.header.frame_id = 'lidar'
        cmd.header.stamp.sec = 1
        cmd.position.x = 2.0
        cmd.position.y = 3.0
        cmd.position.z = 4.0
        cmd.yaw = 0.5
        node.command_callback(cmd)
        assert len(received) == 1
        out = received[0]
        assert (out.position.x, out.position.y, out.position.z) == pytest.approx((3.25, 1.5, -4.0))
        assert out.yaw == pytest.approx(math.pi / 2 - 0.5)
    finally:
        node.destroy_node()
