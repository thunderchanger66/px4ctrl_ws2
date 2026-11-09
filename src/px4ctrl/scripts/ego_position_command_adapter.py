#!/usr/bin/env python3
"""Translate EGO PositionCommand samples into the controller's planner interface."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from quadrotor_msgs.msg import PositionCommand

from px4ctrl.msg import DesiredState


class EgoPositionCommandAdapter(Node):
    def __init__(self):
        super().__init__("ego_position_command_adapter")
        input_topic = self.declare_parameter(
            "input_topic", "/drone_0_planning/pos_cmd"
        ).value
        output_topic = self.declare_parameter(
            "output_topic", "/px4ctrl/desired_state"
        ).value
        self.planner_frame = self.declare_parameter("planner_frame", "lidar").value
        if not input_topic or not output_topic or not self.planner_frame:
            raise ValueError("adapter topics and planner_frame must not be empty")
        self.publisher = self.create_publisher(DesiredState, output_topic, 20)
        self.subscription = self.create_subscription(
            PositionCommand, input_topic, self.on_command, 20
        )

    def on_command(self, command):
        if (
            command.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY
            or command.header.frame_id != self.planner_frame
        ):
            return
        values = (
            command.position.x, command.position.y, command.position.z,
            command.velocity.x, command.velocity.y, command.velocity.z,
            command.acceleration.x, command.acceleration.y, command.acceleration.z,
            command.yaw, command.yaw_dot,
        )
        if not all(math.isfinite(value) for value in values):
            return
        desired = DesiredState()
        desired.header = command.header
        desired.position = command.position
        desired.velocity = command.velocity
        desired.acceleration = command.acceleration
        desired.yaw = command.yaw
        desired.yaw_rate = command.yaw_dot
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
