#!/usr/bin/env python3
"""Publish a manual EGO goal in the planner's lidar frame."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty, Float64


def main(args=None):
    parser = argparse.ArgumentParser(description='Send an EGO goal in lidar coordinates')
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    parser.add_argument('yaw_deg', nargs='?', type=float,
                        help='fixed PX4-local-NED heading in degrees; omit for EGO path auto-yaw')
    parser.add_argument('--auto-yaw', action='store_true',
                        help='explicitly clear manual yaw and follow EGO path heading')
    parser.add_argument('--frame', default='lidar')
    parser.add_argument('--topic', default='/goal_pose')
    parser.add_argument('--manual-yaw-topic', default='/px4ctrl/manual_yaw')
    parser.add_argument('--clear-manual-yaw-topic', default='/px4ctrl/clear_manual_yaw')
    parser.add_argument('--wait-timeout', type=float, default=30.0)
    options = parser.parse_args(args)
    if options.auto_yaw and options.yaw_deg is not None:
        parser.error('use either yaw_deg or --auto-yaw, not both')
    values = (options.x, options.y, options.z)
    if options.yaw_deg is not None:
        values += (options.yaw_deg,)
    if not all(math.isfinite(value) for value in values):
        parser.error('goal coordinates and yaw must be finite')
    if not math.isfinite(options.wait_timeout) or options.wait_timeout <= 0.0:
        parser.error('wait-timeout must be positive')

    rclpy.init()
    node = Node('px4ctrl_manual_goal', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    publisher = node.create_publisher(PoseStamped, options.topic, 10)
    yaw_publisher = node.create_publisher(Float64, options.manual_yaw_topic, 10)
    clear_yaw_publisher = node.create_publisher(Empty, options.clear_manual_yaw_topic, 10)
    deadline = time.monotonic() + options.wait_timeout
    try:
        while rclpy.ok() and publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError(f'no EGO goal subscriber found on {options.topic}')
            rclpy.spin_once(node, timeout_sec=0.1)

        if options.yaw_deg is not None:
            yaw_deadline = time.monotonic() + options.wait_timeout
            while rclpy.ok() and yaw_publisher.get_subscription_count() == 0:
                if time.monotonic() >= yaw_deadline:
                    raise RuntimeError(f'no px4ctrl subscriber found on {options.manual_yaw_topic}')
                rclpy.spin_once(node, timeout_sec=0.1)

        msg = PoseStamped()
        msg.header.frame_id = options.frame
        msg.pose.position.x = options.x
        msg.pose.position.y = options.y
        msg.pose.position.z = options.z
        msg.pose.orientation.w = 1.0
        if options.yaw_deg is not None:
            yaw = Float64()
            yaw.data = math.radians(options.yaw_deg)
        else:
            yaw = Empty()
        for _ in range(5):
            msg.header.stamp = node.get_clock().now().to_msg()
            if options.yaw_deg is not None:
                yaw_publisher.publish(yaw)
            else:
                clear_yaw_publisher.publish(yaw)
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.05)
        heading = (f'fixed PX4 NED heading {options.yaw_deg:g} deg'
                   if options.yaw_deg is not None else 'EGO path auto-yaw')
        print(f'Sent EGO goal ({options.x:g}, {options.y:g}, {options.z:g}) m '
              f'in {options.frame}; heading: {heading}.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
