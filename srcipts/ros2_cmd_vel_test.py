#!/usr/bin/env python3
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--linear-x", type=float, default=0.18)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("cmd_vel_test")
    pub = node.create_publisher(Twist, args.cmd_topic, 1)

    msg = Twist()
    msg.linear.x = args.linear_x
    msg.angular.z = args.angular_z

    deadline = time.time() + args.duration
    period = 1.0 / args.rate
    node.get_logger().info(
        f"Publishing {args.cmd_topic}: linear.x={args.linear_x}, angular.z={args.angular_z}, duration={args.duration}s"
    )
    while rclpy.ok() and time.time() < deadline:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)

    if rclpy.ok():
        pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
