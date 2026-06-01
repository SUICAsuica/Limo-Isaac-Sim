#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-topic", default="/goal")
    parser.add_argument("--x", type=float, default=1.6)
    parser.add_argument("--y", type=float, default=2.75)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--repeat-sec", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("goal_test")
    pub = node.create_publisher(PoseStamped, args.goal_topic, 1)

    msg = PoseStamped()
    msg.header.frame_id = "world"
    msg.pose.position.x = args.x
    msg.pose.position.y = args.y
    msg.pose.position.z = 0.0
    half = args.yaw * 0.5
    msg.pose.orientation.z = math.sin(half)
    msg.pose.orientation.w = math.cos(half)

    deadline = time.time() + max(args.repeat_sec, 0.1)
    node.get_logger().info(f"Publishing {args.goal_topic}: x={args.x}, y={args.y}, yaw={args.yaw}")
    while rclpy.ok() and time.time() < deadline:
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
