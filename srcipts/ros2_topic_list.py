#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("topic_list_once")
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.1)
    for name, types in sorted(node.get_topic_names_and_types()):
        print(f"{name}: {', '.join(types)}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
