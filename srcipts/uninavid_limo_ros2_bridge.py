#!/usr/bin/env python3
import argparse
import base64
import json
import threading
import time

import requests
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class UniNaVidLimoBridge(Node):
    def __init__(self, args):
        super().__init__("uninavid_limo_bridge")
        self.args = args
        self.latest_image = None
        self.lock = threading.Lock()
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 1)
        self.goal_pub = self.create_publisher(PoseStamped, args.goal_topic, 1)
        self.create_subscription(Image, args.image_topic, self.on_image, 1)
        self.timer = self.create_timer(1.0 / args.inference_rate, self.on_timer)
        self.busy = False
        self.log_file = open(args.log_path, "a", encoding="utf-8") if args.log_path else None
        self.get_logger().info(f"Waiting for images on {args.image_topic}")
        self.get_logger().info(f"Control mode: {args.control_mode}")

    def on_image(self, msg):
        if msg.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
            self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
            return

        channels = 4 if msg.encoding in ("rgba8", "bgra8") else 3
        rgb = bytearray(msg.width * msg.height * 3)
        src = msg.data
        out = 0
        for row in range(msg.height):
            row_start = row * msg.step
            for col in range(msg.width):
                i = row_start + col * channels
                if msg.encoding == "rgb8":
                    rgb[out : out + 3] = src[i : i + 3]
                elif msg.encoding == "bgr8":
                    rgb[out : out + 3] = bytes((src[i + 2], src[i + 1], src[i]))
                elif msg.encoding == "rgba8":
                    rgb[out : out + 3] = src[i : i + 3]
                else:
                    rgb[out : out + 3] = bytes((src[i + 2], src[i + 1], src[i]))
                out += 3

        ppm = f"P6\n{msg.width} {msg.height}\n255\n".encode("ascii") + bytes(rgb)

        with self.lock:
            self.latest_image_b64 = base64.b64encode(ppm).decode("ascii")

    def request_actions(self, image_b64):
        payload = {
            "instruction": self.args.instruction,
            "image_b64": image_b64,
        }
        response = requests.post(f"{self.args.server_url}/predict", json=payload, timeout=self.args.timeout)
        response.raise_for_status()
        return response.json()

    def twist_for_action(self, action):
        msg = Twist()
        if action == "forward":
            msg.linear.x = self.args.linear_speed
        elif action == "left":
            msg.angular.z = self.args.angular_speed
        elif action == "right":
            msg.angular.z = -self.args.angular_speed
        return msg

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_goal(self, result):
        trajectory = result.get("trajectory") or []
        if not trajectory:
            self.publish_stop()
            return

        target = trajectory[min(len(trajectory) - 1, self.args.goal_step)]
        if len(target) < 3:
            self.publish_stop()
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.goal_frame
        msg.pose.position.x = float(target[0]) * self.args.goal_scale
        msg.pose.position.y = float(target[1]) * self.args.goal_scale
        msg.pose.position.z = 0.0
        half = float(target[2]) * 0.5
        msg.pose.orientation.z = __import__("math").sin(half)
        msg.pose.orientation.w = __import__("math").cos(half)
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f"Published /goal from Uni-NaVid: frame={msg.header.frame_id}, "
            f"x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, yaw={float(target[2]):.3f}"
        )

    def publish_action(self, action, stop_after=False):
        twist = self.twist_for_action(action)
        duration = self.args.forward_duration if action == "forward" else self.args.turn_duration
        if action == "stop":
            duration = self.args.stop_duration

        deadline = time.time() + duration
        period = 1.0 / self.args.cmd_rate
        while rclpy.ok() and time.time() < deadline:
            self.cmd_pub.publish(twist)
            time.sleep(period)
        if stop_after or action == "stop":
            self.publish_stop()

    def log_result(self, result):
        if not self.log_file:
            return
        self.log_file.write(json.dumps({"time": time.time(), **result}, ensure_ascii=False) + "\n")
        self.log_file.flush()

    def on_timer(self):
        if self.busy:
            return
        with self.lock:
            image_b64 = getattr(self, "latest_image_b64", None)
        if image_b64 is None:
            return

        self.busy = True
        try:
            result = self.request_actions(image_b64)
            actions = result.get("actions", [])
            self.log_result(result)
            self.get_logger().info(f"Uni-NaVid actions: {' '.join(actions)}")
            if self.args.control_mode == "goal":
                self.publish_goal(result)
                return
            if not actions:
                self.publish_stop()
            selected_actions = actions[: self.args.max_actions_per_cycle]
            for index, action in enumerate(selected_actions):
                if not rclpy.ok():
                    break
                self.publish_action(action, stop_after=index == len(selected_actions) - 1)
                if action == "stop":
                    break
        except Exception as exc:
            self.get_logger().error(f"Uni-NaVid bridge error: {exc}")
            self.publish_stop()
        finally:
            self.busy = False


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server-url", default="http://127.0.0.1:8088")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--goal-topic", default="/goal")
    parser.add_argument("--control-mode", choices=("goal", "cmd_vel"), default="cmd_vel")
    parser.add_argument("--goal-frame", default="base_link")
    parser.add_argument("--goal-step", type=int, default=-1)
    parser.add_argument("--goal-scale", type=float, default=1.0)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--linear-speed", type=float, default=0.18)
    parser.add_argument("--angular-speed", type=float, default=0.6)
    parser.add_argument("--forward-duration", type=float, default=0.7)
    parser.add_argument("--turn-duration", type=float, default=0.45)
    parser.add_argument("--stop-duration", type=float, default=0.2)
    parser.add_argument("--cmd-rate", type=float, default=20.0)
    parser.add_argument("--inference-rate", type=float, default=1.0)
    parser.add_argument("--max-actions-per-cycle", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--log-path", default="/tmp/uninavid_limo_actions.jsonl")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = UniNaVidLimoBridge(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
