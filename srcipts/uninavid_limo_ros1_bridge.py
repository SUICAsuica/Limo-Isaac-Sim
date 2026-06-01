#!/usr/bin/env python3
import argparse
import base64
import json
import threading
import time

import requests
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class UniNaVidLimoBridge:
    def __init__(self, args):
        self.args = args
        self.latest_image_b64 = None
        self.latest_stamp = None
        self.lock = threading.Lock()
        self.cmd_pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=1)
        self.image_sub = rospy.Subscriber(args.image_topic, Image, self.on_image, queue_size=1, buff_size=2**24)
        self.log_file = open(args.log_path, "a", encoding="utf-8") if args.log_path else None

    def on_image(self, msg):
        if msg.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
            rospy.logwarn_throttle(5.0, "Unsupported image encoding: %s", msg.encoding)
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
            self.latest_stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()

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

    def publish_action(self, action):
        twist = self.twist_for_action(action)
        duration = self.args.forward_duration if action == "forward" else self.args.turn_duration
        if action == "stop":
            duration = self.args.stop_duration

        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(self.args.cmd_rate)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.cmd_pub.publish(Twist())

    def log_result(self, result):
        if not self.log_file:
            return
        self.log_file.write(json.dumps({"time": time.time(), **result}, ensure_ascii=False) + "\n")
        self.log_file.flush()

    def run(self):
        rospy.loginfo("Waiting for images on %s", self.args.image_topic)
        rate = rospy.Rate(self.args.inference_rate)
        while not rospy.is_shutdown():
            with self.lock:
                image_b64 = self.latest_image_b64
            if image_b64 is None:
                rate.sleep()
                continue

            try:
                result = self.request_actions(image_b64)
                actions = result.get("actions", [])
                self.log_result(result)
                rospy.loginfo("Uni-NaVid actions: %s", " ".join(actions))
                if not actions:
                    self.cmd_pub.publish(Twist())
                for action in actions[: self.args.max_actions_per_cycle]:
                    if rospy.is_shutdown():
                        break
                    self.publish_action(action)
                    if action == "stop":
                        break
            except Exception as exc:
                rospy.logerr("Uni-NaVid bridge error: %s", exc)
                self.cmd_pub.publish(Twist())

            rate.sleep()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server-url", default="http://127.0.0.1:8088")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--linear-speed", type=float, default=0.18)
    parser.add_argument("--angular-speed", type=float, default=0.6)
    parser.add_argument("--forward-duration", type=float, default=0.7)
    parser.add_argument("--turn-duration", type=float, default=0.45)
    parser.add_argument("--stop-duration", type=float, default=0.2)
    parser.add_argument("--cmd-rate", type=float, default=20.0)
    parser.add_argument("--inference-rate", type=float, default=1.0)
    parser.add_argument("--max-actions-per-cycle", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--log-path", default="/tmp/uninavid_limo_actions.jsonl")
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("uninavid_limo_bridge")
    UniNaVidLimoBridge(args).run()


if __name__ == "__main__":
    main()
