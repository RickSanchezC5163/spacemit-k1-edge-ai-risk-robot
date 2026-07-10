#!/usr/bin/env python3
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class VisibleExplorer(Node):
    def __init__(self, args):
        super().__init__("sim_visible_explorer")
        self.args = args
        self.started = time.monotonic()
        self.last_phase = self.started
        self.phase = "forward"
        self.turn_direction = 1.0
        self.status = {}
        self.odom = None
        self.records = []
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.create_subscription(String, args.status_topic, self.status_cb, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 10)
        self.timer = self.create_timer(1.0 / args.rate_hz, self.tick)

    def status_cb(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status = {}

    def odom_cb(self, msg):
        self.odom = msg

    def front_range(self):
        value = self.status.get("front_p10_range_m")
        if value is None:
            value = self.status.get("front_min_range_m")
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def set_phase(self, phase, now):
        if phase == self.phase:
            return
        self.phase = phase
        self.last_phase = now
        record = {"time_s": round(now - self.started, 2), "phase": phase, "front_m": self.front_range()}
        self.records.append(record)
        print("VISIBLE_PHASE", json.dumps(record), flush=True)

    def tick(self):
        now = time.monotonic()
        elapsed = now - self.started
        if elapsed >= self.args.runtime_s:
            self.publish_zero()
            rclpy.shutdown()
            return

        front = self.front_range()
        obstacle = front is not None and front < self.args.hard_stop_m
        phase_elapsed = now - self.last_phase
        if obstacle and self.phase == "forward":
            self.turn_direction *= -1.0
            self.set_phase("reverse_obstacle", now)
        elif self.phase == "reverse_obstacle" and phase_elapsed >= self.args.reverse_duration_s:
            self.set_phase("turn_obstacle", now)
        elif self.phase.startswith("turn") and phase_elapsed >= self.args.turn_duration_s:
            self.set_phase("forward", now)
        elif self.args.forward_duration_s > 0.0 and self.phase == "forward" and phase_elapsed >= self.args.forward_duration_s:
            self.turn_direction *= -1.0
            self.set_phase("turn_sweep", now)

        msg = Twist()
        if self.phase == "reverse_obstacle":
            msg.linear.x = -abs(self.args.reverse_speed)
        elif self.phase == "forward":
            msg.linear.x = self.args.linear_speed
            msg.angular.z = self.args.arc_speed * math.sin(elapsed * self.args.arc_frequency)
        else:
            msg.angular.z = self.turn_direction * self.args.turn_speed
        self.cmd_pub.publish(msg)

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def summary(self):
        pose = None
        if self.odom is not None:
            p = self.odom.pose.pose.position
            pose = {
                "x": round(float(p.x), 3),
                "y": round(float(p.y), 3),
                "yaw": round(yaw_from_quat(self.odom.pose.pose.orientation), 3),
            }
        return {
            "runtime_s": round(time.monotonic() - self.started, 2),
            "final_pose": pose,
            "records": self.records,
        }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cmd-topic", default="/input_cmd_vel")
    parser.add_argument("--status-topic", default="/safety/front_obstacle")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--runtime-s", type=float, default=180.0)
    parser.add_argument("--rate-hz", type=float, default=12.0)
    parser.add_argument("--linear-speed", type=float, default=0.28)
    parser.add_argument("--arc-speed", type=float, default=0.12)
    parser.add_argument("--arc-frequency", type=float, default=0.45)
    parser.add_argument("--turn-speed", type=float, default=1.0)
    parser.add_argument("--turn-duration-s", type=float, default=2.4)
    parser.add_argument("--reverse-speed", type=float, default=0.12)
    parser.add_argument("--reverse-duration-s", type=float, default=1.2)
    parser.add_argument("--forward-duration-s", type=float, default=0.0)
    parser.add_argument("--hard-stop-m", type=float, default=0.42)
    parser.add_argument("--report", default="")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = VisibleExplorer(args)
    print("VISIBLE_READY", flush=True)
    try:
        rclpy.spin(node)
    finally:
        node.publish_zero()
        summary = node.summary()
        if args.report:
            path = Path(args.report)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(summary, indent=2) + "\n")
        print("VISIBLE_SUMMARY", json.dumps(summary), flush=True)
        node.destroy_node()


if __name__ == "__main__":
    main()
