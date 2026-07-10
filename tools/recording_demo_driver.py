#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_odom(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_delta(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class RecordingDemoDriver(Node):
    def __init__(self, args):
        super().__init__("recording_demo_driver")
        self.args = args
        self.odom = None
        self.pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 10)

    def odom_cb(self, msg):
        self.odom = msg

    def publish(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def stop(self, seconds=0.4):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            self.publish()
            time.sleep(0.02)

    def wait_for_odom(self):
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and self.odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom is not None

    def turn_degrees(self, degrees):
        if self.odom is None:
            return
        start = yaw_from_odom(self.odom)
        target = start + math.radians(degrees)
        direction = 1.0 if degrees >= 0 else -1.0
        deadline = time.monotonic() + self.args.turn_timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            current = yaw_from_odom(self.odom)
            remaining = angle_delta(target, current)
            if abs(math.degrees(remaining)) < 2.5:
                break
            speed = direction * self.args.turn_speed
            if abs(math.degrees(remaining)) < 8.0:
                speed *= 0.55
            self.publish(angular=speed)
            time.sleep(0.02)
        self.stop(0.25)

    def drive_forward(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            self.publish(linear=self.args.linear_speed)
            time.sleep(0.02)
        self.stop(0.35)

    def run_demo(self):
        if not self.wait_for_odom():
            self.get_logger().error("No odom received; aborting recording demo.")
            return 1
        self.get_logger().info("Recording demo started: 30deg turns + short guarded drives.")
        self.stop(0.5)
        pattern = [30, 30, -30, 30, -30, -30]
        for i, turn in enumerate(pattern, 1):
            self.get_logger().info(f"Step {i}: turn {turn}deg")
            self.turn_degrees(turn)
            self.get_logger().info(f"Step {i}: forward {self.args.forward_s:.1f}s")
            self.drive_forward(self.args.forward_s)
        self.stop(0.8)
        self.get_logger().info("Recording demo finished.")
        return 0


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cmd-topic", default="/input_cmd_vel")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--turn-speed", type=float, default=0.65)
    parser.add_argument("--linear-speed", type=float, default=0.10)
    parser.add_argument("--forward-s", type=float, default=1.7)
    parser.add_argument("--turn-timeout-s", type=float, default=2.2)
    return parser.parse_args()


def main():
    rclpy.init()
    node = RecordingDemoDriver(parse_args())
    try:
        raise SystemExit(node.run_demo())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
