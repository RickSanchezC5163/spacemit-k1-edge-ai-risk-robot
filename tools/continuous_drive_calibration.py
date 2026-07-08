#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


MAX_LINEAR = 0.45
MAX_ANGULAR = 1.20
MAX_DURATION = 10.0
DEFAULT_DURATION = 3.0
DEFAULT_RAMP_TIME = 0.4
DEFAULT_ZERO_SECONDS = 3.0
DEFAULT_RATE = 50.0
WAIT_FOR_SUBSCRIBER_SEC = 3.0


class OdomStats:
    def __init__(self):
        self.first = None
        self.last = None
        self.max_abs_vx = 0.0
        self.max_abs_wz = 0.0
        self.samples = 0

    def update(self, msg: Odometry):
        if self.first is None:
            self.first = msg
        self.last = msg
        self.samples += 1
        self.max_abs_vx = max(self.max_abs_vx, abs(msg.twist.twist.linear.x))
        self.max_abs_wz = max(self.max_abs_wz, abs(msg.twist.twist.angular.z))

    def summary(self):
        if self.first is None or self.last is None:
            return "odom samples=0"
        dx = self.last.pose.pose.position.x - self.first.pose.pose.position.x
        dy = self.last.pose.pose.position.y - self.first.pose.pose.position.y
        distance = math.hypot(dx, dy)
        return (
            f"odom samples={self.samples}, dx={dx:.3f}m, dy={dy:.3f}m, "
            f"distance={distance:.3f}m, max|vx|={self.max_abs_vx:.3f}m/s, "
            f"max|wz|={self.max_abs_wz:.3f}rad/s"
        )


def reject(message: str) -> None:
    print(f"REFUSE: {message}", file=sys.stderr)
    sys.exit(2)


def publish_for(pub, node, make_msg, duration: float, rate: float) -> None:
    start = time.monotonic()
    period = 1.0 / rate
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= duration:
            break
        pub.publish(make_msg(elapsed))
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)


def main():
    parser = argparse.ArgumentParser(
        description="Guarded continuous /cmd_vel test for C30D PID and mapping-speed calibration."
    )
    parser.add_argument("--linear", type=float, default=0.30, help="steady linear.x in m/s")
    parser.add_argument("--angular", type=float, default=0.0, help="steady angular.z in rad/s")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION, help="steady test duration in seconds")
    parser.add_argument("--ramp-time", type=float, default=DEFAULT_RAMP_TIME, help="linear ramp time in seconds")
    parser.add_argument("--zero-seconds", type=float, default=DEFAULT_ZERO_SECONDS)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE)
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom")
    args = parser.parse_args()

    linear = float(args.linear)
    angular = float(args.angular)
    duration = float(args.duration)
    ramp_time = max(0.0, float(args.ramp_time))
    zero_seconds = max(1.0, float(args.zero_seconds))
    rate = max(1.0, float(args.rate))

    if abs(linear) > MAX_LINEAR:
        reject(f"abs(linear) must be <= {MAX_LINEAR}, got {linear}")
    if abs(angular) > MAX_ANGULAR:
        reject(f"abs(angular) must be <= {MAX_ANGULAR}, got {angular}")
    if duration <= 0.0 or duration > MAX_DURATION:
        reject(f"duration must be > 0 and <= {MAX_DURATION}, got {duration}")
    if abs(linear) > 1e-6 and abs(angular) > 1e-6:
        reject("test either linear or angular motion first, not both at once")

    expected_distance = abs(linear) * duration
    expected_yaw = abs(angular) * duration

    print("SAFETY WARNING")
    print("- This sends a continuous guarded /cmd_vel command, then forced zero speed.")
    print("- A person must physically follow the robot and be ready to lift/disable it.")
    print("- Use this for PID calibration and mapping-speed validation, not unattended navigation.")
    print(f"- Limits: abs(linear)<={MAX_LINEAR:.2f} m/s, abs(angular)<={MAX_ANGULAR:.2f} rad/s, duration<={MAX_DURATION:.1f}s")
    print(f"- Command: linear.x={linear:.3f}, angular.z={angular:.3f}, duration={duration:.3f}s, ramp={ramp_time:.3f}s")
    print(f"- Expected open-loop distance/yaw: {expected_distance:.2f}m / {expected_yaw:.2f}rad")
    print(f"- Forced zero: {zero_seconds:.1f}s")
    confirmation = input("Type YES to execute: ").strip()
    if confirmation != "YES":
        reject("confirmation was not YES")

    rclpy.init()
    node = rclpy.create_node("continuous_drive_calibration")
    pub = node.create_publisher(Twist, args.topic, 10)
    stats = OdomStats()
    node.create_subscription(Odometry, args.odom_topic, stats.update, 20)

    wait_start = time.monotonic()
    while pub.get_subscription_count() < 1 and time.monotonic() - wait_start < WAIT_FOR_SUBSCRIBER_SEC:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() < 1:
        node.destroy_node()
        rclpy.shutdown()
        reject(f"no subscriber on {args.topic}")

    def command_msg(elapsed):
        scale = 1.0
        if ramp_time > 0.0:
            scale = min(1.0, elapsed / ramp_time)
        msg = Twist()
        msg.linear.x = linear * scale
        msg.angular.z = angular * scale
        return msg

    zero = Twist()
    node.get_logger().info(
        f"Continuous test start: linear.x={linear:.3f}, angular.z={angular:.3f}, duration={duration:.3f}s"
    )
    publish_for(pub, node, command_msg, duration, rate)
    publish_for(pub, node, lambda _: zero, zero_seconds, rate)
    pub.publish(zero)
    rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Continuous test complete; zero command sent.")
    print(stats.summary())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
