#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist


MAX_LINEAR = 0.10
MAX_ANGULAR = 0.35
MAX_DURATION = 1.0
DEFAULT_ZERO_SECONDS = 2.0
DEFAULT_RATE = 50.0
WAIT_FOR_SUBSCRIBER_SEC = 3.0


def reject(message: str) -> None:
    print(f"REFUSE: {message}", file=sys.stderr)
    sys.exit(2)


def publish_for(pub, node, msg, duration: float, rate: float) -> None:
    end_time = time.monotonic() + duration
    period = 1.0 / rate
    while time.monotonic() < end_time:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)


def main():
    parser = argparse.ArgumentParser(
        description="One confirmed guarded /cmd_vel pulse for mapping checks."
    )
    parser.add_argument("--linear", type=float, default=0.05, help="linear.x in m/s")
    parser.add_argument("--angular", type=float, default=0.0, help="angular.z in rad/s")
    parser.add_argument("--duration", type=float, default=0.3, help="pulse duration in seconds")
    parser.add_argument("--zero-seconds", type=float, default=DEFAULT_ZERO_SECONDS)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE)
    parser.add_argument("--topic", default="/cmd_vel")
    args = parser.parse_args()

    linear = float(args.linear)
    angular = float(args.angular)
    duration = float(args.duration)
    zero_seconds = max(0.5, float(args.zero_seconds))
    rate = max(1.0, float(args.rate))

    if abs(linear) > MAX_LINEAR:
        reject(f"abs(linear) must be <= {MAX_LINEAR}, got {linear}")
    if abs(angular) > MAX_ANGULAR:
        reject(f"abs(angular) must be <= {MAX_ANGULAR}, got {angular}")
    if duration <= 0.0 or duration > MAX_DURATION:
        reject(f"duration must be > 0 and <= {MAX_DURATION}, got {duration}")

    print("SAFETY WARNING")
    print("- This sends one low-speed /cmd_vel pulse, then forced zero speed.")
    print("- The robot must be lifted or physically guarded.")
    print("- This script does not run a route and does not repeat automatically.")
    print(f"- Limits: abs(linear)<={MAX_LINEAR:.2f} m/s, abs(angular)<={MAX_ANGULAR:.2f} rad/s, duration<={MAX_DURATION:.1f}s")
    print(f"- Command: linear.x={linear:.3f}, angular.z={angular:.3f}, duration={duration:.3f}s")
    print(f"- Forced zero: {zero_seconds:.1f}s")
    confirmation = input("Type YES to execute: ").strip()
    if confirmation != "YES":
        reject("confirmation was not YES")

    rclpy.init()
    node = rclpy.create_node("mapping_pulse_test")
    pub = node.create_publisher(Twist, args.topic, 10)
    wait_start = time.monotonic()
    while pub.get_subscription_count() < 1 and time.monotonic() - wait_start < WAIT_FOR_SUBSCRIBER_SEC:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() < 1:
        node.destroy_node()
        rclpy.shutdown()
        reject(f"no subscriber on {args.topic}")

    pulse = Twist()
    pulse.linear.x = linear
    pulse.angular.z = angular
    zero = Twist()

    node.get_logger().info(
        f"Publishing mapping pulse to {args.topic}: "
        f"linear.x={linear:.3f}, angular.z={angular:.3f}, duration={duration:.3f}s"
    )

    publish_for(pub, node, pulse, duration, rate)
    publish_for(pub, node, zero, zero_seconds, rate)
    pub.publish(zero)
    rclpy.spin_once(node, timeout_sec=0.05)

    node.get_logger().info("Pulse complete; zero command sent.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
