#!/usr/bin/env python3
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist


def main():
    parser = argparse.ArgumentParser(
        description="Publish only zero /cmd_vel for a fixed duration."
    )
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--topic", default="/cmd_vel")
    args = parser.parse_args()

    duration = max(0.1, float(args.duration))
    rate = max(1.0, float(args.rate))

    rclpy.init()
    node = rclpy.create_node("send_safe_zero_cmd")
    pub = node.create_publisher(Twist, args.topic, 10)
    msg = Twist()

    end_time = time.monotonic() + duration
    period = 1.0 / rate

    node.get_logger().info(
        f"Publishing zero Twist to {args.topic} for {duration:.2f}s at {rate:.1f}Hz"
    )

    while time.monotonic() < end_time:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)

    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)
    node.get_logger().info("Zero command complete.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

