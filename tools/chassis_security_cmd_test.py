#!/usr/bin/env python3
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int8


def parse_args():
    parser = argparse.ArgumentParser(
        description="Guarded C30D chassis security and cmd_vel smoke test."
    )
    parser.add_argument("--linear", type=float, default=0.30)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--zero-duration", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--security", type=int, default=1)
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if args.confirm != "YES":
        raise SystemExit("Refusing to move chassis without --confirm YES.")
    if abs(args.linear) > 0.45:
        raise SystemExit("Refusing linear speed above 0.45 m/s.")
    if abs(args.angular) > 0.80:
        raise SystemExit("Refusing angular speed above 0.80 rad/s.")
    if args.duration > 2.0:
        raise SystemExit("Refusing duration above 2.0s.")
    if args.rate <= 0:
        raise SystemExit("--rate must be positive.")
    return args


def publish_for(node, pub, msg, duration, rate):
    period = 1.0 / rate
    end = time.monotonic() + duration
    while time.monotonic() < end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.001)
        time.sleep(period)


def main():
    args = parse_args()
    rclpy.init()
    node = Node("chassis_security_cmd_test")
    security_pub = node.create_publisher(Int8, "/chassis_security", 10)
    cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)

    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    security = Int8()
    security.data = int(args.security)
    publish_for(node, security_pub, security, 0.5, 20.0)

    cmd = Twist()
    cmd.linear.x = args.linear
    cmd.angular.z = args.angular
    publish_for(node, cmd_pub, cmd, args.duration, args.rate)

    zero = Twist()
    publish_for(node, cmd_pub, zero, args.zero_duration, args.rate)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
