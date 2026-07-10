#!/usr/bin/env python3
import argparse

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TwistRelay(Node):
    def __init__(self, args):
        super().__init__("twist_relay")
        self.pub = self.create_publisher(Twist, args.output_topic, 10)
        self.create_subscription(Twist, args.input_topic, self.pub.publish, 10)
        self.get_logger().info(f"Relaying {args.input_topic} -> {args.output_topic}")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", default="/cmd_vel")
    parser.add_argument("--output-topic", default="/input_cmd_vel")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = TwistRelay(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
