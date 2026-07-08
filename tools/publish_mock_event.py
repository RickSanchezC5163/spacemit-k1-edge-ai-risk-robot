#!/usr/bin/env python3
import argparse
import json
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description="Publish one mock perception event.")
    parser.add_argument("--type", default="soft_obstacle", dest="event_type")
    parser.add_argument("--distance", type=float, default=0.8)
    parser.add_argument("--confidence", type=float, default=0.9)
    parser.add_argument("--source", default="manual_mock")
    parser.add_argument("--topic", default="/perception/mock_event")
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--rate", type=float, default=None, help="Publish rate in Hz; overrides --interval.")
    args = parser.parse_args()

    rclpy.init()
    node = Node("publish_mock_event")
    pub = node.create_publisher(String, args.topic, 10)

    event = {
        "event_type": args.event_type,
        "distance_m": args.distance,
        "confidence": max(0.0, min(1.0, args.confidence)),
        "source": args.source,
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
    }
    msg = String()
    msg.data = json.dumps(event, ensure_ascii=False)

    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    interval = max(0.0, args.interval)
    if args.rate is not None and args.rate > 0:
        interval = 1.0 / args.rate

    for _ in range(max(1, args.count)):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(interval)

    node.get_logger().info(f"Published mock event to {args.topic}: {msg.data}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
