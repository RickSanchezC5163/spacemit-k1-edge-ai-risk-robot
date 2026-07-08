#!/usr/bin/env python3
"""Write lightweight ROS telemetry snapshots for the prelim demo UI."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    tmp.replace(path)


class TelemetryWriter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("prelim_telemetry_writer")
        self.args = args
        self.output_path = Path(args.output_dir) / "topic_status.json"
        self.latest_scan: LaserScan | None = None
        self.latest_scan_time = 0.0
        self.latest_odom: Odometry | None = None
        self.latest_odom_time = 0.0
        self.create_subscription(LaserScan, args.scan_topic, self.scan_cb, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 10)
        self.create_timer(args.period_s, self.timer_cb)

    def scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = time.monotonic()

    def odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.latest_odom_time = time.monotonic()

    def scan_snapshot(self) -> Dict[str, Any]:
        msg = self.latest_scan
        if msg is None:
            return {"fresh": False}
        now = time.monotonic()
        valid: List[tuple[float, float]] = []
        front: List[float] = []
        sample: List[Dict[str, float]] = []
        step = max(1, len(msg.ranges) // int(self.args.max_scan_points))
        for index, raw in enumerate(msg.ranges):
            value = float(raw)
            if not math.isfinite(value):
                continue
            if value < float(msg.range_min) or value > float(msg.range_max):
                continue
            angle = float(msg.angle_min) + index * float(msg.angle_increment)
            valid.append((angle, value))
            if abs(angle) <= float(self.args.front_half_angle_rad):
                front.append(value)
            if index % step == 0:
                sample.append({"angle_rad": round(angle, 4), "range_m": round(value, 4)})
        ranges = [item[1] for item in valid]
        return {
            "fresh": now - self.latest_scan_time <= float(self.args.fresh_timeout_s),
            "age_s": round(now - self.latest_scan_time, 3),
            "frame_id": msg.header.frame_id,
            "range_min_m": round(float(msg.range_min), 4),
            "range_max_m": round(float(msg.range_max), 4),
            "valid_count": len(valid),
            "min_range_m": None if not ranges else round(min(ranges), 4),
            "front_min_range_m": None if not front else round(min(front), 4),
            "sample": sample,
        }

    def odom_snapshot(self) -> Dict[str, Any]:
        msg = self.latest_odom
        if msg is None:
            return {"fresh": False}
        now = time.monotonic()
        pose = msg.pose.pose
        twist = msg.twist.twist
        yaw = yaw_from_quat(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        return {
            "fresh": now - self.latest_odom_time <= float(self.args.fresh_timeout_s),
            "age_s": round(now - self.latest_odom_time, 3),
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x_m": round(float(pose.position.x), 4),
            "y_m": round(float(pose.position.y), 4),
            "yaw_deg": round(math.degrees(yaw), 2),
            "linear_x_mps": round(float(twist.linear.x), 4),
            "angular_z_radps": round(float(twist.angular.z), 4),
        }

    def timer_cb(self) -> None:
        data = {
            "updated_at_unix": round(time.time(), 3),
            "scan": self.scan_snapshot(),
            "odom": self.odom_snapshot(),
        }
        write_json(self.output_path, data)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--period-s", type=float, default=0.5)
    parser.add_argument("--fresh-timeout-s", type=float, default=2.0)
    parser.add_argument("--front-half-angle-rad", type=float, default=0.35)
    parser.add_argument("--max-scan-points", type=int, default=180)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = TelemetryWriter(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
