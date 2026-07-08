#!/usr/bin/env python3
import json
import math
import time
from datetime import datetime, timezone

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ScanEventAdapterNode(Node):
    def __init__(self):
        super().__init__("scan_event_adapter_node")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("event_topic", "/perception/mock_event")
        self.declare_parameter("front_sector_deg", 30.0)
        self.declare_parameter("soft_threshold_m", 1.0)
        self.declare_parameter("blocked_threshold_m", 0.5)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("min_valid_range_m", 0.05)
        self.declare_parameter("max_valid_range_m", 8.0)
        self.declare_parameter("dry_run", False)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.front_sector_deg = float(self.get_parameter("front_sector_deg").value)
        self.soft_threshold_m = float(self.get_parameter("soft_threshold_m").value)
        self.blocked_threshold_m = float(self.get_parameter("blocked_threshold_m").value)
        self.publish_period_s = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.min_valid_range_m = float(self.get_parameter("min_valid_range_m").value)
        self.max_valid_range_m = float(self.get_parameter("max_valid_range_m").value)
        self.dry_run = self._param_bool(self.get_parameter("dry_run").value)

        self.publisher = self.create_publisher(String, self.event_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self._last_publish_time = 0.0
        self._last_no_valid_warn = 0.0

        self.get_logger().info(
            "Scan adapter subscribing %s, publishing %s, front_sector=%.1f deg, "
            "soft<%.2fm, blocked<%.2fm, dry_run=%s"
            % (
                self.scan_topic,
                self.event_topic,
                self.front_sector_deg,
                self.soft_threshold_m,
                self.blocked_threshold_m,
                self.dry_run,
            )
        )

    @staticmethod
    def _param_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _scan_cb(self, scan: LaserScan) -> None:
        now = time.monotonic()
        if now - self._last_publish_time < self.publish_period_s:
            return

        try:
            front_min = self._front_min_range(scan)
        except Exception as exc:
            self.get_logger().warn(f"Cannot process LaserScan frame: {exc}")
            return

        if front_min is None:
            if now - self._last_no_valid_warn > 5.0:
                self.get_logger().warn("No valid /scan ranges in configured front sector.")
                self._last_no_valid_warn = now
            return

        event_type = None
        confidence = 0.0
        if front_min < self.blocked_threshold_m:
            event_type = "blocked_path"
            confidence = 0.9
        elif front_min < self.soft_threshold_m:
            event_type = "soft_obstacle"
            confidence = 0.75

        if event_type is None:
            self.get_logger().debug(f"front_min={front_min:.2f}m, no risk event.")
            return

        event = {
            "event_type": event_type,
            "distance_m": round(float(front_min), 3),
            "confidence": confidence,
            "source": "n10p_scan",
            "front_min_range_m": round(float(front_min), 3),
            "front_sector_deg": self.front_sector_deg,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        self._last_publish_time = now
        self.get_logger().info(f"front_min={front_min:.2f}m -> {event_type}")
        if not self.dry_run:
            msg = String()
            msg.data = json.dumps(event, ensure_ascii=False)
            self.publisher.publish(msg)

    def _front_min_range(self, scan: LaserScan):
        if not scan.ranges:
            return None
        if not math.isfinite(scan.angle_increment) or abs(scan.angle_increment) < 1e-9:
            raise ValueError("LaserScan angle_increment is zero or invalid")

        half_sector_rad = math.radians(max(self.front_sector_deg, 0.0) / 2.0)
        front_values = []
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            value_f = float(value)
            if value_f < self.min_valid_range_m or value_f > self.max_valid_range_m:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            if abs(self._normalize_angle(angle)) <= half_sector_rad:
                front_values.append(value_f)
        if not front_values:
            return None
        return min(front_values)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = ScanEventAdapterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
