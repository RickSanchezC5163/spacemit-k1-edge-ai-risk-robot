#!/usr/bin/env python3
import json
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class EventLoggerNode(Node):
    def __init__(self):
        super().__init__("event_logger_node")
        self.declare_parameter("log_dir", "logs/events")
        self.log_dir = Path(str(self.get_parameter("log_dir").value)).expanduser()
        self.log_ready = True
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self.log_ready = False
            self.get_logger().error(f"Cannot create event log directory {self.log_dir}: {exc}")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = self.log_dir / f"events_{stamp}.jsonl"

        self.latest_level = "unknown"
        self.latest_action = "unknown"
        self.create_subscription(String, "/risk/current_event", self._event_cb, 10)
        self.create_subscription(String, "/risk/current_level", self._level_cb, 10)
        self.create_subscription(String, "/risk/recommended_action", self._action_cb, 10)
        self.get_logger().info(f"Event logger writing JSONL to {self.log_path}")

    def _level_cb(self, msg: String) -> None:
        self.latest_level = msg.data

    def _action_cb(self, msg: String) -> None:
        self.latest_action = msg.data

    def _event_cb(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
            if not isinstance(event, dict):
                raise ValueError("event JSON must be an object")
        except Exception as exc:
            self.get_logger().warn(f"Cannot parse current_event JSON: {exc}")
            event = {"event_type": "unknown", "source": "risk_event_parse_error"}

        record = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "event_type": event.get("event_type") or event.get("type") or "unknown",
            "risk_level": event.get("risk_level") or self.latest_level,
            "recommended_action": event.get("recommended_action") or self.latest_action,
            "distance_m": event.get("distance_m"),
            "confidence": event.get("confidence"),
            "source": event.get("source", "unknown"),
        }

        if not self.log_ready:
            self.get_logger().error(f"Event log disabled; cannot write record: {record}")
            return

        try:
            with self.log_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
            self.get_logger().info(
                f"Logged {record['event_type']} level={record['risk_level']} action={record['recommended_action']}"
            )
        except OSError as exc:
            self.get_logger().error(f"Failed to write event log {self.log_path}: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = EventLoggerNode()
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
