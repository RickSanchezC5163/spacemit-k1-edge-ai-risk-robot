#!/usr/bin/env python3
import json
from datetime import datetime, timezone

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class RiskEngineNode(Node):
    def __init__(self):
        super().__init__("risk_engine_node")
        self.event_pub = self.create_publisher(String, "/risk/current_event", 10)
        self.level_pub = self.create_publisher(String, "/risk/current_level", 10)
        self.action_pub = self.create_publisher(String, "/risk/recommended_action", 10)
        self.create_subscription(String, "/perception/mock_event", self._event_cb, 10)
        self.get_logger().info("Risk engine ready. Waiting for /perception/mock_event JSON.")

    def _event_cb(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
            if not isinstance(event, dict):
                raise ValueError("event JSON must be an object")
        except Exception as exc:
            self.get_logger().warn(f"Invalid mock event JSON: {exc}")
            event = {"event_type": "unknown", "raw": msg.data, "source": "unknown"}

        event_type = str(event.get("event_type") or event.get("type") or "unknown")
        distance_m = self._optional_float(event.get("distance_m", event.get("distance")), default=-1.0)
        confidence = self._optional_float(event.get("confidence"), default=0.0)
        source = str(event.get("source") or "unknown")
        risk_level, action = self._assess(event_type, distance_m)

        enriched = dict(event)
        enriched["timestamp"] = datetime.now(timezone.utc).isoformat()
        enriched["event_type"] = event_type
        enriched["distance_m"] = distance_m
        enriched["confidence"] = confidence
        enriched["risk_level"] = risk_level
        enriched["recommended_action"] = action
        enriched["source"] = source

        self._publish(self.event_pub, json.dumps(enriched, ensure_ascii=False))
        self._publish(self.level_pub, risk_level)
        self._publish(self.action_pub, action)
        self.get_logger().info(f"{event_type}: level={risk_level}, action={action}")

    @staticmethod
    def _optional_float(value, default):
        if value is None:
            return default
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _assess(event_type: str, distance_m):
        has_distance = distance_m >= 0.0
        if event_type == "soft_obstacle" and has_distance and distance_m < 1.0:
            return "medium", "stop_and_recheck"
        if event_type == "hard_obstacle" and has_distance and distance_m < 0.8:
            return "high", "stop_and_report"
        if event_type == "blocked_path":
            return "high", "stop_and_report"
        if event_type == "low_light":
            return "medium", "turn_on_light_and_recheck"
        if event_type == "cable_or_wire" and has_distance and distance_m < 1.2:
            return "high", "stop_and_report"
        if event_type == "reflective_noise":
            return "medium", "slow_down_and_recheck"
        return "low", "continue_with_caution"

    @staticmethod
    def _publish(pub, value: str) -> None:
        msg = String()
        msg.data = value
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RiskEngineNode()
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
