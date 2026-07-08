#!/usr/bin/env python3
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32, String


class RiskLightBridgeNode(Node):
    def __init__(self):
        super().__init__("risk_light_bridge_node")
        self.declare_parameter("action_topic", "/risk/recommended_action")
        self.declare_parameter("brightness_topic", "/light/brightness_cmd")
        self.declare_parameter("trigger_action", "turn_on_light_and_recheck")
        self.declare_parameter("on_brightness", 5)
        self.declare_parameter("off_brightness", 0)
        self.declare_parameter("hold_seconds", 8.0)
        self.declare_parameter("ramp_step_percent", 5)
        self.declare_parameter("ramp_period_s", 0.2)
        self.declare_parameter("command_period_s", 1.0)
        self.declare_parameter("dry_run", False)

        self.action_topic = str(self.get_parameter("action_topic").value)
        self.brightness_topic = str(self.get_parameter("brightness_topic").value)
        self.trigger_action = str(self.get_parameter("trigger_action").value)
        self.on_brightness = self._clamp(self.get_parameter("on_brightness").value)
        self.off_brightness = self._clamp(self.get_parameter("off_brightness").value)
        self.hold_seconds = max(0.0, float(self.get_parameter("hold_seconds").value))
        self.ramp_step_percent = max(1, int(self.get_parameter("ramp_step_percent").value))
        self.ramp_period_s = max(0.05, float(self.get_parameter("ramp_period_s").value))
        self.command_period_s = max(0.1, float(self.get_parameter("command_period_s").value))
        self.dry_run = self._param_bool(self.get_parameter("dry_run").value)

        self.publisher = self.create_publisher(Int32, self.brightness_topic, 10)
        self.create_subscription(String, self.action_topic, self._action_cb, 10)
        self.create_timer(self.ramp_period_s, self._timer_cb)

        self._target_brightness = self.off_brightness
        self._current_brightness = self.off_brightness
        self._lit_until = 0.0
        self._last_command_time = 0.0
        self._last_published = None

        mode = "dry-run" if self.dry_run else "publishing"
        self.get_logger().info(
            "Risk light bridge started in %s mode: %s -> %s, trigger=%s, "
            "on=%d%%, off=%d%%, hold=%.1fs, ramp_step=%d%%"
            % (
                mode,
                self.action_topic,
                self.brightness_topic,
                self.trigger_action,
                self.on_brightness,
                self.off_brightness,
                self.hold_seconds,
                self.ramp_step_percent,
            )
        )

    @staticmethod
    def _clamp(value) -> int:
        return max(0, min(100, int(value)))

    @staticmethod
    def _param_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _action_cb(self, msg: String) -> None:
        action = msg.data.strip()
        now = time.monotonic()
        if action == self.trigger_action:
            self._target_brightness = self.on_brightness
            self._lit_until = now + self.hold_seconds
            self.get_logger().info(
                f"Risk action {action!r}; target light brightness {self.on_brightness}% "
                f"until idle for {self.hold_seconds:.1f}s."
            )
            return

        if self._target_brightness != self.off_brightness:
            self.get_logger().info(f"Risk action {action!r}; target light brightness off.")
        self._target_brightness = self.off_brightness
        self._lit_until = 0.0

    def _timer_cb(self) -> None:
        now = time.monotonic()
        if self._lit_until and now >= self._lit_until:
            self._target_brightness = self.off_brightness
            self._lit_until = 0.0
            self.get_logger().info("Light hold expired; target light brightness off.")

        if self._current_brightness < self._target_brightness:
            self._current_brightness = min(
                self._target_brightness, self._current_brightness + self.ramp_step_percent
            )
        elif self._current_brightness > self._target_brightness:
            self._current_brightness = max(
                self._target_brightness, self._current_brightness - self.ramp_step_percent
            )

        should_republish = now - self._last_command_time >= self.command_period_s
        changed = self._last_published != self._current_brightness
        if changed or should_republish:
            self._publish_brightness(self._current_brightness)

    def _publish_brightness(self, brightness: int) -> None:
        self._last_command_time = time.monotonic()
        self._last_published = brightness
        if self.dry_run:
            self.get_logger().info(f"Dry-run light brightness command: {brightness}%")
            return

        msg = Int32()
        msg.data = brightness
        self.publisher.publish(msg)
        self.get_logger().info(f"Published light brightness command: {brightness}%")


def main(args=None):
    rclpy.init(args=args)
    node = RiskLightBridgeNode()
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
