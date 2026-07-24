#!/usr/bin/env python3
"""Bridge ROS brightness commands to the verified K1 PWM7 lamp helper."""

from __future__ import annotations

import subprocess
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32


def action_for_brightness(brightness: int) -> str:
    return "on" if int(brightness) > 0 else "off"


class Pwm7LightNode(Node):
    def __init__(self) -> None:
        super().__init__("pwm7_light_node")
        self.declare_parameter("brightness_topic", "/light/brightness_cmd")
        self.declare_parameter("helper", "/usr/local/sbin/k1-light-mode")
        self.declare_parameter("dry_run", False)
        self.topic = str(self.get_parameter("brightness_topic").value)
        self.helper = Path(str(self.get_parameter("helper").value))
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.last_action: str | None = None
        self.create_subscription(Int32, self.topic, self._brightness_cb, 10)
        self._apply("off")
        self.get_logger().info(
            f"PWM7 hardware light bridge listening on {self.topic}; default=off dry_run={self.dry_run}"
        )

    def _brightness_cb(self, message: Int32) -> None:
        self._apply(action_for_brightness(message.data))

    def _apply(self, action: str) -> None:
        if action == self.last_action:
            return
        if self.dry_run:
            self.get_logger().info(f"Dry-run PWM7 light action: {action}")
        else:
            subprocess.run(
                ["sudo", "-n", str(self.helper), action],
                check=True,
                capture_output=True,
                text=True,
            )
        self.last_action = action
        self.get_logger().info(f"PWM7 light action: {action}")

    def destroy_node(self):
        try:
            self._apply("off")
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Pwm7LightNode()
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
