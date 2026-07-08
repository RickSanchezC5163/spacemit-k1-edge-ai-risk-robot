#!/usr/bin/env python3
import os
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32


class Gpio37LightNode(Node):
    def __init__(self):
        super().__init__("gpio37_light_node")
        self.declare_parameter("gpio", 37)
        self.declare_parameter("freq_hz", 50.0)
        self.declare_parameter("frequency", 50.0)
        self.declare_parameter("min_us", 1100.0)
        self.declare_parameter("max_us", 1900.0)
        self.declare_parameter("default_brightness", 0)
        self.declare_parameter("status_hz", 1.0)
        self.declare_parameter("dry_run", False)

        self.gpio = str(self.get_parameter("gpio").value)
        frequency = float(self.get_parameter("frequency").value)
        freq_hz = float(self.get_parameter("freq_hz").value)
        self.freq_hz = frequency if frequency > 0 else freq_hz
        self.min_us = float(self.get_parameter("min_us").value)
        self.max_us = float(self.get_parameter("max_us").value)
        default_brightness = int(self.get_parameter("default_brightness").value)
        self.dry_run = self._param_bool(self.get_parameter("dry_run").value)

        self.gpio_dir = f"/sys/class/gpio/gpio{self.gpio}"
        self.gpio_value = f"{self.gpio_dir}/value"
        self.gpio_direction = f"{self.gpio_dir}/direction"
        self.period_s = 1.0 / max(self.freq_hz, 1.0)
        self._brightness = self._clamp(default_brightness)
        self._lock = threading.Lock()
        self._running = True
        self._gpio_ok = False

        self.status_pub = self.create_publisher(Int32, "/light/status", 10)
        self.create_subscription(Int32, "/light/brightness_cmd", self._brightness_cb, 10)
        status_period = 1.0 / max(float(self.get_parameter("status_hz").value), 0.1)
        self.create_timer(status_period, self._publish_status)

        self._setup_gpio()
        self._thread = threading.Thread(target=self._pwm_loop, daemon=True)
        self._thread.start()
        mode = "dry-run" if self.dry_run else "gpio"
        self.get_logger().info(
            f"GPIO{self.gpio} light node started in {mode} mode with brightness 0 by default."
        )

    @staticmethod
    def _clamp(value) -> int:
        return max(0, min(100, int(value)))

    @staticmethod
    def _param_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _write(self, path: str, value: str) -> None:
        with open(path, "w", encoding="ascii") as f:
            f.write(str(value))

    def _setup_gpio(self) -> None:
        if self.dry_run:
            self.get_logger().warn("Light node dry_run=true; GPIO writes are disabled.")
            self._gpio_ok = False
            return
        try:
            if not os.path.isdir(self.gpio_dir):
                try:
                    self._write("/sys/class/gpio/export", self.gpio)
                    time.sleep(0.05)
                except OSError:
                    pass
            self._write(self.gpio_direction, "out")
            self._write(self.gpio_value, "0")
            self._gpio_ok = True
        except OSError as exc:
            self._gpio_ok = False
            self.get_logger().error(
                f"Cannot configure GPIO{self.gpio}: {exc}. Run with GPIO permission or sudo."
            )

    def _set_low(self) -> None:
        if self.dry_run or not self._gpio_ok:
            return
        try:
            self._write(self.gpio_value, "0")
        except OSError as exc:
            self.get_logger().warn(f"Failed to pull GPIO{self.gpio} low: {exc}")

    def _brightness_cb(self, msg: Int32) -> None:
        value = self._clamp(msg.data)
        with self._lock:
            self._brightness = value
        self.get_logger().info(f"Light brightness command: {msg.data} -> {value}%")

    def _publish_status(self) -> None:
        with self._lock:
            value = self._brightness
        msg = Int32()
        msg.data = value
        self.status_pub.publish(msg)

    def _brightness_to_pulse_us(self, brightness: int) -> float:
        return self.min_us + (self.max_us - self.min_us) * (brightness / 100.0)

    def _pwm_loop(self) -> None:
        while self._running:
            with self._lock:
                brightness = self._brightness
            if self.dry_run:
                time.sleep(0.05)
                continue
            if not self._gpio_ok or brightness <= 0:
                self._set_low()
                time.sleep(0.05)
                continue

            pulse_s = min(self.period_s, self._brightness_to_pulse_us(brightness) / 1_000_000.0)
            low_s = max(0.0, self.period_s - pulse_s)
            try:
                self._write(self.gpio_value, "1")
                time.sleep(pulse_s)
                self._write(self.gpio_value, "0")
                time.sleep(low_s)
            except OSError as exc:
                self.get_logger().warn(f"GPIO{self.gpio} PWM write failed: {exc}")
                time.sleep(0.5)

        self._set_low()

    def destroy_node(self):
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=1.0)
        self._set_low()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Gpio37LightNode()
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
