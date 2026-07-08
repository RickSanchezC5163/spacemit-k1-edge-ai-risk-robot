#!/usr/bin/env python3
import json
import math
import time
from datetime import datetime, timezone

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String


class AdaptiveLightControllerNode(Node):
    def __init__(self):
        super().__init__("adaptive_light_controller_node")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("brightness_topic", "/light/brightness_cmd")
        self.declare_parameter("status_topic", "/light/adaptive_status")
        self.declare_parameter("target_luma", 75.0)
        self.declare_parameter("dark_pixel_threshold", 50.0)
        self.declare_parameter("min_brightness", 0)
        self.declare_parameter("max_brightness", 5)
        self.declare_parameter("update_rate_hz", 1.0)
        self.declare_parameter("step_limit", 5)
        self.declare_parameter("stable_frames", 3)
        self.declare_parameter("resize_width", 320)
        self.declare_parameter("image_timeout_s", 3.0)
        self.declare_parameter("enable_auto_light", True)
        self.declare_parameter("dry_run", False)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.brightness_topic = str(self.get_parameter("brightness_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.target_luma = float(self.get_parameter("target_luma").value)
        self.dark_pixel_threshold = float(self.get_parameter("dark_pixel_threshold").value)
        self.min_brightness = self._clamp(self.get_parameter("min_brightness").value)
        self.max_brightness = min(25, self._clamp(self.get_parameter("max_brightness").value))
        self.update_period_s = 1.0 / max(float(self.get_parameter("update_rate_hz").value), 0.1)
        self.step_limit = max(1, int(self.get_parameter("step_limit").value))
        self.stable_frames = max(1, int(self.get_parameter("stable_frames").value))
        self.resize_width = max(1, int(self.get_parameter("resize_width").value))
        self.image_timeout_s = max(0.5, float(self.get_parameter("image_timeout_s").value))
        self.enable_auto_light = self._param_bool(self.get_parameter("enable_auto_light").value)
        self.dry_run = self._param_bool(self.get_parameter("dry_run").value)

        if self.min_brightness > self.max_brightness:
            self.min_brightness = self.max_brightness

        self.command_pub = self.create_publisher(Int32, self.brightness_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)
        self.create_timer(self.update_period_s, self._timer_cb)

        self._last_image_time = 0.0
        self._last_stats = None
        self._current_brightness = 0
        self._target_brightness = 0
        self._candidate_brightness = None
        self._candidate_count = 0
        self._last_published = None
        self._last_decode_warn = 0.0
        self._last_timeout_warn = 0.0

        mode = "dry-run" if self.dry_run else "publishing"
        self.get_logger().info(
            "Adaptive light controller started in %s mode: image=%s, cmd=%s, "
            "status=%s, luma_target=%.1f, brightness=%d-%d, rate=%.2fHz, "
            "step=%d, stable_frames=%d, timeout=%.1fs"
            % (
                mode,
                self.image_topic,
                self.brightness_topic,
                self.status_topic,
                self.target_luma,
                self.min_brightness,
                self.max_brightness,
                1.0 / self.update_period_s,
                self.step_limit,
                self.stable_frames,
                self.image_timeout_s,
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

    def _image_cb(self, image: Image) -> None:
        now = time.monotonic()
        try:
            mean_luma, dark_ratio = self._luma_stats(image)
        except Exception as exc:
            if now - self._last_decode_warn > 5.0:
                self.get_logger().warn(f"Cannot process image frame: {exc}")
                self._last_decode_warn = now
            return

        desired = self._desired_brightness(mean_luma)
        self._last_image_time = now
        self._last_stats = {
            "mean_luma": mean_luma,
            "dark_pixel_ratio": dark_ratio,
            "desired_brightness": desired,
            "encoding": str(image.encoding or ""),
            "width": int(image.width),
            "height": int(image.height),
        }

        if not self.enable_auto_light:
            self._set_target(0)
            return

        if desired == self._candidate_brightness:
            self._candidate_count += 1
        else:
            self._candidate_brightness = desired
            self._candidate_count = 1

        if self._candidate_count >= self.stable_frames:
            self._set_target(desired)

    def _timer_cb(self) -> None:
        now = time.monotonic()
        if self._last_image_time == 0.0 or now - self._last_image_time > self.image_timeout_s:
            if now - self._last_timeout_warn > 3.0:
                self.get_logger().warn("No recent camera image; forcing light brightness to 0.")
                self._last_timeout_warn = now
            self._force_off("image_timeout")
            self._publish_status(reason="image_timeout")
            return

        if self._current_brightness < self._target_brightness:
            self._current_brightness = min(
                self._target_brightness, self._current_brightness + self.step_limit
            )
        elif self._current_brightness > self._target_brightness:
            self._current_brightness = max(
                self._target_brightness, self._current_brightness - self.step_limit
            )

        self._publish_brightness(self._current_brightness)
        self._publish_status(reason="adaptive")

    def _set_target(self, brightness: int) -> None:
        bounded = max(self.min_brightness, min(self.max_brightness, int(brightness)))
        if bounded != self._target_brightness:
            self.get_logger().info(
                f"Adaptive light target {self._target_brightness}% -> {bounded}%"
            )
        self._target_brightness = bounded

    def _force_off(self, reason: str) -> None:
        self._target_brightness = 0
        self._current_brightness = 0
        self._candidate_brightness = 0
        self._candidate_count = 0
        self._publish_brightness(0)
        self.get_logger().debug(f"Forced light off: {reason}")

    def _publish_brightness(self, brightness: int) -> None:
        brightness = self._clamp(brightness)
        if brightness == self._last_published:
            return
        self._last_published = brightness
        if self.dry_run:
            self.get_logger().info(f"Dry-run adaptive light brightness command: {brightness}%")
            return
        msg = Int32()
        msg.data = brightness
        self.command_pub.publish(msg)
        self.get_logger().info(f"Adaptive light brightness command: {brightness}%")

    def _publish_status(self, reason: str) -> None:
        stats = self._last_stats or {}
        status = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "reason": reason,
            "enabled": self.enable_auto_light,
            "mean_luma": self._round_or_none(stats.get("mean_luma")),
            "dark_pixel_ratio": self._round_or_none(stats.get("dark_pixel_ratio"), digits=3),
            "desired_brightness": stats.get("desired_brightness"),
            "target_brightness": self._target_brightness,
            "current_brightness": self._current_brightness,
            "min_brightness": self.min_brightness,
            "max_brightness": self.max_brightness,
            "candidate_brightness": self._candidate_brightness,
            "candidate_count": self._candidate_count,
            "stable_frames": self.stable_frames,
            "image_age_s": self._round_or_none(
                time.monotonic() - self._last_image_time if self._last_image_time else None
            ),
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)

    @staticmethod
    def _round_or_none(value, digits=2):
        if value is None:
            return None
        return round(float(value), digits)

    def _desired_brightness(self, mean_luma: float) -> int:
        if mean_luma >= 80.0:
            return 0
        if mean_luma >= 65.0:
            return 10
        if mean_luma >= 50.0:
            return 15
        if mean_luma >= 35.0:
            return 20
        return 25

    def _luma_stats(self, image: Image):
        width = int(image.width)
        height = int(image.height)
        if width <= 0 or height <= 0:
            raise ValueError("image width/height must be positive")

        encoding = str(image.encoding or "").lower()
        data = bytes(image.data)
        step = int(image.step) if int(image.step) > 0 else width
        sample_x = max(1, math.ceil(width / self.resize_width))
        sample_y = sample_x

        pixel_reader = self._pixel_reader(encoding)
        total_luma = 0.0
        dark_count = 0
        count = 0

        for y in range(0, height, sample_y):
            row = y * step
            for x in range(0, width, sample_x):
                offset = row + x * pixel_reader["stride"]
                if offset + pixel_reader["stride"] > len(data):
                    continue
                luma = pixel_reader["fn"](data, offset)
                total_luma += luma
                if luma < self.dark_pixel_threshold:
                    dark_count += 1
                count += 1

        if count == 0:
            raise ValueError(f"no pixels decoded for encoding={image.encoding!r}")
        return total_luma / count, dark_count / count

    @staticmethod
    def _pixel_reader(encoding: str):
        if encoding in ("mono8", "8uc1"):
            return {"stride": 1, "fn": lambda data, offset: float(data[offset])}
        if encoding in ("rgb8", "bgr8"):
            rgb = encoding == "rgb8"

            def read_rgb(data, offset):
                if rgb:
                    r, g, b = data[offset], data[offset + 1], data[offset + 2]
                else:
                    b, g, r = data[offset], data[offset + 1], data[offset + 2]
                return 0.299 * r + 0.587 * g + 0.114 * b

            return {"stride": 3, "fn": read_rgb}
        if encoding in ("rgba8", "bgra8"):
            rgba = encoding == "rgba8"

            def read_rgba(data, offset):
                if rgba:
                    r, g, b = data[offset], data[offset + 1], data[offset + 2]
                else:
                    b, g, r = data[offset], data[offset + 1], data[offset + 2]
                return 0.299 * r + 0.587 * g + 0.114 * b

            return {"stride": 4, "fn": read_rgba}
        if encoding in ("yuyv", "yuyv422", "yuv422"):
            return {"stride": 2, "fn": lambda data, offset: float(data[offset])}
        raise ValueError(f"unsupported image encoding: {encoding!r}")

    def destroy_node(self):
        self._force_off("shutdown")
        time.sleep(0.05)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveLightControllerNode()
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
