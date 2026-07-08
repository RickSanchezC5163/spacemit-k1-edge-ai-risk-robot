#!/usr/bin/env python3
import json
import math
import time
from datetime import datetime, timezone

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraLowLightAdapterNode(Node):
    def __init__(self):
        super().__init__("camera_low_light_adapter_node")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("event_topic", "/perception/mock_event")
        self.declare_parameter("luma_threshold", 55.0)
        self.declare_parameter("dark_pixel_threshold", 50.0)
        self.declare_parameter("dark_ratio_threshold", 0.6)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("resize_width", 320)
        self.declare_parameter("warmup_seconds", 3.0)
        self.declare_parameter("required_consecutive_frames", 2)
        self.declare_parameter("dry_run", False)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.luma_threshold = float(self.get_parameter("luma_threshold").value)
        self.dark_pixel_threshold = float(self.get_parameter("dark_pixel_threshold").value)
        self.dark_ratio_threshold = float(self.get_parameter("dark_ratio_threshold").value)
        self.publish_period_s = 1.0 / max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self.resize_width = max(1, int(self.get_parameter("resize_width").value))
        self.warmup_seconds = max(0.0, float(self.get_parameter("warmup_seconds").value))
        self.required_consecutive_frames = max(
            1, int(self.get_parameter("required_consecutive_frames").value)
        )
        self.dry_run = self._param_bool(self.get_parameter("dry_run").value)

        self.publisher = self.create_publisher(String, self.event_topic, 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)
        self._start_time = time.monotonic()
        self._last_publish_time = 0.0
        self._last_process_time = 0.0
        self._last_decode_warn = 0.0
        self._low_light_streak = 0

        self.get_logger().info(
            "Camera low-light adapter subscribing %s, publishing %s, luma<%.1f, "
            "dark_ratio>%.2f, warmup=%.1fs, consecutive_frames=%d, dry_run=%s"
            % (
                self.image_topic,
                self.event_topic,
                self.luma_threshold,
                self.dark_ratio_threshold,
                self.warmup_seconds,
                self.required_consecutive_frames,
                self.dry_run,
            )
        )

    @staticmethod
    def _param_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _image_cb(self, image: Image) -> None:
        now = time.monotonic()
        if now - self._last_process_time < self.publish_period_s:
            return
        self._last_process_time = now

        try:
            mean_luma, dark_ratio = self._luma_stats(image)
        except Exception as exc:
            if now - self._last_decode_warn > 5.0:
                self.get_logger().warn(f"Cannot process image frame: {exc}")
                self._last_decode_warn = now
            return

        low_light = mean_luma < self.luma_threshold or dark_ratio > self.dark_ratio_threshold
        if not low_light:
            self._low_light_streak = 0
            self.get_logger().debug(
                f"mean_luma={mean_luma:.1f}, dark_pixel_ratio={dark_ratio:.2f}, no low-light event."
            )
            return

        if now - self._start_time < self.warmup_seconds:
            self._low_light_streak = 0
            self.get_logger().debug(
                f"mean_luma={mean_luma:.1f}, dark_pixel_ratio={dark_ratio:.2f}, "
                "ignored during camera warmup."
            )
            return

        self._low_light_streak += 1
        if self._low_light_streak < self.required_consecutive_frames:
            self.get_logger().debug(
                f"mean_luma={mean_luma:.1f}, dark_pixel_ratio={dark_ratio:.2f}, "
                f"low-light candidate {self._low_light_streak}/{self.required_consecutive_frames}."
            )
            return

        if now - self._last_publish_time < self.publish_period_s:
            return

        confidence = self._confidence(mean_luma, dark_ratio)
        event = {
            "event_type": "low_light",
            "distance_m": -1,
            "confidence": confidence,
            "source": "camera_luma",
            "mean_luma": round(float(mean_luma), 2),
            "dark_pixel_ratio": round(float(dark_ratio), 3),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        self._last_publish_time = now
        self.get_logger().info(
            f"mean_luma={mean_luma:.1f}, dark_pixel_ratio={dark_ratio:.2f} -> low_light"
        )
        if not self.dry_run:
            msg = String()
            msg.data = json.dumps(event, ensure_ascii=False)
            self.publisher.publish(msg)

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
            # Approximate YUYV by sampling the luma byte for the addressed pixel.
            return {"stride": 2, "fn": lambda data, offset: float(data[offset])}
        raise ValueError(f"unsupported image encoding: {encoding!r}")

    def _confidence(self, mean_luma: float, dark_ratio: float) -> float:
        luma_score = max(0.0, min(1.0, (self.luma_threshold - mean_luma) / max(self.luma_threshold, 1.0)))
        ratio_score = max(0.0, min(1.0, dark_ratio))
        return round(max(0.8, min(0.98, 0.8 + 0.18 * max(luma_score, ratio_score))), 3)


def main(args=None):
    rclpy.init(args=args)
    node = CameraLowLightAdapterNode()
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
