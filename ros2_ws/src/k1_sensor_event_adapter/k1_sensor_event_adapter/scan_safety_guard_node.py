#!/usr/bin/env python3
import json
import math
import time
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ScanSafetyGuardNode(Node):
    def __init__(self):
        super().__init__("scan_safety_guard_node")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("input_cmd_topic", "/input_cmd_vel")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/safety/front_obstacle")
        self.declare_parameter("event_topic", "/perception/mock_event")
        self.declare_parameter("stop_request_topic", "/chassis/stop_request")
        self.declare_parameter("front_sector_deg", 35.0)
        self.declare_parameter("hard_stop_m", 1.00)
        self.declare_parameter("slow_down_m", 1.60)
        self.declare_parameter("slow_clear_m", 1.75)
        self.declare_parameter("emergency_stop_m", 0.45)
        self.declare_parameter("soft_max_linear", 0.30)
        self.declare_parameter("clear_max_linear", 0.30)
        self.declare_parameter("min_effective_forward", 0.28)
        self.declare_parameter("approach_stop_m", 1.60)
        self.declare_parameter("approach_rate_stop_mps", 0.35)
        self.declare_parameter("ttc_stop_s", 1.20)
        self.declare_parameter("enable_dynamic_stop", False)
        self.declare_parameter("hard_stop_latch_s", 1.50)
        self.declare_parameter("min_front_valid_count", 3)
        self.declare_parameter("min_valid_range_m", 0.05)
        self.declare_parameter("max_valid_range_m", 8.0)
        self.declare_parameter("scan_timeout_s", 0.6)
        self.declare_parameter("cmd_timeout_s", 0.35)
        self.declare_parameter("output_rate_hz", 50.0)
        self.declare_parameter("event_rate_hz", 1.0)
        self.declare_parameter("fail_closed_without_scan", True)
        self.declare_parameter("publish_events", True)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.input_cmd_topic = str(self.get_parameter("input_cmd_topic").value)
        self.output_cmd_topic = str(self.get_parameter("output_cmd_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.stop_request_topic = str(self.get_parameter("stop_request_topic").value)
        self.front_sector_deg = float(self.get_parameter("front_sector_deg").value)
        self.hard_stop_m = float(self.get_parameter("hard_stop_m").value)
        self.slow_down_m = float(self.get_parameter("slow_down_m").value)
        self.slow_clear_m = float(self.get_parameter("slow_clear_m").value)
        self.emergency_stop_m = float(self.get_parameter("emergency_stop_m").value)
        self.soft_max_linear = float(self.get_parameter("soft_max_linear").value)
        self.clear_max_linear = float(self.get_parameter("clear_max_linear").value)
        self.min_effective_forward = float(self.get_parameter("min_effective_forward").value)
        self.approach_stop_m = float(self.get_parameter("approach_stop_m").value)
        self.approach_rate_stop_mps = float(self.get_parameter("approach_rate_stop_mps").value)
        self.ttc_stop_s = float(self.get_parameter("ttc_stop_s").value)
        self.enable_dynamic_stop = self._param_bool(self.get_parameter("enable_dynamic_stop").value)
        self.hard_stop_latch_s = float(self.get_parameter("hard_stop_latch_s").value)
        self.min_front_valid_count = int(self.get_parameter("min_front_valid_count").value)
        self.min_valid_range_m = float(self.get_parameter("min_valid_range_m").value)
        self.max_valid_range_m = float(self.get_parameter("max_valid_range_m").value)
        self.scan_timeout_s = float(self.get_parameter("scan_timeout_s").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.event_period_s = 1.0 / max(float(self.get_parameter("event_rate_hz").value), 0.1)
        output_period_s = 1.0 / max(float(self.get_parameter("output_rate_hz").value), 1.0)
        self.fail_closed_without_scan = self._param_bool(
            self.get_parameter("fail_closed_without_scan").value
        )
        self.publish_events = self._param_bool(self.get_parameter("publish_events").value)

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.event_pub = self.create_publisher(String, self.event_topic, 10)
        self.stop_request_pub = self.create_publisher(String, self.stop_request_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.create_subscription(Twist, self.input_cmd_topic, self._cmd_cb, 10)
        self.create_timer(output_period_s, self._output_tick)

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.last_scan_time = 0.0
        self.front_min = None
        self.front_p10 = None
        self.front_valid_count = 0
        self.prev_front_p10 = None
        self.prev_front_time = 0.0
        self.approach_rate_mps = 0.0
        self.ttc_s = math.inf
        self.hard_stop_latch_until = 0.0
        self.state = "stale_scan" if self.fail_closed_without_scan else "clear"
        self.last_logged_state = self.state
        self.last_status_json = None
        self.last_event_time = 0.0
        self.last_status_time = 0.0
        self.last_diagnostic_log_time = 0.0
        self.last_stop_request_time = 0.0
        self.hard_stop_stop_request_sent = False

        self.get_logger().info(
            "Scan safety guard: %s -> %s using %s, hard<%.2fm, slow<%.2fm"
            % (
                self.input_cmd_topic,
                self.output_cmd_topic,
                self.scan_topic,
                self.hard_stop_m,
                self.slow_down_m,
            )
        )

    @staticmethod
    def _param_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def _scan_cb(self, scan: LaserScan) -> None:
        try:
            now = time.monotonic()
            stats = self._front_stats(scan)
            self.front_min = stats["front_min"]
            self.front_p10 = stats["front_p10"]
            self.front_valid_count = stats["valid_count"]
            self._update_approach_rate(self.front_p10, now)
            self._update_ttc()
            self.last_scan_time = now
            self._update_state(now)
        except Exception as exc:
            self.get_logger().warn(f"Cannot process LaserScan frame: {exc}")

    def _output_tick(self) -> None:
        now = time.monotonic()
        cmd_fresh = now - self.last_cmd_time <= self.cmd_timeout_s
        scan_fresh = now - self.last_scan_time <= self.scan_timeout_s

        if not scan_fresh and self.fail_closed_without_scan:
            self.state = "stale_scan"
            self._log_state_transition("scan_timeout")

        source = self.last_cmd if cmd_fresh else Twist()
        filtered, action = self._filter_cmd(source, scan_fresh, now)
        self.cmd_pub.publish(filtered)
        self._publish_status(now, scan_fresh, cmd_fresh, action)

    def _filter_cmd(self, source: Twist, scan_fresh: bool, now: float):
        result = Twist()
        result.linear.x = float(source.linear.x)
        result.linear.y = float(source.linear.y)
        result.linear.z = float(source.linear.z)
        result.angular.x = float(source.angular.x)
        result.angular.y = float(source.angular.y)
        result.angular.z = float(source.angular.z)

        if not scan_fresh and self.fail_closed_without_scan:
            if result.linear.x > 0.0:
                self._publish_stop_request(now, "stale_scan", source)
            return Twist(), "stale_scan_zero"

        if self.state == "hard_stop" and result.linear.x > 0.0:
            if not self.hard_stop_stop_request_sent:
                self._publish_stop_request(now, "hard_stop", source)
                self.hard_stop_stop_request_sent = True
                return Twist(), "hard_stop_stop_request"
            return Twist(), "hard_stop_latched_zero"

        if result.linear.x > 0.0:
            if result.linear.x < self.min_effective_forward:
                return Twist(), "below_min_motion_zero"
            limit = self._forward_limit()
            if limit <= 0.0:
                return Twist(), "distance_brake_zero"
            if result.linear.x > limit:
                result.linear.x = limit
                return result, "limit_forward_speed"

        return result, "pass"

    def _publish_stop_request(self, now: float, reason: str, source: Twist) -> None:
        if now - self.last_stop_request_time < 0.10:
            return
        request = {
            "request": "STOP_REQUEST",
            "reason": reason,
            "source": "scan_safety_guard",
            "input_linear_x": round(float(source.linear.x), 3),
            "input_angular_z": round(float(source.angular.z), 3),
            "front_min_range_m": None if self.front_min is None else round(float(self.front_min), 3),
            "front_p10_range_m": None if self.front_p10 is None else round(float(self.front_p10), 3),
            "front_valid_count": int(self.front_valid_count),
            "approach_rate_mps": round(float(self.approach_rate_mps), 3),
            "ttc_s": None if math.isinf(self.ttc_s) else round(float(self.ttc_s), 3),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        self.stop_request_pub.publish(msg)
        self.last_stop_request_time = now

    def _forward_limit(self) -> float:
        if self.front_min is None:
            return self.clear_max_linear
        if self._hard_condition():
            return 0.0
        if self.state == "warning":
            return self.soft_max_linear
        return self.clear_max_linear

    def _update_state(self, now: float) -> None:
        if self.front_valid_count < self.min_front_valid_count or self.front_p10 is None:
            return
        old_state = self.state
        if self._hard_condition():
            if old_state != "hard_stop":
                self.hard_stop_stop_request_sent = False
            self.state = "hard_stop"
            self.hard_stop_latch_until = max(
                self.hard_stop_latch_until, now + self.hard_stop_latch_s
            )
            if self.state != old_state:
                self._log_state_transition("hard_condition")
            return

        if self.state == "hard_stop":
            if now < self.hard_stop_latch_until:
                self._log_state_transition("latched")
                return
            self.state = "warning" if self.front_p10 < self.slow_clear_m else "clear"
            self.hard_stop_stop_request_sent = False
            self._log_state_transition("scan")
            return
        if self.front_p10 < self.slow_down_m:
            self.state = "warning"
        else:
            self.state = "clear"
        if self.state != old_state:
            self._log_state_transition("scan")

    def _hard_condition(self) -> bool:
        if self.front_min is not None and self.front_min <= self.emergency_stop_m:
            return True
        if self.front_p10 is None or self.front_valid_count < self.min_front_valid_count:
            return False
        if self.front_p10 <= self.hard_stop_m:
            return True
        if not self.enable_dynamic_stop:
            return False
        forward_cmd_active = (
            time.monotonic() - self.last_cmd_time <= self.cmd_timeout_s
            and float(self.last_cmd.linear.x) > 0.0
        )
        inside_approach_zone = self.front_p10 < self.approach_stop_m
        if (
            forward_cmd_active
            and inside_approach_zone
            and self.approach_rate_mps > self.approach_rate_stop_mps
        ):
            return True
        if forward_cmd_active and inside_approach_zone and self.ttc_s < self.ttc_stop_s:
            return True
        return False

    def _update_approach_rate(self, new_front_p10, now: float) -> None:
        if new_front_p10 is None:
            self.approach_rate_mps = 0.0
            return
        if self.prev_front_p10 is not None and self.prev_front_time > 0.0:
            dt = max(now - self.prev_front_time, 1e-3)
            # Positive means the nearest front obstacle is getting closer.
            self.approach_rate_mps = max(0.0, (self.prev_front_p10 - new_front_p10) / dt)
        self.prev_front_p10 = new_front_p10
        self.prev_front_time = now

    def _update_ttc(self) -> None:
        if self.front_p10 is None or self.approach_rate_mps <= 1e-6:
            self.ttc_s = math.inf
            return
        self.ttc_s = self.front_p10 / self.approach_rate_mps

    def _log_state_transition(self, reason: str) -> None:
        if self.state == self.last_logged_state:
            return
        front_min_text = "none" if self.front_min is None else f"{self.front_min:.3f}m"
        front_p10_text = "none" if self.front_p10 is None else f"{self.front_p10:.3f}m"
        self.get_logger().info(
            "guard_state %s -> %s reason=%s front_min=%s front_p10=%s "
            "valid_count=%d approach_rate=%.3fm/s ttc=%s latch_remaining=%.3fs"
            % (
                self.last_logged_state,
                self.state,
                reason,
                front_min_text,
                front_p10_text,
                self.front_valid_count,
                self.approach_rate_mps,
                self._format_ttc(),
                self._latch_remaining(),
            )
        )
        self.last_logged_state = self.state

    def _log_diagnostics(self, now: float) -> None:
        if now - self.last_diagnostic_log_time < 1.0:
            return
        self.last_diagnostic_log_time = now
        self.get_logger().info(
            "guard_status state=%s front_min=%s front_p10=%s valid_count=%d "
            "approach_rate=%.3fm/s ttc=%s latch_remaining=%.3fs"
            % (
                self.state,
                "none" if self.front_min is None else f"{self.front_min:.3f}m",
                "none" if self.front_p10 is None else f"{self.front_p10:.3f}m",
                self.front_valid_count,
                self.approach_rate_mps,
                self._format_ttc(),
                self._latch_remaining(),
            )
        )

    def _format_ttc(self) -> str:
        return "inf" if math.isinf(self.ttc_s) else f"{self.ttc_s:.3f}s"

    def _latch_remaining(self) -> float:
        return max(0.0, self.hard_stop_latch_until - time.monotonic())

    def _publish_status(self, now: float, scan_fresh: bool, cmd_fresh: bool, action: str) -> None:
        status = {
            "state": self.state,
            "action": action,
            "front_min_range_m": None if self.front_min is None else round(float(self.front_min), 3),
            "front_p10_range_m": None if self.front_p10 is None else round(float(self.front_p10), 3),
            "front_valid_count": int(self.front_valid_count),
            "scan_fresh": bool(scan_fresh),
            "cmd_fresh": bool(cmd_fresh),
            "hard_stop_m": self.hard_stop_m,
            "slow_down_m": self.slow_down_m,
            "emergency_stop_m": self.emergency_stop_m,
            "clear_max_linear": self.clear_max_linear,
            "soft_max_linear": self.soft_max_linear,
            "min_effective_forward": self.min_effective_forward,
            "enable_dynamic_stop": self.enable_dynamic_stop,
            "approach_rate_mps": round(float(self.approach_rate_mps), 3),
            "ttc_s": None if math.isinf(self.ttc_s) else round(float(self.ttc_s), 3),
            "hard_stop_latch_remaining_s": round(self._latch_remaining(), 3),
            "hard_stop_stop_request_sent": bool(self.hard_stop_stop_request_sent),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        status_json = json.dumps(status, ensure_ascii=False)
        if status_json != self.last_status_json or now - self.last_status_time > 1.0:
            msg = String()
            msg.data = status_json
            self.status_pub.publish(msg)
            self.last_status_json = status_json
            self.last_status_time = now
        self._log_diagnostics(now)

        if not self.publish_events or now - self.last_event_time < self.event_period_s:
            return
        if self.state not in ("hard_stop", "warning"):
            return

        event_type = "blocked_path" if self.state == "hard_stop" else "soft_obstacle"
        event = {
            "event_type": event_type,
            "distance_m": -1.0 if self.front_p10 is None else round(float(self.front_p10), 3),
            "confidence": 0.9 if self.state == "hard_stop" else 0.75,
            "source": "scan_safety_guard",
            "front_min_range_m": None if self.front_min is None else round(float(self.front_min), 3),
            "front_p10_range_m": None if self.front_p10 is None else round(float(self.front_p10), 3),
            "front_valid_count": int(self.front_valid_count),
            "front_sector_deg": self.front_sector_deg,
            "guard_action": action,
            "ttc_s": None if math.isinf(self.ttc_s) else round(float(self.ttc_s), 3),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        msg = String()
        msg.data = json.dumps(event, ensure_ascii=False)
        self.event_pub.publish(msg)
        self.last_event_time = now

    def _front_stats(self, scan: LaserScan):
        if not scan.ranges:
            return {"front_min": None, "front_p10": None, "valid_count": 0}
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
            return {"front_min": None, "front_p10": None, "valid_count": 0}
        front_values.sort()
        p10_index = max(0, min(len(front_values) - 1, math.ceil(len(front_values) * 0.10) - 1))
        return {
            "front_min": front_values[0],
            "front_p10": front_values[p10_index],
            "valid_count": len(front_values),
        }

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = ScanSafetyGuardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
