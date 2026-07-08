#!/usr/bin/env python3
import argparse
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


MAX_LINEAR = 0.30
MAX_ANGULAR = 0.0
DEFAULT_RATE = 50.0
DEFAULT_MAX_WALL_TIME = 5.0
DEFAULT_ZERO_SECONDS = 8.0
WAIT_FOR_SUBSCRIBER_SEC = 3.0

STOP_START_RE = re.compile(
    r"stop_kick_start reason=(?P<reason>\S+) "
    r"prev=\((?P<prev_vx>-?\d+\.\d+),(?P<prev_wz>-?\d+\.\d+)\) "
    r"kick=\((?P<kick_vx>-?\d+\.\d+),(?P<kick_wz>-?\d+\.\d+)\) "
    r"duration=(?P<duration>\d+\.\d+)s "
    r"feedback_start=\((?P<fb_vx>-?\d+\.\d+),(?P<fb_wz>-?\d+\.\d+)\)"
)
STOP_END_RE = re.compile(
    r"stop_kick_end phase=(?P<phase>\S+) elapsed=(?P<elapsed>\d+\.\d+)s "
    r"feedback_now=\((?P<fb_vx>-?\d+\.\d+),(?P<fb_wz>-?\d+\.\d+)\)"
)
BASE_STOP_REQUEST_RE = re.compile(
    r"stop_request reason=(?P<reason>\S+) "
    r"prev_serial=\((?P<prev_vx>-?\d+\.\d+),(?P<prev_wz>-?\d+\.\d+)\) "
    r"serial_duration=(?P<serial_duration>\d+\.\d+)s "
    r"front_p10=(?P<front_p10>-?\d+\.?\d*|None)"
)
DIAG_RE = re.compile(
    r"diag .*serial=\((?P<serial_vx>-?\d+\.\d+),(?P<serial_wz>-?\d+\.\d+)\) "
    r"feedback=\((?P<fb_vx>-?\d+\.\d+),(?P<fb_wz>-?\d+\.\d+)\)"
)


def reject(message: str) -> None:
    print(f"REFUSE: {message}", file=sys.stderr)
    sys.exit(2)


def as_float_dict(match):
    data = match.groupdict()
    for key, value in list(data.items()):
        if key not in ("reason", "phase"):
            data[key] = None if value == "None" else float(value)
    return data


class GuardStopRequestValidation(Node):
    def __init__(self, args):
        super().__init__("guard_stop_request_validation")
        self.args = args
        self.input_pub = self.create_publisher(Twist, args.input_cmd_topic, 10)
        self.guarded_pub = self.create_publisher(Twist, args.guarded_cmd_topic, 10)
        self.stop_request_pub = self.create_publisher(String, args.stop_request_topic, 10)
        self.status = None
        self.last_status_time = 0.0
        self.status_samples = []
        self.state_changes = []
        self.last_state = None
        self.stop_requests = []
        self.guarded_samples = []
        self.start_odom = None
        self.latest_odom = None
        self.create_subscription(String, args.status_topic, self.status_cb, 30)
        self.create_subscription(String, args.stop_request_topic, self.stop_request_cb, 30)
        self.create_subscription(Twist, args.guarded_cmd_topic, self.guarded_cb, 30)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 20)

    def status_cb(self, msg: String) -> None:
        now = time.monotonic()
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.status = status
        self.last_status_time = now
        self.status_samples.append((now, status))
        state = status.get("state")
        if state != self.last_state:
            self.state_changes.append(
                (
                    now,
                    state,
                    status.get("front_min_range_m"),
                    status.get("front_p10_range_m"),
                    status.get("approach_rate_mps"),
                    status.get("ttc_s"),
                )
            )
            self.last_state = state

    def stop_request_cb(self, msg: String) -> None:
        self.stop_requests.append((time.monotonic(), msg.data))

    def guarded_cb(self, msg: Twist) -> None:
        self.guarded_samples.append((time.monotonic(), float(msg.linear.x), float(msg.angular.z)))

    def odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if self.start_odom is None:
            self.start_odom = msg

    def wait_ready(self):
        start = time.monotonic()
        while self.input_pub.get_subscription_count() < 1 and time.monotonic() - start < WAIT_FOR_SUBSCRIBER_SEC:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.input_pub.get_subscription_count() < 1:
            reject(f"no subscriber on {self.args.input_cmd_topic}")

        start = time.monotonic()
        while self.status is None and time.monotonic() - start < WAIT_FOR_SUBSCRIBER_SEC:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.status is None:
            reject(f"no status on {self.args.status_topic}")

    def publish_zero_for(self, duration: float) -> None:
        zero = Twist()
        end = time.monotonic() + duration
        period = 1.0 / self.args.rate
        while time.monotonic() < end:
            self.input_pub.publish(zero)
            self.guarded_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self.input_pub.publish(zero)
        self.guarded_pub.publish(zero)

    def publish_emergency_stop_request(self, reason: str) -> None:
        status = self.status or {}
        request = {
            "request": "STOP_REQUEST",
            "reason": reason,
            "source": "guard_stop_request_validation",
            "input_linear_x": round(float(self.args.linear), 3),
            "input_angular_z": round(float(self.args.angular), 3),
            "front_min_range_m": status.get("front_min_range_m"),
            "front_p10_range_m": status.get("front_p10_range_m"),
            "front_valid_count": status.get("front_valid_count"),
            "approach_rate_mps": status.get("approach_rate_mps"),
            "ttc_s": status.get("ttc_s"),
            "timestamp_monotonic": round(time.monotonic(), 3),
        }
        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        for _ in range(10):
            self.stop_request_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def odom_forward_distance(self) -> float:
        if self.start_odom is None or self.latest_odom is None:
            return 0.0
        sx = self.start_odom.pose.pose.position.x
        sy = self.start_odom.pose.pose.position.y
        lx = self.latest_odom.pose.pose.position.x
        ly = self.latest_odom.pose.pose.position.y
        return math.hypot(lx - sx, ly - sy)

    def emergency_reason(self, now: float):
        if self.last_status_time and now - self.last_status_time > self.args.no_status_timeout_s:
            return f"no_status_{now - self.last_status_time:.2f}s"
        status = self.status or {}
        front_min = status.get("front_min_range_m")
        front_p10 = status.get("front_p10_range_m")
        if front_min is not None and front_min <= self.args.emergency_front_min_m:
            return f"front_min_{front_min:.3f}_m"
        if front_p10 is not None and front_p10 <= self.args.emergency_front_p10_m:
            return f"front_p10_{front_p10:.3f}_m"
        odom_distance = self.odom_forward_distance()
        if odom_distance >= self.args.max_odom_forward_m:
            return f"odom_forward_{odom_distance:.3f}_m"
        return None

    def run(self):
        log_path = Path(self.args.base_log)
        if not log_path.exists():
            reject(f"base log does not exist: {log_path}")
        log_offset = log_path.stat().st_size
        self.wait_ready()

        start_status = dict(self.status)
        print("START_STATUS", json.dumps(start_status, ensure_ascii=False))
        if start_status.get("state") == "hard_stop":
            reject("guard is already hard_stop before motion; move obstacle farther or reset latch")

        cmd = Twist()
        cmd.linear.x = self.args.linear
        cmd.angular.z = self.args.angular
        start = time.monotonic()
        period = 1.0 / self.args.rate
        trigger_time = None
        trigger_reason = None
        emergency = None

        print(
            f"RUN guard_stop_request linear={self.args.linear:.3f} "
            f"max_wall_time={self.args.max_wall_time_s:.2f}s"
        )
        while True:
            now = time.monotonic()
            elapsed = now - start
            rclpy.spin_once(self, timeout_sec=0.0)
            emergency = self.emergency_reason(now)
            if emergency is not None:
                trigger_time = elapsed
                trigger_reason = f"EMERGENCY:{emergency}"
                break
            if self.stop_requests:
                trigger_time = elapsed
                trigger_reason = "STOP_REQUEST_TOPIC"
                break
            if elapsed >= self.args.max_wall_time_s:
                trigger_time = elapsed
                trigger_reason = "MAX_WALL_TIME"
                break
            self.input_pub.publish(cmd)
            time.sleep(period)

        if trigger_reason and trigger_reason.startswith("EMERGENCY"):
            print(f"EMERGENCY_STOP reason={trigger_reason} t={trigger_time:.3f}s")
            self.publish_emergency_stop_request(trigger_reason)
            self.publish_zero_for(self.args.zero_seconds)
            if self.args.kill_base_on_emergency:
                subprocess.run(["pkill", "-f", "wheeltec_tank_base_safe.py"], check=False)
        else:
            print(f"TRIGGER reason={trigger_reason} t={trigger_time:.3f}s")
            self.publish_zero_for(self.args.zero_seconds)

        time.sleep(self.args.log_wait_s)
        rclpy.spin_once(self, timeout_sec=0.1)
        appended_log = log_path.read_text(errors="ignore")[log_offset:]
        self.print_report(start, trigger_reason, appended_log)

    def print_report(self, start_time: float, trigger_reason: str, appended_log: str) -> None:
        starts = [as_float_dict(m) for m in STOP_START_RE.finditer(appended_log)]
        ends = [as_float_dict(m) for m in STOP_END_RE.finditer(appended_log)]
        base_stop_requests = [as_float_dict(m) for m in BASE_STOP_REQUEST_RE.finditer(appended_log)]
        diags = [as_float_dict(m) for m in DIAG_RE.finditer(appended_log)]

        stop_start = next((item for item in starts if item.get("reason") == "stop_request"), None)
        stop_end = ends[-1] if ends else None
        base_stop_request = base_stop_requests[0] if base_stop_requests else None
        final_diag = diags[-1] if diags else None
        guarded_after_start = [s for s in self.guarded_samples if s[0] >= start_time]
        guarded_nonzero = [s for s in guarded_after_start if abs(s[1]) > 0.05 or abs(s[2]) > 0.05]
        guarded_zero = [s for s in guarded_after_start if abs(s[1]) < 1e-6 and abs(s[2]) < 1e-6]
        stop_requests_after_start = [s for s in self.stop_requests if s[0] >= start_time]
        state_changes = [
            (
                round(t - start_time, 3),
                state,
                front_min,
                front_p10,
                approach,
                ttc,
            )
            for t, state, front_min, front_p10, approach, ttc in self.state_changes
            if t >= start_time
        ]

        checks = [
            ("triggered_by_stop_request", trigger_reason == "STOP_REQUEST_TOPIC"),
            ("stop_request_topic_seen", bool(stop_requests_after_start)),
            ("base_stop_request_seen", base_stop_request is not None),
            ("stop_kick_start_reason_stop_request", stop_start is not None),
            ("stop_kick_end_seen", stop_end is not None),
            ("guarded_nonzero_seen", bool(guarded_nonzero)),
            ("guarded_zero_seen", bool(guarded_zero)),
        ]
        if base_stop_request is not None and stop_start is not None:
            expected_duration = (
                base_stop_request["serial_duration"] * self.args.brake_time_ratio
                + self.args.brake_time_offset
            )
            expected_duration = max(expected_duration, self.args.stop_kick_min_duration)
            if self.args.stop_kick_max_duration > 0.0:
                expected_duration = min(expected_duration, self.args.stop_kick_max_duration)
            checks.append(
                (
                    "base_prev_matches_stop_kick_prev",
                    abs(base_stop_request["prev_vx"] - stop_start["prev_vx"]) <= self.args.speed_tolerance
                    and abs(base_stop_request["prev_wz"] - stop_start["prev_wz"]) <= self.args.speed_tolerance,
                )
            )
            checks.append(
                (
                    "kick_matches_reverse_speed_gain",
                    abs(
                        stop_start["kick_vx"]
                        - (
                            -math.copysign(
                                min(abs(base_stop_request["prev_vx"]) * self.args.brake_speed_gain, MAX_LINEAR),
                                base_stop_request["prev_vx"],
                            )
                            if abs(base_stop_request["prev_vx"]) > 1e-6
                            else 0.0
                        )
                    ) <= self.args.speed_tolerance
                    and abs(
                        stop_start["kick_wz"]
                        - (
                            -math.copysign(
                                min(abs(base_stop_request["prev_wz"]) * self.args.brake_speed_gain, MAX_ANGULAR),
                                base_stop_request["prev_wz"],
                            )
                            if abs(base_stop_request["prev_wz"]) > 1e-6
                            else 0.0
                        )
                    ) <= self.args.speed_tolerance,
                )
            )
            checks.append(
                (
                    "kick_duration_matches_serial_motion_ratio",
                    abs(stop_start["duration"] - expected_duration) <= self.args.duration_tolerance,
                )
            )
        else:
            checks.extend(
                [
                    ("base_prev_matches_stop_kick_prev", False),
                    ("kick_matches_reverse_speed_gain", False),
                    ("kick_duration_matches_serial_motion_ratio", False),
                ]
            )
        if final_diag is not None:
            checks.append(("serial_zero_final", abs(final_diag["serial_vx"]) <= 1e-6 and abs(final_diag["serial_wz"]) <= 1e-6))
            checks.append(("feedback_near_zero_final", math.hypot(final_diag["fb_vx"], final_diag["fb_wz"]) <= self.args.feedback_tolerance))
        else:
            checks.append(("serial_zero_final", False))
            checks.append(("feedback_near_zero_final", False))

        print("GUARD_STOP_REQUEST_REPORT")
        print("trigger_reason:", trigger_reason)
        print("state_changes:", state_changes)
        print("stop_requests_seen:", len(stop_requests_after_start))
        print("guarded_nonzero_samples:", len(guarded_nonzero))
        print("guarded_zero_samples:", len(guarded_zero))
        print("odom_forward_m:", round(self.odom_forward_distance(), 3))
        print("base_stop_request:", base_stop_request if base_stop_request is not None else "NONE")
        print("stop_kick_start:", stop_start if stop_start is not None else "NONE")
        print("stop_kick_end:", stop_end if stop_end is not None else "NONE")
        print("final_diag:", final_diag if final_diag is not None else "NONE")
        print("checks:")
        for name, ok in checks:
            print(f"  {name}: {'PASS' if ok else 'FAIL'}")
        print("RESULT:", "PASS" if all(ok for _, ok in checks) else "FAIL")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Validate dynamic obstacle hard_stop -> STOP_REQUEST -> matched reverse braking."
    )
    parser.add_argument("--linear", type=float, default=0.30)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--max-wall-time-s", type=float, default=DEFAULT_MAX_WALL_TIME)
    parser.add_argument("--zero-seconds", type=float, default=DEFAULT_ZERO_SECONDS)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE)
    parser.add_argument("--input-cmd-topic", default="/input_cmd_vel")
    parser.add_argument("--guarded-cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--status-topic", default="/safety/front_obstacle")
    parser.add_argument("--stop-request-topic", default="/chassis/stop_request")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--base-log", default="/home/soc/edge-ai-robot-k1/logs/dynamic_base_latest.log")
    parser.add_argument("--emergency-front-p10-m", type=float, default=0.70)
    parser.add_argument("--emergency-front-min-m", type=float, default=0.40)
    parser.add_argument("--max-odom-forward-m", type=float, default=1.20)
    parser.add_argument("--no-status-timeout-s", type=float, default=0.50)
    parser.add_argument("--log-wait-s", type=float, default=1.0)
    parser.add_argument("--speed-tolerance", type=float, default=0.03)
    parser.add_argument("--duration-tolerance", type=float, default=0.20)
    parser.add_argument("--brake-time-ratio", type=float, default=0.45)
    parser.add_argument("--brake-time-offset", type=float, default=0.0)
    parser.add_argument("--brake-speed-gain", type=float, default=1.0)
    parser.add_argument("--stop-kick-min-duration", type=float, default=0.12)
    parser.add_argument("--stop-kick-max-duration", type=float, default=1.50)
    parser.add_argument("--feedback-tolerance", type=float, default=0.03)
    parser.add_argument("--kill-base-on-emergency", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if args.confirm != "YES":
        reject("requires --confirm YES")
    if abs(args.linear) > MAX_LINEAR:
        reject(f"abs(linear) must be <= {MAX_LINEAR}")
    if abs(args.angular) > MAX_ANGULAR:
        reject("angular motion is disabled for this straight guard test")
    if args.max_wall_time_s <= 0.0 or args.max_wall_time_s > 10.0:
        reject("max-wall-time-s must be > 0 and <= 10")
    if args.zero_seconds < 5.0:
        reject("zero-seconds must be >= 5.0")
    if args.emergency_front_p10_m <= args.emergency_front_min_m:
        reject("emergency-front-p10-m must be greater than emergency-front-min-m")
    if args.max_odom_forward_m <= 0.0:
        reject("max-odom-forward-m must be positive")
    if args.brake_time_ratio <= 0.0:
        reject("brake-time-ratio must be positive")
    if args.brake_time_offset < 0.0:
        reject("brake-time-offset must be non-negative")
    if args.brake_speed_gain <= 0.0:
        reject("brake-speed-gain must be positive")
    if args.stop_kick_min_duration < 0.0 or args.stop_kick_max_duration < 0.0:
        reject("stop kick duration bounds must be non-negative")
    return args


def main():
    args = parse_args()
    print("SAFETY WARNING")
    print("- This continuously publishes /input_cmd_vel until guard emits STOP_REQUEST.")
    print("- PASS requires STOP_REQUEST plus base stop_kick_start reason=stop_request.")
    print("- Expected kick duration is serial_duration * brake_time_ratio, clamped by min/max.")
    print("- Emergency stop publishes zero to input/guarded topics and a direct STOP_REQUEST.")
    print("- A person must physically guard the robot. Do not run unattended.")
    print(
        f"- Command: linear.x={args.linear:.3f}, max_wall={args.max_wall_time_s:.1f}s, "
        f"emergency_p10={args.emergency_front_p10_m:.2f}m, max_odom={args.max_odom_forward_m:.2f}m, "
        f"brake_ratio={args.brake_time_ratio:.2f}, brake_offset={args.brake_time_offset:.2f}s, "
        f"brake_speed_gain={args.brake_speed_gain:.2f}"
    )
    rclpy.init()
    node = GuardStopRequestValidation(args)
    try:
        node.run()
    finally:
        node.publish_zero_for(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
