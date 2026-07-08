#!/usr/bin/env python3
import argparse
import math
import re
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


MAX_LINEAR = 0.30
MAX_ANGULAR = 0.0
MAX_DURATION = 1.0
DEFAULT_RATE = 50.0
DEFAULT_ZERO_SECONDS = 4.0
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
DIAG_RE = re.compile(
    r"diag .*serial=\((?P<serial_vx>-?\d+\.\d+),(?P<serial_wz>-?\d+\.\d+)\) "
    r"feedback=\((?P<fb_vx>-?\d+\.\d+),(?P<fb_wz>-?\d+\.\d+)\)"
)
BASE_STOP_REQUEST_RE = re.compile(
    r"stop_request reason=(?P<reason>\S+) "
    r"prev_serial=\((?P<prev_vx>-?\d+\.\d+),(?P<prev_wz>-?\d+\.\d+)\) "
    r"serial_duration=(?P<serial_duration>\d+\.\d+)s "
    r"front_p10=(?P<front_p10>-?\d+\.?\d*|None)"
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


class StopChainValidation(Node):
    def __init__(self, args):
        super().__init__("stop_chain_validation")
        self.args = args
        self.cmd_pub = self.create_publisher(Twist, args.input_cmd_topic, 10)
        self.guarded_samples = []
        self.status_samples = []
        self.stop_requests = []
        self.create_subscription(Twist, args.guarded_cmd_topic, self.guarded_cb, 20)
        self.create_subscription(String, args.status_topic, self.status_cb, 20)
        self.create_subscription(String, args.stop_request_topic, self.stop_request_cb, 20)

    def guarded_cb(self, msg: Twist):
        self.guarded_samples.append((time.monotonic(), float(msg.linear.x), float(msg.angular.z)))

    def status_cb(self, msg: String):
        self.status_samples.append((time.monotonic(), msg.data))

    def stop_request_cb(self, msg: String):
        self.stop_requests.append((time.monotonic(), msg.data))

    def wait_ready(self):
        start = time.monotonic()
        while self.cmd_pub.get_subscription_count() < 1 and time.monotonic() - start < WAIT_FOR_SUBSCRIBER_SEC:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.cmd_pub.get_subscription_count() < 1:
            reject(f"no subscriber on {self.args.input_cmd_topic}")

        start = time.monotonic()
        while not self.guarded_samples and time.monotonic() - start < WAIT_FOR_SUBSCRIBER_SEC:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.guarded_samples:
            reject(f"no samples on {self.args.guarded_cmd_topic}")

    def publish_for(self, msg: Twist, duration: float):
        end = time.monotonic() + duration
        period = 1.0 / self.args.rate
        while time.monotonic() < end:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def run_active_zero_test(self):
        self.wait_ready()
        nonzero = Twist()
        nonzero.linear.x = self.args.linear
        nonzero.angular.z = self.args.angular
        zero = Twist()

        start = time.monotonic()
        print(f"RUN active_zero linear={self.args.linear:.3f} duration={self.args.duration:.3f}s")
        self.publish_for(nonzero, self.args.duration)
        self.publish_for(zero, self.args.zero_seconds)
        self.cmd_pub.publish(zero)
        rclpy.spin_once(self, timeout_sec=0.1)
        return start

    def run(self):
        log_path = Path(self.args.base_log)
        if not log_path.exists():
            reject(f"base log does not exist: {log_path}")
        offset = log_path.stat().st_size
        start = self.run_active_zero_test()
        time.sleep(self.args.log_wait_s)
        rclpy.spin_once(self, timeout_sec=0.1)
        appended = log_path.read_text(errors="ignore")[offset:]
        self.print_report(start, appended)

    def print_report(self, start_time: float, appended_log: str):
        guarded_after_start = [s for s in self.guarded_samples if s[0] >= start_time]
        nonzero_guarded = [s for s in guarded_after_start if abs(s[1]) > 0.05 or abs(s[2]) > 0.05]
        zero_guarded = [s for s in guarded_after_start if abs(s[1]) < 1e-6 and abs(s[2]) < 1e-6]

        starts = [as_float_dict(m) for m in STOP_START_RE.finditer(appended_log)]
        ends = [as_float_dict(m) for m in STOP_END_RE.finditer(appended_log)]
        diags = [as_float_dict(m) for m in DIAG_RE.finditer(appended_log)]
        base_stop_requests = [as_float_dict(m) for m in BASE_STOP_REQUEST_RE.finditer(appended_log)]

        start = starts[-1] if starts else None
        end = ends[-1] if ends else None
        diag = diags[-1] if diags else None
        base_stop_request = base_stop_requests[0] if base_stop_requests else None

        effective_motion_duration = 0.0
        if nonzero_guarded:
            effective_motion_duration = max(0.0, nonzero_guarded[-1][0] - nonzero_guarded[0][0])

        checks = []
        checks.append(("guarded_nonzero_seen", bool(nonzero_guarded)))
        checks.append(("guarded_zero_seen", bool(zero_guarded)))
        checks.append(("stop_kick_start_seen", start is not None))
        checks.append(("stop_kick_end_seen", end is not None))

        if start is not None:
            prev_vx = start["prev_vx"]
            prev_wz = start["prev_wz"]
            kick_vx = start["kick_vx"]
            kick_wz = start["kick_wz"]
            duration = start["duration"]
            expected_kick_vx = (
                -math.copysign(min(abs(prev_vx) * self.args.brake_speed_gain, MAX_LINEAR), prev_vx)
                if abs(prev_vx) > 1e-6
                else 0.0
            )
            expected_kick_wz = (
                -math.copysign(min(abs(prev_wz) * self.args.brake_speed_gain, MAX_ANGULAR), prev_wz)
                if abs(prev_wz) > 1e-6
                else 0.0
            )
            checks.append(
                (
                    "kick_matches_reverse_speed_gain",
                    abs(kick_vx - expected_kick_vx) <= self.args.speed_tolerance
                    and abs(kick_wz - expected_kick_wz) <= self.args.speed_tolerance,
                )
            )
            checks.append(("kick_duration_matches_motion", abs(duration - self.args.duration) <= self.args.duration_tolerance))
            effective_duration = (
                base_stop_request["serial_duration"]
                if base_stop_request is not None
                else effective_motion_duration
                if effective_motion_duration > 0.05
                else self.args.duration
            )
            expected_duration = (
                effective_duration * self.args.brake_time_ratio
                + self.args.brake_time_offset
            )
            expected_duration = max(expected_duration, self.args.stop_kick_min_duration)
            if self.args.stop_kick_max_duration > 0.0:
                expected_duration = min(expected_duration, self.args.stop_kick_max_duration)
            checks[-1] = (
                "kick_duration_matches_serial_motion_ratio",
                abs(duration - expected_duration) <= self.args.duration_tolerance,
            )
        if diag is not None:
            checks.append(("serial_zero_final", abs(diag["serial_vx"]) <= 1e-6 and abs(diag["serial_wz"]) <= 1e-6))
            checks.append(("feedback_near_zero_final", math.hypot(diag["fb_vx"], diag["fb_wz"]) <= self.args.feedback_tolerance))
        else:
            checks.append(("serial_zero_final", False))
            checks.append(("feedback_near_zero_final", False))

        print("STOP_CHAIN_REPORT")
        print("guarded_nonzero_samples:", len(nonzero_guarded))
        print("guarded_zero_samples:", len(zero_guarded))
        print("effective_guarded_motion_s:", round(effective_motion_duration, 3))
        print("stop_requests_seen:", len([s for s in self.stop_requests if s[0] >= start_time]))
        print("base_stop_request:", base_stop_request if base_stop_request is not None else "NONE")
        print("stop_kick_start:", start if start is not None else "NONE")
        print("stop_kick_end:", end if end is not None else "NONE")
        print("final_diag:", diag if diag is not None else "NONE")
        print("checks:")
        for name, ok in checks:
            print(f"  {name}: {'PASS' if ok else 'FAIL'}")
        print("RESULT:", "PASS" if all(ok for _, ok in checks) else "FAIL")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Validate C30D stop chain: guarded cmd -> base stop_kick -> serial zero -> feedback near zero."
    )
    parser.add_argument("--linear", type=float, default=0.30)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--zero-seconds", type=float, default=DEFAULT_ZERO_SECONDS)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE)
    parser.add_argument("--input-cmd-topic", default="/input_cmd_vel")
    parser.add_argument("--guarded-cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--status-topic", default="/safety/front_obstacle")
    parser.add_argument("--stop-request-topic", default="/chassis/stop_request")
    parser.add_argument("--base-log", default="/home/soc/edge-ai-robot-k1/logs/dynamic_base_latest.log")
    parser.add_argument("--log-wait-s", type=float, default=1.0)
    parser.add_argument("--speed-tolerance", type=float, default=0.03)
    parser.add_argument("--duration-tolerance", type=float, default=0.25)
    parser.add_argument("--brake-time-ratio", type=float, default=0.45)
    parser.add_argument("--brake-time-offset", type=float, default=0.0)
    parser.add_argument("--brake-speed-gain", type=float, default=1.0)
    parser.add_argument("--stop-kick-min-duration", type=float, default=0.12)
    parser.add_argument("--stop-kick-max-duration", type=float, default=1.50)
    parser.add_argument("--feedback-tolerance", type=float, default=0.03)
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if args.confirm != "YES":
        reject("requires --confirm YES")
    if abs(args.linear) > MAX_LINEAR:
        reject(f"abs(linear) must be <= {MAX_LINEAR}")
    if abs(args.angular) > MAX_ANGULAR:
        reject("angular motion is disabled for this stop-chain test")
    if args.duration <= 0.0 or args.duration > MAX_DURATION:
        reject(f"duration must be > 0 and <= {MAX_DURATION}")
    if args.zero_seconds < 3.0:
        reject("zero-seconds must be >= 3.0")
    if args.rate <= 0.0:
        reject("rate must be positive")
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
    print("- This sends one short forward command, then zero, to validate matched reverse braking.")
    print("- It validates stop_kick_start/stop_kick_end, kick speed/ratio-duration, serial zero, and feedback near zero.")
    print("- Do not run unattended. Do not use this for obstacle approach validation.")
    rclpy.init()
    node = StopChainValidation(args)
    try:
        node.run()
    finally:
        zero = Twist()
        for _ in range(10):
            node.cmd_pub.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
