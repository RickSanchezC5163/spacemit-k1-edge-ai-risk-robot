#!/usr/bin/env python3
import argparse
import json
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
DEFAULT_ZERO_SECONDS = 3.0
DEFAULT_BACKSTOP_M = 1.8
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


def reject(message: str) -> None:
    print(f"REFUSE: {message}", file=sys.stderr)
    sys.exit(2)


def as_float_dict(match):
    data = match.groupdict()
    for key, value in list(data.items()):
        if key not in ("reason", "phase"):
            data[key] = float(value)
    return data


class GuardedApproachLogTest(Node):
    def __init__(self, args):
        super().__init__("guarded_approach_log_test")
        self.args = args
        self.status = None
        self.samples = []
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.create_subscription(String, args.status_topic, self.status_cb, 20)

    def status_cb(self, msg: String) -> None:
        now = time.monotonic()
        try:
            item = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.status = item
        self.samples.append((now, item))

    def wait_ready(self) -> None:
        start = time.monotonic()
        while self.cmd_pub.get_subscription_count() < 1 and time.monotonic() - start < WAIT_FOR_SUBSCRIBER_SEC:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.cmd_pub.get_subscription_count() < 1:
            reject(f"no subscriber on {self.args.cmd_topic}")

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
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self.cmd_pub.publish(zero)

    def run_test(self) -> None:
        log_path = Path(self.args.base_log)
        if not log_path.exists():
            reject(f"base log does not exist: {log_path}")
        log_offset = log_path.stat().st_size
        self.wait_ready()
        start_status = dict(self.status)
        start_p10 = start_status.get("front_p10_range_m")
        if start_p10 is not None and start_p10 <= self.args.backstop_m:
            reject(
                f"front_p10 is already <= backstop ({start_p10:.3f}m <= {self.args.backstop_m:.3f}m); "
                "move obstacle farther before running"
            )

        cmd = Twist()
        cmd.linear.x = self.args.linear
        cmd.angular.z = self.args.angular
        period = 1.0 / self.args.rate
        start = time.monotonic()
        stop_reason = "duration_elapsed"
        stop_time = None

        print("START_STATUS", json.dumps(start_status, ensure_ascii=False))
        while time.monotonic() - start < self.args.duration:
            rclpy.spin_once(self, timeout_sec=0.0)
            status = self.status or {}
            p10 = status.get("front_p10_range_m")
            state = status.get("state")
            ttc = status.get("ttc_s")
            approach = status.get("approach_rate_mps")
            elapsed = time.monotonic() - start
            if p10 is not None and p10 <= self.args.backstop_m:
                stop_reason = f"backstop_p10_{p10:.3f}_m"
                stop_time = elapsed
                break
            if state == "hard_stop":
                stop_reason = "guard_hard_stop"
                stop_time = elapsed
                break
            if int(elapsed * 10) != int((elapsed - period) * 10):
                print(
                    f"SAMPLE t={elapsed:.2f}s state={state} p10={p10} "
                    f"approach={approach} ttc={ttc}"
                )
            self.cmd_pub.publish(cmd)
            time.sleep(period)

        if stop_time is None:
            stop_time = time.monotonic() - start
        print(f"STOP reason={stop_reason} t={stop_time:.3f}s")
        self.publish_zero_for(self.args.zero_seconds)
        time.sleep(self.args.log_wait_s)
        rclpy.spin_once(self, timeout_sec=0.1)
        self.print_summary(start)
        appended_log = log_path.read_text(errors="ignore")[log_offset:]
        self.print_stop_chain_report(appended_log, stop_time)

    def print_summary(self, start_time: float) -> None:
        parsed = [(t - start_time, item) for t, item in self.samples if t >= start_time]
        print(f"samples={len(parsed)}")
        if not parsed:
            return
        state_changes = []
        last_state = None
        min_p10 = None
        first_warning = None
        first_hard = None
        for t, item in parsed:
            state = item.get("state")
            p10 = item.get("front_p10_range_m")
            if p10 is not None:
                min_p10 = p10 if min_p10 is None else min(min_p10, p10)
            if state != last_state:
                state_changes.append(
                    (
                        round(t, 3),
                        state,
                        p10,
                        item.get("approach_rate_mps"),
                        item.get("ttc_s"),
                        item.get("hard_stop_latch_remaining_s"),
                    )
                )
                last_state = state
            if first_warning is None and state == "warning":
                first_warning = (t, item)
            if first_hard is None and state == "hard_stop":
                first_hard = (t, item)
        print("min_p10:", min_p10)
        print(
            "first_warning:",
            "NONE"
            if first_warning is None
            else (
                round(first_warning[0], 3),
                first_warning[1].get("front_p10_range_m"),
                first_warning[1].get("approach_rate_mps"),
                first_warning[1].get("ttc_s"),
            ),
        )
        print(
            "first_hard_stop:",
            "NONE"
            if first_hard is None
            else (
                round(first_hard[0], 3),
                first_hard[1].get("front_p10_range_m"),
                first_hard[1].get("approach_rate_mps"),
                first_hard[1].get("ttc_s"),
            ),
        )
        print("state_changes:", state_changes)

    def print_stop_chain_report(self, appended_log: str, expected_motion_duration: float) -> None:
        starts = [as_float_dict(m) for m in STOP_START_RE.finditer(appended_log)]
        ends = [as_float_dict(m) for m in STOP_END_RE.finditer(appended_log)]
        diags = [as_float_dict(m) for m in DIAG_RE.finditer(appended_log)]
        start = starts[-1] if starts else None
        end = ends[-1] if ends else None
        diag = diags[-1] if diags else None

        checks = [
            ("stop_kick_start_seen", start is not None),
            ("stop_kick_end_seen", end is not None),
        ]
        if start is not None:
            expected_kick_vx = (
                -math.copysign(min(abs(start["prev_vx"]) * self.args.brake_speed_gain, MAX_LINEAR), start["prev_vx"])
                if abs(start["prev_vx"]) > 1e-6
                else 0.0
            )
            checks.append(
                (
                    "kick_matches_reverse_speed_gain",
                    abs(start["kick_vx"] - expected_kick_vx) <= self.args.speed_tolerance,
                )
            )
            checks.append(
                (
                    "kick_duration_matches_motion",
                    abs(start["duration"] - expected_motion_duration) <= self.args.duration_tolerance,
                )
            )
        if diag is not None:
            checks.append(("serial_zero_final", abs(diag["serial_vx"]) <= 1e-6 and abs(diag["serial_wz"]) <= 1e-6))
            checks.append(("feedback_near_zero_final", math.hypot(diag["fb_vx"], diag["fb_wz"]) <= self.args.feedback_tolerance))
        else:
            checks.append(("serial_zero_final", False))
            checks.append(("feedback_near_zero_final", False))

        print("STOP_CHAIN_REPORT")
        print("stop_kick_start:", start if start is not None else "NONE")
        print("stop_kick_end:", end if end is not None else "NONE")
        print("final_diag:", diag if diag is not None else "NONE")
        for name, ok in checks:
            print(f"  {name}: {'PASS' if ok else 'FAIL'}")
        print("STOP_CHAIN_RESULT:", "PASS" if all(ok for _, ok in checks) else "FAIL")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Short guarded approach logger with p10 backstop for scan_safety_guard validation."
    )
    parser.add_argument("--linear", type=float, default=0.30)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--backstop-m", type=float, default=DEFAULT_BACKSTOP_M)
    parser.add_argument("--zero-seconds", type=float, default=DEFAULT_ZERO_SECONDS)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE)
    parser.add_argument("--cmd-topic", default="/input_cmd_vel")
    parser.add_argument("--status-topic", default="/safety/front_obstacle")
    parser.add_argument("--base-log", default="/home/soc/edge-ai-robot-k1/logs/dynamic_base_latest.log")
    parser.add_argument("--log-wait-s", type=float, default=1.0)
    parser.add_argument("--speed-tolerance", type=float, default=0.03)
    parser.add_argument("--duration-tolerance", type=float, default=0.25)
    parser.add_argument("--brake-speed-gain", type=float, default=1.0)
    parser.add_argument("--feedback-tolerance", type=float, default=0.03)
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if args.confirm != "YES":
        reject("requires --confirm YES")
    if abs(args.linear) > MAX_LINEAR:
        reject(f"abs(linear) must be <= {MAX_LINEAR}")
    if abs(args.angular) > MAX_ANGULAR:
        reject("angular motion is disabled for this forward approach test")
    if args.duration <= 0.0 or args.duration > MAX_DURATION:
        reject(f"duration must be > 0 and <= {MAX_DURATION}")
    if args.brake_speed_gain <= 0.0:
        reject("brake-speed-gain must be positive")
    if args.backstop_m < 1.5:
        reject("backstop must be >= 1.5m for this validation")
    if args.zero_seconds < 2.0:
        reject("zero-seconds must be >= 2.0")
    if args.rate <= 0.0:
        reject("rate must be positive")
    return args


def main():
    args = parse_args()
    print("SAFETY WARNING")
    print("- This publishes a short /input_cmd_vel command for guarded manual mapping validation.")
    print("- It stops early if front_p10 reaches the backstop distance or guard enters hard_stop.")
    print("- Stop means publishing zero so the Python tank base can run matched reverse braking.")
    print("- The base must be launched with stop_kick_match_cmd/duration enabled.")
    print("- It does not start Nav2, RRT, autonomous exploration, or any route loop.")
    print("- A person must physically guard the robot and be ready to lift/disable it.")
    print(
        f"- Command: linear.x={args.linear:.3f}, duration<={args.duration:.3f}s, "
        f"backstop={args.backstop_m:.3f}m, zero={args.zero_seconds:.1f}s"
    )

    rclpy.init()
    node = GuardedApproachLogTest(args)
    try:
        node.run_test()
    finally:
        node.publish_zero_for(0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
