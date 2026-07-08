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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


MAX_LINEAR = 0.45
MAX_DURATION = 2.0
DEFAULT_RATE = 50.0
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


def percentile(values, pct):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round((len(ordered) - 1) * pct))))
    return ordered[index]


class WallBrakeCalibration(Node):
    def __init__(self, args):
        super().__init__("wall_brake_calibration")
        self.args = args
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.stop_request_pub = self.create_publisher(String, args.stop_request_topic, 10)
        self.create_subscription(LaserScan, args.scan_topic, self.scan_cb, 20)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 20)
        self.latest_front = None
        self.front_samples = []
        self.start_odom = None
        self.latest_odom = None
        self.odom_samples = []
        self.drive_start_odom = None
        self.stop_odom = None
        self.final_odom = None
        self.settle_result = None

    def scan_cb(self, msg: LaserScan) -> None:
        half = math.radians(self.args.front_sector_deg) * 0.5
        values = []
        angle = msg.angle_min
        for value in msg.ranges:
            if -half <= angle <= half and math.isfinite(value):
                if msg.range_min <= value <= msg.range_max:
                    values.append(float(value))
            angle += msg.angle_increment
        if not values:
            front = {
                "t": time.monotonic(),
                "min": None,
                "p10": None,
                "count": 0,
            }
        else:
            front = {
                "t": time.monotonic(),
                "min": min(values),
                "p10": percentile(values, 0.10),
                "count": len(values),
            }
        self.latest_front = front
        self.front_samples.append(front)

    def odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.odom_samples.append((time.monotonic(), msg))
        if self.start_odom is None:
            self.start_odom = msg

    def wait_ready(self):
        start = time.monotonic()
        while self.cmd_pub.get_subscription_count() < 1 and time.monotonic() - start < WAIT_FOR_SUBSCRIBER_SEC:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.cmd_pub.get_subscription_count() < 1:
            reject(f"no subscriber on {self.args.cmd_topic}")

        start = time.monotonic()
        while self.latest_front is None and time.monotonic() - start < WAIT_FOR_SUBSCRIBER_SEC:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_front is None:
            reject(f"no scan samples on {self.args.scan_topic}")

    def publish_zero_for(self, duration: float) -> None:
        zero = Twist()
        end = time.monotonic() + duration
        period = 1.0 / self.args.rate
        while time.monotonic() < end:
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self.cmd_pub.publish(zero)

    def publish_zero_until_front_stable(self):
        zero = Twist()
        start = time.monotonic()
        max_duration = max(self.args.max_settle_s, self.args.settle_s)
        end = start + max_duration
        period = 1.0 / self.args.rate
        result = {
            "settled": False,
            "elapsed_s": 0.0,
            "latest_p10_m": None,
            "window_p10_delta_m": None,
            "window_samples": 0,
        }

        while time.monotonic() < end:
            now = time.monotonic()
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.0)

            recent = [
                s
                for s in self.front_samples
                if s["p10"] is not None and now - s["t"] <= self.args.stable_window_s
            ]
            if recent:
                p10_values = [s["p10"] for s in recent]
                result.update(
                    {
                        "elapsed_s": round(now - start, 3),
                        "latest_p10_m": recent[-1]["p10"],
                        "window_p10_delta_m": max(p10_values) - min(p10_values),
                        "window_samples": len(recent),
                    }
                )
                window_age = recent[-1]["t"] - recent[0]["t"]
                if (
                    now - start >= self.args.settle_s
                    and window_age >= self.args.stable_window_s * 0.75
                    and result["window_p10_delta_m"] <= self.args.stable_delta_m
                ):
                    result["settled"] = True
                    break
            time.sleep(period)

        self.cmd_pub.publish(zero)
        return result

    def publish_stop_request(self, reason: str) -> None:
        front = self.latest_front or {}
        request = {
            "request": "STOP_REQUEST",
            "reason": reason,
            "source": "wall_brake_calibration",
            "front_min_range_m": front.get("min"),
            "front_p10_range_m": front.get("p10"),
            "front_valid_count": front.get("count"),
            "input_linear_x": self.args.linear,
            "input_angular_z": 0.0,
            "timestamp_monotonic": round(time.monotonic(), 3),
        }
        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        for _ in range(10):
            self.stop_request_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def odom_distance(self):
        start = self.drive_start_odom or self.start_odom
        end = self.final_odom or self.latest_odom
        if start is None or end is None:
            return 0.0
        sx = start.pose.pose.position.x
        sy = start.pose.pose.position.y
        lx = end.pose.pose.position.x
        ly = end.pose.pose.position.y
        return math.hypot(lx - sx, ly - sy)

    def odom_snapshot(self, msg):
        if msg is None:
            return None
        pos = msg.pose.pose.position
        twist = msg.twist.twist
        return {
            "x": pos.x,
            "y": pos.y,
            "linear_x": twist.linear.x,
            "angular_z": twist.angular.z,
        }

    def odom_delta(self, start, end):
        if start is None or end is None:
            return None
        sx = start.pose.pose.position.x
        sy = start.pose.pose.position.y
        ex = end.pose.pose.position.x
        ey = end.pose.pose.position.y
        return {
            "dx": ex - sx,
            "dy": ey - sy,
            "distance": math.hypot(ex - sx, ey - sy),
        }

    def front_summary(self, start_time: float, end_time: float):
        samples = [s for s in self.front_samples if start_time <= s["t"] <= end_time and s["p10"] is not None]
        if not samples:
            return None
        p10_values = [s["p10"] for s in samples]
        min_values = [s["min"] for s in samples if s["min"] is not None]
        return {
            "samples": len(samples),
            "first_p10": samples[0]["p10"],
            "last_p10": samples[-1]["p10"],
            "min_p10": min(p10_values),
            "min_front": min(min_values) if min_values else None,
        }

    def run(self):
        log_path = Path(self.args.base_log)
        if not log_path.exists():
            reject(f"base log does not exist: {log_path}")
        log_offset = log_path.stat().st_size
        self.wait_ready()

        pre_start = time.monotonic()
        while time.monotonic() - pre_start < self.args.pre_sample_s:
            rclpy.spin_once(self, timeout_sec=0.05)
        pre = self.front_summary(pre_start, time.monotonic())
        if pre is None:
            reject("no front distance samples during pre-sample")
        if pre["last_p10"] < self.args.min_start_p10_m:
            reject(
                f"front wall too close: p10={pre['last_p10']:.3f}m < min_start_p10={self.args.min_start_p10_m:.3f}m"
            )

        print("START_FRONT", json.dumps(pre, ensure_ascii=False))

        cmd = Twist()
        cmd.linear.x = self.args.linear
        period = 1.0 / self.args.rate
        self.drive_start_odom = self.latest_odom
        drive_start = time.monotonic()
        stop_reason = "scheduled_stop"
        while True:
            now = time.monotonic()
            elapsed = now - drive_start
            rclpy.spin_once(self, timeout_sec=0.0)
            front = self.latest_front or {}
            front_min = front.get("min")
            front_p10 = front.get("p10")
            if front_min is not None and front_min <= self.args.emergency_front_min_m:
                stop_reason = f"emergency_front_min_{front_min:.3f}"
                break
            if front_p10 is not None and front_p10 <= self.args.emergency_front_p10_m:
                stop_reason = f"emergency_front_p10_{front_p10:.3f}"
                break
            if elapsed >= self.args.duration:
                break
            self.cmd_pub.publish(cmd)
            time.sleep(period)

        stop_time = time.monotonic()
        self.stop_odom = self.latest_odom
        stop_front = dict(self.latest_front or {})
        print(
            "STOP_REQUEST_POINT",
            json.dumps(
                {
                    "reason": stop_reason,
                    "drive_elapsed_s": round(stop_time - drive_start, 3),
                    "front_min": stop_front.get("min"),
                    "front_p10": stop_front.get("p10"),
                    "front_count": stop_front.get("count"),
                },
                ensure_ascii=False,
            ),
        )
        self.publish_stop_request(stop_reason)
        self.settle_result = self.publish_zero_until_front_stable()
        self.final_odom = self.latest_odom
        print("SETTLE_RESULT", json.dumps(self.settle_result, ensure_ascii=False))
        time.sleep(self.args.log_wait_s)
        rclpy.spin_once(self, timeout_sec=0.1)

        end_time = time.monotonic()
        whole = self.front_summary(pre_start, end_time)
        post = self.front_summary(stop_time, end_time)
        appended_log = log_path.read_text(errors="ignore")[log_offset:]
        self.print_report(pre, stop_front, post, whole, appended_log, stop_reason)

    def print_report(self, pre, stop_front, post, whole, appended_log, stop_reason):
        starts = [as_float_dict(m) for m in STOP_START_RE.finditer(appended_log)]
        ends = [as_float_dict(m) for m in STOP_END_RE.finditer(appended_log)]
        base_stop_requests = [as_float_dict(m) for m in BASE_STOP_REQUEST_RE.finditer(appended_log)]
        diags = [as_float_dict(m) for m in DIAG_RE.finditer(appended_log)]
        stop_start = next((item for item in starts if item.get("reason") == "stop_request"), None)
        stop_end = ends[-1] if ends else None
        base_stop_request = base_stop_requests[0] if base_stop_requests else None
        final_diag = diags[-1] if diags else None

        start_p10 = pre.get("last_p10")
        stop_p10 = stop_front.get("p10")
        final_p10 = post.get("last_p10") if post else None
        min_p10 = whole.get("min_p10") if whole else None

        report = {
            "linear_mps": self.args.linear,
            "command_duration_s": self.args.duration,
            "stop_reason": stop_reason,
            "start_p10_m": start_p10,
            "stop_request_p10_m": stop_p10,
            "final_p10_m": final_p10,
            "min_p10_m": min_p10,
            "approach_to_stop_request_m": None if start_p10 is None or stop_p10 is None else start_p10 - stop_p10,
            "approach_total_m": None if start_p10 is None or final_p10 is None else start_p10 - final_p10,
            "post_stop_extra_approach_m": None if stop_p10 is None or final_p10 is None else stop_p10 - final_p10,
            "odom_distance_m": self.odom_distance(),
            "odom_start": self.odom_snapshot(self.drive_start_odom),
            "odom_stop_request": self.odom_snapshot(self.stop_odom),
            "odom_final": self.odom_snapshot(self.final_odom),
            "odom_to_stop_delta": self.odom_delta(self.drive_start_odom, self.stop_odom),
            "odom_total_delta": self.odom_delta(self.drive_start_odom, self.final_odom),
            "base_stop_request": base_stop_request,
            "stop_kick_start": stop_start,
            "stop_kick_end": stop_end,
            "final_diag": final_diag,
            "settle_result": self.settle_result,
        }
        scan_total = report["approach_total_m"]
        odom_total = report["odom_total_delta"]["distance"] if report["odom_total_delta"] else None
        scan_to_stop = report["approach_to_stop_request_m"]
        odom_to_stop = report["odom_to_stop_delta"]["distance"] if report["odom_to_stop_delta"] else None
        report["scan_odom_ratio"] = {
            "total_scan_over_odom": (
                None if scan_total is None or odom_total is None or odom_total <= 1e-6 else scan_total / odom_total
            ),
            "to_stop_scan_over_odom": (
                None if scan_to_stop is None or odom_to_stop is None or odom_to_stop <= 1e-6 else scan_to_stop / odom_to_stop
            ),
            "odom_total_over_scan": (
                None if scan_total is None or abs(scan_total) <= 1e-6 or odom_total is None else odom_total / scan_total
            ),
        }

        print("WALL_BRAKE_REPORT")
        print(json.dumps(report, ensure_ascii=False, indent=2))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure wall distance change for one C30D direct drive + matched STOP_REQUEST braking run."
    )
    parser.add_argument("--linear", type=float, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--stop-request-topic", default="/chassis/stop_request")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--base-log", default="/home/soc/edge-ai-robot-k1/logs/dynamic_base_latest.log")
    parser.add_argument("--front-sector-deg", type=float, default=20.0)
    parser.add_argument("--min-start-p10-m", type=float, default=2.0)
    parser.add_argument("--emergency-front-p10-m", type=float, default=0.80)
    parser.add_argument("--emergency-front-min-m", type=float, default=0.50)
    parser.add_argument("--pre-sample-s", type=float, default=1.0)
    parser.add_argument("--settle-s", type=float, default=5.0)
    parser.add_argument("--max-settle-s", type=float, default=18.0)
    parser.add_argument("--stable-window-s", type=float, default=2.0)
    parser.add_argument("--stable-delta-m", type=float, default=0.02)
    parser.add_argument("--log-wait-s", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE)
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if args.confirm != "YES":
        reject("requires --confirm YES")
    if args.linear <= 0.0 or args.linear > MAX_LINEAR:
        reject(f"linear must be > 0 and <= {MAX_LINEAR}")
    if args.duration <= 0.0 or args.duration > MAX_DURATION:
        reject(f"duration must be > 0 and <= {MAX_DURATION}")
    if args.emergency_front_p10_m <= args.emergency_front_min_m:
        reject("emergency-front-p10-m must be greater than emergency-front-min-m")
    if args.min_start_p10_m <= args.emergency_front_p10_m:
        reject("min-start-p10-m must be greater than emergency-front-p10-m")
    return args


def main():
    args = parse_args()
    print("SAFETY WARNING")
    print("- This bypasses scan_safety_guard and publishes directly to /cmd_vel_guarded.")
    print("- Use only with the robot pointed at a wall, with a human ready to lift or power off.")
    print("- One run only: fixed forward speed, then STOP_REQUEST matched reverse braking.")
    print(
        f"- Command linear.x={args.linear:.3f}m/s for {args.duration:.3f}s; "
        f"emergency p10={args.emergency_front_p10_m:.2f}m."
    )
    rclpy.init()
    node = WallBrakeCalibration(args)
    try:
        node.run()
    finally:
        node.publish_zero_for(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
