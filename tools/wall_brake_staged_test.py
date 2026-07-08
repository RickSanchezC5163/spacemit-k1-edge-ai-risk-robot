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
DEFAULT_RATE = 50.0
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


def reject(message):
    print(f"REFUSE: {message}", file=sys.stderr)
    sys.exit(2)


def as_float_dict(match):
    data = match.groupdict()
    for key, value in list(data.items()):
        if key not in ("reason", "phase"):
            data[key] = float(value)
    return data


def percentile(values, pct):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round((len(ordered) - 1) * pct))))
    return ordered[index]


class WallBrakeStagedTest(Node):
    def __init__(self, args):
        super().__init__("wall_brake_staged_test")
        self.args = args
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.stop_pub = self.create_publisher(String, args.stop_request_topic, 10)
        self.create_subscription(LaserScan, args.scan_topic, self.scan_cb, 20)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 20)
        self.front_samples = []
        self.latest_front = None
        self.start_odom = None
        self.stop_odom = None
        self.final_odom = None
        self.latest_odom = None
        self.supplements = []

    def scan_cb(self, msg):
        half = math.radians(self.args.front_sector_deg) * 0.5
        vals = []
        for i, value in enumerate(msg.ranges):
            if not math.isfinite(value) or not (msg.range_min <= value <= msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if -half <= angle <= half:
                vals.append(float(value))
        vals.sort()
        if vals:
            front = {
                "t": time.monotonic(),
                "min": vals[0],
                "p10": percentile(vals, 0.10),
                "median": vals[len(vals) // 2],
                "count": len(vals),
            }
        else:
            front = {"t": time.monotonic(), "min": None, "p10": None, "median": None, "count": 0}
        self.latest_front = front
        self.front_samples.append(front)

    def odom_cb(self, msg):
        self.latest_odom = msg

    def wait_ready(self):
        start = time.monotonic()
        while self.cmd_pub.get_subscription_count() < 1 and time.monotonic() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.cmd_pub.get_subscription_count() < 1:
            reject(f"no subscriber on {self.args.cmd_topic}")
        start = time.monotonic()
        while self.latest_front is None and time.monotonic() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_front is None or self.latest_front["p10"] is None:
            reject("no valid front scan")

    def publish_zero_for(self, duration):
        zero = Twist()
        end = time.monotonic() + duration
        period = 1.0 / self.args.rate
        while time.monotonic() < end:
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self.cmd_pub.publish(zero)

    def publish_stop_request(self, reason, force_vx=None, force_duration=None, burst_count=10):
        front = self.latest_front or {}
        request = {
            "request": "STOP_REQUEST",
            "reason": reason,
            "source": "wall_brake_staged_test",
            "front_min_range_m": front.get("min"),
            "front_p10_range_m": front.get("p10"),
            "front_valid_count": front.get("count"),
            "timestamp_monotonic": round(time.monotonic(), 3),
        }
        if force_vx is not None and force_duration is not None:
            request["force_kick_vx"] = force_vx
            request["force_kick_wz"] = 0.0
            request["force_kick_duration"] = force_duration
        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        for _ in range(burst_count):
            self.stop_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def front_summary(self, start_time, end_time):
        samples = [
            s for s in self.front_samples
            if start_time <= s["t"] <= end_time and s["p10"] is not None
        ]
        if not samples:
            return None
        p10 = [s["p10"] for s in samples]
        return {
            "samples": len(samples),
            "first_p10": samples[0]["p10"],
            "last_p10": samples[-1]["p10"],
            "min_p10": min(p10),
            "max_p10": max(p10),
        }

    def recent_approach_delta(self, now):
        recent = [
            s for s in self.front_samples
            if s["p10"] is not None and now - s["t"] <= self.args.approach_window_s
        ]
        if len(recent) < 3:
            return 0.0, None, None
        return recent[0]["p10"] - recent[-1]["p10"], recent[0]["p10"], recent[-1]["p10"]

    def odom_distance(self, start, end):
        if start is None or end is None:
            return None
        sx = start.pose.pose.position.x
        sy = start.pose.pose.position.y
        ex = end.pose.pose.position.x
        ey = end.pose.pose.position.y
        return math.hypot(ex - sx, ey - sy)

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
        if pre is None or pre["last_p10"] < self.args.min_start_p10_m:
            reject(f"front p10 too close for staged test: {pre}")
        print("START_FRONT", json.dumps(pre, ensure_ascii=False))

        cmd = Twist()
        cmd.linear.x = self.args.linear
        self.start_odom = self.latest_odom
        period = 1.0 / self.args.rate
        drive_start = time.monotonic()
        while True:
            now = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.0)
            front = self.latest_front or {}
            if front.get("min") is not None and front["min"] <= self.args.emergency_front_min_m:
                break
            if front.get("p10") is not None and front["p10"] <= self.args.emergency_front_p10_m:
                break
            if now - drive_start >= self.args.duration:
                break
            self.cmd_pub.publish(cmd)
            time.sleep(period)

        stop_time = time.monotonic()
        stop_front = dict(self.latest_front or {})
        self.stop_odom = self.latest_odom
        print("STOP_REQUEST_POINT", json.dumps({
            "drive_elapsed_s": round(stop_time - drive_start, 3),
            "front_min": stop_front.get("min"),
            "front_p10": stop_front.get("p10"),
            "front_count": stop_front.get("count"),
        }, ensure_ascii=False))
        self.publish_stop_request("staged_primary_stop", burst_count=10)

        next_check = stop_time + self.args.first_check_delay_s
        last_supplement = 0.0
        end = stop_time + self.args.max_settle_s
        while time.monotonic() < end:
            now = time.monotonic()
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.0)
            front = self.latest_front or {}
            if front.get("min") is not None and front["min"] <= self.args.emergency_front_min_m:
                next_check = now
            if now >= next_check and len(self.supplements) < self.args.max_supplements:
                delta, old_p10, new_p10 = self.recent_approach_delta(now)
                if delta >= self.args.supplement_delta_m:
                    self.publish_stop_request(
                        "staged_supplemental_brake",
                        force_vx=-abs(self.args.supplement_vx),
                        force_duration=self.args.supplement_duration,
                        burst_count=1,
                    )
                    self.supplements.append({
                        "t_after_stop_s": round(now - stop_time, 3),
                        "old_p10": old_p10,
                        "new_p10": new_p10,
                        "approach_delta_m": delta,
                        "force_vx": -abs(self.args.supplement_vx),
                        "duration_s": self.args.supplement_duration,
                    })
                    last_supplement = now
                    next_check = now + self.args.after_supplement_delay_s
                else:
                    next_check = now + self.args.approach_window_s * 0.5

            stable = self.front_summary(max(stop_time, now - self.args.stable_window_s), now)
            if (
                now - stop_time >= self.args.min_settle_s
                and stable is not None
                and stable["max_p10"] - stable["min_p10"] <= self.args.stable_delta_m
                and now - last_supplement >= self.args.after_supplement_delay_s
            ):
                break
            time.sleep(period)

        self.publish_zero_for(1.0)
        self.final_odom = self.latest_odom
        final_time = time.monotonic()
        post = self.front_summary(stop_time, final_time)
        whole = self.front_summary(pre_start, final_time)
        appended = log_path.read_text(errors="ignore")[log_offset:]
        starts = [as_float_dict(m) for m in STOP_START_RE.finditer(appended)]
        ends = [as_float_dict(m) for m in STOP_END_RE.finditer(appended)]
        start_p10 = pre["last_p10"]
        stop_p10 = stop_front.get("p10")
        final_p10 = post["last_p10"] if post else None
        odom_total = self.odom_distance(self.start_odom, self.final_odom)
        report = {
            "linear_mps": self.args.linear,
            "command_duration_s": self.args.duration,
            "start_p10_m": start_p10,
            "stop_request_p10_m": stop_p10,
            "final_p10_m": final_p10,
            "min_p10_m": whole["min_p10"] if whole else None,
            "approach_to_stop_request_m": None if stop_p10 is None else start_p10 - stop_p10,
            "approach_total_m": None if final_p10 is None else start_p10 - final_p10,
            "post_stop_extra_approach_m": None if stop_p10 is None or final_p10 is None else stop_p10 - final_p10,
            "odom_total_m": odom_total,
            "supplements": self.supplements,
            "stop_kick_starts": starts,
            "stop_kick_ends": ends,
        }
        print("WALL_BRAKE_STAGED_REPORT")
        print(json.dumps(report, ensure_ascii=False, indent=2))


def parse_args():
    parser = argparse.ArgumentParser(description="One-shot wall brake test with scan-triggered supplemental braking.")
    parser.add_argument("--linear", type=float, default=0.30)
    parser.add_argument("--duration", type=float, default=1.50)
    parser.add_argument("--cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--stop-request-topic", default="/chassis/stop_request")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--base-log", default="/home/soc/edge-ai-robot-k1/logs/dynamic_base_latest.log")
    parser.add_argument("--front-sector-deg", type=float, default=20.0)
    parser.add_argument("--min-start-p10-m", type=float, default=2.80)
    parser.add_argument("--emergency-front-p10-m", type=float, default=0.70)
    parser.add_argument("--emergency-front-min-m", type=float, default=0.45)
    parser.add_argument("--pre-sample-s", type=float, default=1.0)
    parser.add_argument("--first-check-delay-s", type=float, default=0.55)
    parser.add_argument("--approach-window-s", type=float, default=0.30)
    parser.add_argument("--supplement-delta-m", type=float, default=0.02)
    parser.add_argument("--supplement-vx", type=float, default=0.45)
    parser.add_argument("--supplement-duration", type=float, default=0.22)
    parser.add_argument("--after-supplement-delay-s", type=float, default=0.75)
    parser.add_argument("--max-supplements", type=int, default=2)
    parser.add_argument("--min-settle-s", type=float, default=4.0)
    parser.add_argument("--max-settle-s", type=float, default=14.0)
    parser.add_argument("--stable-window-s", type=float, default=1.5)
    parser.add_argument("--stable-delta-m", type=float, default=0.02)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE)
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()
    if args.confirm != "YES":
        reject("requires --confirm YES")
    if args.linear <= 0.0 or args.linear > MAX_LINEAR:
        reject(f"linear must be > 0 and <= {MAX_LINEAR}")
    if args.duration <= 0.0 or args.duration > 2.0:
        reject("duration must be > 0 and <= 2.0")
    if args.supplement_duration <= 0.0 or args.supplement_duration > 0.5:
        reject("supplement-duration must be > 0 and <= 0.5")
    if args.max_supplements < 0 or args.max_supplements > 3:
        reject("max-supplements must be between 0 and 3")
    return args


def main():
    args = parse_args()
    print("SAFETY WARNING")
    print("- One staged brake test only. Human guard required.")
    print("- Publishes forward command, primary STOP_REQUEST, then scan-triggered forced brake only if still approaching.")
    rclpy.init()
    node = WallBrakeStagedTest(args)
    try:
        node.run()
    finally:
        node.publish_zero_for(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
