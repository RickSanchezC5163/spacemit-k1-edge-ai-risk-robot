#!/usr/bin/env python3
import argparse
import json
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import MapMetaData, Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String


MAX_LINEAR = 0.30
MAX_ANGULAR = 0.80
MAX_DURATION = 1.0
MAX_YAW_AMPLIFIED_DURATION = 3.0
MAX_ODOM_FORWARD_TARGET_M = 0.30
MAX_ODOM_TURN_TARGET_DEG = 30.0
MAX_ODOM_FORWARD_TIMEOUT_S = 6.0
MAX_ODOM_TURN_TIMEOUT_S = 6.0
DEFAULT_RATE = 50.0
DEFAULT_ZERO_HOLD_S = 4.0
WAIT_READY_S = 8.0

DIAG_RE = re.compile(
    r"diag .*cmd=\((?P<cmd_vx>-?\d+\.\d+),(?P<cmd_wz>-?\d+\.\d+)\) "
    r"serial=\((?P<serial_vx>-?\d+\.\d+),(?P<serial_wz>-?\d+\.\d+)\) "
    r"feedback=\((?P<fb_vx>-?\d+\.\d+),(?P<fb_wz>-?\d+\.\d+)\)"
)


def reject(message: str) -> None:
    print(f"REFUSE: {message}", file=sys.stderr)
    sys.exit(2)


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def odom_snapshot(msg: Odometry):
    if msg is None:
        return None
    pos = msg.pose.pose.position
    twist = msg.twist.twist
    yaw = yaw_from_odom(msg)
    return {
        "x": round(float(pos.x), 4),
        "y": round(float(pos.y), 4),
        "yaw_rad": round(float(yaw), 5),
        "yaw_deg": round(math.degrees(yaw), 2),
        "linear_x": round(float(twist.linear.x), 4),
        "angular_z": round(float(twist.angular.z), 4),
    }


def map_snapshot(msg: MapMetaData):
    if msg is None:
        return None
    return {
        "resolution": round(float(msg.resolution), 5),
        "width": int(msg.width),
        "height": int(msg.height),
        "origin_x": round(float(msg.origin.position.x), 4),
        "origin_y": round(float(msg.origin.position.y), 4),
    }


def scan_percentile(values, percentile: float):
    if not values:
        return None
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * percentile))
    return ordered[index]


def parse_diag(text: str):
    match = DIAG_RE.search(text)
    if not match:
        return None
    result = {}
    for key, value in match.groupdict().items():
        result[key] = round(float(value), 4)
    return result


def signed_stats(values):
    if not values:
        return {
            "samples": 0,
            "mean": None,
            "max_abs": None,
            "min": None,
            "max": None,
        }
    return {
        "samples": len(values),
        "mean": round(sum(values) / len(values), 4),
        "max_abs": round(max(abs(v) for v in values), 4),
        "min": round(min(values), 4),
        "max": round(max(values), 4),
    }


def window_values(samples, start_time: float, end_time: float, index: int):
    return [sample[index] for sample in samples if start_time <= sample[0] <= end_time]


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def odom_delta_values(start_msg: Odometry, end_msg: Odometry):
    if start_msg is None or end_msg is None:
        return {
            "delta_x": 0.0,
            "delta_y": 0.0,
            "delta_yaw_rad": 0.0,
            "forward_delta_m": 0.0,
            "lateral_delta_m": 0.0,
        }
    start_pos = start_msg.pose.pose.position
    end_pos = end_msg.pose.pose.position
    yaw0 = yaw_from_odom(start_msg)
    dx = float(end_pos.x - start_pos.x)
    dy = float(end_pos.y - start_pos.y)
    dyaw = normalize_angle(yaw_from_odom(end_msg) - yaw0)
    forward_delta = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    lateral_delta = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
    return {
        "delta_x": float(dx),
        "delta_y": float(dy),
        "delta_yaw_rad": float(dyaw),
        "forward_delta_m": float(forward_delta),
        "lateral_delta_m": float(lateral_delta),
    }


def odom_delta(start_msg: Odometry, end_msg: Odometry):
    raw = odom_delta_values(start_msg, end_msg)
    return {
        "delta_x": round(raw["delta_x"], 4),
        "delta_y": round(raw["delta_y"], 4),
        "delta_yaw_rad": round(raw["delta_yaw_rad"], 5),
        "delta_yaw_deg": round(math.degrees(raw["delta_yaw_rad"]), 2),
        "forward_delta_m": round(raw["forward_delta_m"], 4),
        "lateral_delta_m": round(raw["lateral_delta_m"], 4),
    }


class GuardedAutoMappingMicro(Node):
    def __init__(self, args):
        super().__init__("guarded_auto_mapping_micro")
        self.args = args
        self.cmd_pub = self.create_publisher(Twist, args.input_cmd_topic, 10)
        self.save_client = self.create_client(SaveMap, args.save_map_service)

        self.latest_scan = None
        self.latest_scan_time = 0.0
        self.latest_odom = None
        self.latest_odom_time = 0.0
        self.latest_map = None
        self.latest_map_time = 0.0
        self.latest_status = None
        self.latest_status_time = 0.0
        self.latest_guarded_cmd = None
        self.latest_guarded_time = 0.0
        self.latest_robot_vel = None
        self.latest_robot_vel_time = 0.0
        self.latest_diag = None
        self.latest_diag_time = 0.0
        self.odom_samples = []
        self.guarded_samples = []
        self.robot_vel_samples = []
        self.diag_samples = []
        self.pending_save = None
        self.completed_async_saves = []

        self.create_subscription(LaserScan, args.scan_topic, self.scan_cb, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 20)
        self.create_subscription(MapMetaData, args.map_metadata_topic, self.map_cb, 10)
        self.create_subscription(String, args.status_topic, self.status_cb, 20)
        self.create_subscription(Twist, args.guarded_cmd_topic, self.guarded_cb, 20)
        self.create_subscription(Log, "/rosout", self.rosout_cb, 50)
        self.create_subscription(Vector3, args.robot_vel_topic, self.robot_vel_cb, 20)

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg
        self.latest_scan_time = time.monotonic()

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg
        self.latest_odom_time = time.monotonic()
        self.odom_samples.append(
            (
                self.latest_odom_time,
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
                yaw_from_odom(msg),
            )
        )

    def map_cb(self, msg: MapMetaData):
        self.latest_map = msg
        self.latest_map_time = time.monotonic()

    def status_cb(self, msg: String):
        try:
            self.latest_status = json.loads(msg.data)
            self.latest_status_time = time.monotonic()
        except json.JSONDecodeError:
            self.get_logger().warn("Ignored non-JSON front obstacle status.")

    def guarded_cb(self, msg: Twist):
        self.latest_guarded_cmd = msg
        self.latest_guarded_time = time.monotonic()
        self.guarded_samples.append(
            (
                self.latest_guarded_time,
                float(msg.linear.x),
                float(msg.angular.z),
            )
        )

    def robot_vel_cb(self, msg):
        self.latest_robot_vel = msg
        self.latest_robot_vel_time = time.monotonic()
        self.robot_vel_samples.append(
            (
                self.latest_robot_vel_time,
                float(msg.x),
                float(msg.z),
            )
        )

    def rosout_cb(self, msg: Log):
        if "wheeltec_tank_base" not in msg.name:
            return
        diag = parse_diag(msg.msg)
        if diag is not None:
            self.latest_diag = diag
            self.latest_diag_time = time.monotonic()
            self.diag_samples.append((self.latest_diag_time, diag))

    def spin_for(self, duration: float, step: float = 0.05):
        end = time.monotonic() + max(0.0, duration)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=step)
            self.poll_pending_save()

    def wait_ready(self):
        deadline = time.monotonic() + WAIT_READY_S
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.poll_pending_save()
            if (
                self.cmd_pub.get_subscription_count() >= 1
                and self.latest_scan is not None
                and self.latest_odom is not None
                and self.latest_map is not None
                and self.latest_status is not None
            ):
                return
        missing = []
        if self.cmd_pub.get_subscription_count() < 1:
            missing.append(f"subscriber on {self.args.input_cmd_topic}")
        if self.latest_scan is None:
            missing.append(self.args.scan_topic)
        if self.latest_odom is None:
            missing.append(self.args.odom_topic)
        if self.latest_map is None:
            missing.append(self.args.map_metadata_topic)
        if self.latest_status is None:
            missing.append(self.args.status_topic)
        reject("not ready: missing " + ", ".join(missing))

    def wait_policy_ready(self, require_cmd_subscriber: bool):
        deadline = time.monotonic() + WAIT_READY_S
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.poll_pending_save()
            cmd_ready = (
                not require_cmd_subscriber
                or self.cmd_pub.get_subscription_count() >= 1
            )
            if (
                cmd_ready
                and self.latest_odom is not None
                and self.latest_map is not None
                and self.latest_status is not None
                and self.latest_robot_vel is not None
                and self.latest_diag is not None
            ):
                return
        missing = []
        if require_cmd_subscriber and self.cmd_pub.get_subscription_count() < 1:
            missing.append(f"subscriber on {self.args.input_cmd_topic}")
        if self.latest_odom is None:
            missing.append(self.args.odom_topic)
        if self.latest_map is None:
            missing.append(self.args.map_metadata_topic)
        if self.latest_status is None:
            missing.append(self.args.status_topic)
        if self.latest_robot_vel is None:
            missing.append(self.args.robot_vel_topic)
        if self.latest_diag is None:
            missing.append("base diag on /rosout")
        reject("policy not ready: missing " + ", ".join(missing))

    def freshness(self):
        now = time.monotonic()
        return {
            "scan_fresh": now - self.latest_scan_time <= self.args.fresh_timeout_s,
            "odom_fresh": now - self.latest_odom_time <= self.args.fresh_timeout_s,
            "map_fresh": now - self.latest_map_time <= self.args.fresh_timeout_s,
            "status_fresh": now - self.latest_status_time <= self.args.fresh_timeout_s,
            "guarded_cmd_fresh": now - self.latest_guarded_time <= self.args.fresh_timeout_s,
            "robot_vel_fresh": now - self.latest_robot_vel_time <= self.args.fresh_timeout_s,
            "diag_fresh": now - self.latest_diag_time <= self.args.diag_fresh_timeout_s,
        }

    def precheck(self):
        self.spin_for(self.args.precheck_sample_s)
        status = dict(self.latest_status or {})
        front_p10 = status.get("front_p10_range_m")
        front_min = status.get("front_min_range_m")
        return {
            "freshness": self.freshness(),
            "front_state": status.get("state"),
            "front_action": status.get("action"),
            "front_min_range_m": front_min,
            "front_p10_range_m": front_p10,
            "front_valid_count": status.get("front_valid_count"),
            "odom": odom_snapshot(self.latest_odom),
            "map": map_snapshot(self.latest_map),
            "scan_sectors": self.scan_sector_snapshot(),
        }

    def scan_sector_snapshot(self):
        scan = self.latest_scan
        if scan is None:
            return {"available": False, "reason": "no_scan"}
        sectors = {
            "front": (-15.0, 15.0),
            "left": (15.0, 75.0),
            "right": (-75.0, -15.0),
            "left45": (30.0, 60.0),
            "right45": (-60.0, -30.0),
        }
        values = {name: [] for name in sectors}
        range_min = float(scan.range_min) if scan.range_min else 0.0
        range_max = float(scan.range_max) if scan.range_max else float("inf")
        for index, raw_range in enumerate(scan.ranges):
            if (
                raw_range is None
                or math.isnan(raw_range)
                or math.isinf(raw_range)
                or raw_range <= max(0.0, range_min)
                or raw_range >= range_max
            ):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            angle_deg = math.degrees(normalize_angle(angle))
            for name, (lower, upper) in sectors.items():
                if lower <= angle_deg <= upper:
                    values[name].append(float(raw_range))
        result = {"available": True}
        for name, sector_values in values.items():
            result[name] = {
                "count": len(sector_values),
                "min": (
                    None
                    if not sector_values
                    else round(float(min(sector_values)), 4)
                ),
                "p10": (
                    None
                    if not sector_values
                    else round(float(scan_percentile(sector_values, 0.10)), 4)
                ),
            }
        return result

    def zero_status_snapshot(self):
        guarded_ok = False
        if self.latest_guarded_cmd is not None:
            guarded_ok = (
                abs(float(self.latest_guarded_cmd.linear.x)) <= self.args.zero_tolerance
                and abs(float(self.latest_guarded_cmd.angular.z)) <= self.args.zero_tolerance
            )
        robot_vel_ok = False
        robot_vel = None
        if self.latest_robot_vel is not None:
            robot_vel = {
                "feedback_vx": round(float(self.latest_robot_vel.x), 4),
                "feedback_wz": round(float(self.latest_robot_vel.z), 4),
            }
            robot_vel_ok = (
                abs(float(self.latest_robot_vel.x)) <= self.args.feedback_tolerance
                and abs(float(self.latest_robot_vel.z)) <= self.args.feedback_tolerance
            )
        diag_ok = False
        if self.latest_diag is not None and self.freshness()["diag_fresh"]:
            diag_ok = (
                abs(self.latest_diag["cmd_vx"]) <= self.args.zero_tolerance
                and abs(self.latest_diag["cmd_wz"]) <= self.args.zero_tolerance
                and abs(self.latest_diag["serial_vx"]) <= self.args.zero_tolerance
                and abs(self.latest_diag["serial_wz"]) <= self.args.zero_tolerance
                and abs(self.latest_diag["fb_vx"]) <= self.args.feedback_tolerance
                and abs(self.latest_diag["fb_wz"]) <= self.args.feedback_tolerance
            )
        return {
            "guarded_cmd_zero_ok": guarded_ok,
            "robot_vel_zero_ok": robot_vel_ok,
            "diag_zero_ok": diag_ok,
            "base_zero_ok": guarded_ok and robot_vel_ok and diag_ok,
            "latest_robot_vel": robot_vel,
            "latest_diag": self.latest_diag,
        }

    def zero_hold(self, duration: float):
        zero = Twist()
        started = time.monotonic()
        min_hold_s = min(
            max(float(self.args.zero_min_hold_s), 0.0),
            max(float(duration), 0.2),
        )
        max_hold_s = max(float(duration), min_hold_s, 0.2)
        poll_s = max(float(self.args.zero_poll_s), 0.02)
        required_confirmations = max(int(self.args.zero_confirm_samples), 1)
        deadline = started + max_hold_s
        min_deadline = started + min_hold_s
        next_check = min_deadline
        confirm_count = 0
        checks = []
        final_zero = None
        period = 1.0 / self.args.rate
        while rclpy.ok() and time.monotonic() < deadline:
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.0)
            self.poll_pending_save()
            now = time.monotonic()
            if now >= next_check:
                final_zero = self.zero_status_snapshot()
                if final_zero.get("base_zero_ok"):
                    confirm_count += 1
                else:
                    confirm_count = 0
                checks.append(
                    {
                        "elapsed_s": round(float(now - started), 3),
                        "base_zero_ok": bool(final_zero.get("base_zero_ok")),
                        "confirm_count": confirm_count,
                        "robot_vel": final_zero.get("latest_robot_vel"),
                        "diag": final_zero.get("latest_diag"),
                    }
                )
                if confirm_count >= required_confirmations:
                    break
                next_check = now + poll_s
            time.sleep(period)
        self.cmd_pub.publish(zero)
        self.spin_for(0.05)
        if final_zero is None:
            final_zero = self.zero_status_snapshot()
        elapsed = time.monotonic() - started
        return {
            "mode": "event_driven",
            "requested_max_s": round(float(duration), 3),
            "min_hold_s": round(float(min_hold_s), 3),
            "max_hold_s": round(float(max_hold_s), 3),
            "poll_s": round(float(poll_s), 3),
            "required_confirmations": required_confirmations,
            "confirm_count": confirm_count,
            "elapsed_s": round(float(elapsed), 3),
            "stop_kick_s": round(float(min(elapsed, min_hold_s)), 3),
            "adaptive_wait_s": round(float(max(0.0, elapsed - min_hold_s)), 3),
            "timed_out": confirm_count < required_confirmations,
            "base_zero_ok": bool(final_zero.get("base_zero_ok")),
            "base_zero": final_zero,
            "checks": checks[-10:],
        }

    def publish_motion(self, linear: float, angular: float, duration: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        start = time.monotonic()
        end = start + duration
        period = 1.0 / self.args.rate
        while rclpy.ok() and time.monotonic() < end:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            self.poll_pending_save()
            time.sleep(period)
        return start, time.monotonic()

    def publish_drive_once(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def front_block_reason(self, front_gate_m=None):
        status = dict(self.latest_status or {})
        front_p10 = status.get("front_p10_range_m")
        if front_p10 is None:
            return "front_p10 unavailable"
        try:
            front_p10_value = float(front_p10)
        except (TypeError, ValueError):
            return f"front_p10 invalid: {front_p10}"
        gate_m = self.args.forward_front_p10_min_m if front_gate_m is None else float(front_gate_m)
        if front_p10_value < gate_m:
            return f"front_p10 {front_p10_value:.3f} < {gate_m:.2f}"
        return None

    def telemetry_summary(self, command_start: float, command_end: float, segment_end: float):
        guarded_vx_cmd = window_values(self.guarded_samples, command_start, command_end, 1)
        guarded_vx_full = window_values(self.guarded_samples, command_start, segment_end, 1)
        guarded_wz_cmd = window_values(self.guarded_samples, command_start, command_end, 2)
        guarded_wz_full = window_values(self.guarded_samples, command_start, segment_end, 2)
        robot_vx_cmd = window_values(self.robot_vel_samples, command_start, command_end, 1)
        robot_vx_full = window_values(self.robot_vel_samples, command_start, segment_end, 1)
        robot_wz_cmd = window_values(self.robot_vel_samples, command_start, command_end, 2)
        robot_wz_full = window_values(self.robot_vel_samples, command_start, segment_end, 2)
        odom_vx_cmd = window_values(self.odom_samples, command_start, command_end, 1)
        odom_vx_full = window_values(self.odom_samples, command_start, segment_end, 1)
        odom_wz_cmd = window_values(self.odom_samples, command_start, command_end, 2)
        odom_wz_full = window_values(self.odom_samples, command_start, segment_end, 2)
        diag_cmd = [sample[1] for sample in self.diag_samples if command_start <= sample[0] <= segment_end]
        return {
            "command_window_s": round(command_end - command_start, 3),
            "full_window_s": round(segment_end - command_start, 3),
            "guarded_linear_x_command": signed_stats(guarded_vx_cmd),
            "guarded_linear_x_full": signed_stats(guarded_vx_full),
            "guarded_angular_z_command": signed_stats(guarded_wz_cmd),
            "guarded_angular_z_full": signed_stats(guarded_wz_full),
            "robot_vel_linear_x_command": signed_stats(robot_vx_cmd),
            "robot_vel_linear_x_full": signed_stats(robot_vx_full),
            "robot_vel_angular_z_command": signed_stats(robot_wz_cmd),
            "robot_vel_angular_z_full": signed_stats(robot_wz_full),
            "odom_twist_linear_x_command": signed_stats(odom_vx_cmd),
            "odom_twist_linear_x_full": signed_stats(odom_vx_full),
            "odom_twist_angular_z_command": signed_stats(odom_wz_cmd),
            "odom_twist_angular_z_full": signed_stats(odom_wz_full),
            "diag_samples": len(diag_cmd),
            "diag_cmd_vx": signed_stats([diag["cmd_vx"] for diag in diag_cmd]),
            "diag_cmd_wz": signed_stats([diag["cmd_wz"] for diag in diag_cmd]),
            "diag_serial_vx": signed_stats([diag["serial_vx"] for diag in diag_cmd]),
            "diag_serial_wz": signed_stats([diag["serial_wz"] for diag in diag_cmd]),
            "diag_feedback_vx": signed_stats([diag["fb_vx"] for diag in diag_cmd]),
            "diag_feedback_wz": signed_stats([diag["fb_wz"] for diag in diag_cmd]),
        }

    def base_zero_status(self, wait_s=None):
        if wait_s is None:
            wait_s = self.args.zero_check_wait_s
        if wait_s > 0.0:
            self.spin_for(wait_s)
        return self.zero_status_snapshot()

    def policy_observation_zero_status(self):
        zero = self.base_zero_status(wait_s=0.0)
        has_guarded_sample = self.latest_guarded_cmd is not None
        base_zero_ok = zero["robot_vel_zero_ok"] and zero["diag_zero_ok"]
        if has_guarded_sample:
            base_zero_ok = base_zero_ok and zero["guarded_cmd_zero_ok"]
        zero["base_zero_ok"] = base_zero_ok
        zero["policy_zero_basis"] = (
            "guarded_robot_diag" if has_guarded_sample else "robot_diag_no_guarded_sample"
        )
        return zero

    def map_file_status(self, prefix: str):
        files = {}
        for suffix in (".pgm", ".yaml"):
            path = Path(f"{prefix}{suffix}")
            files[suffix.lstrip(".")] = {
                "path": str(path),
                "exists": path.exists(),
                "size": path.stat().st_size if path.exists() else None,
            }
        return {
            "prefix": prefix,
            "files": files,
            "all_exist": all(item["exists"] for item in files.values()),
        }

    def save_map_attempt(self, prefix: str, attempt: int):
        started = time.monotonic()
        request = SaveMap.Request()
        request.name.data = prefix
        future = self.save_client.call_async(request)
        deadline = time.monotonic() + self.args.service_timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        elapsed_s = round(time.monotonic() - started, 3)
        if not future.done():
            return {
                "attempt": attempt,
                "ok": False,
                "elapsed_s": elapsed_s,
                "error": "save_map timeout",
                "file_status": self.map_file_status(prefix),
            }
        result = future.result()
        result_code = None if result is None else int(result.result)
        if result_code == 0:
            self.spin_for(0.2)
        file_status = self.map_file_status(prefix)
        ok = result_code == 0 and file_status["all_exist"]
        response = {
            "attempt": attempt,
            "ok": ok,
            "elapsed_s": elapsed_s,
            "result_code": result_code,
            "response": str(result),
            "file_status": file_status,
        }
        if not ok:
            if result_code == 0:
                response["error"] = "save_map files missing"
            else:
                response["error"] = "save_map failed"
        return response

    def save_map(self, prefix: str):
        if not self.save_client.wait_for_service(timeout_sec=self.args.service_timeout_s):
            return {"prefix": prefix, "ok": False, "error": "save_map service unavailable"}
        attempts = []
        max_attempts = self.args.save_map_retries + 1
        for attempt in range(1, max_attempts + 1):
            attempt_result = self.save_map_attempt(prefix, attempt)
            attempts.append(attempt_result)
            self.emit_save_attempt(attempt_result)
            if attempt_result["ok"]:
                return {
                    "prefix": prefix,
                    "ok": True,
                    "attempt_count": attempt,
                    "attempts": attempts,
                    "result_code": attempt_result.get("result_code"),
                    "response": attempt_result.get("response"),
                    "file_status": attempt_result.get("file_status"),
                }
            if attempt < max_attempts:
                self.spin_for(self.args.save_map_retry_delay_s)
        last = attempts[-1] if attempts else {}
        return {
            "prefix": prefix,
            "ok": False,
            "attempt_count": len(attempts),
            "attempts": attempts,
            "result_code": last.get("result_code"),
            "response": last.get("response"),
            "file_status": last.get("file_status"),
            "error": last.get("error", "save_map failed"),
        }

    def emit_save_attempt(self, attempt_result):
        if self.args.console_mode == "compact":
            status = "ok" if attempt_result.get("ok") else "failed"
            prefix = ((attempt_result.get("file_status") or {}).get("prefix")) or ""
            print(
                f"SAVE_MAP_ATTEMPT {status} attempt={attempt_result.get('attempt')} "
                f"elapsed={attempt_result.get('elapsed_s')}s prefix={prefix}"
            )
            return
        print("SAVE_MAP_ATTEMPT", json.dumps(attempt_result, ensure_ascii=False))

    def _finalize_save_record(self, pending, timed_out=False):
        record = pending["record"]
        future = pending["future"]
        elapsed_s = round(time.monotonic() - pending["started"], 3)
        record["elapsed_s"] = elapsed_s
        if timed_out or not future.done():
            record.update(
                {
                    "ok": False,
                    "status": "timeout",
                    "error": "async save_map timeout",
                    "file_status": self.map_file_status(record["prefix"]),
                }
            )
        else:
            try:
                result = future.result()
            except Exception as exc:
                record.update(
                    {
                        "ok": False,
                        "status": "exception",
                        "error": str(exc),
                        "file_status": self.map_file_status(record["prefix"]),
                    }
                )
                self.completed_async_saves.append(record)
                self.pending_save = None
                if self.args.console_mode == "compact":
                    print(
                        f"ASYNC_SAVE_DONE failed reason={record.get('reason')} "
                        f"elapsed={record.get('elapsed_s')}s prefix={record.get('prefix')}"
                    )
                else:
                    print("ASYNC_SAVE_DONE", json.dumps(record, ensure_ascii=False))
                return record
            result_code = None if result is None else int(result.result)
            file_status = self.map_file_status(record["prefix"])
            ok = result_code == 0 and file_status["all_exist"]
            record.update(
                {
                    "ok": ok,
                    "status": "completed",
                    "result_code": result_code,
                    "response": str(result),
                    "file_status": file_status,
                }
            )
            if not ok:
                if result_code == 0:
                    record["error"] = "save_map files missing"
                else:
                    record["error"] = "save_map failed"
        self.completed_async_saves.append(record)
        self.pending_save = None
        if self.args.console_mode == "compact":
            status = "ok" if record.get("ok") else "failed"
            print(
                f"ASYNC_SAVE_DONE {status} reason={record.get('reason')} "
                f"elapsed={record.get('elapsed_s')}s prefix={record.get('prefix')}"
            )
        else:
            print("ASYNC_SAVE_DONE", json.dumps(record, ensure_ascii=False))
        return record

    def poll_pending_save(self):
        if self.pending_save is None:
            return None
        if self.pending_save["future"].done():
            return self._finalize_save_record(self.pending_save)
        return None

    def wait_pending_save(self, reason: str):
        if self.pending_save is None:
            return None
        pending = self.pending_save
        pending["record"]["wait_reason"] = reason
        deadline = time.monotonic() + self.args.service_timeout_s
        while rclpy.ok() and not pending["future"].done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._finalize_save_record(
            pending,
            timed_out=not pending["future"].done(),
        )

    def start_async_save(self, prefix: str, reason: str, step_index=None):
        self.poll_pending_save()
        if self.pending_save is not None:
            return {
                "prefix": prefix,
                "ok": None,
                "status": "skipped_pending",
                "async": True,
                "reason": reason,
                "step_index": step_index,
                "pending_prefix": self.pending_save["record"].get("prefix"),
            }
        if not self.save_client.wait_for_service(timeout_sec=0.1):
            return {
                "prefix": prefix,
                "ok": False,
                "status": "service_unavailable",
                "async": True,
                "reason": reason,
                "step_index": step_index,
                "error": "save_map service unavailable",
            }
        request = SaveMap.Request()
        request.name.data = prefix
        started = time.monotonic()
        future = self.save_client.call_async(request)
        record = {
            "prefix": prefix,
            "ok": None,
            "status": "pending",
            "async": True,
            "reason": reason,
            "step_index": step_index,
            "started_monotonic_s": round(started, 3),
        }
        self.pending_save = {
            "record": record,
            "future": future,
            "started": started,
        }
        if self.args.console_mode == "compact":
            print(f"ASYNC_SAVE_START reason={reason} prefix={prefix}")
        else:
            print("ASYNC_SAVE_START", json.dumps(record, ensure_ascii=False))
        return record

    def segment_timing_breakdown(
        self,
        segment_start: float,
        precheck_start: float,
        precheck_end: float,
        motion_start: float,
        motion_end: float,
        zero_report,
        postcheck_start: float,
        postcheck_end: float,
        segment_end: float,
    ):
        zero_report = zero_report or {}
        precheck_s = max(0.0, precheck_end - precheck_start)
        motion_s = max(0.0, motion_end - motion_start)
        postcheck_s = max(0.0, postcheck_end - postcheck_start)
        stop_kick_s = float(zero_report.get("stop_kick_s") or 0.0)
        base_zero_s = float(zero_report.get("elapsed_s") or 0.0)
        total_s = max(0.0, segment_end - segment_start)
        accounted_s = precheck_s + motion_s + base_zero_s + postcheck_s
        return {
            "state_wait_time_s": round(precheck_s, 3),
            "decision_time_s": 0.0,
            "motion_execution_time_s": round(motion_s, 3),
            "stop_kick_time_s": round(stop_kick_s, 3),
            "base_zero_wait_time_s": round(base_zero_s, 3),
            "adaptive_zero_extra_wait_s": round(float(zero_report.get("adaptive_wait_s") or 0.0), 3),
            "map_save_time_s": 0.0,
            "postcheck_time_s": round(postcheck_s, 3),
            "loop_overhead_time_s": round(max(0.0, total_s - accounted_s), 3),
            "total_time_s": round(total_s, 3),
            "zero_wait": zero_report,
        }

    def combine_timing_breakdowns(self, children):
        totals = {
            "state_wait_time_s": 0.0,
            "decision_time_s": 0.0,
            "motion_execution_time_s": 0.0,
            "stop_kick_time_s": 0.0,
            "base_zero_wait_time_s": 0.0,
            "adaptive_zero_extra_wait_s": 0.0,
            "map_save_time_s": 0.0,
            "postcheck_time_s": 0.0,
            "loop_overhead_time_s": 0.0,
            "total_time_s": 0.0,
        }
        for child in children:
            timing = child.get("timing_breakdown") or {}
            for key in totals:
                value = timing.get(key)
                if value is not None:
                    totals[key] += float(value)
        return {key: round(value, 3) for key, value in totals.items()}

    def action_timing_breakdown(self, action_record, motion: str):
        if action_record is None:
            return self.combine_timing_breakdowns([])
        if motion == "arc30":
            return self.combine_timing_breakdowns(action_record.get("records", []))
        return dict(action_record.get("timing_breakdown") or {})

    def policy_step_timing_breakdown(
        self,
        step_start: float,
        state_ready_time: float,
        decision_done_time: float,
        base_before_start: float,
        base_before_end: float,
        motion_start: float,
        motion_end: float,
        action_timing,
        map_save_start,
        map_save_end,
        postcheck_start: float,
        postcheck_end: float,
        step_end: float,
    ):
        action_timing = action_timing or {}
        state_wait_s = max(0.0, state_ready_time - step_start)
        decision_s = max(0.0, decision_done_time - state_ready_time)
        pre_action_zero_s = max(0.0, base_before_end - base_before_start)
        motion_wall_s = max(0.0, motion_end - motion_start)
        motion_execution_s = float(action_timing.get("motion_execution_time_s") or 0.0)
        stop_kick_s = float(action_timing.get("stop_kick_time_s") or 0.0)
        base_zero_s = float(action_timing.get("base_zero_wait_time_s") or 0.0)
        adaptive_zero_s = float(action_timing.get("adaptive_zero_extra_wait_s") or 0.0)
        map_save_s = 0.0
        if map_save_start is not None and map_save_end is not None:
            map_save_s = max(0.0, map_save_end - map_save_start)
        postcheck_s = max(0.0, postcheck_end - postcheck_start)
        total_s = max(0.0, step_end - step_start)
        accounted_s = (
            state_wait_s
            + decision_s
            + pre_action_zero_s
            + motion_wall_s
            + map_save_s
            + postcheck_s
        )
        return {
            "state_wait_time_s": round(state_wait_s, 3),
            "decision_time_s": round(decision_s, 3),
            "pre_action_zero_check_time_s": round(pre_action_zero_s, 3),
            "motion_wall_time_s": round(motion_wall_s, 3),
            "motion_execution_time_s": round(motion_execution_s, 3),
            "stop_kick_time_s": round(stop_kick_s, 3),
            "base_zero_wait_time_s": round(base_zero_s, 3),
            "adaptive_zero_extra_wait_s": round(adaptive_zero_s, 3),
            "map_save_time_s": round(map_save_s, 3),
            "postcheck_time_s": round(postcheck_s, 3),
            "loop_overhead_time_s": round(max(0.0, total_s - accounted_s), 3),
            "step_total_time_s": round(total_s, 3),
            "action_timing_breakdown": action_timing,
        }

    def segment_record(self, name, kind, linear, angular, duration, blocked=False, reason=None):
        segment_start = time.monotonic()
        precheck_start = segment_start
        start = odom_snapshot(self.latest_odom)
        start_msg = self.latest_odom
        pre = self.precheck()
        precheck_end = time.monotonic()
        if blocked:
            command_start = time.monotonic()
            zero_report = self.zero_hold(self.args.zero_hold_s)
            command_end = command_start
            zero = zero_report["base_zero"]
            postcheck_start = time.monotonic()
            post = self.precheck()
            postcheck_end = time.monotonic()
            segment_end = time.monotonic()
            end_msg = self.latest_odom
            end = odom_snapshot(end_msg)
            dx = 0.0
            dy = 0.0
            dyaw = 0.0
            if start_msg is not None and end_msg is not None:
                dx = end_msg.pose.pose.position.x - start_msg.pose.pose.position.x
                dy = end_msg.pose.pose.position.y - start_msg.pose.pose.position.y
                dyaw = normalize_angle(yaw_from_odom(end_msg) - yaw_from_odom(start_msg))
            return {
                "name": name,
                "kind": kind,
                "command": {"linear": linear, "angular": angular, "duration_s": duration},
                "blocked": True,
                "blocked_reason": reason,
                "precheck": pre,
                "odom_start": start,
                "odom_end": end,
                "delta_x": round(float(dx), 4),
                "delta_y": round(float(dy), 4),
                "delta_yaw_rad": round(float(dyaw), 5),
                "delta_yaw_deg": round(math.degrees(dyaw), 2),
                "map_width": self.latest_map.width if self.latest_map else None,
                "map_height": self.latest_map.height if self.latest_map else None,
                "telemetry": self.telemetry_summary(command_start, command_end, segment_end),
                "base_zero": zero,
                "postcheck": post,
                "timing_breakdown": self.segment_timing_breakdown(
                    segment_start,
                    precheck_start,
                    precheck_end,
                    command_start,
                    command_end,
                    zero_report,
                    postcheck_start,
                    postcheck_end,
                    segment_end,
                ),
            }

        command_start, command_end = self.publish_motion(linear, angular, duration)
        zero_report = self.zero_hold(self.args.zero_hold_s)
        zero = zero_report["base_zero"]
        postcheck_start = time.monotonic()
        post = self.precheck()
        postcheck_end = time.monotonic()
        segment_end = time.monotonic()
        end_msg = self.latest_odom
        end = odom_snapshot(end_msg)
        dx = 0.0
        dy = 0.0
        dyaw = 0.0
        if start_msg is not None and end_msg is not None:
            dx = end_msg.pose.pose.position.x - start_msg.pose.pose.position.x
            dy = end_msg.pose.pose.position.y - start_msg.pose.pose.position.y
            dyaw = normalize_angle(yaw_from_odom(end_msg) - yaw_from_odom(start_msg))
        return {
            "name": name,
            "kind": kind,
            "command": {"linear": linear, "angular": angular, "duration_s": duration},
            "blocked": False,
            "precheck": pre,
            "odom_start": start,
            "odom_end": end,
            "delta_x": round(float(dx), 4),
            "delta_y": round(float(dy), 4),
            "delta_yaw_rad": round(float(dyaw), 5),
            "delta_yaw_deg": round(math.degrees(dyaw), 2),
            "map_width": self.latest_map.width if self.latest_map else None,
            "map_height": self.latest_map.height if self.latest_map else None,
            "telemetry": self.telemetry_summary(command_start, command_end, segment_end),
            "base_zero": zero,
            "postcheck": post,
            "timing_breakdown": self.segment_timing_breakdown(
                segment_start,
                precheck_start,
                precheck_end,
                command_start,
                command_end,
                zero_report,
                postcheck_start,
                postcheck_end,
                segment_end,
            ),
        }

    def odom_target_record(self, name, kind, target, max_linear, max_angular, timeout_s):
        segment_start = time.monotonic()
        precheck_start = segment_start
        pre = self.precheck()
        precheck_end = time.monotonic()
        start_msg = self.latest_odom
        start = odom_snapshot(start_msg)
        command_start = time.monotonic()
        command_end = command_start
        stop_reason = "ros_shutdown"
        target_reached = False
        period = 1.0 / self.args.rate
        signed_target_yaw = float(target.get("yaw_deg", 0.0))
        yaw_sign = 1.0 if signed_target_yaw >= 0.0 else -1.0
        target_yaw_rad = math.radians(abs(signed_target_yaw))
        target_distance = float(target.get("distance_m", 0.0))

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.poll_pending_save()
            now = time.monotonic()
            if now - command_start >= timeout_s:
                stop_reason = "timeout"
                break

            fresh = self.freshness()
            stale = [
                key.replace("_fresh", "")
                for key in ("scan_fresh", "odom_fresh", "status_fresh")
                if not fresh.get(key, False)
            ]
            if stale:
                stop_reason = "stale_" + "_".join(stale)
                break

            current_delta = odom_delta(start_msg, self.latest_odom)
            if kind == "forward":
                block_reason = self.front_block_reason()
                if block_reason is not None:
                    stop_reason = "front_blocked: " + block_reason
                    break
                if current_delta["forward_delta_m"] >= target_distance:
                    stop_reason = "target_reached"
                    target_reached = True
                    break
                self.publish_drive_once(max_linear, 0.0)
            elif kind == "turn":
                raw_yaw_delta = normalize_angle(
                    yaw_from_odom(self.latest_odom) - yaw_from_odom(start_msg)
                )
                if yaw_sign * raw_yaw_delta >= target_yaw_rad:
                    stop_reason = "target_reached"
                    target_reached = True
                    break
                self.publish_drive_once(0.0, yaw_sign * abs(max_angular))
            else:
                stop_reason = f"unsupported_kind: {kind}"
                break
            time.sleep(period)

        command_end = time.monotonic()
        zero_report = self.zero_hold(self.args.zero_hold_s)
        zero = zero_report["base_zero"]
        postcheck_start = time.monotonic()
        post = self.precheck()
        postcheck_end = time.monotonic()
        segment_end = time.monotonic()
        end_msg = self.latest_odom
        end = odom_snapshot(end_msg)
        actual = odom_delta(start_msg, end_msg)
        return {
            "name": name,
            "kind": kind,
            "control": "odom_target",
            "target": target,
            "command": {
                "max_linear": round(float(max_linear), 4),
                "max_angular": round(float(max_angular), 4),
                "timeout_s": round(float(timeout_s), 3),
            },
            "duration_used_s": round(command_end - command_start, 3),
            "target_reached": target_reached,
            "stop_reason": stop_reason,
            "precheck": pre,
            "postcheck": post,
            "front_p10_start": pre.get("front_p10_range_m"),
            "front_p10_end": post.get("front_p10_range_m"),
            "odom_start": start,
            "odom_end": end,
            "actual_odom_delta": actual,
            "delta_x": actual["delta_x"],
            "delta_y": actual["delta_y"],
            "delta_yaw_rad": actual["delta_yaw_rad"],
            "delta_yaw_deg": actual["delta_yaw_deg"],
            "actual_forward_delta_m": actual["forward_delta_m"],
            "actual_lateral_delta_m": actual["lateral_delta_m"],
            "map_width": self.latest_map.width if self.latest_map else None,
            "map_height": self.latest_map.height if self.latest_map else None,
            "map": map_snapshot(self.latest_map),
            "telemetry": self.telemetry_summary(command_start, command_end, segment_end),
            "base_zero": zero,
            "timing_breakdown": self.segment_timing_breakdown(
                segment_start,
                precheck_start,
                precheck_end,
                command_start,
                command_end,
                zero_report,
                postcheck_start,
                postcheck_end,
                segment_end,
            ),
        }

    def move_forward_by_odom(self, name, target_m, max_speed, timeout_s):
        return self.odom_target_record(
            name=name,
            kind="forward",
            target={"distance_m": round(float(target_m), 4)},
            max_linear=max_speed,
            max_angular=0.0,
            timeout_s=timeout_s,
        )

    def turn_by_odom(self, name, target_deg, max_angular, timeout_s):
        return self.odom_target_record(
            name=name,
            kind="turn",
            target={"yaw_deg": round(float(target_deg), 3)},
            max_linear=0.0,
            max_angular=max_angular,
            timeout_s=timeout_s,
        )

    def forward_threshold_record(self, speed: float):
        segment_start = time.monotonic()
        precheck_start = segment_start
        pre = self.precheck()
        precheck_end = time.monotonic()
        start_msg = self.latest_odom
        start = odom_snapshot(start_msg)
        block_reason = self.front_block_reason()
        if block_reason is not None:
            command_start = time.monotonic()
            zero_report = self.zero_hold(self.args.zero_hold_s)
            zero = zero_report["base_zero"]
            postcheck_start = time.monotonic()
            post = self.precheck()
            postcheck_end = time.monotonic()
            segment_end = time.monotonic()
            actual = odom_delta(start_msg, self.latest_odom)
            return {
                "speed_mps": round(float(speed), 3),
                "blocked": True,
                "stop_reason": "front_blocked: " + block_reason,
                "movement_detected": False,
                "time_to_motion_s": None,
                "precheck": pre,
                "postcheck": post,
                "odom_start": start,
                "odom_end": odom_snapshot(self.latest_odom),
                "actual_odom_delta": actual,
                "front_p10_start": pre.get("front_p10_range_m"),
                "front_p10_end": post.get("front_p10_range_m"),
                "telemetry": self.telemetry_summary(command_start, command_start, segment_end),
                "base_zero": zero,
                "timing_breakdown": self.segment_timing_breakdown(
                    segment_start,
                    precheck_start,
                    precheck_end,
                    command_start,
                    command_start,
                    zero_report,
                    postcheck_start,
                    postcheck_end,
                    segment_end,
                ),
            }

        command_start = time.monotonic()
        command_end = command_start
        stop_reason = "duration_complete"
        movement_detected = False
        time_to_motion = None
        first_motion = None
        max_forward_delta = 0.0
        max_odom_vx = 0.0
        max_robot_vx = 0.0
        period = 1.0 / self.args.rate

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.poll_pending_save()
            now = time.monotonic()
            if now - command_start >= self.args.threshold_pulse_s:
                break

            fresh = self.freshness()
            stale = [
                key.replace("_fresh", "")
                for key in ("scan_fresh", "odom_fresh", "status_fresh")
                if not fresh.get(key, False)
            ]
            if stale:
                stop_reason = "stale_" + "_".join(stale)
                break

            block_reason = self.front_block_reason()
            if block_reason is not None:
                stop_reason = "front_blocked: " + block_reason
                break

            raw_delta = odom_delta_values(start_msg, self.latest_odom)
            forward_delta = raw_delta["forward_delta_m"]
            odom_vx = 0.0
            if self.latest_odom is not None:
                odom_vx = float(self.latest_odom.twist.twist.linear.x)
            robot_vx = 0.0
            if self.latest_robot_vel is not None:
                robot_vx = float(self.latest_robot_vel.x)
            max_forward_delta = max(max_forward_delta, forward_delta)
            max_odom_vx = max(max_odom_vx, abs(odom_vx))
            max_robot_vx = max(max_robot_vx, abs(robot_vx))

            if (
                not movement_detected
                and (
                    forward_delta >= self.args.threshold_detect_m
                    or abs(odom_vx) >= self.args.threshold_detect_vx
                    or abs(robot_vx) >= self.args.threshold_detect_vx
                )
            ):
                movement_detected = True
                time_to_motion = now - command_start
                first_motion = {
                    "time_s": round(float(time_to_motion), 3),
                    "forward_delta_m": round(float(forward_delta), 4),
                    "odom_vx": round(float(odom_vx), 4),
                    "robot_vx": round(float(robot_vx), 4),
                }
                stop_reason = "movement_detected"
                if not self.args.threshold_continue_after_first_motion:
                    break

            self.publish_drive_once(speed, 0.0)
            time.sleep(period)

        command_end = time.monotonic()
        zero_report = self.zero_hold(self.args.zero_hold_s)
        zero = zero_report["base_zero"]
        postcheck_start = time.monotonic()
        post = self.precheck()
        postcheck_end = time.monotonic()
        segment_end = time.monotonic()
        actual = odom_delta(start_msg, self.latest_odom)
        return {
            "speed_mps": round(float(speed), 3),
            "blocked": False,
            "stop_reason": stop_reason,
            "movement_detected": movement_detected,
            "time_to_motion_s": None if time_to_motion is None else round(float(time_to_motion), 3),
            "first_motion": first_motion,
            "pulse_duration_s": round(command_end - command_start, 3),
            "max_forward_delta_during_command_m": round(float(max_forward_delta), 4),
            "max_odom_vx_during_command": round(float(max_odom_vx), 4),
            "max_robot_vx_during_command": round(float(max_robot_vx), 4),
            "precheck": pre,
            "postcheck": post,
            "odom_start": start,
            "odom_end": odom_snapshot(self.latest_odom),
            "actual_odom_delta": actual,
            "front_p10_start": pre.get("front_p10_range_m"),
            "front_p10_end": post.get("front_p10_range_m"),
            "telemetry": self.telemetry_summary(command_start, command_end, segment_end),
            "base_zero": zero,
            "timing_breakdown": self.segment_timing_breakdown(
                segment_start,
                precheck_start,
                precheck_end,
                command_start,
                command_end,
                zero_report,
                postcheck_start,
                postcheck_end,
                segment_end,
            ),
        }

    def run_forward_threshold(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        speeds = []
        value = self.args.threshold_start_speed
        while value <= self.args.threshold_max_speed + 1e-9:
            speeds.append(round(float(value), 3))
            value += self.args.threshold_step_speed

        records = []
        first_motion_speed = None
        sequence_stop_reason = None
        for speed in speeds:
            record = self.forward_threshold_record(speed)
            records.append(record)
            print("FORWARD_THRESHOLD_STEP", json.dumps(record, ensure_ascii=False))
            if record.get("movement_detected"):
                first_motion_speed = speed
                if not self.args.threshold_continue_after_first_motion:
                    sequence_stop_reason = f"first_motion_at_{speed:.2f}mps"
                    break
            if record.get("blocked") or str(record.get("stop_reason", "")).startswith("front_blocked"):
                sequence_stop_reason = record["stop_reason"]
                break

        return {
            "mode": "forward-threshold",
            "control": "speed_step_short_pulse",
            "first_motion_speed_mps": first_motion_speed,
            "sequence_stop_reason": sequence_stop_reason,
            "records": records,
        }

    def staged_forward_record(
        self,
        name: str,
        target_m: float,
        fast_speed: float,
        mid_speed: float,
        slow_speed: float,
        mid_zone_m: float,
        slow_zone_m: float,
        brake_margin_m: float,
        timeout_s: float,
    ):
        segment_start = time.monotonic()
        precheck_start = segment_start
        pre = self.precheck()
        precheck_end = time.monotonic()
        original_start_msg = self.latest_odom
        original_start = odom_snapshot(original_start_msg)
        control_start_msg = original_start_msg
        control_start = odom_snapshot(control_start_msg)
        block_reason = self.front_block_reason()
        command_start = time.monotonic()
        command_end = command_start
        control_start_time = command_start
        samples = []
        stop_reason = "ros_shutdown"
        phase = "pre_roll"
        time_to_motion = None
        first_motion = None
        max_control_forward_delta = 0.0
        max_total_forward_delta = 0.0
        max_odom_vx = 0.0
        max_robot_vx = 0.0
        period = 1.0 / self.args.rate

        if block_reason is not None:
            zero_report = self.zero_hold(self.args.zero_hold_s)
            zero = zero_report["base_zero"]
            postcheck_start = time.monotonic()
            post = self.precheck()
            postcheck_end = time.monotonic()
            segment_end = time.monotonic()
            actual = odom_delta(original_start_msg, self.latest_odom)
            return {
                "name": name,
                "control": "staged",
                "blocked": True,
                "stop_reason": "front_blocked: " + block_reason,
                "target_m": round(float(target_m), 4),
                "actual_final_forward_m": actual["forward_delta_m"],
                "overshoot_m": round(actual["forward_delta_m"] - target_m, 4),
                "time_to_motion_s": None,
                "first_motion": None,
                "duration_used_s": 0.0,
                "precheck": pre,
                "postcheck": post,
                "odom_start": original_start,
                "control_odom_start": control_start,
                "odom_end": odom_snapshot(self.latest_odom),
                "actual_odom_delta": actual,
                "samples": samples,
                "telemetry": self.telemetry_summary(command_start, command_start, segment_end),
                "base_zero": zero,
                "timing_breakdown": self.segment_timing_breakdown(
                    segment_start,
                    precheck_start,
                    precheck_end,
                    command_start,
                    command_start,
                    zero_report,
                    postcheck_start,
                    postcheck_end,
                    segment_end,
                ),
            }

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.poll_pending_save()
            now = time.monotonic()
            elapsed = now - command_start
            if elapsed >= timeout_s:
                stop_reason = "timeout"
                break

            fresh = self.freshness()
            stale = [
                key.replace("_fresh", "")
                for key in ("scan_fresh", "odom_fresh", "status_fresh")
                if not fresh.get(key, False)
            ]
            if stale:
                stop_reason = "stale_" + "_".join(stale)
                break

            block_reason = self.front_block_reason()
            if block_reason is not None:
                stop_reason = "front_blocked: " + block_reason
                break

            total_delta = odom_delta_values(original_start_msg, self.latest_odom)
            total_forward_delta = total_delta["forward_delta_m"]
            odom_vx = 0.0
            if self.latest_odom is not None:
                odom_vx = float(self.latest_odom.twist.twist.linear.x)
            robot_vx = 0.0
            if self.latest_robot_vel is not None:
                robot_vx = float(self.latest_robot_vel.x)

            max_total_forward_delta = max(max_total_forward_delta, total_forward_delta)
            max_odom_vx = max(max_odom_vx, abs(odom_vx))
            max_robot_vx = max(max_robot_vx, abs(robot_vx))

            if phase == "pre_roll":
                motion_detected = (
                    total_forward_delta >= self.args.threshold_detect_m
                    or abs(odom_vx) >= self.args.threshold_detect_vx
                )
                if motion_detected:
                    phase = "staged"
                    time_to_motion = elapsed
                    control_start_msg = self.latest_odom
                    control_start = odom_snapshot(control_start_msg)
                    control_start_time = now
                    first_motion = {
                        "time_s": round(float(elapsed), 3),
                        "total_forward_delta_m": round(float(total_forward_delta), 4),
                        "odom_vx": round(float(odom_vx), 4),
                        "robot_vx": round(float(robot_vx), 4),
                    }
                else:
                    status = dict(self.latest_status or {})
                    samples.append(
                        {
                            "phase": phase,
                            "time_s": round(float(elapsed), 3),
                            "total_forward_delta_m": round(float(total_forward_delta), 4),
                            "control_forward_delta_m": 0.0,
                            "remaining_m": round(float(target_m), 4),
                            "effective_brake_margin_m": None,
                            "cmd_linear": round(float(slow_speed), 4),
                            "odom_vx": round(float(odom_vx), 4),
                            "robot_vx": round(float(robot_vx), 4),
                            "front_p10": status.get("front_p10_range_m"),
                            "guard_state": status.get("state"),
                            "guard_action": status.get("action"),
                        }
                    )
                    self.publish_drive_once(slow_speed, 0.0)
                    time.sleep(period)
                    continue

            control_delta = odom_delta_values(control_start_msg, self.latest_odom)
            control_forward_delta = control_delta["forward_delta_m"]
            remaining = target_m - control_forward_delta
            max_control_forward_delta = max(max_control_forward_delta, control_forward_delta)
            effective_brake_margin = max(
                brake_margin_m,
                abs(odom_vx) * self.args.forward_brake_coef_s
                + self.args.forward_static_brake_margin_m,
            )

            if remaining <= effective_brake_margin:
                stop_reason = "brake_margin_dynamic"
                break
            if remaining <= slow_zone_m:
                cmd_linear = slow_speed
            elif remaining <= mid_zone_m:
                cmd_linear = mid_speed
            else:
                cmd_linear = fast_speed

            status = dict(self.latest_status or {})
            samples.append(
                {
                    "phase": phase,
                    "time_s": round(float(elapsed), 3),
                    "phase_time_s": round(float(now - control_start_time), 3),
                    "total_forward_delta_m": round(float(total_forward_delta), 4),
                    "control_forward_delta_m": round(float(control_forward_delta), 4),
                    "remaining_m": round(float(remaining), 4),
                    "effective_brake_margin_m": round(float(effective_brake_margin), 4),
                    "cmd_linear": round(float(cmd_linear), 4),
                    "odom_vx": round(float(odom_vx), 4),
                    "robot_vx": round(float(robot_vx), 4),
                    "front_p10": status.get("front_p10_range_m"),
                    "guard_state": status.get("state"),
                    "guard_action": status.get("action"),
                }
            )
            self.publish_drive_once(cmd_linear, 0.0)
            time.sleep(period)

        command_end = time.monotonic()
        zero_report = self.zero_hold(self.args.zero_hold_s)
        zero = zero_report["base_zero"]
        postcheck_start = time.monotonic()
        post = self.precheck()
        postcheck_end = time.monotonic()
        segment_end = time.monotonic()
        actual = odom_delta(original_start_msg, self.latest_odom)
        control_actual = odom_delta(control_start_msg, self.latest_odom)
        actual_forward = actual["forward_delta_m"]
        control_forward = control_actual["forward_delta_m"]
        return {
            "name": name,
            "control": "staged",
            "phases": ["pre_roll", "staged"],
            "blocked": False,
            "target_m": round(float(target_m), 4),
            "profile": {
                "fast_speed": round(float(fast_speed), 4),
                "mid_speed": round(float(mid_speed), 4),
                "slow_speed": round(float(slow_speed), 4),
                "mid_zone_m": round(float(mid_zone_m), 4),
                "slow_zone_m": round(float(slow_zone_m), 4),
                "brake_margin_m": round(float(brake_margin_m), 4),
                "brake_coef_s": round(float(self.args.forward_brake_coef_s), 3),
                "static_brake_margin_m": round(
                    float(self.args.forward_static_brake_margin_m), 4
                ),
                "pre_roll_speed": round(float(slow_speed), 4),
                "pre_roll_detect_m": round(float(self.args.threshold_detect_m), 4),
                "pre_roll_detect_vx": round(float(self.args.threshold_detect_vx), 4),
                "timeout_s": round(float(timeout_s), 3),
            },
            "actual_final_forward_m": actual_forward,
            "overshoot_m": round(float(actual_forward - target_m), 4),
            "control_final_forward_m": control_forward,
            "control_overshoot_m": round(float(control_forward - target_m), 4),
            "stop_reason": stop_reason,
            "time_to_motion_s": None if time_to_motion is None else round(float(time_to_motion), 3),
            "first_motion": first_motion,
            "duration_used_s": round(command_end - command_start, 3),
            "staged_duration_s": round(command_end - control_start_time, 3),
            "max_forward_delta_during_command_m": round(float(max_control_forward_delta), 4),
            "max_total_forward_delta_during_command_m": round(float(max_total_forward_delta), 4),
            "max_odom_vx_during_command": round(float(max_odom_vx), 4),
            "max_robot_vx_during_command": round(float(max_robot_vx), 4),
            "precheck": pre,
            "postcheck": post,
            "front_p10_start": pre.get("front_p10_range_m"),
            "front_p10_end": post.get("front_p10_range_m"),
            "odom_start": original_start,
            "control_odom_start": control_start,
            "odom_end": odom_snapshot(self.latest_odom),
            "actual_odom_delta": actual,
            "control_odom_delta": control_actual,
            "map_width": self.latest_map.width if self.latest_map else None,
            "map_height": self.latest_map.height if self.latest_map else None,
            "map": map_snapshot(self.latest_map),
            "samples": samples,
            "telemetry": self.telemetry_summary(command_start, command_end, segment_end),
            "base_zero": zero,
            "timing_breakdown": self.segment_timing_breakdown(
                segment_start,
                precheck_start,
                precheck_end,
                command_start,
                command_end,
                zero_report,
                postcheck_start,
                postcheck_end,
                segment_end,
            ),
        }

    def run_forward_staged(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        if self.args.staged_test_set == "abc":
            cases = [
                {
                    "name": "staged_A_target_0p15",
                    "target_m": 0.15,
                    "fast_speed": 0.20,
                    "mid_speed": 0.15,
                    "slow_speed": 0.10,
                    "mid_zone_m": 0.12,
                    "slow_zone_m": 0.06,
                    "brake_margin_m": 0.03,
                    "timeout_s": self.args.forward_timeout_s,
                },
                {
                    "name": "staged_B_target_0p20",
                    "target_m": 0.20,
                    "fast_speed": 0.20,
                    "mid_speed": 0.15,
                    "slow_speed": 0.10,
                    "mid_zone_m": 0.12,
                    "slow_zone_m": 0.06,
                    "brake_margin_m": 0.03,
                    "timeout_s": self.args.forward_timeout_s,
                },
                {
                    "name": "staged_C_target_0p20_brake_0p05",
                    "target_m": 0.20,
                    "fast_speed": 0.20,
                    "mid_speed": 0.12,
                    "slow_speed": 0.10,
                    "mid_zone_m": 0.12,
                    "slow_zone_m": 0.06,
                    "brake_margin_m": 0.05,
                    "timeout_s": self.args.forward_timeout_s,
                },
            ]
        else:
            cases = [
                {
                    "name": "staged_single",
                    "target_m": self.args.forward_target_m,
                    "fast_speed": self.args.forward_fast_speed,
                    "mid_speed": self.args.forward_mid_speed,
                    "slow_speed": self.args.forward_slow_speed,
                    "mid_zone_m": self.args.forward_mid_zone_m,
                    "slow_zone_m": self.args.forward_slow_zone_m,
                    "brake_margin_m": self.args.forward_brake_margin_m,
                    "timeout_s": self.args.forward_timeout_s,
                }
            ]

        records = []
        sequence_stop_reason = None
        for case in cases:
            record = self.staged_forward_record(**case)
            records.append(record)
            print("FORWARD_STAGED_SEGMENT", json.dumps(record, ensure_ascii=False))
            if record.get("blocked") or str(record.get("stop_reason", "")).startswith("front_blocked"):
                sequence_stop_reason = f"{record['name']}: {record['stop_reason']}"
                break
            if not record.get("base_zero", {}).get("base_zero_ok"):
                sequence_stop_reason = f"{record['name']}: base_zero_failed"
                break

        return {
            "mode": "forward-staged",
            "control_mode": "staged",
            "test_set": self.args.staged_test_set,
            "sequence_stop_reason": sequence_stop_reason,
            "records": records,
        }

    def run_yaw_calibration(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        cases = [
            ("yaw_pos_0p40", 0.40, 1.0),
            ("yaw_neg_0p40", -0.40, 1.0),
            ("yaw_pos_0p80", 0.80, 1.0),
            ("yaw_neg_0p80", -0.80, 1.0),
        ]
        records = []
        for name, angular, duration in cases:
            record = self.segment_record(name, "yaw_calibration", 0.0, angular, duration)
            records.append(record)
            print("YAW_CALIBRATION_SEGMENT", json.dumps(record, ensure_ascii=False))
        return {"mode": "yaw-calibration", "records": records}

    def run_yaw_amplified(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        cases = [
            ("yaw_amp_pos", self.args.yaw_angular, self.args.yaw_duration_s),
            ("yaw_amp_neg", -self.args.yaw_angular, self.args.yaw_duration_s),
        ]
        records = []
        for name, angular, duration in cases:
            record = self.segment_record(name, "yaw_amplified", 0.0, angular, duration)
            record["expected_yaw_delta_deg"] = round(math.degrees(angular * duration), 2)
            records.append(record)
            print("YAW_AMPLIFIED_SEGMENT", json.dumps(record, ensure_ascii=False))
        return {
            "mode": "yaw-amplified",
            "yaw_angular": self.args.yaw_angular,
            "yaw_duration_s": self.args.yaw_duration_s,
            "records": records,
        }

    def run_turn_threshold(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        if self.args.turn_threshold_set == "strong":
            cases = [
                ("turn_pos_0p50_1p0s", 0.50, 1.0),
                ("turn_neg_0p50_1p0s", -0.50, 1.0),
                ("turn_pos_0p80_0p5s", 0.80, 0.5),
                ("turn_neg_0p80_0p5s", -0.80, 0.5),
                ("turn_pos_0p80_1p0s", 0.80, 1.0),
                ("turn_neg_0p80_1p0s", -0.80, 1.0),
            ]
        else:
            cases = [
                ("turn_pos_0p20_1p0s", 0.20, 1.0),
                ("turn_neg_0p20_1p0s", -0.20, 1.0),
                ("turn_pos_0p30_1p0s", 0.30, 1.0),
                ("turn_neg_0p30_1p0s", -0.30, 1.0),
                ("turn_pos_0p50_0p5s", 0.50, 0.5),
                ("turn_neg_0p50_0p5s", -0.50, 0.5),
            ]
        records = []
        for name, angular, duration in cases:
            record = self.turn_threshold_record(name, angular, duration)
            records.append(record)
            print("TURN_THRESHOLD_SEGMENT", json.dumps(record, ensure_ascii=False))
        return {
            "mode": "turn-threshold",
            "threshold_set": self.args.turn_threshold_set,
            "detect_deg": self.args.turn_threshold_detect_deg,
            "detect_wz": self.args.turn_threshold_detect_wz,
            "records": records,
        }

    def run_turn_duration_sweep(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        cases = [
            ("turn_pos_0p80_0p5s", 0.80, 0.5),
            ("turn_pos_0p80_1p0s", 0.80, 1.0),
            ("turn_pos_0p80_1p5s", 0.80, 1.5),
            ("turn_pos_0p80_2p0s", 0.80, 2.0),
            ("turn_neg_0p80_0p5s", -0.80, 0.5),
            ("turn_neg_0p80_1p0s", -0.80, 1.0),
            ("turn_neg_0p80_1p5s", -0.80, 1.5),
            ("turn_neg_0p80_2p0s", -0.80, 2.0),
        ]
        records = []
        for name, angular, duration in cases:
            record = self.turn_threshold_record(name, angular, duration)
            records.append(record)
            print("TURN_DURATION_SWEEP_SEGMENT", json.dumps(record, ensure_ascii=False))
            if not record.get("base_zero", {}).get("base_zero_ok"):
                break
        return {
            "mode": "turn-duration-sweep",
            "angular_rad_s": 0.80,
            "detect_deg": self.args.turn_threshold_detect_deg,
            "detect_wz": self.args.turn_threshold_detect_wz,
            "records": records,
        }

    def run_arc_turn_threshold(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        cases = [
            ("arc_lin0p10_pos0p50_1p0s", 0.10, 0.50, 1.0),
            ("arc_lin0p10_neg0p50_1p0s", 0.10, -0.50, 1.0),
            ("arc_lin0p12_pos0p80_1p0s", 0.12, 0.80, 1.0),
            ("arc_lin0p12_neg0p80_1p0s", 0.12, -0.80, 1.0),
        ]
        records = []
        sequence_stop_reason = None
        for name, linear, angular, duration in cases:
            record = self.turn_threshold_record(
                name,
                angular,
                duration,
                linear=linear,
                kind="arc_turn_threshold",
                front_check=True,
            )
            records.append(record)
            print("ARC_TURN_THRESHOLD_SEGMENT", json.dumps(record, ensure_ascii=False))
            if record.get("blocked") or str(record.get("stop_reason", "")).startswith("front_blocked"):
                sequence_stop_reason = f"{name}: {record.get('stop_reason')}"
                break
            if not record.get("base_zero", {}).get("base_zero_ok"):
                sequence_stop_reason = f"{name}: base_zero_failed"
                break
        return {
            "mode": "arc-turn-threshold",
            "sequence_stop_reason": sequence_stop_reason,
            "front_p10_min_m": self.args.forward_front_p10_min_m,
            "records": records,
        }

    def run_arc_step_repeat(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        sequence_start_msg = self.latest_odom
        records = []
        sequence_stop_reason = None
        directions = (("left", 1.0), ("right", -1.0))

        for direction, sign in directions:
            group_start_msg = self.latest_odom
            for repeat_index in range(1, self.args.arc_step_repeats + 1):
                angular = sign * abs(self.args.arc_step_angular)
                name = (
                    f"arc_step_{direction}_{repeat_index}_"
                    f"lin{self.args.arc_step_linear:.2f}_wz{angular:+.2f}_"
                    f"{self.args.arc_step_duration_s:.2f}s"
                )
                record = self.turn_threshold_record(
                    name,
                    angular,
                    self.args.arc_step_duration_s,
                    linear=self.args.arc_step_linear,
                    kind="arc_step_repeat",
                    front_check=True,
                )
                group_delta = odom_delta(group_start_msg, self.latest_odom)
                sequence_delta = odom_delta(sequence_start_msg, self.latest_odom)
                record["direction"] = direction
                record["repeat_index"] = repeat_index
                record["group_cumulative_yaw_deg"] = group_delta["delta_yaw_deg"]
                record["group_cumulative_forward_m"] = group_delta["forward_delta_m"]
                record["group_cumulative_lateral_m"] = group_delta["lateral_delta_m"]
                record["sequence_cumulative_yaw_deg"] = sequence_delta["delta_yaw_deg"]
                record["sequence_cumulative_forward_m"] = sequence_delta["forward_delta_m"]
                record["sequence_cumulative_lateral_m"] = sequence_delta["lateral_delta_m"]
                records.append(record)
                print("ARC_STEP_REPEAT_SEGMENT", json.dumps(record, ensure_ascii=False))

                if record.get("blocked") or str(record.get("stop_reason", "")).startswith("front_blocked"):
                    sequence_stop_reason = f"{name}: {record.get('stop_reason')}"
                    break
                if not record.get("base_zero", {}).get("base_zero_ok"):
                    sequence_stop_reason = f"{name}: base_zero_failed"
                    break
            if sequence_stop_reason is not None:
                break

        return {
            "mode": "arc-step-repeat",
            "sequence_stop_reason": sequence_stop_reason,
            "front_p10_min_m": self.args.forward_front_p10_min_m,
            "arc_step": {
                "linear": round(float(self.args.arc_step_linear), 4),
                "angular_abs": round(float(abs(self.args.arc_step_angular)), 4),
                "duration_s": round(float(self.args.arc_step_duration_s), 3),
                "repeats_per_direction": int(self.args.arc_step_repeats),
            },
            "records": records,
        }

    def arc_fast_calib_cases(self):
        if self.args.arc_fast_profile == "g1":
            return (
                ("lin0p10_wz0p70_1p2s", 0.10, 0.70, 1.2),
                ("lin0p10_wz0p80_1p2s", 0.10, 0.80, 1.2),
                ("lin0p12_wz0p70_1p2s", 0.12, 0.70, 1.2),
                ("lin0p12_wz0p80_1p0s", 0.12, 0.80, 1.0),
            )
        return (
            ("lin0p10_wz0p50_1p0s", 0.10, 0.50, 1.0),
            ("lin0p10_wz0p55_1p0s", 0.10, 0.55, 1.0),
            ("lin0p10_wz0p60_1p0s", 0.10, 0.60, 1.0),
            ("lin0p10_wz0p55_1p2s", 0.10, 0.55, 1.2),
        )

    def arc_fast_directions(self):
        if self.args.arc_fast_direction == "both":
            return (("left", 1.0), ("right", -1.0))
        if self.args.arc_fast_direction == "left":
            return (("left", 1.0),)
        return (("right", -1.0),)

    def run_arc_fast_calib(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        sequence_start_msg = self.latest_odom
        records = []
        sequence_stop_reason = None
        target_lower = 20.0
        target_upper = 35.0
        front_danger_min_m = 0.20
        front_observe_p10_m = 0.30
        old_gate = self.args.forward_front_p10_min_m
        self.args.forward_front_p10_min_m = self.args.arc_fast_front_p10_min_m

        try:
            for case_index, (case_name, linear, angular_abs, duration) in enumerate(
                self.arc_fast_calib_cases(),
                start=1,
            ):
                for direction, sign in self.arc_fast_directions():
                    angular = sign * angular_abs
                    name = f"arc_fast_calib_{case_index:02d}_{case_name}_{direction}"
                    record = self.turn_threshold_record(
                        name,
                        angular,
                        duration,
                        linear=linear,
                        kind="arc_fast_calib",
                        front_check=True,
                    )
                    pre = record.get("precheck") or {}
                    post = record.get("postcheck") or {}
                    front_p10_before = pre.get("front_p10_range_m")
                    front_p10_after = post.get("front_p10_range_m")
                    front_min_after = post.get("front_min_range_m")
                    abs_yaw = abs(float(record.get("delta_yaw_deg") or 0.0))
                    base_zero_ok = bool((record.get("base_zero") or {}).get("base_zero_ok"))
                    front_improved = (
                        front_p10_before is not None
                        and front_p10_after is not None
                        and float(front_p10_after) > float(front_p10_before)
                    )
                    front_min_safe = (
                        front_min_after is None or float(front_min_after) >= front_danger_min_m
                    )
                    front_not_danger = (
                        front_min_safe
                        and (front_p10_after is None or float(front_p10_after) >= front_observe_p10_m)
                    )
                    yaw_in_band = target_lower <= abs_yaw <= target_upper
                    timing = record.get("timing_breakdown") or {}
                    sequence_delta = odom_delta(sequence_start_msg, self.latest_odom)

                    record["case_index"] = case_index
                    record["case_name"] = case_name
                    record["direction"] = direction
                    record["arc_fast_candidate"] = {
                        "linear": round(float(linear), 4),
                        "angular": round(float(angular), 4),
                        "duration_s": round(float(duration), 3),
                    }
                    record["acceptance"] = {
                        "target_abs_yaw_min_deg": target_lower,
                        "target_abs_yaw_max_deg": target_upper,
                        "front_danger_min_m": front_danger_min_m,
                        "front_observe_p10_m": front_observe_p10_m,
                    }
                    record["abs_yaw_delta_deg"] = round(float(abs_yaw), 2)
                    record["yaw_in_target_band"] = yaw_in_band
                    record["front_p10_before"] = front_p10_before
                    record["front_p10_after"] = front_p10_after
                    record["front_improved"] = front_improved
                    record["front_min_safe"] = front_min_safe
                    record["front_not_danger"] = front_not_danger
                    record["base_zero_ok"] = base_zero_ok
                    record["elapsed_s"] = timing.get("total_time_s")
                    record["candidate_ok"] = (
                        not record.get("blocked", False)
                        and yaw_in_band
                        and base_zero_ok
                        and front_min_safe
                        and (front_not_danger or front_improved)
                    )
                    record["sequence_cumulative_yaw_deg"] = sequence_delta["delta_yaw_deg"]
                    record["sequence_cumulative_forward_m"] = sequence_delta["forward_delta_m"]
                    record["sequence_cumulative_lateral_m"] = sequence_delta["lateral_delta_m"]
                    records.append(record)
                    self.emit_arc_fast_calib_record(record)

                    if record.get("blocked") or str(record.get("stop_reason", "")).startswith("front_blocked"):
                        sequence_stop_reason = f"{name}: {record.get('stop_reason')}"
                        break
                    if not base_zero_ok:
                        sequence_stop_reason = f"{name}: base_zero_failed"
                        break
                    if front_min_after is not None and float(front_min_after) < front_danger_min_m:
                        sequence_stop_reason = (
                            f"{name}: front_min_after {float(front_min_after):.3f} "
                            f"< {front_danger_min_m:.2f}"
                        )
                        break
                if sequence_stop_reason is not None:
                    break
        finally:
            self.args.forward_front_p10_min_m = old_gate

        ok_records = [record for record in records if record.get("candidate_ok")]
        fastest_ok = None
        if ok_records:
            fastest_ok = min(
                ok_records,
                key=lambda record: float(record.get("elapsed_s") or 9999.0),
            )

        return {
            "mode": "arc-fast-calib",
            "arc_fast_profile": self.args.arc_fast_profile,
            "sequence_stop_reason": sequence_stop_reason,
            "front_p10_min_m": self.args.arc_fast_front_p10_min_m,
            "acceptance": {
                "target_abs_yaw_min_deg": target_lower,
                "target_abs_yaw_max_deg": target_upper,
                "front_danger_min_m": front_danger_min_m,
                "front_observe_p10_m": front_observe_p10_m,
            },
            "directions": [direction for direction, _ in self.arc_fast_directions()],
            "cases": [
                {
                    "case_name": case_name,
                    "linear": linear,
                    "angular_abs": angular_abs,
                    "duration_s": duration,
                }
                for case_name, linear, angular_abs, duration in self.arc_fast_calib_cases()
            ],
            "records": records,
            "ok_count": len(ok_records),
            "fastest_ok": None
            if fastest_ok is None
            else {
                "name": fastest_ok.get("name"),
                "case_name": fastest_ok.get("case_name"),
                "direction": fastest_ok.get("direction"),
                "command": fastest_ok.get("command"),
                "abs_yaw_delta_deg": fastest_ok.get("abs_yaw_delta_deg"),
                "forward_drift_m": fastest_ok.get("forward_drift_m"),
                "lateral_drift_m": fastest_ok.get("lateral_drift_m"),
                "front_p10_before": fastest_ok.get("front_p10_before"),
                "front_p10_after": fastest_ok.get("front_p10_after"),
                "elapsed_s": fastest_ok.get("elapsed_s"),
            },
        }

    def run_arc_yaw_closed(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        if self.args.arc_yaw_direction == "both":
            directions = (("left", 1.0), ("right", -1.0))
        elif self.args.arc_yaw_direction == "left":
            directions = (("left", 1.0),)
        else:
            directions = (("right", -1.0),)

        records = []
        sequence_stop_reason = None
        target_abs = float(self.args.arc_yaw_target_deg)
        tolerance = float(self.args.arc_yaw_tolerance_deg)
        overshoot_epsilon = float(self.args.arc_yaw_overshoot_epsilon_deg)
        lower_bound = target_abs - tolerance
        upper_bound = target_abs + tolerance
        accepted_upper_bound = upper_bound + overshoot_epsilon

        for direction, sign in directions:
            group_start_msg = self.latest_odom
            direction_records = []
            direction_stop_reason = "max_steps_reached"
            final_cumulative_yaw = 0.0
            final_cumulative_forward = 0.0
            final_cumulative_lateral = 0.0

            for step_index in range(1, self.args.arc_max_steps + 1):
                angular = sign * abs(self.args.arc_step_angular)
                name = (
                    f"arc_yaw_closed_{direction}_{step_index}_"
                    f"target{target_abs:.1f}_tol{tolerance:.1f}"
                )
                record = self.turn_threshold_record(
                    name,
                    angular,
                    self.args.arc_step_duration_s,
                    linear=self.args.arc_step_linear,
                    kind="arc_yaw_closed_step",
                    front_check=True,
                )
                group_delta = odom_delta(group_start_msg, self.latest_odom)
                cumulative_yaw = group_delta["delta_yaw_deg"]
                signed_progress = sign * cumulative_yaw
                record["direction"] = direction
                record["step_index"] = step_index
                record["target_yaw_deg"] = round(float(sign * target_abs), 2)
                record["target_band_deg"] = {
                    "lower": round(float(sign * lower_bound), 2),
                    "upper": round(float(sign * upper_bound), 2),
                    "abs_lower": round(float(lower_bound), 2),
                    "abs_upper": round(float(upper_bound), 2),
                }
                record["cumulative_yaw_deg"] = cumulative_yaw
                record["cumulative_abs_yaw_deg"] = round(float(abs(cumulative_yaw)), 2)
                record["cumulative_forward_m"] = group_delta["forward_delta_m"]
                record["cumulative_lateral_m"] = group_delta["lateral_delta_m"]
                record["target_band_reached"] = lower_bound <= signed_progress <= accepted_upper_bound
                record["target_band_strict_reached"] = lower_bound <= signed_progress <= upper_bound
                record["target_accepted_by_overshoot_epsilon"] = (
                    upper_bound < signed_progress <= accepted_upper_bound
                )
                record["target_overshot"] = signed_progress > accepted_upper_bound
                direction_records.append(record)
                self.emit_arc_yaw_closed_step(record)

                final_cumulative_yaw = cumulative_yaw
                final_cumulative_forward = group_delta["forward_delta_m"]
                final_cumulative_lateral = group_delta["lateral_delta_m"]

                if record.get("blocked") or str(record.get("stop_reason", "")).startswith("front_blocked"):
                    direction_stop_reason = f"{name}: {record.get('stop_reason')}"
                    sequence_stop_reason = direction_stop_reason
                    break
                if not record.get("base_zero", {}).get("base_zero_ok"):
                    direction_stop_reason = f"{name}: base_zero_failed"
                    sequence_stop_reason = direction_stop_reason
                    break
                if record["target_band_reached"]:
                    direction_stop_reason = "target_band_reached"
                    break
                if record["target_overshot"]:
                    direction_stop_reason = "target_overshot"
                    break

            summary = {
                "direction": direction,
                "target_yaw_deg": round(float(sign * target_abs), 2),
                "target_tolerance_deg": round(float(tolerance), 2),
                "target_band_abs_deg": {
                    "lower": round(float(lower_bound), 2),
                    "upper": round(float(upper_bound), 2),
                    "accepted_upper": round(float(accepted_upper_bound), 2),
                },
                "steps_used": len(direction_records),
                "final_cumulative_yaw_deg": final_cumulative_yaw,
                "final_cumulative_abs_yaw_deg": round(float(abs(final_cumulative_yaw)), 2),
                "final_cumulative_forward_m": final_cumulative_forward,
                "final_cumulative_lateral_m": final_cumulative_lateral,
                "stop_reason": direction_stop_reason,
                "target_band_reached": (
                    lower_bound <= sign * final_cumulative_yaw <= accepted_upper_bound
                ),
                "target_band_strict_reached": lower_bound <= sign * final_cumulative_yaw <= upper_bound,
                "target_accepted_by_overshoot_epsilon": (
                    upper_bound < sign * final_cumulative_yaw <= accepted_upper_bound
                ),
                "timing_breakdown": self.combine_timing_breakdowns(direction_records),
                "records": direction_records,
            }
            records.append(summary)
            self.emit_arc_yaw_closed_direction(summary)

            if sequence_stop_reason is not None:
                break

        return {
            "mode": "arc-yaw-closed",
            "sequence_stop_reason": sequence_stop_reason,
            "front_p10_min_m": self.args.forward_front_p10_min_m,
            "arc_step": {
                "linear": round(float(self.args.arc_step_linear), 4),
                "angular_abs": round(float(abs(self.args.arc_step_angular)), 4),
                "duration_s": round(float(self.args.arc_step_duration_s), 3),
                "max_steps": int(self.args.arc_max_steps),
            },
            "target": {
                "yaw_deg_abs": round(float(target_abs), 2),
                "tolerance_deg": round(float(tolerance), 2),
                "band_abs_deg": {
                    "lower": round(float(lower_bound), 2),
                    "upper": round(float(upper_bound), 2),
                    "accepted_upper": round(float(accepted_upper_bound), 2),
                },
                "overshoot_epsilon_deg": round(float(overshoot_epsilon), 2),
            },
            "directions": records,
        }

    def arc_yaw_closed_direction_record(self, direction: str, name_prefix: str, front_gate_m=None):
        if direction not in ("left", "right"):
            raise ValueError(f"unsupported arc direction: {direction}")
        sign = 1.0 if direction == "left" else -1.0
        target_abs = float(self.args.arc_yaw_target_deg)
        tolerance = float(self.args.arc_yaw_tolerance_deg)
        overshoot_epsilon = float(self.args.arc_yaw_overshoot_epsilon_deg)
        lower_bound = target_abs - tolerance
        upper_bound = target_abs + tolerance
        accepted_upper_bound = upper_bound + overshoot_epsilon
        group_start_msg = self.latest_odom
        records = []
        stop_reason = "max_steps_reached"
        final_cumulative_yaw = 0.0
        final_cumulative_forward = 0.0
        final_cumulative_lateral = 0.0

        for step_index in range(1, self.args.arc_max_steps + 1):
            angular = sign * abs(self.args.arc_step_angular)
            name = f"{name_prefix}_{direction}_{step_index}"
            record = self.turn_threshold_record(
                name,
                angular,
                self.args.arc_step_duration_s,
                linear=self.args.arc_step_linear,
                kind="arc_yaw_closed_step",
                front_check=True,
                front_gate_m=front_gate_m,
            )
            group_delta = odom_delta(group_start_msg, self.latest_odom)
            cumulative_yaw = group_delta["delta_yaw_deg"]
            signed_progress = sign * cumulative_yaw
            record["direction"] = direction
            record["step_index"] = step_index
            record["target_yaw_deg"] = round(float(sign * target_abs), 2)
            record["target_band_deg"] = {
                "lower": round(float(sign * lower_bound), 2),
                "upper": round(float(sign * upper_bound), 2),
                "accepted_upper": round(float(sign * accepted_upper_bound), 2),
                "abs_lower": round(float(lower_bound), 2),
                "abs_upper": round(float(upper_bound), 2),
                "abs_accepted_upper": round(float(accepted_upper_bound), 2),
            }
            record["cumulative_yaw_deg"] = cumulative_yaw
            record["cumulative_abs_yaw_deg"] = round(float(abs(cumulative_yaw)), 2)
            record["cumulative_forward_m"] = group_delta["forward_delta_m"]
            record["cumulative_lateral_m"] = group_delta["lateral_delta_m"]
            record["target_band_reached"] = lower_bound <= signed_progress <= accepted_upper_bound
            record["target_band_strict_reached"] = lower_bound <= signed_progress <= upper_bound
            record["target_accepted_by_overshoot_epsilon"] = (
                upper_bound < signed_progress <= accepted_upper_bound
            )
            record["target_overshot"] = signed_progress > accepted_upper_bound
            records.append(record)
            self.emit_arc_yaw_closed_step(record)

            final_cumulative_yaw = cumulative_yaw
            final_cumulative_forward = group_delta["forward_delta_m"]
            final_cumulative_lateral = group_delta["lateral_delta_m"]

            if record.get("blocked") or str(record.get("stop_reason", "")).startswith("front_blocked"):
                stop_reason = f"{name}: {record.get('stop_reason')}"
                break
            if not record.get("base_zero", {}).get("base_zero_ok"):
                stop_reason = f"{name}: base_zero_failed"
                break
            if record["target_band_reached"]:
                stop_reason = "target_band_reached"
                break
            if record["target_overshot"]:
                stop_reason = "target_overshot"
                break

        return {
            "direction": direction,
            "target_yaw_deg": round(float(sign * target_abs), 2),
            "target_tolerance_deg": round(float(tolerance), 2),
            "target_band_abs_deg": {
                "lower": round(float(lower_bound), 2),
                "upper": round(float(upper_bound), 2),
                "accepted_upper": round(float(accepted_upper_bound), 2),
            },
            "steps_used": len(records),
            "final_cumulative_yaw_deg": final_cumulative_yaw,
            "final_cumulative_abs_yaw_deg": round(float(abs(final_cumulative_yaw)), 2),
            "final_cumulative_forward_m": final_cumulative_forward,
            "final_cumulative_lateral_m": final_cumulative_lateral,
            "stop_reason": stop_reason,
            "target_band_reached": (
                lower_bound <= sign * final_cumulative_yaw <= accepted_upper_bound
            ),
            "target_band_strict_reached": lower_bound <= sign * final_cumulative_yaw <= upper_bound,
            "target_accepted_by_overshoot_epsilon": (
                upper_bound < sign * final_cumulative_yaw <= accepted_upper_bound
            ),
            "timing_breakdown": self.combine_timing_breakdowns(records),
            "records": records,
        }

    def policy_float(self, value):
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def select_policy_action(self, pre):
        profile = self.args.behavior_profile
        freshness = pre.get("freshness") or {}
        required_fresh = (
            "odom_fresh",
            "status_fresh",
            "robot_vel_fresh",
            "diag_fresh",
        )
        stale = [
            key.replace("_fresh", "")
            for key in required_fresh
            if not freshness.get(key, False)
        ]
        front_min = self.policy_float(pre.get("front_min_range_m"))
        front_p10 = self.policy_float(pre.get("front_p10_range_m"))
        decision = {
            "profile": profile,
            "front_min": None if front_min is None else round(front_min, 4),
            "front_p10": None if front_p10 is None else round(front_p10, 4),
            "selected_action": "NOT_READY",
            "action_reason": "",
            "threshold_band": "not_ready",
        }

        if stale:
            decision["action_reason"] = "stale_" + "_".join(stale)
            return decision
        if pre.get("map") is None:
            decision["action_reason"] = "map_metadata_unavailable"
            return decision
        if front_p10 is None:
            decision["action_reason"] = "front_p10_unavailable"
            return decision
        if profile == "interaction_mode" and front_min is None:
            decision["action_reason"] = "front_min_unavailable"
            return decision

        if profile == "mapping_safe_mode":
            if front_p10 < 0.50:
                action = "HARD_STOP"
                reason = "front_p10 < 0.50m mapping hard stop"
                band = "mapping_front_p10_lt_0p50"
            elif front_p10 < 0.80:
                action = "HOLD_AND_SAVE"
                reason = "0.50m <= front_p10 < 0.80m"
                band = "mapping_0p50_to_0p80"
            elif front_p10 < 1.20:
                action = "ARC30_PREFERRED"
                reason = "0.80m <= front_p10 < 1.20m"
                band = "mapping_0p80_to_1p20"
            else:
                action = "FORWARD_ALLOWED"
                reason = "front_p10 >= 1.20m"
                band = "mapping_ge_1p20"
        else:
            if front_min < 0.20:
                action = "HARD_STOP"
                reason = "front_min < 0.20m interaction hard stop"
                band = "interaction_front_min_lt_0p20"
            elif front_p10 < 0.30:
                action = "HOLD_AND_CAPTURE"
                reason = "front_p10 < 0.30m"
                band = "interaction_front_p10_lt_0p30"
            elif front_p10 < 0.40:
                action = "HOLD_SAVE_OBSERVE"
                reason = "0.30m <= front_p10 < 0.40m"
                band = "interaction_0p30_to_0p40"
            elif front_p10 < 0.60:
                action = "ARC30_OR_FORWARD_0P05"
                reason = "0.40m <= front_p10 < 0.60m"
                band = "interaction_0p40_to_0p60"
            elif front_p10 < 0.80:
                action = "ARC30_OR_FORWARD_0P10"
                reason = "0.60m <= front_p10 < 0.80m"
                band = "interaction_0p60_to_0p80"
            else:
                action = "FORWARD_0P15_OR_ARC30"
                reason = "front_p10 >= 0.80m"
                band = "interaction_ge_0p80"

        decision["selected_action"] = action
        decision["action_reason"] = reason
        decision["threshold_band"] = band
        return decision

    def policy_arc_plan(self, arc_direction: str, arc_direction_reason: str, front_gate_m: float):
        if self.args.policy_arc_mode == "fast":
            return {
                "execution_action": f"ARC_FAST_{arc_direction.upper()}",
                "motion": "arc_fast",
                "front_gate_m": front_gate_m,
                "forward_target_m": None,
                "arc_direction": arc_direction,
                "arc_direction_reason": arc_direction_reason,
                "capture_placeholder": False,
                "arc_mode": "fast",
            }
        return {
            "execution_action": f"ARC30_{arc_direction.upper()}",
            "motion": "arc30",
            "front_gate_m": front_gate_m,
            "forward_target_m": None,
            "arc_direction": arc_direction,
            "arc_direction_reason": arc_direction_reason,
            "capture_placeholder": False,
            "arc_mode": "precise",
        }

    def policy_execution_plan(self, selected_action: str, pre=None):
        arc_direction, arc_direction_reason = self.resolve_policy_arc_direction(pre)
        if selected_action in (
            "NOT_READY",
            "HARD_STOP",
            "HOLD_AND_SAVE",
            "HOLD_AND_CAPTURE",
            "HOLD_SAVE_OBSERVE",
        ):
            return {
                "execution_action": "HOLD",
                "motion": "hold",
                "front_gate_m": None,
                "forward_target_m": None,
                "arc_direction": None,
                "arc_direction_reason": None,
                "capture_placeholder": selected_action == "HOLD_AND_CAPTURE",
                "arc_mode": None,
            }
        if selected_action == "ARC30_PREFERRED":
            return self.policy_arc_plan(arc_direction, arc_direction_reason, 0.80)
        if selected_action == "FORWARD_ALLOWED":
            return {
                "execution_action": "FORWARD_0P15",
                "motion": "forward",
                "front_gate_m": 1.20,
                "forward_target_m": 0.15,
                "arc_direction": None,
                "arc_direction_reason": None,
                "capture_placeholder": False,
                "arc_mode": None,
            }
        if selected_action == "ARC30_OR_FORWARD_0P05":
            if self.args.policy_close_action == "forward":
                return {
                    "execution_action": "FORWARD_0P05",
                    "motion": "forward",
                    "front_gate_m": 0.40,
                    "forward_target_m": 0.05,
                    "arc_direction": None,
                    "arc_direction_reason": None,
                    "capture_placeholder": False,
                    "arc_mode": None,
                }
            return self.policy_arc_plan(arc_direction, arc_direction_reason, 0.40)
        if selected_action == "ARC30_OR_FORWARD_0P10":
            if self.args.policy_mid_action == "forward":
                return {
                    "execution_action": "FORWARD_0P10",
                    "motion": "forward",
                    "front_gate_m": 0.60,
                    "forward_target_m": 0.10,
                    "arc_direction": None,
                    "arc_direction_reason": None,
                    "capture_placeholder": False,
                    "arc_mode": None,
                }
            return self.policy_arc_plan(arc_direction, arc_direction_reason, 0.60)
        if selected_action == "FORWARD_0P15_OR_ARC30":
            if self.args.policy_normal_action == "arc30":
                return self.policy_arc_plan(arc_direction, arc_direction_reason, 0.80)
            return {
                "execution_action": "FORWARD_0P15",
                "motion": "forward",
                "front_gate_m": 0.80,
                "forward_target_m": 0.15,
                "arc_direction": None,
                "arc_direction_reason": None,
                "capture_placeholder": False,
                "arc_mode": None,
            }
        return {
            "execution_action": "HOLD",
            "motion": "hold",
            "front_gate_m": None,
            "forward_target_m": None,
            "arc_direction": None,
            "arc_direction_reason": None,
            "capture_placeholder": False,
            "arc_mode": None,
        }

    def resolve_policy_arc_direction(self, pre):
        configured = self.args.policy_arc_direction
        if configured in ("left", "right"):
            return configured, f"configured_{configured}"
        sectors = (pre or {}).get("scan_sectors") or {}
        left = ((sectors.get("left") or {}).get("p10"))
        right = ((sectors.get("right") or {}).get("p10"))
        if left is None and right is None:
            return "left", "auto_no_side_scan_fallback_left"
        if left is None:
            return "right", "auto_right_only_side_scan"
        if right is None:
            return "left", "auto_left_only_side_scan"
        if left >= right:
            return "left", f"auto_left_clearer_p10_{left:.3f}_vs_{right:.3f}"
        return "right", f"auto_right_clearer_p10_{right:.3f}_vs_{left:.3f}"

    def policy_forward_zones(self, target_m: float):
        if target_m <= 0.05:
            mid_zone = 0.04
            slow_zone = 0.035
        elif target_m <= 0.10:
            mid_zone = 0.08
            slow_zone = 0.05
        else:
            mid_zone = 0.12
            slow_zone = 0.07
        brake_margin = min(
            self.args.forward_brake_margin_m,
            max(0.015, slow_zone - 0.005),
        )
        return mid_zone, slow_zone, brake_margin

    def run_policy_forward_once(self, name: str, target_m: float, front_gate_m: float):
        old_gate = self.args.forward_front_p10_min_m
        self.args.forward_front_p10_min_m = front_gate_m
        try:
            mid_zone, slow_zone, brake_margin = self.policy_forward_zones(target_m)
            return self.staged_forward_record(
                name=name,
                target_m=target_m,
                fast_speed=self.args.forward_fast_speed,
                mid_speed=self.args.forward_mid_speed,
                slow_speed=self.args.forward_slow_speed,
                mid_zone_m=mid_zone,
                slow_zone_m=slow_zone,
                brake_margin_m=brake_margin,
                timeout_s=self.args.forward_timeout_s,
            )
        finally:
            self.args.forward_front_p10_min_m = old_gate

    def run_policy_arc_once(self, name: str, front_gate_m: float, direction: str):
        return self.arc_yaw_closed_direction_record(direction, name, front_gate_m=front_gate_m)

    def run_policy_arc_fast_once(self, name: str, front_gate_m: float, direction: str):
        if direction not in ("left", "right"):
            raise ValueError(f"unsupported arc direction: {direction}")
        sign = 1.0 if direction == "left" else -1.0
        angular = sign * abs(self.args.policy_arc_fast_angular)
        record = self.turn_threshold_record(
            name,
            angular,
            self.args.policy_arc_fast_duration_s,
            linear=self.args.policy_arc_fast_linear,
            kind="guarded_policy_arc_fast",
            front_check=True,
            front_gate_m=front_gate_m,
        )
        pre = record.get("precheck") or {}
        post = record.get("postcheck") or {}
        front_min_after = post.get("front_min_range_m")
        front_p10_after = post.get("front_p10_range_m")
        abs_yaw = abs(float(record.get("delta_yaw_deg") or 0.0))
        record["direction"] = direction
        record["arc_mode"] = "fast"
        record["front_gate_m"] = round(float(front_gate_m), 3)
        record["policy_arc_fast"] = {
            "linear": round(float(self.args.policy_arc_fast_linear), 4),
            "angular": round(float(angular), 4),
            "angular_abs": round(float(abs(self.args.policy_arc_fast_angular)), 4),
            "duration_s": round(float(self.args.policy_arc_fast_duration_s), 3),
        }
        record["front_p10_before"] = pre.get("front_p10_range_m")
        record["front_p10_after"] = front_p10_after
        record["front_min_after"] = front_min_after
        record["front_min_safe"] = front_min_after is None or float(front_min_after) >= 0.20
        record["abs_yaw_delta_deg"] = round(float(abs_yaw), 2)
        record["fast_expected_yaw_band_deg"] = {"lower": 20.0, "upper": 35.0}
        record["fast_yaw_in_expected_band"] = 20.0 <= abs_yaw <= 35.0
        record["base_zero_ok"] = bool((record.get("base_zero") or {}).get("base_zero_ok"))
        return record

    def policy_hold_record(self, name: str, decision, plan, odom_before_msg):
        segment_start = time.monotonic()
        zero_report = self.zero_hold(self.args.zero_hold_s)
        zero = zero_report["base_zero"]
        postcheck_start = time.monotonic()
        post = self.precheck()
        postcheck_end = time.monotonic()
        segment_end = time.monotonic()
        return {
            "name": name,
            "kind": "guarded_policy_hold",
            "decision": decision,
            "execution_plan": plan,
            "executed": False,
            "base_zero": zero,
            "base_zero_ok": zero.get("base_zero_ok"),
            "odom_before": odom_snapshot(odom_before_msg),
            "odom_after": odom_snapshot(self.latest_odom),
            "odom_delta": odom_delta(odom_before_msg, self.latest_odom),
            "postcheck": post,
            "timing_breakdown": self.segment_timing_breakdown(
                segment_start,
                segment_start,
                segment_start,
                segment_start,
                segment_start,
                zero_report,
                postcheck_start,
                postcheck_end,
                segment_end,
            ),
        }

    def policy_record_base_zero_ok(self, action_record, motion: str):
        if motion == "forward":
            return bool(action_record.get("base_zero", {}).get("base_zero_ok"))
        if motion == "arc30":
            steps = action_record.get("records", [])
            return bool(steps) and all(
                step.get("base_zero", {}).get("base_zero_ok") for step in steps
            )
        return bool(action_record.get("base_zero", {}).get("base_zero_ok"))

    def is_policy_critical_event(self, decision, plan, stop_reason):
        selected = (decision or {}).get("selected_action")
        if selected in {
            "HARD_STOP",
            "HOLD_AND_CAPTURE",
            "HOLD_SAVE_OBSERVE",
            "HOLD_AND_SAVE",
        }:
            return True
        if (plan or {}).get("motion") == "hold":
            return True
        if (plan or {}).get("capture_placeholder"):
            return True
        if "hard_stop" in str(stop_reason or "").lower():
            return True
        if "max_consecutive_fast_arc" in str(stop_reason or ""):
            return True
        return False

    def policy_step_save_prefix(self, step_index, suffix="policy"):
        if step_index is None:
            return f"{self.args.map_prefix}_{suffix}"
        return f"{self.args.map_prefix}_{step_index:02d}_{suffix}"

    def write_step_checkpoint(self, record):
        checkpoint_path = Path(self.args.report).with_suffix(".checkpoints.jsonl")
        checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
        with checkpoint_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        return str(checkpoint_path)

    def save_policy_step_map(
        self,
        prefix: str,
        step_index,
        decision,
        plan,
        stop_reason,
        base_zero_ok: bool,
        saved_maps,
    ):
        if not base_zero_ok or decision.get("selected_action") == "NOT_READY":
            return None, "not_saved_not_ready_or_base_not_zero"

        policy = self.args.save_policy
        critical = self.is_policy_critical_event(decision, plan, stop_reason)
        reason = "critical" if critical else "normal"
        if policy == "every_step":
            result = self.save_map(prefix)
            result.update({"save_policy": policy, "reason": reason, "async": False})
            saved_maps.append(result)
            return result, "sync_every_step"

        if critical:
            waited = self.wait_pending_save("before_critical_save")
            result = self.save_map(prefix)
            result.update(
                {
                    "save_policy": policy,
                    "reason": reason,
                    "async": False,
                    "waited_pending_save": waited,
                }
            )
            saved_maps.append(result)
            return result, "sync_critical"

        if policy == "critical_or_end":
            return {
                "prefix": prefix,
                "ok": None,
                "status": "checkpoint_only",
                "save_policy": policy,
                "reason": reason,
            }, "checkpoint_only"

        if policy == "every_n_steps" and step_index % self.args.save_every_n != 0:
            return {
                "prefix": prefix,
                "ok": None,
                "status": "skipped_not_n_step",
                "save_policy": policy,
                "reason": reason,
                "save_every_n": self.args.save_every_n,
            }, "skipped_not_n_step"

        if policy in ("every_n_steps", "pipelined_critical"):
            result = self.start_async_save(prefix, reason=reason, step_index=step_index)
            result["save_policy"] = policy
            saved_maps.append(result)
            return result, str(result.get("status"))

        return {
            "prefix": prefix,
            "ok": False,
            "status": "unknown_save_policy",
            "save_policy": policy,
            "reason": reason,
            "error": f"unknown save_policy {policy}",
        }, "unknown_save_policy"

    def save_policy_final_map(self, saved_maps):
        started = time.monotonic()
        waited = self.wait_pending_save("before_final_save")
        prefix = f"{self.args.map_prefix}_final"
        result = self.save_map(prefix)
        ended = time.monotonic()
        result.update(
            {
                "save_policy": self.args.save_policy,
                "reason": "run_end",
                "async": False,
                "waited_pending_save": waited,
                "elapsed_with_pending_wait_s": round(ended - started, 3),
            }
        )
        saved_maps.append(result)
        return result

    def map_save_label(self, record):
        if not record:
            return "none"
        status = record.get("status")
        if status in {"pending", "skipped_pending", "checkpoint_only", "skipped_not_n_step"}:
            return status
        if record.get("ok") is True:
            return "saved"
        if record.get("ok") is False:
            return "failed"
        return str(status or "unknown")

    def emit_policy_run_step(self, record):
        if self.args.console_mode != "compact":
            print("GUARDED_POLICY_RUN_STEP", json.dumps(record, ensure_ascii=False))
            return
        odom = record.get("odom_delta") or {}
        timing = record.get("timing_breakdown") or {}
        print(
            f"STEP {record.get('step_index')} {record.get('execution_action')} "
            f"executed={str(record.get('executed')).lower()} "
            f"stop={record.get('stop_reason') or 'none'} "
            f"fwd={odom.get('forward_delta_m')}m yaw={odom.get('delta_yaw_deg')}deg "
            f"zero={'ok' if record.get('base_zero_ok') else 'bad'} "
            f"map={self.map_save_label(record.get('map_save'))} "
            f"elapsed={timing.get('step_total_time_s')}s"
        )

    def emit_policy_step_result(self, result):
        if self.args.console_mode != "compact":
            print("GUARDED_POLICY_STEP", json.dumps(result, ensure_ascii=False))
            return
        odom = result.get("odom_delta") or {}
        timing = result.get("timing_breakdown") or {}
        print(
            f"STEP {result.get('execution_action')} "
            f"executed={str(result.get('executed')).lower()} "
            f"stop={result.get('stop_reason') or 'none'} "
            f"fwd={odom.get('forward_delta_m')}m yaw={odom.get('delta_yaw_deg')}deg "
            f"zero={'ok' if result.get('base_zero_ok') else 'bad'} "
            f"map={self.map_save_label(result.get('map_save'))} "
            f"final_map={self.map_save_label(result.get('final_map_save'))} "
            f"elapsed={timing.get('step_total_time_s')}s"
        )

    def emit_policy_run_result(self, result):
        if self.args.console_mode != "compact":
            print("GUARDED_POLICY_RUN", json.dumps(result, ensure_ascii=False))
            return
        odom = result.get("odom_delta") or {}
        print(
            f"RUN stop={result.get('sequence_stop_reason')} "
            f"steps={result.get('step_count')} executed={result.get('executed_count')} "
            f"zero={'ok' if result.get('base_zero_ok') else 'bad'} "
            f"final_map={self.map_save_label(result.get('final_map_save'))} "
            f"fwd={odom.get('forward_delta_m')}m yaw={odom.get('delta_yaw_deg')}deg"
        )

    def emit_arc_yaw_closed_step(self, record):
        if self.args.console_mode != "compact":
            print("ARC_YAW_CLOSED_STEP", json.dumps(record, ensure_ascii=False))
            return
        timing = record.get("timing_breakdown") or {}
        print(
            f"ARC_STEP {record.get('direction')}#{record.get('step_index')} "
            f"yaw={record.get('yaw_delta_after_settle_deg')}deg "
            f"cum={record.get('cumulative_yaw_deg')}deg "
            f"zero={'ok' if (record.get('base_zero') or {}).get('base_zero_ok') else 'bad'} "
            f"stop={record.get('stop_reason')} "
            f"elapsed={timing.get('total_time_s')}s"
        )

    def emit_arc_yaw_closed_direction(self, summary):
        if self.args.console_mode != "compact":
            print("ARC_YAW_CLOSED_DIRECTION", json.dumps(summary, ensure_ascii=False))
            return
        print(
            f"ARC_DIRECTION {summary.get('direction')} "
            f"steps={summary.get('steps_used')} "
            f"yaw={summary.get('final_cumulative_yaw_deg')}deg "
            f"stop={summary.get('stop_reason')}"
        )

    def emit_arc_fast_calib_record(self, record):
        if self.args.console_mode != "compact":
            print("ARC_FAST_CALIB_SEGMENT", json.dumps(record, ensure_ascii=False))
            return
        print(
            f"ARC_FAST {record.get('direction')} {record.get('case_name')} "
            f"yaw={record.get('delta_yaw_deg')}deg "
            f"abs={record.get('abs_yaw_delta_deg')}deg "
            f"fwd={record.get('forward_drift_m')}m "
            f"lat={record.get('lateral_drift_m')}m "
            f"front={record.get('front_p10_before')}->{record.get('front_p10_after')}m "
            f"zero={'ok' if record.get('base_zero_ok') else 'bad'} "
            f"ok={record.get('candidate_ok')} "
            f"elapsed={record.get('elapsed_s')}s"
        )

    def run_save_map_only(self):
        self.wait_policy_ready(require_cmd_subscriber=False)
        pre = self.precheck()
        zero_before = self.policy_observation_zero_status()
        odom_before = self.latest_odom
        prefix = f"{self.args.map_prefix}_save_only"
        save_result = self.save_map(prefix)
        post = self.precheck()
        zero_after = self.policy_observation_zero_status()
        result = {
            "mode": "save-map-only",
            "executed": False,
            "published_input_cmd_vel": False,
            "odom_before": odom_snapshot(odom_before),
            "odom_after": odom_snapshot(self.latest_odom),
            "odom_delta": odom_delta(odom_before, self.latest_odom),
            "base_zero_before": zero_before,
            "base_zero_after": zero_after,
            "base_zero_ok": zero_after.get("base_zero_ok"),
            "precheck": pre,
            "postcheck": post,
            "map_saved": bool(save_result.get("ok")),
            "map_save": save_result,
            "stop_reason": None if save_result.get("ok") else "map_save_failed",
        }
        print("SAVE_MAP_ONLY", json.dumps(result, ensure_ascii=False))
        return result

    def run_guarded_policy_dry_run(self):
        self.wait_policy_ready(require_cmd_subscriber=False)
        records = []
        action_counts = {}
        start = time.monotonic()
        sample_index = 0
        while rclpy.ok():
            elapsed = time.monotonic() - start
            if self.args.policy_samples > 0 and sample_index >= self.args.policy_samples:
                break
            if self.args.policy_samples <= 0 and elapsed >= self.args.policy_duration_s:
                break
            sample_start = time.monotonic()
            pre = self.precheck()
            state_ready_time = time.monotonic()
            decision = self.select_policy_action(pre)
            plan = self.policy_execution_plan(decision["selected_action"], pre)
            decision_done_time = time.monotonic()
            zero_start = time.monotonic()
            zero = self.policy_observation_zero_status()
            zero_end = time.monotonic()
            sample_end = time.monotonic()
            state_wait_s = max(0.0, state_ready_time - sample_start)
            decision_s = max(0.0, decision_done_time - state_ready_time)
            zero_check_s = max(0.0, zero_end - zero_start)
            total_s = max(0.0, sample_end - sample_start)
            accounted_s = state_wait_s + decision_s + zero_check_s
            selected = decision["selected_action"]
            action_counts[selected] = action_counts.get(selected, 0) + 1
            record = {
                "sample_index": sample_index + 1,
                "elapsed_s": round(float(elapsed), 3),
                "profile": decision["profile"],
                "front_min": decision["front_min"],
                "front_p10": decision["front_p10"],
                "selected_action": selected,
                "action_reason": decision["action_reason"],
                "threshold_band": decision["threshold_band"],
                "would_select_action": selected,
                "would_execute_action": plan["execution_action"],
                "arc_direction": plan.get("arc_direction"),
                "arc_direction_reason": plan.get("arc_direction_reason"),
                "arc_mode": plan.get("arc_mode"),
                "executed": False,
                "base_zero_ok": zero.get("base_zero_ok"),
                "base_zero": zero,
                "odom_before": pre.get("odom"),
                "odom_after": pre.get("odom"),
                "map_saved": False,
                "stop_reason": None if selected != "NOT_READY" else decision["action_reason"],
                "scan_sectors": pre.get("scan_sectors"),
                "precheck": pre,
                "timing_breakdown": {
                    "state_wait_time_s": round(state_wait_s, 3),
                    "decision_time_s": round(decision_s, 3),
                    "pre_action_zero_check_time_s": round(zero_check_s, 3),
                    "motion_execution_time_s": 0.0,
                    "stop_kick_time_s": 0.0,
                    "base_zero_wait_time_s": 0.0,
                    "adaptive_zero_extra_wait_s": 0.0,
                    "map_save_time_s": 0.0,
                    "postcheck_time_s": 0.0,
                    "loop_overhead_time_s": round(max(0.0, total_s - accounted_s), 3),
                    "step_total_time_s": round(total_s, 3),
                },
            }
            records.append(record)
            if self.args.console_mode == "compact":
                print(
                    f"DRY_SAMPLE {record['sample_index']} "
                    f"action={record['would_select_action']} exec={record['would_execute_action']} "
                    f"front_p10={record['front_p10']} zero={'ok' if record['base_zero_ok'] else 'bad'} "
                    f"elapsed={record['timing_breakdown']['step_total_time_s']}s"
                )
            else:
                print("GUARDED_POLICY_DRY_RUN_SAMPLE", json.dumps(record, ensure_ascii=False))
            sample_index += 1
            self.spin_for(self.args.policy_sample_period_s)

        return {
            "mode": "guarded-policy-dry-run",
            "profile": self.args.behavior_profile,
            "duration_s": round(float(time.monotonic() - start), 3),
            "sample_count": len(records),
            "action_counts": action_counts,
            "records": records,
        }

    def run_guarded_policy_step(self):
        self.wait_policy_ready(require_cmd_subscriber=True)
        step_start = time.monotonic()
        entry_zero_report = self.zero_hold(self.args.zero_hold_s)
        precheck_start = time.monotonic()
        pre = self.precheck()
        state_ready_time = time.monotonic()
        decision = self.select_policy_action(pre)
        plan = self.policy_execution_plan(decision["selected_action"], pre)
        decision_done_time = time.monotonic()
        odom_before_msg = self.latest_odom
        base_before_start = time.monotonic()
        base_before = self.base_zero_status(wait_s=0.0)
        base_before_end = time.monotonic()
        action_record = None
        map_save = None
        map_save_start = None
        map_save_end = None
        saved_maps = []
        capture_event = None
        executed = False
        stop_reason = None
        motion_start = time.monotonic()

        if not base_before.get("base_zero_ok"):
            stop_reason = "base_not_zero_before_action"
        elif decision["selected_action"] == "NOT_READY":
            stop_reason = decision["action_reason"]
        elif plan["motion"] == "hold":
            action_record = self.policy_hold_record(
                "guarded_policy_hold",
                decision,
                plan,
                odom_before_msg,
            )
            if not action_record.get("base_zero_ok"):
                stop_reason = "base_zero_failed"
        elif plan["motion"] == "forward":
            action_record = self.run_policy_forward_once(
                "guarded_policy_forward",
                plan["forward_target_m"],
                plan["front_gate_m"],
            )
            executed = not action_record.get("blocked", False)
            if action_record.get("blocked") or str(action_record.get("stop_reason", "")).startswith("front_blocked"):
                stop_reason = action_record.get("stop_reason")
            elif not self.policy_record_base_zero_ok(action_record, plan["motion"]):
                stop_reason = "base_zero_failed"
        elif plan["motion"] == "arc30":
            action_record = self.run_policy_arc_once(
                "guarded_policy_arc30",
                plan["front_gate_m"],
                plan["arc_direction"],
            )
            executed = any(not step.get("blocked", False) for step in action_record.get("records", []))
            if not action_record.get("target_band_reached"):
                stop_reason = action_record.get("stop_reason")
            for step in action_record.get("records", []):
                if not step.get("base_zero", {}).get("base_zero_ok"):
                    stop_reason = "base_zero_failed"
                    break
        elif plan["motion"] == "arc_fast":
            action_record = self.run_policy_arc_fast_once(
                "guarded_policy_arc_fast",
                plan["front_gate_m"],
                plan["arc_direction"],
            )
            executed = not action_record.get("blocked", False)
            action_stop = str(action_record.get("stop_reason", ""))
            if action_record.get("blocked") or action_stop.startswith("front_blocked"):
                stop_reason = action_record.get("stop_reason")
            elif not self.policy_record_base_zero_ok(action_record, plan["motion"]):
                stop_reason = "base_zero_failed"
            elif not action_record.get("front_min_safe", True):
                stop_reason = (
                    f"front_min_after {action_record.get('front_min_after')} < 0.20"
                )

        motion_end = time.monotonic()

        base_zero_ok = False
        if action_record is not None:
            base_zero_ok = self.policy_record_base_zero_ok(action_record, plan["motion"])
        else:
            base_zero_ok = base_before.get("base_zero_ok")

        if action_record is not None and not base_zero_ok:
            if stop_reason is None:
                stop_reason = "base_zero_failed"
            elif "base_zero_failed" not in str(stop_reason):
                stop_reason = f"{stop_reason}; base_zero_failed"

        map_save_status = None
        if base_zero_ok and decision["selected_action"] != "NOT_READY":
            map_save_start = time.monotonic()
            map_save, map_save_status = self.save_policy_step_map(
                f"{self.args.map_prefix}_policy_step",
                1,
                decision,
                plan,
                stop_reason,
                base_zero_ok,
                saved_maps,
            )
            map_save_end = time.monotonic()
            if stop_reason is None and map_save and map_save.get("ok") is False:
                stop_reason = "map_save_failed"

        if plan.get("capture_placeholder"):
            capture_event = {
                "type": "placeholder_capture",
                "capture_reason": decision["action_reason"],
                "odom": odom_snapshot(self.latest_odom),
                "front_p10": decision["front_p10"],
                "map_file": None if map_save is None else map_save.get("prefix"),
            }

        final_map_save = None
        final_map_save_start = time.monotonic()
        if base_zero_ok and decision["selected_action"] != "NOT_READY":
            final_map_save = self.save_policy_final_map(saved_maps)
            if stop_reason is None and not final_map_save.get("ok"):
                stop_reason = "final_map_save_failed"
        final_map_save_end = time.monotonic()

        postcheck_start = time.monotonic()
        post = self.precheck()
        postcheck_end = time.monotonic()
        step_end = time.monotonic()
        action_timing = self.action_timing_breakdown(action_record, plan["motion"])
        timing = self.policy_step_timing_breakdown(
            step_start,
            state_ready_time,
            decision_done_time,
            base_before_start,
            base_before_end,
            motion_start,
            motion_end,
            action_timing,
            map_save_start,
            map_save_end,
            postcheck_start,
            postcheck_end,
            step_end,
        )
        timing["entry_zero_wait"] = entry_zero_report
        timing["final_map_save_time_s"] = round(final_map_save_end - final_map_save_start, 3)
        result = {
            "mode": "guarded-policy-step",
            "profile": decision["profile"],
            "policy_arc_mode": self.args.policy_arc_mode,
            "front_min": decision["front_min"],
            "front_p10": decision["front_p10"],
            "selected_action": decision["selected_action"],
            "action_reason": decision["action_reason"],
            "threshold_band": decision["threshold_band"],
            "execution_action": plan["execution_action"],
            "arc_direction": plan.get("arc_direction"),
            "arc_direction_reason": plan.get("arc_direction_reason"),
            "arc_mode": plan.get("arc_mode"),
            "executed": executed,
            "base_zero_ok": base_zero_ok,
            "base_zero_before": base_before,
            "odom_before": odom_snapshot(odom_before_msg),
            "odom_after": odom_snapshot(self.latest_odom),
            "odom_delta": odom_delta(odom_before_msg, self.latest_odom),
            "map_saved": bool(
                (map_save and map_save.get("ok"))
                or (final_map_save and final_map_save.get("ok"))
            ),
            "map_save": map_save,
            "map_save_status": map_save_status,
            "final_map_save": final_map_save,
            "saved_maps": saved_maps,
            "capture_event": capture_event,
            "stop_reason": stop_reason,
            "precheck": pre,
            "postcheck": post,
            "action_record": action_record,
            "timing_breakdown": timing,
        }
        result["checkpoint_path"] = self.write_step_checkpoint(result)
        self.emit_policy_step_result(result)
        return result

    def run_guarded_policy_run(self):
        self.wait_policy_ready(require_cmd_subscriber=True)
        entry_zero_report = self.zero_hold(self.args.zero_hold_s)
        records = []
        saved_maps = []
        sequence_stop_reason = None
        sequence_start_msg = self.latest_odom
        start = time.monotonic()
        consecutive_fast_arc = 0
        cumulative_positive_forward_m = 0.0

        for step_index in range(1, self.args.policy_max_steps + 1):
            elapsed = time.monotonic() - start
            if elapsed >= self.args.policy_max_runtime_s:
                sequence_stop_reason = "max_runtime_reached_before_step"
                break

            step_start = time.monotonic()
            pre = self.precheck()
            state_ready_time = time.monotonic()
            decision = self.select_policy_action(pre)
            plan = self.policy_execution_plan(decision["selected_action"], pre)
            limit_stop_reason = None
            if (
                plan["motion"] == "arc_fast"
                and consecutive_fast_arc >= self.args.policy_max_consecutive_fast_arc
            ):
                limit_stop_reason = "max_consecutive_fast_arc_reached"
                plan = dict(plan)
                plan["execution_action"] = "HOLD_MAX_FAST_ARC"
                plan["motion"] = "hold"
                plan["arc_mode"] = "fast"
            decision_done_time = time.monotonic()
            odom_before_msg = self.latest_odom
            base_before_start = time.monotonic()
            base_before = self.base_zero_status(wait_s=0.0)
            base_before_end = time.monotonic()
            action_record = None
            map_save = None
            map_save_start = None
            map_save_end = None
            capture_event = None
            executed = False
            stop_reason = None
            continue_after_front_block = False
            motion_start = time.monotonic()

            if not base_before.get("base_zero_ok"):
                stop_reason = "base_not_zero_before_action"
            elif decision["selected_action"] == "NOT_READY":
                stop_reason = decision["action_reason"]
            elif plan["motion"] == "hold":
                action_record = self.policy_hold_record(
                    f"guarded_policy_run_{step_index:02d}_hold",
                    decision,
                    plan,
                    odom_before_msg,
                )
                if limit_stop_reason is not None:
                    stop_reason = limit_stop_reason
                if not action_record.get("base_zero_ok"):
                    stop_reason = "base_zero_failed"
            elif plan["motion"] == "forward":
                action_record = self.run_policy_forward_once(
                    f"guarded_policy_run_{step_index:02d}_forward",
                    plan["forward_target_m"],
                    plan["front_gate_m"],
                )
                executed = not action_record.get("blocked", False)
                action_stop = str(action_record.get("stop_reason", ""))
                if action_record.get("blocked") or action_stop.startswith("front_blocked"):
                    stop_reason = action_record.get("stop_reason")
                    continue_after_front_block = str(stop_reason).startswith("front_blocked")
                elif not self.policy_record_base_zero_ok(action_record, plan["motion"]):
                    stop_reason = "base_zero_failed"
            elif plan["motion"] == "arc30":
                action_record = self.run_policy_arc_once(
                    f"guarded_policy_run_{step_index:02d}_arc30",
                    plan["front_gate_m"],
                    plan["arc_direction"],
                )
                executed = any(
                    not step.get("blocked", False)
                    for step in action_record.get("records", [])
                )
                if not action_record.get("target_band_reached"):
                    stop_reason = action_record.get("stop_reason")
                    continue_after_front_block = "front_blocked" in str(stop_reason)
                for step in action_record.get("records", []):
                    if not step.get("base_zero", {}).get("base_zero_ok"):
                        stop_reason = "base_zero_failed"
                        continue_after_front_block = False
                        break
            elif plan["motion"] == "arc_fast":
                action_record = self.run_policy_arc_fast_once(
                    f"guarded_policy_run_{step_index:02d}_arc_fast",
                    plan["front_gate_m"],
                    plan["arc_direction"],
                )
                executed = not action_record.get("blocked", False)
                action_stop = str(action_record.get("stop_reason", ""))
                if action_record.get("blocked") or action_stop.startswith("front_blocked"):
                    stop_reason = action_record.get("stop_reason")
                    continue_after_front_block = str(stop_reason).startswith("front_blocked")
                elif not self.policy_record_base_zero_ok(action_record, plan["motion"]):
                    stop_reason = "base_zero_failed"
                elif not action_record.get("front_min_safe", True):
                    stop_reason = (
                        f"front_min_after {action_record.get('front_min_after')} < 0.20"
                    )
                    continue_after_front_block = False

            motion_end = time.monotonic()

            if plan["motion"] == "arc_fast" and executed and stop_reason is None:
                consecutive_fast_arc += 1
            elif plan["motion"] != "arc_fast":
                consecutive_fast_arc = 0

            if action_record is not None:
                base_zero_ok = self.policy_record_base_zero_ok(action_record, plan["motion"])
            else:
                base_zero_ok = base_before.get("base_zero_ok")

            if action_record is not None and not base_zero_ok:
                if stop_reason is None:
                    stop_reason = "base_zero_failed"
                elif "base_zero_failed" not in str(stop_reason):
                    stop_reason = f"{stop_reason}; base_zero_failed"
                continue_after_front_block = False

            map_save_status = None
            if base_zero_ok and decision["selected_action"] != "NOT_READY":
                map_save_start = time.monotonic()
                map_save, map_save_status = self.save_policy_step_map(
                    self.policy_step_save_prefix(step_index),
                    step_index,
                    decision,
                    plan,
                    stop_reason,
                    base_zero_ok,
                    saved_maps,
                )
                map_save_end = time.monotonic()
                if stop_reason is None and map_save and map_save.get("ok") is False:
                    stop_reason = "map_save_failed"
                if map_save and map_save.get("ok") is False:
                    continue_after_front_block = False

            if plan.get("capture_placeholder"):
                capture_event = {
                    "type": "placeholder_capture",
                    "capture_reason": decision["action_reason"],
                    "odom": odom_snapshot(self.latest_odom),
                    "front_p10": decision["front_p10"],
                    "map_file": None if map_save is None else map_save.get("prefix"),
                }

            postcheck_start = time.monotonic()
            post = self.precheck()
            postcheck_end = time.monotonic()
            step_end = time.monotonic()
            action_timing = self.action_timing_breakdown(action_record, plan["motion"])
            timing = self.policy_step_timing_breakdown(
                step_start,
                state_ready_time,
                decision_done_time,
                base_before_start,
                base_before_end,
                motion_start,
                motion_end,
                action_timing,
                map_save_start,
                map_save_end,
                postcheck_start,
                postcheck_end,
                step_end,
            )
            step_odom_delta = odom_delta(odom_before_msg, self.latest_odom)
            step_forward_delta_m = float(step_odom_delta.get("forward_delta_m") or 0.0)
            step_positive_forward_m = max(0.0, step_forward_delta_m)
            cumulative_positive_forward_m += step_positive_forward_m
            sequence_odom_delta = odom_delta(sequence_start_msg, self.latest_odom)
            total_forward_limit_stop_reason = None
            if cumulative_positive_forward_m >= self.args.policy_max_total_forward_m:
                total_forward_limit_stop_reason = (
                    "max_total_forward_reached: "
                    f"{cumulative_positive_forward_m:.3f} >= "
                    f"{self.args.policy_max_total_forward_m:.3f}"
                )
                continue_after_front_block = False
                if stop_reason is None:
                    stop_reason = total_forward_limit_stop_reason
            record = {
                "step_index": step_index,
                "elapsed_s": round(float(time.monotonic() - start), 3),
                "profile": decision["profile"],
                "front_min": decision["front_min"],
                "front_p10": decision["front_p10"],
                "selected_action": decision["selected_action"],
                "action_reason": decision["action_reason"],
                "threshold_band": decision["threshold_band"],
                "execution_action": plan["execution_action"],
                "arc_direction": plan.get("arc_direction"),
                "arc_direction_reason": plan.get("arc_direction_reason"),
                "arc_mode": plan.get("arc_mode"),
                "consecutive_fast_arc": consecutive_fast_arc,
                "executed": executed,
                "base_zero_ok": base_zero_ok,
                "base_zero_before": base_before,
                "odom_before": odom_snapshot(odom_before_msg),
                "odom_after": odom_snapshot(self.latest_odom),
                "odom_delta": step_odom_delta,
                "sequence_odom_delta": sequence_odom_delta,
                "step_positive_forward_m": round(float(step_positive_forward_m), 4),
                "cumulative_positive_forward_m": round(
                    float(cumulative_positive_forward_m), 4
                ),
                "policy_max_total_forward_m": self.args.policy_max_total_forward_m,
                "sequence_limit_stop_reason": total_forward_limit_stop_reason,
                "map_saved": bool(map_save and map_save.get("ok")),
                "map_save": map_save,
                "map_save_status": map_save_status,
                "capture_event": capture_event,
                "stop_reason": stop_reason,
                "sequence_continue_after_front_block": bool(continue_after_front_block),
                "precheck": pre,
                "postcheck": post,
                "action_record": action_record,
                "timing_breakdown": timing,
            }
            record["checkpoint_path"] = self.write_step_checkpoint(record)
            records.append(record)
            self.emit_policy_run_step(record)

            if total_forward_limit_stop_reason is not None:
                sequence_stop_reason = total_forward_limit_stop_reason
                break
            if stop_reason is not None and not continue_after_front_block:
                sequence_stop_reason = stop_reason
                break
            if plan["motion"] == "hold":
                sequence_stop_reason = f"hold_action_{decision['selected_action']}"
                break
            if time.monotonic() - start >= self.args.policy_max_runtime_s:
                sequence_stop_reason = "max_runtime_reached_after_step"
                break

        if sequence_stop_reason is None:
            if len(records) >= self.args.policy_max_steps:
                sequence_stop_reason = "max_steps_reached"
            else:
                sequence_stop_reason = "completed"

        post = self.precheck()
        base_after = self.base_zero_status(wait_s=0.0)
        if base_after.get("base_zero_ok"):
            final_map_save = self.save_policy_final_map(saved_maps)
            if not final_map_save.get("ok") and sequence_stop_reason is None:
                sequence_stop_reason = "final_map_save_failed"
        else:
            final_map_save = {
                "ok": False,
                "status": "skipped_base_not_zero",
                "reason": "run_end",
                "save_policy": self.args.save_policy,
                "error": "final map skipped because base_zero_ok is false",
            }
        result = {
            "mode": "guarded-policy-run",
            "profile": self.args.behavior_profile,
            "policy_arc_mode": self.args.policy_arc_mode,
            "policy_max_consecutive_fast_arc": self.args.policy_max_consecutive_fast_arc,
            "policy_max_total_forward_m": self.args.policy_max_total_forward_m,
            "max_steps": self.args.policy_max_steps,
            "max_runtime_s": self.args.policy_max_runtime_s,
            "step_count": len(records),
            "executed_count": sum(1 for record in records if record.get("executed")),
            "sequence_stop_reason": sequence_stop_reason,
            "base_zero_ok": base_after.get("base_zero_ok"),
            "base_zero_after": base_after,
            "entry_zero_wait": entry_zero_report,
            "final_map_saved": bool(final_map_save.get("ok")),
            "final_map_save": final_map_save,
            "odom_start": odom_snapshot(sequence_start_msg),
            "odom_end": odom_snapshot(self.latest_odom),
            "odom_delta": odom_delta(sequence_start_msg, self.latest_odom),
            "cumulative_positive_forward_m": round(float(cumulative_positive_forward_m), 4),
            "postcheck": post,
            "saved_maps": saved_maps,
            "records": records,
        }
        self.emit_policy_run_result(result)
        return result

    def run_spatial_micro(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        records = []
        saved_maps = []
        sequence_stop_reason = None

        initial_save = self.save_map(f"{self.args.map_prefix}_00_initial")
        saved_maps.append(initial_save)
        print("SPATIAL_MICRO_SAVE_MAP", json.dumps(initial_save, ensure_ascii=False))
        if not initial_save.get("ok"):
            sequence_stop_reason = "initial_map_save_failed"
            return {
                "mode": "spatial-micro-run",
                "sequence_stop_reason": sequence_stop_reason,
                "records": records,
                "saved_maps": saved_maps,
            }

        sequence_start_msg = self.latest_odom

        first_forward = self.staged_forward_record(
            name="spatial_forward_1_0p15",
            target_m=self.args.spatial_forward1_target_m,
            fast_speed=self.args.forward_fast_speed,
            mid_speed=self.args.forward_mid_speed,
            slow_speed=self.args.forward_slow_speed,
            mid_zone_m=min(0.12, self.args.spatial_forward1_target_m),
            slow_zone_m=min(0.07, max(0.04, self.args.spatial_forward1_target_m - 0.02)),
            brake_margin_m=self.args.forward_brake_margin_m,
            timeout_s=self.args.forward_timeout_s,
        )
        first_forward["sequence_index"] = 1
        records.append(first_forward)
        print("SPATIAL_MICRO_SEGMENT", json.dumps(first_forward, ensure_ascii=False))
        save_result = self.save_map(f"{self.args.map_prefix}_01_forward_0p15")
        first_forward["map_save"] = save_result
        saved_maps.append(save_result)
        print("SPATIAL_MICRO_SAVE_MAP", json.dumps(save_result, ensure_ascii=False))
        if (
            first_forward.get("blocked")
            or str(first_forward.get("stop_reason", "")).startswith("front_blocked")
            or not first_forward.get("base_zero", {}).get("base_zero_ok")
        ):
            sequence_stop_reason = f"{first_forward['name']}: {first_forward.get('stop_reason')}"

        if sequence_stop_reason is None:
            arc = self.arc_yaw_closed_direction_record(
                self.args.spatial_arc_direction,
                "spatial_arc_yaw_closed",
            )
            arc["sequence_index"] = 2
            records.append(arc)
            print("SPATIAL_MICRO_SEGMENT", json.dumps(arc, ensure_ascii=False))
            save_result = self.save_map(f"{self.args.map_prefix}_02_arc_{self.args.spatial_arc_direction}")
            arc["map_save"] = save_result
            saved_maps.append(save_result)
            print("SPATIAL_MICRO_SAVE_MAP", json.dumps(save_result, ensure_ascii=False))
            if not arc.get("target_band_reached"):
                sequence_stop_reason = f"arc_{self.args.spatial_arc_direction}: {arc.get('stop_reason')}"
            for step in arc.get("records", []):
                if not step.get("base_zero", {}).get("base_zero_ok"):
                    sequence_stop_reason = f"{step['name']}: base_zero_failed"
                    break

        if sequence_stop_reason is None:
            second_forward = self.staged_forward_record(
                name="spatial_forward_2_0p10",
                target_m=self.args.spatial_forward2_target_m,
                fast_speed=self.args.forward_fast_speed,
                mid_speed=self.args.forward_mid_speed,
                slow_speed=self.args.forward_slow_speed,
                mid_zone_m=min(0.08, self.args.spatial_forward2_target_m),
                slow_zone_m=min(0.05, max(0.04, self.args.spatial_forward2_target_m - 0.02)),
                brake_margin_m=self.args.forward_brake_margin_m,
                timeout_s=self.args.forward_timeout_s,
            )
            second_forward["sequence_index"] = 3
            records.append(second_forward)
            print("SPATIAL_MICRO_SEGMENT", json.dumps(second_forward, ensure_ascii=False))
            save_result = self.save_map(f"{self.args.map_prefix}_03_forward_0p10")
            second_forward["map_save"] = save_result
            saved_maps.append(save_result)
            print("SPATIAL_MICRO_SAVE_MAP", json.dumps(save_result, ensure_ascii=False))
            if (
                second_forward.get("blocked")
                or str(second_forward.get("stop_reason", "")).startswith("front_blocked")
                or not second_forward.get("base_zero", {}).get("base_zero_ok")
            ):
                sequence_stop_reason = f"{second_forward['name']}: {second_forward.get('stop_reason')}"

        sequence_delta = odom_delta(sequence_start_msg, self.latest_odom)
        return {
            "mode": "spatial-micro-run",
            "sequence": "F0.15 -> L30_arc -> F0.10",
            "sequence_stop_reason": sequence_stop_reason,
            "front_p10_min_m": self.args.forward_front_p10_min_m,
            "sequence_odom_delta": sequence_delta,
            "records": records,
            "saved_maps": saved_maps,
        }

    def run_spatial_s(self):
        self.wait_ready()
        self.zero_hold(self.args.zero_hold_s)
        records = []
        saved_maps = []
        sequence_stop_reason = None

        initial_save = self.save_map(f"{self.args.map_prefix}_00_initial")
        saved_maps.append(initial_save)
        print("SPATIAL_S_SAVE_MAP", json.dumps(initial_save, ensure_ascii=False))
        if not initial_save.get("ok"):
            return {
                "mode": "spatial-s-run",
                "sequence_stop_reason": "initial_map_save_failed",
                "records": records,
                "saved_maps": saved_maps,
            }

        sequence_start_msg = self.latest_odom
        sequence = [
            ("forward", "spatial_s_01_forward_0p15", 0.15, "01_forward_0p15"),
            ("arc", "spatial_s_02_arc_left", "left", "02_arc_left"),
            ("forward", "spatial_s_03_forward_0p10", 0.10, "03_forward_0p10_after_left"),
            ("arc", "spatial_s_04_arc_right", "right", "04_arc_right"),
            ("forward", "spatial_s_05_forward_0p10", 0.10, "05_forward_0p10_after_right"),
        ]

        for index, item in enumerate(sequence, start=1):
            kind = item[0]
            if kind == "forward":
                _, name, target_m, save_suffix = item
                record = self.staged_forward_record(
                    name=name,
                    target_m=target_m,
                    fast_speed=self.args.forward_fast_speed,
                    mid_speed=self.args.forward_mid_speed,
                    slow_speed=self.args.forward_slow_speed,
                    mid_zone_m=min(0.12 if target_m >= 0.15 else 0.08, target_m),
                    slow_zone_m=min(0.07 if target_m >= 0.15 else 0.05, max(0.04, target_m - 0.02)),
                    brake_margin_m=self.args.forward_brake_margin_m,
                    timeout_s=self.args.forward_timeout_s,
                )
            else:
                _, name, direction, save_suffix = item
                record = self.arc_yaw_closed_direction_record(direction, name)

            record["sequence_index"] = index
            records.append(record)
            print("SPATIAL_S_SEGMENT", json.dumps(record, ensure_ascii=False))

            save_result = self.save_map(f"{self.args.map_prefix}_{save_suffix}")
            record["map_save"] = save_result
            saved_maps.append(save_result)
            print("SPATIAL_S_SAVE_MAP", json.dumps(save_result, ensure_ascii=False))

            if kind == "forward":
                failed = (
                    record.get("blocked")
                    or str(record.get("stop_reason", "")).startswith("front_blocked")
                    or not record.get("base_zero", {}).get("base_zero_ok")
                )
                if failed:
                    sequence_stop_reason = f"{record.get('name')}: {record.get('stop_reason')}"
                    break
            else:
                if not record.get("target_band_reached"):
                    sequence_stop_reason = f"{record.get('direction')}_arc: {record.get('stop_reason')}"
                    break
                for step in record.get("records", []):
                    if not step.get("base_zero", {}).get("base_zero_ok"):
                        sequence_stop_reason = f"{step.get('name')}: base_zero_failed"
                        break
                if sequence_stop_reason is not None:
                    break

        sequence_delta = odom_delta(sequence_start_msg, self.latest_odom)
        return {
            "mode": "spatial-s-run",
            "sequence": "F0.15 -> L30_arc -> F0.10 -> R30_arc -> F0.10",
            "sequence_stop_reason": sequence_stop_reason,
            "front_p10_min_m": self.args.forward_front_p10_min_m,
            "sequence_odom_delta": sequence_delta,
            "records": records,
            "saved_maps": saved_maps,
        }

    def turn_threshold_record(
        self,
        name: str,
        angular: float,
        duration: float,
        linear: float = 0.0,
        kind: str = "turn_threshold",
        front_check: bool = False,
        front_gate_m=None,
    ):
        segment_start = time.monotonic()
        precheck_start = segment_start
        start = odom_snapshot(self.latest_odom)
        start_msg = self.latest_odom
        pre = self.precheck()
        precheck_end = time.monotonic()
        command_start = time.monotonic()
        command_end = command_start
        period = 1.0 / self.args.rate
        time_to_yaw_motion = None
        first_yaw_motion = None
        max_odom_wz = 0.0
        max_robot_wz = 0.0
        max_abs_yaw_during_command = 0.0
        max_abs_forward_drift = 0.0
        max_abs_lateral_drift = 0.0
        samples = []

        if front_check:
            block_reason = self.front_block_reason(front_gate_m)
            if block_reason is not None:
                zero_report = self.zero_hold(self.args.zero_hold_s)
                zero = zero_report["base_zero"]
                postcheck_start = time.monotonic()
                post = self.precheck()
                postcheck_end = time.monotonic()
                segment_end = time.monotonic()
                end_msg = self.latest_odom
                actual = odom_delta(start_msg, end_msg)
                telemetry = self.telemetry_summary(command_start, command_start, segment_end)
                return {
                    "name": name,
                    "kind": kind,
                    "blocked": True,
                    "stop_reason": "front_blocked: " + block_reason,
                    "command": {
                        "linear": round(float(linear), 4),
                        "angular": round(float(angular), 4),
                        "duration_s": round(float(duration), 3),
                    },
                    "precheck": pre,
                    "postcheck": post,
                    "odom_start": start,
                    "odom_end": odom_snapshot(end_msg),
                    "delta_yaw_deg": actual["delta_yaw_deg"],
                    "forward_drift_m": actual["forward_delta_m"],
                    "lateral_drift_m": actual["lateral_delta_m"],
                    "telemetry": telemetry,
                    "base_zero": zero,
                    "timing_breakdown": self.segment_timing_breakdown(
                        segment_start,
                        precheck_start,
                        precheck_end,
                        command_start,
                        command_start,
                        zero_report,
                        postcheck_start,
                        postcheck_end,
                        segment_end,
                    ),
                }

        while rclpy.ok() and time.monotonic() - command_start < duration:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.poll_pending_save()
            now = time.monotonic()
            elapsed = now - command_start
            current_delta = odom_delta_values(start_msg, self.latest_odom)
            yaw_delta_deg = math.degrees(current_delta["delta_yaw_rad"])
            forward_drift = current_delta["forward_delta_m"]
            lateral_drift = current_delta["lateral_delta_m"]
            odom_wz = 0.0
            if self.latest_odom is not None:
                odom_wz = float(self.latest_odom.twist.twist.angular.z)
            robot_wz = 0.0
            if self.latest_robot_vel is not None:
                robot_wz = float(self.latest_robot_vel.z)

            max_odom_wz = max(max_odom_wz, abs(odom_wz))
            max_robot_wz = max(max_robot_wz, abs(robot_wz))
            max_abs_yaw_during_command = max(max_abs_yaw_during_command, abs(yaw_delta_deg))
            max_abs_forward_drift = max(max_abs_forward_drift, abs(forward_drift))
            max_abs_lateral_drift = max(max_abs_lateral_drift, abs(lateral_drift))

            if time_to_yaw_motion is None and (
                abs(yaw_delta_deg) >= self.args.turn_threshold_detect_deg
                or abs(odom_wz) >= self.args.turn_threshold_detect_wz
            ):
                time_to_yaw_motion = elapsed
                first_yaw_motion = {
                    "time_s": round(float(elapsed), 3),
                    "yaw_delta_deg": round(float(yaw_delta_deg), 2),
                    "odom_wz": round(float(odom_wz), 4),
                    "robot_wz": round(float(robot_wz), 4),
                }

            samples.append(
                {
                    "time_s": round(float(elapsed), 3),
                    "yaw_delta_deg": round(float(yaw_delta_deg), 2),
                    "odom_wz": round(float(odom_wz), 4),
                    "robot_wz": round(float(robot_wz), 4),
                    "forward_drift_m": round(float(forward_drift), 4),
                    "lateral_drift_m": round(float(lateral_drift), 4),
                }
            )
            self.publish_drive_once(linear, angular)
            time.sleep(period)

        command_end = time.monotonic()
        command_end_msg = self.latest_odom
        command_end_odom = odom_snapshot(command_end_msg)
        command_end_delta = odom_delta(start_msg, command_end_msg)
        zero_report = self.zero_hold(self.args.zero_hold_s)
        zero = zero_report["base_zero"]
        postcheck_start = time.monotonic()
        post = self.precheck()
        postcheck_end = time.monotonic()
        segment_end = time.monotonic()
        end_msg = self.latest_odom
        end = odom_snapshot(end_msg)
        actual = odom_delta(start_msg, end_msg)
        expected = math.degrees(angular * duration)
        actual_yaw = float(actual["delta_yaw_deg"])
        telemetry = self.telemetry_summary(command_start, command_end, segment_end)
        return {
            "name": name,
            "kind": kind,
            "command": {
                "linear": round(float(linear), 4),
                "angular": round(float(angular), 4),
                "duration_s": round(float(duration), 3),
            },
            "blocked": False,
            "stop_reason": "duration_elapsed",
            "expected_yaw_delta_deg": round(float(expected), 2),
            "signed_yaw_error_deg": round(float(actual_yaw - expected), 2),
            "abs_yaw_error_deg": round(float(abs(actual_yaw) - abs(expected)), 2),
            "turned_detected": abs(actual_yaw) >= self.args.turn_threshold_detect_deg,
            "overshoot_detected": (
                abs(actual_yaw) > abs(expected) + self.args.turn_threshold_detect_deg
            ),
            "time_to_yaw_motion_s": (
                None if time_to_yaw_motion is None else round(float(time_to_yaw_motion), 3)
            ),
            "first_yaw_motion": first_yaw_motion,
            "max_odom_wz": round(float(max_odom_wz), 4),
            "max_robot_wz": round(float(max_robot_wz), 4),
            "max_abs_yaw_during_command_deg": round(float(max_abs_yaw_during_command), 2),
            "max_abs_forward_drift_during_command_m": round(float(max_abs_forward_drift), 4),
            "max_abs_lateral_drift_during_command_m": round(float(max_abs_lateral_drift), 4),
            "odom_at_cmd_end": command_end_odom,
            "odom_delta_at_cmd_end": command_end_delta,
            "yaw_delta_at_cmd_end_deg": command_end_delta["delta_yaw_deg"],
            "yaw_delta_after_settle_deg": actual["delta_yaw_deg"],
            "settle_extra_yaw_deg": round(
                float(actual["delta_yaw_deg"] - command_end_delta["delta_yaw_deg"]), 2
            ),
            "forward_delta_at_cmd_end_m": command_end_delta["forward_delta_m"],
            "lateral_delta_at_cmd_end_m": command_end_delta["lateral_delta_m"],
            "settle_extra_forward_m": round(
                float(actual["forward_delta_m"] - command_end_delta["forward_delta_m"]), 4
            ),
            "settle_extra_lateral_m": round(
                float(actual["lateral_delta_m"] - command_end_delta["lateral_delta_m"]), 4
            ),
            "forward_drift_m": actual["forward_delta_m"],
            "lateral_drift_m": actual["lateral_delta_m"],
            "precheck": pre,
            "postcheck": post,
            "odom_start": start,
            "odom_end": end,
            "delta_x": actual["delta_x"],
            "delta_y": actual["delta_y"],
            "delta_yaw_rad": actual["delta_yaw_rad"],
            "delta_yaw_deg": actual["delta_yaw_deg"],
            "map_width": self.latest_map.width if self.latest_map else None,
            "map_height": self.latest_map.height if self.latest_map else None,
            "map": map_snapshot(self.latest_map),
            "samples": samples,
            "telemetry": telemetry,
            "diag_cmd_wz_max": telemetry["diag_cmd_wz"]["max_abs"],
            "diag_serial_wz_max": telemetry["diag_serial_wz"]["max_abs"],
            "diag_feedback_wz_max": telemetry["diag_feedback_wz"]["max_abs"],
            "base_zero": zero,
            "timing_breakdown": self.segment_timing_breakdown(
                segment_start,
                precheck_start,
                precheck_end,
                command_start,
                command_end,
                zero_report,
                postcheck_start,
                postcheck_end,
                segment_end,
            ),
        }

    def run_odom_micro(self):
        self.wait_ready()
        sequence = [
            ("odom_forward_1", "forward", self.args.odom_forward_m),
            ("odom_turn_left", "turn", self.args.odom_turn_deg),
            ("odom_forward_2", "forward", self.args.odom_forward_m),
            ("odom_turn_right", "turn", -self.args.odom_turn_deg),
            ("odom_forward_3", "forward", self.args.odom_forward_m),
        ]
        records = []
        saved_maps = []
        self.zero_hold(self.args.zero_hold_s)
        sequence_stop_reason = None
        for index, (name, kind, target_value) in enumerate(sequence, start=1):
            if kind == "forward":
                record = self.move_forward_by_odom(
                    name,
                    target_value,
                    self.args.forward_linear,
                    self.args.odom_forward_timeout_s,
                )
            else:
                record = self.turn_by_odom(
                    name,
                    target_value,
                    self.args.turn_angular,
                    self.args.odom_turn_timeout_s,
                )

            should_stop = not record["target_reached"]
            should_save = (
                index % self.args.save_every_segments == 0
                or index == len(sequence)
                or should_stop
            )
            if should_save:
                prefix = f"{self.args.map_prefix}_{index:02d}_{name}"
                save_result = self.save_map(prefix)
                record["map_save"] = save_result
                saved_maps.append(save_result)
                print("ODOM_MICRO_SAVE_MAP", json.dumps(save_result, ensure_ascii=False))

            records.append(record)
            print("ODOM_MICRO_SEGMENT", json.dumps(record, ensure_ascii=False))

            if should_stop:
                sequence_stop_reason = f"{name}: {record['stop_reason']}"
                break

        return {
            "mode": "odom-micro-run",
            "control": "odom_target",
            "sequence_stop_reason": sequence_stop_reason,
            "records": records,
            "saved_maps": saved_maps,
        }

    def run_micro(self):
        self.wait_ready()
        sequence = [
            ("forward_1", "forward", self.args.forward_linear, 0.0, 0.8),
            ("turn_left", "turn", 0.0, self.args.turn_angular, 0.6),
            ("forward_2", "forward", self.args.forward_linear, 0.0, 0.8),
            ("turn_right", "turn", 0.0, -self.args.turn_angular, 0.6),
            ("forward_3", "forward", self.args.forward_linear, 0.0, 0.8),
        ]
        records = []
        saved_maps = []
        self.zero_hold(self.args.zero_hold_s)
        for index, (name, kind, linear, angular, duration) in enumerate(sequence, start=1):
            pre = self.precheck()
            blocked = False
            reason = None
            front_p10 = pre.get("front_p10_range_m")
            if kind == "forward" and (
                front_p10 is None or float(front_p10) < self.args.forward_front_p10_min_m
            ):
                blocked = True
                reason = f"front_p10 {front_p10} < {self.args.forward_front_p10_min_m:.2f}"
            record = self.segment_record(name, kind, linear, angular, duration, blocked, reason)
            records.append(record)
            print("MICRO_MAPPING_SEGMENT", json.dumps(record, ensure_ascii=False))

            should_save = index % self.args.save_every_segments == 0 or index == len(sequence)
            if should_save:
                prefix = f"{self.args.map_prefix}_{index:02d}"
                save_result = self.save_map(prefix)
                saved_maps.append(save_result)
                print("MICRO_MAPPING_SAVE_MAP", json.dumps(save_result, ensure_ascii=False))
        return {"mode": "micro-run", "records": records, "saved_maps": saved_maps}


def write_report(path: str, payload):
    report_path = Path(path)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Supervised guarded yaw calibration and fixed-primitive micro mapping."
    )
    parser.add_argument(
        "--mode",
        choices=(
            "yaw-calibration",
            "yaw-amplified",
            "turn-threshold",
            "turn-duration-sweep",
            "arc-turn-threshold",
            "arc-step-repeat",
            "arc-fast-calib",
            "arc-yaw-closed",
            "spatial-micro-run",
            "spatial-s-run",
            "save-map-only",
            "guarded-policy-dry-run",
            "guarded-policy-step",
            "guarded-policy-run",
            "forward-threshold",
            "forward-staged",
            "odom-micro-run",
            "micro-run",
        ),
        required=True,
    )
    parser.add_argument("--control-mode", choices=("staged",), default="staged")
    parser.add_argument("--input-cmd-topic", default="/input_cmd_vel")
    parser.add_argument("--guarded-cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--map-metadata-topic", default="/map_metadata")
    parser.add_argument("--status-topic", default="/safety/front_obstacle")
    parser.add_argument("--robot-vel-topic", default="/robot_vel")
    parser.add_argument("--save-map-service", default="/slam_toolbox/save_map")
    parser.add_argument("--save-map-retries", type=int, default=2)
    parser.add_argument("--save-map-retry-delay-s", type=float, default=2.0)
    parser.add_argument("--forward-linear", type=float, default=0.28)
    parser.add_argument("--turn-angular", type=float, default=0.50)
    parser.add_argument("--turn-threshold-set", choices=("basic", "strong"), default="basic")
    parser.add_argument("--turn-threshold-detect-deg", type=float, default=1.0)
    parser.add_argument("--turn-threshold-detect-wz", type=float, default=0.03)
    parser.add_argument("--arc-turn-linear", type=float, default=0.10)
    parser.add_argument("--arc-step-linear", type=float, default=0.10)
    parser.add_argument("--arc-step-angular", type=float, default=0.50)
    parser.add_argument("--arc-step-duration-s", type=float, default=1.0)
    parser.add_argument("--arc-step-repeats", type=int, default=3)
    parser.add_argument("--arc-yaw-target-deg", type=float, default=30.0)
    parser.add_argument("--arc-yaw-tolerance-deg", type=float, default=6.0)
    parser.add_argument("--arc-yaw-overshoot-epsilon-deg", type=float, default=1.5)
    parser.add_argument("--arc-max-steps", type=int, default=4)
    parser.add_argument("--arc-yaw-direction", choices=("left", "right", "both"), default="both")
    parser.add_argument("--arc-fast-profile", choices=("g0", "g1"), default="g0")
    parser.add_argument("--arc-fast-direction", choices=("left", "right", "both"), default="both")
    parser.add_argument("--arc-fast-front-p10-min-m", type=float, default=0.40)
    parser.add_argument("--spatial-forward1-target-m", type=float, default=0.15)
    parser.add_argument("--spatial-forward2-target-m", type=float, default=0.10)
    parser.add_argument("--spatial-arc-direction", choices=("left", "right"), default="left")
    parser.add_argument(
        "--behavior-profile",
        choices=("mapping_safe_mode", "interaction_mode"),
        default="mapping_safe_mode",
    )
    parser.add_argument("--policy-duration-s", type=float, default=30.0)
    parser.add_argument("--policy-samples", type=int, default=0)
    parser.add_argument("--policy-sample-period-s", type=float, default=1.0)
    parser.add_argument("--policy-max-steps", type=int, default=3)
    parser.add_argument("--policy-max-runtime-s", type=float, default=120.0)
    parser.add_argument("--policy-max-total-forward-m", type=float, default=1.0)
    parser.add_argument("--policy-arc-direction", choices=("left", "right", "auto"), default="auto")
    parser.add_argument("--policy-arc-mode", choices=("precise", "fast"), default="precise")
    parser.add_argument("--policy-max-consecutive-fast-arc", type=int, default=2)
    parser.add_argument("--policy-arc-fast-linear", type=float, default=0.12)
    parser.add_argument("--policy-arc-fast-angular", type=float, default=0.80)
    parser.add_argument("--policy-arc-fast-duration-s", type=float, default=1.0)
    parser.add_argument("--policy-close-action", choices=("arc30", "forward"), default="arc30")
    parser.add_argument("--policy-mid-action", choices=("arc30", "forward"), default="arc30")
    parser.add_argument("--policy-normal-action", choices=("forward", "arc30"), default="forward")
    parser.add_argument("--yaw-angular", type=float, default=0.80)
    parser.add_argument("--yaw-duration-s", type=float, default=2.0)
    parser.add_argument("--odom-forward-m", type=float, default=0.20)
    parser.add_argument("--odom-turn-deg", type=float, default=15.0)
    parser.add_argument("--odom-forward-timeout-s", type=float, default=3.0)
    parser.add_argument("--odom-turn-timeout-s", type=float, default=4.0)
    parser.add_argument("--threshold-start-speed", type=float, default=0.10)
    parser.add_argument("--threshold-max-speed", type=float, default=0.30)
    parser.add_argument("--threshold-step-speed", type=float, default=0.10)
    parser.add_argument("--threshold-pulse-s", type=float, default=1.0)
    parser.add_argument("--threshold-detect-m", type=float, default=0.01)
    parser.add_argument("--threshold-detect-vx", type=float, default=0.02)
    parser.add_argument("--threshold-continue-after-first-motion", action="store_true")
    parser.add_argument("--staged-test-set", choices=("single", "abc"), default="single")
    parser.add_argument("--forward-target-m", type=float, default=0.20)
    parser.add_argument("--forward-fast-speed", type=float, default=0.20)
    parser.add_argument("--forward-mid-speed", type=float, default=0.15)
    parser.add_argument("--forward-slow-speed", type=float, default=0.10)
    parser.add_argument("--forward-mid-zone-m", type=float, default=0.12)
    parser.add_argument("--forward-slow-zone-m", type=float, default=0.06)
    parser.add_argument("--forward-brake-margin-m", type=float, default=0.03)
    parser.add_argument("--forward-brake-coef-s", type=float, default=1.0)
    parser.add_argument("--forward-static-brake-margin-m", type=float, default=0.02)
    parser.add_argument("--forward-timeout-s", type=float, default=5.0)
    parser.add_argument("--forward-front-p10-min-m", type=float, default=1.60)
    parser.add_argument("--zero-hold-s", type=float, default=DEFAULT_ZERO_HOLD_S)
    parser.add_argument("--zero-min-hold-s", type=float, default=0.8)
    parser.add_argument("--zero-poll-s", type=float, default=0.1)
    parser.add_argument("--zero-confirm-samples", type=int, default=3)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE)
    parser.add_argument("--fresh-timeout-s", type=float, default=1.0)
    parser.add_argument("--diag-fresh-timeout-s", type=float, default=2.0)
    parser.add_argument("--precheck-sample-s", type=float, default=0.4)
    parser.add_argument("--zero-check-wait-s", type=float, default=0.8)
    parser.add_argument("--zero-tolerance", type=float, default=0.005)
    parser.add_argument("--feedback-tolerance", type=float, default=0.03)
    parser.add_argument("--service-timeout-s", type=float, default=10.0)
    parser.add_argument("--save-every-segments", type=int, default=1)
    parser.add_argument(
        "--save-policy",
        choices=("every_step", "every_n_steps", "critical_or_end", "pipelined_critical"),
        default="every_step",
    )
    parser.add_argument("--save-every-n", type=int, default=2)
    parser.add_argument("--max-pending-saves", type=int, default=1)
    parser.add_argument("--console-mode", choices=("full", "compact"), default="full")
    parser.add_argument(
        "--map-prefix",
        default=f"/home/soc/edge-ai-robot-k1/maps/guarded_auto_micro_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    parser.add_argument(
        "--report",
        default=f"/home/soc/edge-ai-robot-k1/logs/guarded_auto_mapping_micro_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
    )
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()

    if args.confirm != "YES":
        reject("requires --confirm YES")
    if args.forward_linear <= 0.0 or args.forward_linear > MAX_LINEAR:
        reject(f"forward-linear must be > 0 and <= {MAX_LINEAR}")
    if args.turn_angular <= 0.0 or args.turn_angular > MAX_ANGULAR:
        reject(f"turn-angular must be > 0 and <= {MAX_ANGULAR}")
    if args.arc_turn_linear <= 0.0 or args.arc_turn_linear > MAX_LINEAR:
        reject(f"arc-turn-linear must be > 0 and <= {MAX_LINEAR}")
    if args.arc_step_linear <= 0.0 or args.arc_step_linear > MAX_LINEAR:
        reject(f"arc-step-linear must be > 0 and <= {MAX_LINEAR}")
    if args.arc_step_angular <= 0.0 or args.arc_step_angular > MAX_ANGULAR:
        reject(f"arc-step-angular must be > 0 and <= {MAX_ANGULAR}")
    if args.arc_step_duration_s < 0.5 or args.arc_step_duration_s > 1.5:
        reject("arc-step-duration-s must be between 0.5 and 1.5")
    if args.arc_step_repeats < 1 or args.arc_step_repeats > 3:
        reject("arc-step-repeats must be between 1 and 3")
    if args.arc_yaw_target_deg <= 0.0 or args.arc_yaw_target_deg > 90.0:
        reject("arc-yaw-target-deg must be > 0 and <= 90")
    if args.arc_yaw_tolerance_deg <= 0.0 or args.arc_yaw_tolerance_deg >= args.arc_yaw_target_deg:
        reject("arc-yaw-tolerance-deg must be > 0 and < arc-yaw-target-deg")
    if args.arc_yaw_overshoot_epsilon_deg < 0.0 or args.arc_yaw_overshoot_epsilon_deg > 5.0:
        reject("arc-yaw-overshoot-epsilon-deg must be >= 0 and <= 5")
    if args.arc_max_steps < 1 or args.arc_max_steps > 4:
        reject("arc-max-steps must be between 1 and 4")
    if args.arc_fast_front_p10_min_m < 0.30 or args.arc_fast_front_p10_min_m > 1.20:
        reject("arc-fast-front-p10-min-m must be between 0.30 and 1.20")
    if args.spatial_forward1_target_m <= 0.0 or args.spatial_forward1_target_m > MAX_ODOM_FORWARD_TARGET_M:
        reject(f"spatial-forward1-target-m must be > 0 and <= {MAX_ODOM_FORWARD_TARGET_M}")
    if args.spatial_forward2_target_m <= 0.0 or args.spatial_forward2_target_m > MAX_ODOM_FORWARD_TARGET_M:
        reject(f"spatial-forward2-target-m must be > 0 and <= {MAX_ODOM_FORWARD_TARGET_M}")
    if args.spatial_forward2_target_m < 0.08:
        reject("spatial-forward2-target-m must be >= 0.08 for current staged zones")
    if args.policy_duration_s < 1.0 or args.policy_duration_s > 300.0:
        reject("policy-duration-s must be between 1 and 300")
    if args.policy_samples < 0 or args.policy_samples > 300:
        reject("policy-samples must be between 0 and 300")
    if args.policy_sample_period_s < 0.2 or args.policy_sample_period_s > 5.0:
        reject("policy-sample-period-s must be between 0.2 and 5.0")
    if args.policy_max_steps < 1 or args.policy_max_steps > 7:
        reject("policy-max-steps must be between 1 and 7")
    if args.policy_max_runtime_s < 10.0 or args.policy_max_runtime_s > 180.0:
        reject("policy-max-runtime-s must be between 10 and 180")
    if args.policy_max_total_forward_m < 0.10 or args.policy_max_total_forward_m > 3.0:
        reject("policy-max-total-forward-m must be between 0.10 and 3.0")
    if args.policy_max_consecutive_fast_arc < 1 or args.policy_max_consecutive_fast_arc > 3:
        reject("policy-max-consecutive-fast-arc must be between 1 and 3")
    if args.policy_arc_fast_linear <= 0.0 or args.policy_arc_fast_linear > MAX_LINEAR:
        reject(f"policy-arc-fast-linear must be > 0 and <= {MAX_LINEAR}")
    if args.policy_arc_fast_angular <= 0.0 or args.policy_arc_fast_angular > MAX_ANGULAR:
        reject(f"policy-arc-fast-angular must be > 0 and <= {MAX_ANGULAR}")
    if args.policy_arc_fast_duration_s < 0.5 or args.policy_arc_fast_duration_s > 1.5:
        reject("policy-arc-fast-duration-s must be between 0.5 and 1.5")
    if args.yaw_angular <= 0.0 or args.yaw_angular > MAX_ANGULAR:
        reject(f"yaw-angular must be > 0 and <= {MAX_ANGULAR}")
    if args.yaw_duration_s < 1.0 or args.yaw_duration_s > MAX_YAW_AMPLIFIED_DURATION:
        reject(f"yaw-duration-s must be between 1.0 and {MAX_YAW_AMPLIFIED_DURATION}")
    if args.turn_threshold_detect_deg <= 0.0 or args.turn_threshold_detect_deg > 5.0:
        reject("turn-threshold-detect-deg must be > 0 and <= 5.0")
    if args.turn_threshold_detect_wz <= 0.0 or args.turn_threshold_detect_wz > 0.20:
        reject("turn-threshold-detect-wz must be > 0 and <= 0.20")
    if args.odom_forward_m <= 0.0 or args.odom_forward_m > MAX_ODOM_FORWARD_TARGET_M:
        reject(f"odom-forward-m must be > 0 and <= {MAX_ODOM_FORWARD_TARGET_M}")
    if args.odom_turn_deg <= 0.0 or args.odom_turn_deg > MAX_ODOM_TURN_TARGET_DEG:
        reject(f"odom-turn-deg must be > 0 and <= {MAX_ODOM_TURN_TARGET_DEG}")
    if args.odom_forward_timeout_s < 1.0 or args.odom_forward_timeout_s > MAX_ODOM_FORWARD_TIMEOUT_S:
        reject(
            f"odom-forward-timeout-s must be between 1.0 and {MAX_ODOM_FORWARD_TIMEOUT_S}"
        )
    if args.odom_turn_timeout_s < 1.0 or args.odom_turn_timeout_s > MAX_ODOM_TURN_TIMEOUT_S:
        reject(f"odom-turn-timeout-s must be between 1.0 and {MAX_ODOM_TURN_TIMEOUT_S}")
    if args.threshold_start_speed <= 0.0 or args.threshold_start_speed > MAX_LINEAR:
        reject(f"threshold-start-speed must be > 0 and <= {MAX_LINEAR}")
    if args.threshold_max_speed < args.threshold_start_speed or args.threshold_max_speed > MAX_LINEAR:
        reject(f"threshold-max-speed must be >= start and <= {MAX_LINEAR}")
    if args.threshold_step_speed <= 0.0 or args.threshold_step_speed > MAX_LINEAR:
        reject(f"threshold-step-speed must be > 0 and <= {MAX_LINEAR}")
    if args.threshold_pulse_s < 0.2 or args.threshold_pulse_s > 2.0:
        reject("threshold-pulse-s must be between 0.2 and 2.0")
    if args.threshold_detect_m <= 0.0 or args.threshold_detect_m > 0.05:
        reject("threshold-detect-m must be > 0 and <= 0.05")
    if args.threshold_detect_vx <= 0.0 or args.threshold_detect_vx > 0.10:
        reject("threshold-detect-vx must be > 0 and <= 0.10")
    if args.forward_target_m <= 0.0 or args.forward_target_m > MAX_ODOM_FORWARD_TARGET_M:
        reject(f"forward-target-m must be > 0 and <= {MAX_ODOM_FORWARD_TARGET_M}")
    for label, value in (
        ("forward-fast-speed", args.forward_fast_speed),
        ("forward-mid-speed", args.forward_mid_speed),
        ("forward-slow-speed", args.forward_slow_speed),
    ):
        if value <= 0.0 or value > MAX_LINEAR:
            reject(f"{label} must be > 0 and <= {MAX_LINEAR}")
    if not (args.forward_slow_speed <= args.forward_mid_speed <= args.forward_fast_speed):
        reject("staged speeds must satisfy slow <= mid <= fast")
    if args.forward_brake_margin_m <= 0.0 or args.forward_brake_margin_m > 0.10:
        reject("forward-brake-margin-m must be > 0 and <= 0.10")
    if args.forward_brake_coef_s < 0.0 or args.forward_brake_coef_s > 3.0:
        reject("forward-brake-coef-s must be >= 0 and <= 3.0")
    if args.forward_static_brake_margin_m < 0.0 or args.forward_static_brake_margin_m > 0.10:
        reject("forward-static-brake-margin-m must be >= 0 and <= 0.10")
    if args.forward_slow_zone_m <= args.forward_brake_margin_m:
        reject("forward-slow-zone-m must be greater than forward-brake-margin-m")
    if args.forward_mid_zone_m <= args.forward_slow_zone_m:
        reject("forward-mid-zone-m must be greater than forward-slow-zone-m")
    if args.forward_mid_zone_m > args.forward_target_m:
        reject("forward-mid-zone-m must be <= forward-target-m")
    if args.forward_timeout_s < 1.0 or args.forward_timeout_s > 8.0:
        reject("forward-timeout-s must be between 1.0 and 8.0")
    if args.zero_hold_s < 1.0 or args.zero_hold_s > 8.0:
        reject("zero-hold-s must be between 1 and 8")
    if args.zero_min_hold_s < 0.2 or args.zero_min_hold_s > args.zero_hold_s:
        reject("zero-min-hold-s must be >= 0.2 and <= zero-hold-s")
    if args.zero_poll_s < 0.05 or args.zero_poll_s > 0.5:
        reject("zero-poll-s must be between 0.05 and 0.5")
    if args.zero_confirm_samples < 1 or args.zero_confirm_samples > 10:
        reject("zero-confirm-samples must be between 1 and 10")
    if args.save_every_segments < 1 or args.save_every_segments > 2:
        reject("save-every-segments must be 1 or 2")
    if args.save_every_n < 1 or args.save_every_n > 10:
        reject("save-every-n must be between 1 and 10")
    if args.max_pending_saves != 1:
        reject("current save pipeline supports exactly one pending save")
    if args.save_map_retries < 0 or args.save_map_retries > 5:
        reject("save-map-retries must be between 0 and 5")
    if args.save_map_retry_delay_s < 0.5 or args.save_map_retry_delay_s > 10.0:
        reject("save-map-retry-delay-s must be between 0.5 and 10.0")
    return args


def main():
    args = parse_args()
    print("SAFETY WARNING")
    if args.mode in ("guarded-policy-dry-run", "save-map-only"):
        print("- This mode does not publish /input_cmd_vel.")
    else:
        print("- This publishes only to /input_cmd_vel and expects scan_safety_guard to drive /cmd_vel_guarded.")
    print("- It does not start RRT, AMCL, Nav2 goals, or open-ended exploration.")
    print("- Run only while physically supervised; be ready to lift or disable the robot.")
    print(
        f"- mode={args.mode} zero_wait=event(min={args.zero_min_hold_s:.1f}s,"
        f"max={args.zero_hold_s:.1f}s,poll={args.zero_poll_s:.2f}s,"
        f"confirm={args.zero_confirm_samples}) "
        f"forward={args.forward_linear:.2f}m/s turn={args.turn_angular:.2f}rad/s "
        f"yaw_amp={args.yaw_angular:.2f}rad/s*{args.yaw_duration_s:.1f}s "
        f"odom_forward={args.odom_forward_m:.2f}m odom_turn={args.odom_turn_deg:.1f}deg "
        f"threshold={args.threshold_start_speed:.2f}:{args.threshold_step_speed:.2f}:{args.threshold_max_speed:.2f} "
        f"staged_target={args.forward_target_m:.2f}m staged={args.forward_fast_speed:.2f}/"
        f"{args.forward_mid_speed:.2f}/{args.forward_slow_speed:.2f}m/s "
        f"brake=base{args.forward_brake_margin_m:.2f}+vx*{args.forward_brake_coef_s:.2f}"
        f"+static{args.forward_static_brake_margin_m:.2f} "
        f"front_p10_min={args.forward_front_p10_min_m:.2f}m "
        f"arc_fast={args.arc_fast_profile}/{args.arc_fast_direction}/"
        f"front_p10_min{args.arc_fast_front_p10_min_m:.2f}m "
        f"behavior_profile={args.behavior_profile} "
        f"policy_arc_mode={args.policy_arc_mode} "
        f"policy_arc_fast={args.policy_arc_fast_linear:.2f}/"
        f"{args.policy_arc_fast_angular:.2f}/{args.policy_arc_fast_duration_s:.1f}s "
        f"max_fast_arc={args.policy_max_consecutive_fast_arc} "
        f"policy_max_steps={args.policy_max_steps} policy_max_runtime={args.policy_max_runtime_s:.1f}s "
        f"policy_max_total_forward={args.policy_max_total_forward_m:.2f}m "
        f"save_policy={args.save_policy} console={args.console_mode}"
    )
    rclpy.init()
    node = GuardedAutoMappingMicro(args)
    try:
        if args.mode == "yaw-calibration":
            result = node.run_yaw_calibration()
        elif args.mode == "yaw-amplified":
            result = node.run_yaw_amplified()
        elif args.mode == "turn-threshold":
            result = node.run_turn_threshold()
        elif args.mode == "turn-duration-sweep":
            result = node.run_turn_duration_sweep()
        elif args.mode == "arc-turn-threshold":
            result = node.run_arc_turn_threshold()
        elif args.mode == "arc-step-repeat":
            result = node.run_arc_step_repeat()
        elif args.mode == "arc-fast-calib":
            result = node.run_arc_fast_calib()
        elif args.mode == "arc-yaw-closed":
            result = node.run_arc_yaw_closed()
        elif args.mode == "spatial-micro-run":
            result = node.run_spatial_micro()
        elif args.mode == "spatial-s-run":
            result = node.run_spatial_s()
        elif args.mode == "save-map-only":
            result = node.run_save_map_only()
        elif args.mode == "guarded-policy-dry-run":
            result = node.run_guarded_policy_dry_run()
        elif args.mode == "guarded-policy-step":
            result = node.run_guarded_policy_step()
        elif args.mode == "guarded-policy-run":
            result = node.run_guarded_policy_run()
        elif args.mode == "forward-threshold":
            result = node.run_forward_threshold()
        elif args.mode == "forward-staged":
            result = node.run_forward_staged()
        elif args.mode == "odom-micro-run":
            result = node.run_odom_micro()
        else:
            result = node.run_micro()
        payload = {
            "started_at": datetime.now().isoformat(timespec="seconds"),
            "args": vars(args),
            "result": result,
        }
        write_report(args.report, payload)
        print("REPORT", args.report)
        if args.console_mode == "compact":
            if result.get("mode") == "guarded-policy-run":
                print(
                    f"RESULT_SUMMARY stop={result.get('sequence_stop_reason')} "
                    f"steps={result.get('step_count')} executed={result.get('executed_count')} "
                    f"base_zero={result.get('base_zero_ok')} "
                    f"final_map={bool((result.get('final_map_save') or {}).get('ok'))} "
                    f"report={args.report}"
                )
            elif result.get("mode") == "guarded-policy-step":
                print(
                    f"RESULT_SUMMARY stop={result.get('stop_reason') or 'none'} "
                    f"executed={result.get('executed')} base_zero={result.get('base_zero_ok')} "
                    f"map_saved={result.get('map_saved')} report={args.report}"
                )
            elif result.get("mode") == "arc-fast-calib":
                fastest = result.get("fastest_ok") or {}
                print(
                    f"RESULT_SUMMARY mode=arc-fast-calib "
                    f"stop={result.get('sequence_stop_reason') or 'none'} "
                    f"records={len(result.get('records') or [])} "
                    f"ok={result.get('ok_count')} "
                    f"fastest={fastest.get('case_name')}:{fastest.get('direction')} "
                    f"report={args.report}"
                )
            else:
                print(f"RESULT_SUMMARY mode={result.get('mode')} report={args.report}")
        else:
            print("RESULT_JSON", json.dumps(result, ensure_ascii=False))
    finally:
        try:
            if args.mode not in ("guarded-policy-dry-run", "save-map-only"):
                node.zero_hold(1.0)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
