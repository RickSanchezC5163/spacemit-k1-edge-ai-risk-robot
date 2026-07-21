#!/usr/bin/env python3
"""Serve calibrated K1 motion controls with a live SLAM map and robot pose."""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from k1_calibrated_motion import MotionSemantic, get_semantic, list_semantics


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class OdomSnapshot:
    stamp: float
    sequence: int
    x: float
    y: float
    yaw: float
    linear_x: float
    angular_z: float
    age_s: float


class SemanticMappingNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("k1_semantic_mapping_server")
        self.args = args
        self.lock = threading.RLock()
        self.cancel_motion = threading.Event()
        self.motion_thread: Optional[threading.Thread] = None
        self.odom: Optional[Odometry] = None
        self.odom_monotonic = 0.0
        self.odom_sequence = 0
        self.map_payload: Optional[dict[str, Any]] = None
        self.map_version = 0
        self.map_pose: Optional[dict[str, float]] = None
        self.map_pose_monotonic = 0.0
        self.front_min_m = math.inf
        self.rear_min_m = math.inf
        self.scan_monotonic = 0.0
        self.trail: deque[dict[str, float]] = deque(maxlen=args.max_trail_points)
        self.motion: dict[str, Any] = {
            "state": "idle",
            "phase": "IDLE",
            "semantic": None,
            "label": "待命",
            "result": None,
            "progress": 0.0,
            "target": 0.0,
            "unit": None,
            "started_at": None,
            "finished_at": None,
        }
        self.hold_zero = False
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 20)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(OccupancyGrid, args.map_topic, self.on_map, map_qos)
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan, scan_qos)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=3.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.25, self.update_map_pose)
        self.create_timer(0.02, self.publish_held_zero)
        self.log_file = args.log_jsonl.open("a", encoding="utf-8") if args.log_jsonl else None

    def log_event(self, event: dict[str, Any]) -> None:
        payload = {"wall_time": time.time(), **event}
        self.get_logger().info(json.dumps(payload, ensure_ascii=False))
        if self.log_file:
            self.log_file.write(json.dumps(payload, ensure_ascii=False) + "\n")
            self.log_file.flush()

    def on_odom(self, msg: Odometry) -> None:
        with self.lock:
            self.odom = msg
            self.odom_monotonic = time.monotonic()
            self.odom_sequence += 1

    def on_map(self, msg: OccupancyGrid) -> None:
        q = msg.info.origin.orientation
        payload = {
            "frame_id": msg.header.frame_id or self.args.map_frame,
            "resolution": float(msg.info.resolution),
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "origin": {
                "x": float(msg.info.origin.position.x),
                "y": float(msg.info.origin.position.y),
                "yaw": yaw_from_quaternion(q),
            },
            "data": list(msg.data),
        }
        with self.lock:
            self.map_version += 1
            payload["version"] = self.map_version
            self.map_payload = payload

    def on_scan(self, msg: LaserScan) -> None:
        sector = math.radians(self.args.emergency_sector_deg)
        front = math.inf
        rear = math.inf
        for index, value in enumerate(msg.ranges):
            distance = float(value)
            if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
                continue
            angle = normalize_angle(msg.angle_min + index * msg.angle_increment)
            if abs(angle) <= sector:
                front = min(front, distance)
            if abs(normalize_angle(angle - math.pi)) <= sector:
                rear = min(rear, distance)
        with self.lock:
            self.front_min_m = front
            self.rear_min_m = rear
            self.scan_monotonic = time.monotonic()

    def update_map_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.map_frame,
                self.args.base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return
        pose = {
            "x": float(transform.transform.translation.x),
            "y": float(transform.transform.translation.y),
            "yaw": yaw_from_quaternion(transform.transform.rotation),
        }
        with self.lock:
            self.map_pose = pose
            self.map_pose_monotonic = time.monotonic()
            if not self.trail:
                self.trail.append(dict(pose))
            else:
                last = self.trail[-1]
                moved = math.hypot(pose["x"] - last["x"], pose["y"] - last["y"])
                turned = abs(normalize_angle(pose["yaw"] - last["yaw"]))
                if moved >= 0.015 or turned >= math.radians(4.0):
                    self.trail.append(dict(pose))

    def _odom_snapshot(self) -> Optional[OdomSnapshot]:
        now = time.monotonic()
        with self.lock:
            if self.odom is None:
                return None
            pose = self.odom.pose.pose
            twist = self.odom.twist.twist
            stamp = self.odom.header.stamp
            return OdomSnapshot(
                stamp=float(stamp.sec) + float(stamp.nanosec) * 1e-9,
                sequence=self.odom_sequence,
                x=float(pose.position.x),
                y=float(pose.position.y),
                yaw=yaw_from_quaternion(pose.orientation),
                linear_x=float(twist.linear.x),
                angular_z=float(twist.angular.z),
                age_s=max(0.0, now - self.odom_monotonic),
            )

    def _inputs_ready(self) -> tuple[bool, str]:
        snapshot = self._odom_snapshot()
        if snapshot is None or snapshot.age_s > self.args.input_max_age_s:
            return False, "odom_stale"
        if self.cmd_pub.get_subscription_count() < 1:
            return False, "cmd_vel_has_no_subscriber"
        return True, "ready"

    def start_motion(self, semantic_code: str) -> tuple[bool, str]:
        semantic = get_semantic(semantic_code)
        with self.lock:
            if self.motion_thread and self.motion_thread.is_alive():
                return False, "motion_busy"
            if self.motion["state"] == "failed":
                return False, "controller_failed_zero_hold"
        ready, reason = self._inputs_ready()
        if not ready:
            return False, reason
        with self.lock:
            if self.motion_thread and self.motion_thread.is_alive():
                return False, "motion_busy"
            if self.motion["state"] == "failed":
                return False, "controller_failed_zero_hold"
            self.cancel_motion.clear()
            self.hold_zero = False
            self.motion = {
                "state": "running",
                "phase": "EXECUTING",
                "semantic": semantic.code,
                "label": semantic.label,
                "result": None,
                "progress": 0.0,
                "target": semantic.odom_cutoff,
                "unit": "m" if semantic.kind == "drive" else "deg",
                "started_at": time.time(),
                "finished_at": None,
            }
            self.motion_thread = threading.Thread(
                target=self._run_motion,
                args=(semantic,),
                name=f"semantic-{semantic.code.lower()}",
                daemon=True,
            )
            self.motion_thread.start()
        return True, "accepted"

    def _set_progress(self, value: float) -> None:
        with self.lock:
            self.motion["progress"] = round(value, 4)

    def _set_phase(self, phase: str) -> None:
        with self.lock:
            self.motion["phase"] = phase

    def _run_motion(self, semantic: MotionSemantic) -> None:
        self.log_event({"event": "semantic_start", **semantic.public_dict()})
        result = "timeout"
        final_progress = 0.0
        try:
            if semantic.kind == "drive":
                result, final_progress = self._run_drive(semantic)
            else:
                result, final_progress = self._run_turn(semantic)
        except Exception as exc:  # noqa: BLE001 - always stop the real chassis.
            result = f"error:{type(exc).__name__}"
            self.log_event({"event": "semantic_exception", "error": str(exc)})
        finally:
            self._set_phase("ZERO_FLUSH")
            with self.lock:
                self.hold_zero = True
            self.publish_zero(self.args.stop_settle_s)
            self._set_phase("WAIT_SETTLED")
            settled = self._wait_settled()
            if not settled:
                result = "settle_timeout"
            successful = result in {"distance_reached", "angle_reached", "cancelled"} and settled
            with self.lock:
                self.motion.update(
                    {
                        "state": "idle" if successful else "failed",
                        "phase": "IDLE" if successful else "FAILED",
                        "result": result,
                        "progress": round(final_progress, 4),
                        "finished_at": time.time(),
                    }
                )
                self.hold_zero = not successful
            self.log_event(
                {
                    "event": "semantic_result",
                    "semantic": semantic.code,
                    "result": result,
                    "progress": round(final_progress, 4),
                }
            )

    def _wait_settled(self) -> bool:
        deadline = time.monotonic() + self.args.settle_timeout_s
        last_sequence = -1
        settled_samples = 0
        while time.monotonic() < deadline:
            self.cmd_pub.publish(Twist())
            snapshot = self._odom_snapshot()
            if (
                snapshot is not None
                and snapshot.age_s <= self.args.input_max_age_s
                and snapshot.sequence != last_sequence
            ):
                last_sequence = snapshot.sequence
                if (
                    abs(snapshot.linear_x) < self.args.settle_linear_mps
                    and abs(snapshot.angular_z) < self.args.settle_angular_rps
                ):
                    settled_samples += 1
                    if settled_samples >= self.args.settle_samples:
                        return True
                else:
                    settled_samples = 0
            time.sleep(0.02)
        return False

    def _run_drive(self, semantic: MotionSemantic) -> tuple[str, float]:
        start = self._odom_snapshot()
        if start is None or start.age_s > self.args.input_max_age_s:
            return "odom_stale", 0.0
        command = Twist()
        command.linear.x = semantic.direction * semantic.speed
        deadline = time.monotonic() + semantic.timeout_s
        progress = 0.0
        period = 1.0 / self.args.command_rate_hz
        yaw_hold = self.args.advance_yaw_hold and semantic.direction > 0
        yaw_ref = start.yaw
        last_progress = time.monotonic()
        best_progress = 0.0
        saturation_started: Optional[float] = None
        next_diagnostic = time.monotonic()
        while time.monotonic() < deadline:
            if self.cancel_motion.is_set():
                return "cancelled", progress
            with self.lock:
                clearance = self.front_min_m if semantic.direction > 0 else self.rear_min_m
            if math.isfinite(clearance) and clearance < self.args.emergency_distance_m:
                return "lidar_emergency_stop", progress
            snapshot = self._odom_snapshot()
            if snapshot is None or snapshot.age_s > self.args.input_max_age_s:
                return "odom_stale", progress
            dx = snapshot.x - start.x
            dy = snapshot.y - start.y
            progress = semantic.direction * (
                dx * math.cos(start.yaw) + dy * math.sin(start.yaw)
            )
            self._set_progress(progress)
            if progress >= semantic.odom_cutoff - self.args.distance_tolerance_m:
                return "distance_reached", progress

            now = time.monotonic()
            if semantic.direction > 0:
                if progress >= best_progress + self.args.advance_progress_delta_m:
                    best_progress = progress
                    last_progress = now
                elif now - last_progress > self.args.advance_no_progress_timeout_s:
                    return "no_distance_progress", progress

            yaw_error = 0.0
            command.angular.z = 0.0
            if yaw_hold:
                yaw_error = normalize_angle(yaw_ref - snapshot.yaw)
                if abs(yaw_error) > math.radians(self.args.advance_max_yaw_error_deg):
                    return "yaw_error_limit", progress
                if abs(yaw_error) > self.args.advance_yaw_deadband_rad:
                    correction = clamp(
                        self.args.advance_yaw_kp * yaw_error,
                        -self.args.advance_max_angular_z,
                        self.args.advance_max_angular_z,
                    )
                    if 0.0 < abs(correction) < self.args.advance_min_effective_angular_z:
                        correction = math.copysign(self.args.advance_min_effective_angular_z, correction)
                    command.angular.z = correction

                saturated = (
                    self.args.advance_max_angular_z > 0.0
                    and abs(command.angular.z) >= self.args.advance_max_angular_z - 1e-9
                )
                if saturated:
                    if saturation_started is None:
                        saturation_started = now
                    elif now - saturation_started > self.args.advance_saturation_timeout_s:
                        return "yaw_correction_saturated", progress
                else:
                    saturation_started = None

                if now >= next_diagnostic:
                    self.log_event(
                        {
                            "event": "advance_yaw_hold",
                            "yaw_ref": yaw_ref,
                            "yaw": snapshot.yaw,
                            "yaw_error": yaw_error,
                            "angular_cmd": command.angular.z,
                            "angular_feedback": snapshot.angular_z,
                            "progress": progress,
                        }
                    )
                    next_diagnostic = now + 1.0 / self.args.advance_diagnostic_hz
            self.cmd_pub.publish(command)
            time.sleep(period)
        return "timeout", progress

    def _run_turn(self, semantic: MotionSemantic) -> tuple[str, float]:
        pose = self._odom_snapshot()
        if pose is None or pose.age_s > self.args.input_max_age_s:
            return "odom_stale", 0.0
        last_yaw = pose.yaw
        accumulated = 0.0
        command = Twist()
        command.angular.z = semantic.direction * semantic.speed
        deadline = time.monotonic() + semantic.timeout_s
        period = 1.0 / self.args.command_rate_hz
        while time.monotonic() < deadline:
            if self.cancel_motion.is_set():
                return "cancelled", math.degrees(accumulated)
            pose = self._odom_snapshot()
            if pose is None or pose.age_s > self.args.input_max_age_s:
                return "odom_stale", math.degrees(accumulated)
            accumulated += normalize_angle(pose.yaw - last_yaw)
            last_yaw = pose.yaw
            directed_deg = semantic.direction * math.degrees(accumulated)
            self._set_progress(directed_deg)
            if directed_deg >= semantic.odom_cutoff:
                return "angle_reached", directed_deg
            self.cmd_pub.publish(command)
            time.sleep(period)
        return "timeout", semantic.direction * math.degrees(accumulated)

    def publish_zero(self, duration_s: float = 1.0) -> None:
        zero = Twist()
        deadline = time.monotonic() + max(0.1, duration_s)
        while time.monotonic() < deadline:
            self.cmd_pub.publish(zero)
            time.sleep(0.02)

    def publish_held_zero(self) -> None:
        with self.lock:
            hold_zero = self.hold_zero
        if hold_zero:
            self.cmd_pub.publish(Twist())

    def stop_motion(self) -> None:
        self.cancel_motion.set()
        with self.lock:
            self.hold_zero = True
        self.cmd_pub.publish(Twist())

    def reset_trail(self) -> None:
        with self.lock:
            self.trail.clear()
            if self.map_pose:
                self.trail.append(dict(self.map_pose))

    def state_snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            odom = None
            if self.odom:
                pose = self.odom.pose.pose
                twist = self.odom.twist.twist
                stamp = self.odom.header.stamp
                odom = {
                    "stamp": round(float(stamp.sec) + float(stamp.nanosec) * 1e-9, 9),
                    "x": round(float(pose.position.x), 4),
                    "y": round(float(pose.position.y), 4),
                    "yaw": round(yaw_from_quaternion(pose.orientation), 6),
                    "yaw_deg": round(math.degrees(yaw_from_quaternion(pose.orientation)), 2),
                    "linear_x": round(float(twist.linear.x), 4),
                    "angular_z": round(float(twist.angular.z), 4),
                    "age_s": round(now - self.odom_monotonic, 3),
                }
            map_pose = None
            if self.map_pose:
                map_pose = {
                    "x": round(self.map_pose["x"], 4),
                    "y": round(self.map_pose["y"], 4),
                    "yaw_deg": round(math.degrees(self.map_pose["yaw"]), 2),
                    "age_s": round(now - self.map_pose_monotonic, 3),
                }
            return {
                "server_time": time.time(),
                "motion": dict(self.motion),
                "odom": odom,
                "map_pose": map_pose,
                "scan": {
                    "front_m": None if not math.isfinite(self.front_min_m) else round(self.front_min_m, 3),
                    "rear_m": None if not math.isfinite(self.rear_min_m) else round(self.rear_min_m, 3),
                    "age_s": None if not self.scan_monotonic else round(now - self.scan_monotonic, 3),
                },
                "map": None
                if self.map_payload is None
                else {
                    key: self.map_payload[key]
                    for key in ("version", "frame_id", "resolution", "width", "height", "origin")
                },
                "trail": list(self.trail),
                "semantics": [item.public_dict() for item in list_semantics()],
            }

    def map_snapshot(self) -> Optional[dict[str, Any]]:
        with self.lock:
            if self.map_payload is None:
                return None
            return {
                **{key: value for key, value in self.map_payload.items() if key != "data"},
                "data": list(self.map_payload["data"]),
            }

    def close(self) -> None:
        self.stop_motion()
        self.publish_zero(0.6)
        if self.log_file:
            self.log_file.close()


def make_handler(node: SemanticMappingNode, html_path: Path):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def _send(self, status: int, payload: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _send_json(self, status: int, payload: Any) -> None:
            self._send(
                status,
                json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
                "application/json; charset=utf-8",
            )

        def _json_body(self) -> dict[str, Any]:
            length = min(int(self.headers.get("Content-Length", "0")), 4096)
            raw = self.rfile.read(length)
            return json.loads(raw.decode("utf-8")) if raw else {}

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/api/state":
                self._send_json(200, node.state_snapshot())
                return
            if parsed.path == "/api/map":
                snapshot = node.map_snapshot()
                if snapshot is None:
                    self._send_json(503, {"error": "map_unavailable"})
                    return
                requested = parse_qs(parsed.query).get("version", [None])[0]
                if requested is not None and int(requested) == int(snapshot["version"]):
                    self._send(204, b"", "application/json; charset=utf-8")
                    return
                self._send_json(200, snapshot)
                return
            if parsed.path in ("/", "/index.html"):
                try:
                    html_bytes = html_path.read_bytes()
                except OSError as exc:
                    self._send_json(500, {"error": f"html_unavailable:{exc}"})
                    return
                self._send(200, html_bytes, "text/html; charset=utf-8")
                return
            self._send_json(404, {"error": "not_found"})

        def do_POST(self) -> None:  # noqa: N802
            try:
                body = self._json_body()
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json(400, {"error": "invalid_json"})
                return
            if self.path == "/api/motion":
                try:
                    accepted, reason = node.start_motion(str(body.get("semantic", "")))
                except ValueError as exc:
                    self._send_json(400, {"error": str(exc)})
                    return
                self._send_json(202 if accepted else 409, {"accepted": accepted, "reason": reason})
                return
            if self.path == "/api/stop":
                node.stop_motion()
                self._send_json(200, {"stopped": True})
                return
            if self.path == "/api/trail/reset":
                node.reset_trail()
                self._send_json(200, {"reset": True})
                return
            self._send_json(404, {"error": "not_found"})

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8769)
    parser.add_argument("--html", type=Path, default=Path(__file__).with_name("k1_semantic_mapping_controller.html"))
    parser.add_argument("--cmd-topic", default="/cmd_vel_raw")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--command-rate-hz", type=float, default=30.0)
    parser.add_argument("--stop-settle-s", type=float, default=0.8)
    parser.add_argument("--settle-timeout-s", type=float, default=2.0)
    parser.add_argument("--settle-linear-mps", type=float, default=0.03)
    parser.add_argument("--settle-angular-rps", type=float, default=0.04)
    parser.add_argument("--settle-samples", type=int, default=5)
    parser.add_argument("--distance-tolerance-m", type=float, default=0.005)
    parser.add_argument("--emergency-distance-m", type=float, default=0.10)
    parser.add_argument("--emergency-sector-deg", type=float, default=15.0)
    parser.add_argument("--input-max-age-s", type=float, default=0.20)
    parser.add_argument(
        "--advance-yaw-hold",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--advance-yaw-kp", type=float, default=1.0)
    parser.add_argument("--advance-yaw-deadband-rad", type=float, default=0.03)
    parser.add_argument("--advance-max-angular-z", type=float, default=0.08)
    parser.add_argument("--advance-min-effective-angular-z", type=float, default=0.0)
    parser.add_argument("--advance-max-yaw-error-deg", type=float, default=15.0)
    parser.add_argument("--advance-no-progress-timeout-s", type=float, default=1.0)
    parser.add_argument("--advance-progress-delta-m", type=float, default=0.01)
    parser.add_argument("--advance-saturation-timeout-s", type=float, default=0.75)
    parser.add_argument("--advance-diagnostic-hz", type=float, default=5.0)
    parser.add_argument("--max-trail-points", type=int, default=1500)
    parser.add_argument("--log-jsonl", type=Path)
    args = parser.parse_args()
    positive_values = {
        "command_rate_hz": args.command_rate_hz,
        "stop_settle_s": args.stop_settle_s,
        "settle_timeout_s": args.settle_timeout_s,
        "settle_linear_mps": args.settle_linear_mps,
        "settle_angular_rps": args.settle_angular_rps,
        "input_max_age_s": args.input_max_age_s,
        "advance_max_angular_z": args.advance_max_angular_z,
        "advance_max_yaw_error_deg": args.advance_max_yaw_error_deg,
        "advance_no_progress_timeout_s": args.advance_no_progress_timeout_s,
        "advance_progress_delta_m": args.advance_progress_delta_m,
        "advance_saturation_timeout_s": args.advance_saturation_timeout_s,
        "advance_diagnostic_hz": args.advance_diagnostic_hz,
    }
    for name, value in positive_values.items():
        if value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.settle_samples < 1:
        parser.error("--settle-samples must be at least 1")
    if args.advance_yaw_kp < 0.0 or args.advance_yaw_deadband_rad < 0.0:
        parser.error("advance yaw gain and deadband must be non-negative")
    if not 0.0 <= args.advance_min_effective_angular_z <= args.advance_max_angular_z:
        parser.error("advance minimum angular velocity must be between zero and the maximum")
    if args.log_jsonl:
        args.log_jsonl.parent.mkdir(parents=True, exist_ok=True)
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = SemanticMappingNode(args)
    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(node, args.html))
    http_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    http_thread.start()
    print(f"K1 semantic mapping controller: http://{args.host}:{args.port}/", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        httpd.server_close()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
