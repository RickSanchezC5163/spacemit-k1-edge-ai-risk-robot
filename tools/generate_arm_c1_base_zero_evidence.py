#!/usr/bin/env python3
"""Generate read-only base-zero evidence for the Arm-C1 hardware gate.

Default usage is offline extraction from an existing episode_report.json. That
mode is useful for dry-run and documentation, but it is explicitly not valid
for Arm-C1 hardware execution.

The optional --ros-live mode creates a read-only ROS subscriber node on K1 and
does not publish cmd_vel or open any serial port.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = ROOT / "outputs" / "arm_c1_base_zero_evidence_v1"

NUMBER_RE = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
DIAG_RE = re.compile(
    rf"diag .*cmd=\((?P<cmd_vx>{NUMBER_RE}),(?P<cmd_wz>{NUMBER_RE})\) "
    rf"serial=\((?P<serial_vx>{NUMBER_RE}),(?P<serial_wz>{NUMBER_RE})\) "
    rf"feedback=\((?P<fb_vx>{NUMBER_RE}),(?P<fb_wz>{NUMBER_RE})\)"
)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def bool_or_none(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    return None


def extract_base_zero_ok(raw: Dict[str, Any]) -> Optional[bool]:
    for key in ("base_zero_ok_before_arm", "base_zero_ok_before", "base_zero_ok"):
        if key in raw:
            return bool_or_none(raw.get(key))
    base_zero = raw.get("base_zero") or {}
    if isinstance(base_zero, dict) and "base_zero_ok" in base_zero:
        return bool_or_none(base_zero.get("base_zero_ok"))
    policy_state = raw.get("policy_state") or {}
    if isinstance(policy_state, dict) and "base_zero_ok" in policy_state:
        return bool_or_none(policy_state.get("base_zero_ok"))
    return None


def extract_published_cmd_vel(raw: Dict[str, Any]) -> Optional[bool]:
    for key in ("published_cmd_vel", "published_cmd_vel_before", "published_cmd_vel_during_arm"):
        if key in raw:
            return bool_or_none(raw.get(key))
    base_zero = raw.get("base_zero") or {}
    if isinstance(base_zero, dict) and "published_cmd_vel" in base_zero:
        return bool_or_none(base_zero.get("published_cmd_vel"))
    summary = raw.get("summary") or {}
    if isinstance(summary, dict) and "published_cmd_vel" in summary:
        return bool_or_none(summary.get("published_cmd_vel"))
    return None


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)


def header_to_dict(msg: Any) -> Dict[str, Any]:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    return {
        "frame_id": str(getattr(getattr(msg, "header", None), "frame_id", "")),
        "stamp": {
            "sec": int(getattr(stamp, "sec", 0)),
            "nanosec": int(getattr(stamp, "nanosec", 0)),
        },
    }


def odom_to_dict(msg: Any) -> Optional[Dict[str, Any]]:
    if msg is None:
        return None
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    twist = msg.twist.twist
    yaw = yaw_from_quaternion(ori)
    return {
        "header": header_to_dict(msg),
        "child_frame_id": str(msg.child_frame_id),
        "pose": {
            "position": {"x": float(pos.x), "y": float(pos.y), "z": float(pos.z)},
            "orientation": {
                "x": float(ori.x),
                "y": float(ori.y),
                "z": float(ori.z),
                "w": float(ori.w),
            },
            "yaw_rad": float(yaw),
            "yaw_deg": float(math.degrees(yaw)),
        },
        "twist": {
            "linear": {"x": float(twist.linear.x), "y": float(twist.linear.y), "z": float(twist.linear.z)},
            "angular": {
                "x": float(twist.angular.x),
                "y": float(twist.angular.y),
                "z": float(twist.angular.z),
            },
        },
    }


def summarize_action_results(action_results: List[Dict[str, Any]]) -> Dict[str, Any]:
    succeeded_statuses = {"succeeded", "succeeded_dry_run"}
    relevant = [item for item in action_results if item.get("status") in succeeded_statuses]
    base_zero_values = [item.get("base_zero_ok_before") for item in relevant]
    cmd_vel_values = [item.get("published_cmd_vel") for item in action_results]
    return {
        "action_result_count": len(action_results),
        "succeeded_or_dryrun_count": len(relevant),
        "base_zero_ok_before_values": base_zero_values,
        "all_succeeded_results_base_zero_true": bool(relevant)
        and all(value is True for value in base_zero_values)
        and len(base_zero_values) == len(relevant),
        "any_published_cmd_vel_true": any(value is True for value in cmd_vel_values),
        "published_cmd_vel_values": cmd_vel_values,
    }


def build_offline_evidence(episode_report_path: Path) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    report = load_json(episode_report_path)
    if not isinstance(report, dict):
        raise ValueError("episode_report must be a JSON object")

    errors: List[Dict[str, Any]] = []
    policy_state = report.get("policy_state") or {}
    summary = report.get("summary") or {}
    action_results = report.get("action_results") or []
    if not isinstance(policy_state, dict):
        policy_state = {}
    if not isinstance(summary, dict):
        summary = {}
    if not isinstance(action_results, list):
        action_results = []

    source_base_zero = policy_state.get("base_zero") or {}
    policy_base_zero_ok = extract_base_zero_ok(policy_state)
    base_zero_snapshot_ok = extract_base_zero_ok(source_base_zero) if isinstance(source_base_zero, dict) else None
    result_summary = summarize_action_results(
        [item for item in action_results if isinstance(item, dict)]
    )
    action_results_ok = result_summary["all_succeeded_results_base_zero_true"]
    if action_results:
        base_zero_ok_before_arm = bool(action_results_ok and policy_base_zero_ok is True)
    else:
        base_zero_ok_before_arm = policy_base_zero_ok is True or base_zero_snapshot_ok is True

    published_candidates = [
        extract_published_cmd_vel(policy_state),
        extract_published_cmd_vel(source_base_zero) if isinstance(source_base_zero, dict) else None,
        extract_published_cmd_vel(summary),
    ]
    any_action_published = bool(result_summary["any_published_cmd_vel_true"])
    if any(value is True for value in published_candidates) or any_action_published:
        published_cmd_vel: Optional[bool] = True
    elif any(value is False for value in published_candidates) or action_results:
        published_cmd_vel = False
    else:
        published_cmd_vel = None

    if policy_base_zero_ok is not True and base_zero_snapshot_ok is not True:
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": "offline_base_zero_extract",
                "error": "source episode_report does not prove policy_state.base_zero_ok=true",
            }
        )
    if published_cmd_vel is not False:
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": "offline_base_zero_extract",
                "error": "source episode_report does not prove published_cmd_vel=false",
            }
        )

    evidence = {
        "schema_version": "arm_c1_base_zero_evidence_v1",
        "generated_at": now_iso(),
        "generator": "tools/generate_arm_c1_base_zero_evidence.py",
        "evidence_type": "offline_episode_report_snapshot",
        "source_mode": "offline_episode_report",
        "valid_for_arm_c1_hardware": False,
        "hardware_block_reason": "offline episode snapshots are audit evidence only; Arm-C1 hardware requires live_base_zero_observation",
        "read_only": True,
        "ros_node_created_by_this_script": False,
        "cmd_vel_publisher_created": False,
        "cmd_vel_published_by_this_script": False,
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "base_zero_checked_live": False,
        "base_zero_ok_before_arm": base_zero_ok_before_arm,
        "published_cmd_vel": published_cmd_vel,
        "source_episode_report": str(episode_report_path),
        "source_episode_id": report.get("episode_id"),
        "source_protocol_version": report.get("protocol_version"),
        "source_policy_state_id": policy_state.get("state_id"),
        "source_policy_timestamp": policy_state.get("timestamp"),
        "base_zero": source_base_zero,
        "odom": policy_state.get("odom"),
        "checks": {
            "policy_state_base_zero_ok": policy_base_zero_ok,
            "base_zero_snapshot_ok": base_zero_snapshot_ok,
            "source_summary_published_cmd_vel": summary.get("published_cmd_vel"),
            "action_results": result_summary,
            "offline_snapshot_can_unlock_hardware": False,
        },
        "claim_boundary": [
            "This offline evidence can support dry-run and documentation only.",
            "This offline evidence must not unlock Arm-C1 hardware execution.",
            "No ROS process, cmd_vel publisher, serial port, or mechanical arm action is used in offline mode.",
        ],
    }
    return evidence, errors


def run_live_evidence(args: argparse.Namespace) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    try:
        import rclpy
        from geometry_msgs.msg import Twist, Vector3
        from nav_msgs.msg import Odometry
        from rcl_interfaces.msg import Log
        from rclpy.node import Node
    except Exception as exc:  # noqa: BLE001 - live mode must explain import failure.
        raise RuntimeError(f"ROS imports unavailable for --ros-live mode: {exc}") from exc

    class BaseZeroObservationNode(Node):
        def __init__(self) -> None:
            super().__init__("arm_c1_base_zero_evidence_readonly")
            self.latest_odom = None
            self.latest_odom_time = 0.0
            self.latest_guarded_cmd = None
            self.latest_guarded_cmd_time = 0.0
            self.latest_robot_vel = None
            self.latest_robot_vel_time = 0.0
            self.latest_diag = None
            self.latest_diag_time = 0.0
            self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 20)
            self.create_subscription(Twist, args.guarded_cmd_topic, self._guarded_cmd_cb, 20)
            self.create_subscription(Vector3, args.robot_vel_topic, self._robot_vel_cb, 20)
            self.create_subscription(Log, "/rosout", self._rosout_cb, 50)

        def _odom_cb(self, msg: Any) -> None:
            self.latest_odom = msg
            self.latest_odom_time = time.monotonic()

        def _guarded_cmd_cb(self, msg: Any) -> None:
            self.latest_guarded_cmd = msg
            self.latest_guarded_cmd_time = time.monotonic()

        def _robot_vel_cb(self, msg: Any) -> None:
            self.latest_robot_vel = msg
            self.latest_robot_vel_time = time.monotonic()

        def _rosout_cb(self, msg: Any) -> None:
            if "wheeltec_tank_base" not in str(msg.name):
                return
            match = DIAG_RE.search(str(msg.msg))
            if not match:
                return
            self.latest_diag = {key: float(value) for key, value in match.groupdict().items()}
            self.latest_diag_time = time.monotonic()

        def base_zero_snapshot(self) -> Dict[str, Any]:
            now = time.monotonic()
            odom_fresh = (
                self.latest_odom is not None
                and now - self.latest_odom_time <= args.fresh_timeout_s
            )
            guarded_fresh = (
                self.latest_guarded_cmd is not None
                and now - self.latest_guarded_cmd_time <= args.fresh_timeout_s
            )
            robot_vel_fresh = (
                self.latest_robot_vel is not None
                and now - self.latest_robot_vel_time <= args.fresh_timeout_s
            )
            diag_fresh = (
                self.latest_diag is not None
                and now - self.latest_diag_time <= args.diag_fresh_timeout_s
            )

            odom_twist = None
            odom_zero_ok = False
            if self.latest_odom is not None:
                twist = self.latest_odom.twist.twist
                odom_twist = {
                    "linear_x": float(twist.linear.x),
                    "angular_z": float(twist.angular.z),
                }
                odom_zero_ok = (
                    odom_fresh
                    and abs(float(twist.linear.x)) <= args.feedback_tolerance
                    and abs(float(twist.angular.z)) <= args.feedback_tolerance
                )

            guarded_cmd = None
            guarded_cmd_zero_ok = False
            if self.latest_guarded_cmd is not None:
                guarded_cmd = {
                    "linear_x": float(self.latest_guarded_cmd.linear.x),
                    "angular_z": float(self.latest_guarded_cmd.angular.z),
                }
                guarded_cmd_zero_ok = (
                    guarded_fresh
                    and abs(float(self.latest_guarded_cmd.linear.x)) <= args.zero_tolerance
                    and abs(float(self.latest_guarded_cmd.angular.z)) <= args.zero_tolerance
                )

            robot_vel = None
            robot_vel_zero_ok = False
            if self.latest_robot_vel is not None:
                robot_vel = {
                    "feedback_vx": float(self.latest_robot_vel.x),
                    "feedback_wz": float(self.latest_robot_vel.z),
                }
                robot_vel_zero_ok = (
                    robot_vel_fresh
                    and abs(float(self.latest_robot_vel.x)) <= args.feedback_tolerance
                    and abs(float(self.latest_robot_vel.z)) <= args.feedback_tolerance
                )

            diag_zero_ok = False
            if self.latest_diag is not None:
                diag_zero_ok = (
                    diag_fresh
                    and abs(float(self.latest_diag["cmd_vx"])) <= args.zero_tolerance
                    and abs(float(self.latest_diag["cmd_wz"])) <= args.zero_tolerance
                    and abs(float(self.latest_diag["serial_vx"])) <= args.zero_tolerance
                    and abs(float(self.latest_diag["serial_wz"])) <= args.zero_tolerance
                    and abs(float(self.latest_diag["fb_vx"])) <= args.feedback_tolerance
                    and abs(float(self.latest_diag["fb_wz"])) <= args.feedback_tolerance
                )

            guarded_required_ok = guarded_cmd_zero_ok or (
                not args.require_guarded_cmd and self.latest_guarded_cmd is None
            )
            robot_required_ok = robot_vel_zero_ok or (
                not args.require_robot_vel and self.latest_robot_vel is None
            )
            diag_required_ok = diag_zero_ok or (
                not args.require_base_diag and self.latest_diag is None
            )
            base_zero_ok = bool(
                odom_zero_ok and guarded_required_ok and robot_required_ok and diag_required_ok
            )
            basis = ["odom"]
            if self.latest_guarded_cmd is not None or args.require_guarded_cmd:
                basis.append("guarded_cmd")
            if self.latest_robot_vel is not None or args.require_robot_vel:
                basis.append("robot_vel")
            if self.latest_diag is not None or args.require_base_diag:
                basis.append("base_diag")

            return {
                "base_zero_ok": base_zero_ok,
                "policy_zero_basis": "+".join(basis),
                "published_cmd_vel": False,
                "freshness": {
                    "odom_fresh": odom_fresh,
                    "guarded_cmd_fresh": guarded_fresh,
                    "robot_vel_fresh": robot_vel_fresh,
                    "diag_fresh": diag_fresh,
                },
                "required": {
                    "guarded_cmd": bool(args.require_guarded_cmd),
                    "robot_vel": bool(args.require_robot_vel),
                    "base_diag": bool(args.require_base_diag),
                },
                "odom_zero_ok": odom_zero_ok,
                "guarded_cmd_zero_ok": guarded_cmd_zero_ok,
                "robot_vel_zero_ok": robot_vel_zero_ok,
                "diag_zero_ok": diag_zero_ok,
                "latest_odom_twist": odom_twist,
                "latest_guarded_cmd": guarded_cmd,
                "latest_robot_vel": robot_vel,
                "latest_diag": self.latest_diag,
            }

        def wait_base_zero(self) -> Tuple[Dict[str, Any], Optional[Dict[str, Any]]]:
            started = time.monotonic()
            deadline = started + max(args.zero_hold_s, args.zero_min_hold_s, 0.1)
            min_deadline = started + max(args.zero_min_hold_s, 0.0)
            poll_s = max(args.zero_poll_s, 0.02)
            next_check = min_deadline
            confirm_count = 0
            checks: List[Dict[str, Any]] = []
            final_zero = self.base_zero_snapshot()
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.monotonic()
                if now < next_check:
                    continue
                final_zero = self.base_zero_snapshot()
                if final_zero.get("base_zero_ok"):
                    confirm_count += 1
                else:
                    confirm_count = 0
                checks.append(
                    {
                        "elapsed_s": round(now - started, 3),
                        "base_zero_ok": bool(final_zero.get("base_zero_ok")),
                        "confirm_count": int(confirm_count),
                        "freshness": final_zero.get("freshness"),
                        "latest_odom_twist": final_zero.get("latest_odom_twist"),
                        "latest_guarded_cmd": final_zero.get("latest_guarded_cmd"),
                        "latest_robot_vel": final_zero.get("latest_robot_vel"),
                        "latest_diag": final_zero.get("latest_diag"),
                    }
                )
                if confirm_count >= args.zero_confirm_samples:
                    break
                next_check = now + poll_s

            elapsed_s = time.monotonic() - started
            final_zero["mode"] = "observe_only_no_cmd_vel_publish"
            final_zero["elapsed_s"] = round(elapsed_s, 3)
            final_zero["min_hold_s"] = round(float(args.zero_min_hold_s), 3)
            final_zero["max_hold_s"] = round(float(args.zero_hold_s), 3)
            final_zero["poll_s"] = round(float(poll_s), 3)
            final_zero["required_confirmations"] = int(args.zero_confirm_samples)
            final_zero["confirm_count"] = int(confirm_count)
            final_zero["timed_out"] = confirm_count < args.zero_confirm_samples
            final_zero["checks"] = checks[-10:]
            return final_zero, odom_to_dict(self.latest_odom)

    errors: List[Dict[str, Any]] = []
    rclpy.init()
    node = BaseZeroObservationNode()
    try:
        base_zero, odom = node.wait_base_zero()
        valid_for_hw = bool(
            base_zero.get("base_zero_ok") is True
            and base_zero.get("published_cmd_vel") is False
            and base_zero.get("timed_out") is False
        )
        if not valid_for_hw:
            errors.append(
                {
                    "timestamp": now_iso(),
                    "stage": "live_base_zero_observation",
                    "error": "live base-zero observation did not satisfy Arm-C1 hardware gate",
                }
            )
        evidence = {
            "schema_version": "arm_c1_base_zero_evidence_v1",
            "generated_at": now_iso(),
            "generator": "tools/generate_arm_c1_base_zero_evidence.py",
            "evidence_type": "live_base_zero_observation",
            "source_mode": "ros_live_readonly",
            "valid_for_arm_c1_hardware": valid_for_hw,
            "hardware_block_reason": None if valid_for_hw else "live base-zero gate failed",
            "read_only": True,
            "ros_node_created_by_this_script": True,
            "cmd_vel_publisher_created": False,
            "cmd_vel_published_by_this_script": False,
            "serial_port_opened": False,
            "serial_bytes_written": 0,
            "base_zero_checked_live": True,
            "base_zero_ok_before_arm": bool(base_zero.get("base_zero_ok")),
            "published_cmd_vel": False,
            "topics": {
                "odom": args.odom_topic,
                "guarded_cmd": args.guarded_cmd_topic,
                "robot_vel": args.robot_vel_topic,
                "base_diag": "/rosout wheeltec_tank_base diag",
            },
            "base_zero": base_zero,
            "odom": odom,
            "checks": {
                "zero_confirm_samples": int(args.zero_confirm_samples),
                "zero_min_hold_s": float(args.zero_min_hold_s),
                "zero_hold_s": float(args.zero_hold_s),
                "fresh_timeout_s": float(args.fresh_timeout_s),
                "diag_fresh_timeout_s": float(args.diag_fresh_timeout_s),
                "zero_tolerance": float(args.zero_tolerance),
                "feedback_tolerance": float(args.feedback_tolerance),
            },
            "claim_boundary": [
                "This is a live read-only base-zero observation.",
                "The script creates no cmd_vel publisher and opens no serial port.",
                "This evidence only satisfies Arm-C1 hardware precondition if valid_for_arm_c1_hardware=true and the consuming gate accepts its freshness.",
            ],
        }
        return evidence, errors
    finally:
        node.destroy_node()
        rclpy.shutdown()


def write_readme(output_dir: Path, evidence: Dict[str, Any]) -> None:
    text = f"""# Arm-C1 Base-Zero Evidence

This directory contains read-only base-zero evidence for Arm-C1 gate checks.

## Result

- evidence_type: `{evidence.get('evidence_type')}`
- source_mode: `{evidence.get('source_mode')}`
- base_zero_ok_before_arm: `{evidence.get('base_zero_ok_before_arm')}`
- published_cmd_vel: `{evidence.get('published_cmd_vel')}`
- valid_for_arm_c1_hardware: `{evidence.get('valid_for_arm_c1_hardware')}`
- cmd_vel_published_by_this_script: `{evidence.get('cmd_vel_published_by_this_script')}`
- serial_port_opened: `{evidence.get('serial_port_opened')}`
- serial_bytes_written: `{evidence.get('serial_bytes_written')}`

## Boundary

- Offline episode snapshots are audit evidence only.
- Offline episode snapshots must not unlock Arm-C1 hardware execution.
- Arm-C1 hardware execution requires live evidence with
  `evidence_type=live_base_zero_observation` and
  `valid_for_arm_c1_hardware=true`.
- This script never controls the mechanical arm.
"""
    write_text(output_dir / "README.md", text)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--from-episode-report", default=None)
    source.add_argument("--ros-live", action="store_true")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--guarded-cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--robot-vel-topic", default="/robot_vel")
    parser.add_argument("--fresh-timeout-s", type=float, default=2.0)
    parser.add_argument("--diag-fresh-timeout-s", type=float, default=2.0)
    parser.add_argument("--zero-hold-s", type=float, default=4.0)
    parser.add_argument("--zero-min-hold-s", type=float, default=0.8)
    parser.add_argument("--zero-poll-s", type=float, default=0.1)
    parser.add_argument("--zero-confirm-samples", type=int, default=3)
    parser.add_argument("--zero-tolerance", type=float, default=0.005)
    parser.add_argument("--feedback-tolerance", type=float, default=0.03)
    parser.add_argument("--require-guarded-cmd", action="store_true")
    parser.add_argument("--no-require-robot-vel", dest="require_robot_vel", action="store_false")
    parser.add_argument("--no-require-base-diag", dest="require_base_diag", action="store_false")
    parser.set_defaults(require_robot_vel=True, require_base_diag=True)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        if args.from_episode_report:
            evidence, errors = build_offline_evidence(Path(args.from_episode_report))
        else:
            evidence, errors = run_live_evidence(args)
    except Exception as exc:  # noqa: BLE001 - write failed evidence for audit.
        errors = [
            {
                "timestamp": now_iso(),
                "stage": "generate_base_zero_evidence",
                "error": str(exc),
            }
        ]
        evidence = {
            "schema_version": "arm_c1_base_zero_evidence_v1",
            "generated_at": now_iso(),
            "generator": "tools/generate_arm_c1_base_zero_evidence.py",
            "evidence_type": "failed",
            "source_mode": "ros_live_readonly" if args.ros_live else "offline_episode_report",
            "valid_for_arm_c1_hardware": False,
            "base_zero_ok_before_arm": False,
            "published_cmd_vel": None,
            "read_only": True,
            "cmd_vel_published_by_this_script": False,
            "serial_port_opened": False,
            "serial_bytes_written": 0,
            "errors": errors,
        }

    write_json(output_dir / "base_zero_evidence.json", evidence)
    write_json(output_dir / "errors.json", errors)
    write_readme(output_dir, evidence)
    result = {
        "ok": not errors,
        "evidence_type": evidence.get("evidence_type"),
        "base_zero_ok_before_arm": evidence.get("base_zero_ok_before_arm"),
        "published_cmd_vel": evidence.get("published_cmd_vel"),
        "valid_for_arm_c1_hardware": evidence.get("valid_for_arm_c1_hardware"),
        "output_dir": str(output_dir),
        "errors": errors,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
