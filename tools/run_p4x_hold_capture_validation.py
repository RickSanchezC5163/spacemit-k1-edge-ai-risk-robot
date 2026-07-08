#!/usr/bin/env python3
"""Run P4-X D435 HOLD_CAPTURE evidence-chain validation."""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rcl_interfaces.msg import Log


ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from edge_robot_protocol import (  # noqa: E402
    ACTION_HOLD_CAPTURE,
    STATUS_FAILED_SAFE,
    STATUS_SUCCEEDED,
    ActionResult,
    EpisodeReport,
    PolicyAction,
    PolicyState,
    now_iso,
    to_jsonable,
)
from d435_capture_once import (  # noqa: E402
    DEFAULT_OUTPUT_ROOT,
    D435CaptureNode,
    odom_to_dict,
    write_json,
)
from mock_risk_detector import compute_mock_risk_point  # noqa: E402


NUMBER_RE = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
DIAG_RE = re.compile(
    rf"diag .*cmd=\((?P<cmd_vx>{NUMBER_RE}),(?P<cmd_wz>{NUMBER_RE})\) "
    rf"serial=\((?P<serial_vx>{NUMBER_RE}),(?P<serial_wz>{NUMBER_RE})\) "
    rf"feedback=\((?P<fb_vx>{NUMBER_RE}),(?P<fb_wz>{NUMBER_RE})\)"
)


class P4XHoldCaptureNode(D435CaptureNode):
    def __init__(self, args: argparse.Namespace):
        super().__init__(
            args.rgb_topic,
            args.depth_topic,
            args.camera_info_topic,
            args.odom_topic,
            fresh_timeout_s=args.fresh_timeout_s,
        )
        self.guarded_cmd_topic = args.guarded_cmd_topic
        self.robot_vel_topic = args.robot_vel_topic
        self.latest_guarded_cmd: Optional[Twist] = None
        self.latest_guarded_cmd_time = 0.0
        self.latest_robot_vel: Optional[Vector3] = None
        self.latest_robot_vel_time = 0.0
        self.latest_diag: Optional[Dict[str, float]] = None
        self.latest_diag_time = 0.0
        self.create_subscription(Twist, args.guarded_cmd_topic, self._guarded_cmd_cb, 20)
        self.create_subscription(Vector3, args.robot_vel_topic, self._robot_vel_cb, 20)
        self.create_subscription(Log, "/rosout", self._rosout_cb, 50)

    def _guarded_cmd_cb(self, msg: Twist) -> None:
        self.latest_guarded_cmd = msg
        self.latest_guarded_cmd_time = time.monotonic()

    def _robot_vel_cb(self, msg: Vector3) -> None:
        self.latest_robot_vel = msg
        self.latest_robot_vel_time = time.monotonic()

    def _rosout_cb(self, msg: Log) -> None:
        if "wheeltec_tank_base" not in str(msg.name):
            return
        match = DIAG_RE.search(str(msg.msg))
        if not match:
            return
        self.latest_diag = {key: float(value) for key, value in match.groupdict().items()}
        self.latest_diag_time = time.monotonic()

    def base_zero_snapshot(self, args: argparse.Namespace) -> Dict[str, Any]:
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
                and abs(self.latest_diag["cmd_vx"]) <= args.zero_tolerance
                and abs(self.latest_diag["cmd_wz"]) <= args.zero_tolerance
                and abs(self.latest_diag["serial_vx"]) <= args.zero_tolerance
                and abs(self.latest_diag["serial_wz"]) <= args.zero_tolerance
                and abs(self.latest_diag["fb_vx"]) <= args.feedback_tolerance
                and abs(self.latest_diag["fb_wz"]) <= args.feedback_tolerance
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

    def wait_base_zero(self, args: argparse.Namespace) -> Dict[str, Any]:
        started = time.monotonic()
        deadline = started + max(args.zero_hold_s, args.zero_min_hold_s, 0.1)
        min_deadline = started + max(args.zero_min_hold_s, 0.0)
        poll_s = max(args.zero_poll_s, 0.02)
        next_check = min_deadline
        confirm_count = 0
        checks: List[Dict[str, Any]] = []
        final_zero = self.base_zero_snapshot(args)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now < next_check:
                continue
            final_zero = self.base_zero_snapshot(args)
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
        return final_zero


def append_error(
    errors: List[Dict[str, Any]],
    tool: str,
    action_id: str,
    capture_id: str,
    error: str,
    stage: str,
) -> Dict[str, Any]:
    item = {
        "timestamp": now_iso(),
        "tool": tool,
        "stage": stage,
        "action_id": action_id,
        "capture_id": capture_id,
        "error": error,
    }
    errors.append(item)
    return item


def write_status_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    fieldnames = [
        "sequence",
        "action_id",
        "capture_id",
        "status",
        "base_zero_ok_before",
        "capture_meta_path",
        "risk_point_path",
        "error",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def episode_summary(count: int, results: List[ActionResult], min_successes: int) -> Dict[str, Any]:
    succeeded = sum(1 for result in results if result.status == STATUS_SUCCEEDED)
    failed_safe = sum(1 for result in results if result.status == STATUS_FAILED_SAFE)
    return {
        "requested_captures": int(count),
        "completed_actions": len(results),
        "succeeded": int(succeeded),
        "failed_safe": int(failed_safe),
        "success_rate": None if not results else round(succeeded / len(results), 3),
        "min_successes": int(min_successes),
        "acceptance_10_runs_9_success": bool(count >= 10 and succeeded >= 9),
        "published_cmd_vel": False,
    }


def write_episode_report(
    path: Path,
    episode_id: str,
    started_at: str,
    output_root: Path,
    latest_state: Optional[PolicyState],
    actions: List[PolicyAction],
    results: List[ActionResult],
    captures: List[Any],
    risk_points: List[Any],
    errors: List[Dict[str, Any]],
    count: int,
    min_successes: int,
) -> None:
    report = EpisodeReport(
        episode_id=episode_id,
        started_at=started_at,
        ended_at=now_iso(),
        policy_state=latest_state,
        actions=actions,
        action_results=results,
        captures=captures,
        risk_points=risk_points,
        summary=episode_summary(count, results, min_successes),
        errors=errors,
        output_root=str(output_root),
    )
    write_json(path, report)


def write_output_readme(
    path: Path,
    episode_id: str,
    summary: Dict[str, Any],
    args: argparse.Namespace,
) -> None:
    text = f"""# P4-X D435 HOLD_CAPTURE Evidence Chain

episode_id: `{episode_id}`

This output directory was generated by `tools/run_p4x_hold_capture_validation.py`.
The runner does not create any cmd_vel publishers and records `published_cmd_vel=false`
in every `ActionResult`.

## Summary

- requested_captures: `{summary['requested_captures']}`
- succeeded: `{summary['succeeded']}`
- failed_safe: `{summary['failed_safe']}`
- min_successes: `{summary['min_successes']}`
- acceptance_10_runs_9_success: `{summary['acceptance_10_runs_9_success']}`

## Files

- `d435_topic_audit.json` / `d435_topic_audit.md`: generated by the separate topic audit tool.
- `captures/<capture_id>/rgb.png`
- `captures/<capture_id>/depth_raw.npy`
- `captures/<capture_id>/depth_vis.png`
- `captures/<capture_id>/camera_info.json`
- `captures/<capture_id>/odom.json`
- `captures/<capture_id>/capture_meta.json`
- `captures/<capture_id>/risk_point.json`
- `captures/<capture_id>/action_result.json`
- `p4x_hold_capture_status.csv`
- `episode_report.json`
- `errors.json`

## Topics

- rgb: `{args.rgb_topic}`
- depth: `{args.depth_topic}`
- camera_info: `{args.camera_info_topic}`
- odom: `{args.odom_topic}`
- guarded_cmd: `{args.guarded_cmd_topic}`
- robot_vel: `{args.robot_vel_topic}`
"""
    path.write_text(text, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run consecutive P4-X HOLD_CAPTURE validation captures."
    )
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--episode-id", default=None)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--min-successes", type=int, default=None)
    parser.add_argument("--rgb-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/camera/depth/image_rect_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--guarded-cmd-topic", default="/cmd_vel_guarded")
    parser.add_argument("--robot-vel-topic", default="/robot_vel")
    parser.add_argument("--bbox", default=None, help="Mock detector bbox x,y,w,h.")
    parser.add_argument("--capture-timeout-s", type=float, default=8.0)
    parser.add_argument("--fresh-timeout-s", type=float, default=2.0)
    parser.add_argument("--diag-fresh-timeout-s", type=float, default=2.0)
    parser.add_argument("--zero-hold-s", type=float, default=4.0)
    parser.add_argument("--zero-min-hold-s", type=float, default=0.8)
    parser.add_argument("--zero-poll-s", type=float, default=0.1)
    parser.add_argument("--zero-confirm-samples", type=int, default=3)
    parser.add_argument("--zero-tolerance", type=float, default=0.005)
    parser.add_argument("--feedback-tolerance", type=float, default=0.03)
    parser.add_argument("--inter-capture-delay-s", type=float, default=0.2)
    parser.add_argument("--warmup-s", type=float, default=1.0)
    parser.add_argument("--depth-scale-m", type=float, default=None)
    parser.add_argument("--require-guarded-cmd", action="store_true")
    parser.add_argument("--no-require-robot-vel", dest="require_robot_vel", action="store_false")
    parser.add_argument("--no-require-base-diag", dest="require_base_diag", action="store_false")
    parser.set_defaults(require_robot_vel=True, require_base_diag=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_root = Path(args.output_dir)
    output_root.mkdir(parents=True, exist_ok=True)
    captures_root = output_root / "captures"
    episode_id = args.episode_id or f"p4x_hold_capture_{time.strftime('%Y%m%d_%H%M%S')}"
    min_successes = args.min_successes
    if min_successes is None:
        min_successes = 9 if args.count >= 10 else max(1, args.count)

    errors: List[Dict[str, Any]] = []
    actions: List[PolicyAction] = []
    results: List[ActionResult] = []
    captures: List[Any] = []
    risk_points: List[Any] = []
    status_rows: List[Dict[str, Any]] = []
    latest_state: Optional[PolicyState] = None
    started_at = now_iso()
    episode_path = output_root / "episode_report.json"
    status_path = output_root / "p4x_hold_capture_status.csv"
    errors_path = output_root / "errors.json"

    rclpy.init()
    node = P4XHoldCaptureNode(args)
    try:
        node.spin_for(args.warmup_s)
        for sequence in range(1, args.count + 1):
            action_id = f"{episode_id}_action_{sequence:02d}"
            capture_id = f"{episode_id}_capture_{sequence:02d}"
            capture_dir = captures_root / capture_id
            capture_dir.mkdir(parents=True, exist_ok=True)
            action_started_at = now_iso()
            action = PolicyAction(
                action_id=action_id,
                action_type=ACTION_HOLD_CAPTURE,
                requested_at=action_started_at,
                requires_base_zero=True,
                publishes_cmd_vel=False,
                reason="P4-X D435 HOLD_CAPTURE evidence-chain validation",
                params={
                    "sequence": sequence,
                    "capture_id": capture_id,
                    "bbox": args.bbox,
                },
            )
            actions.append(action)

            base_zero = node.wait_base_zero(args)
            base_zero_ok = bool(base_zero.get("base_zero_ok"))
            latest_state = PolicyState(
                state_id=f"{episode_id}_state_{sequence:02d}",
                timestamp=now_iso(),
                base_zero_ok=base_zero_ok,
                base_zero=base_zero,
                odom=odom_to_dict(node.latest_odom),
                source="run_p4x_hold_capture_validation",
            )

            result: ActionResult
            if not base_zero_ok:
                error = "base_zero_ok=false before HOLD_CAPTURE; capture skipped"
                append_error(errors, "run_p4x_hold_capture_validation", action_id, capture_id, error, "base_zero")
                write_json(
                    capture_dir / "errors.json",
                    [err for err in errors if err.get("capture_id") == capture_id],
                )
                result = ActionResult(
                    action_id=action_id,
                    action_type=ACTION_HOLD_CAPTURE,
                    status=STATUS_FAILED_SAFE,
                    started_at=action_started_at,
                    ended_at=now_iso(),
                    base_zero_ok_before=False,
                    published_cmd_vel=False,
                    capture_id=capture_id,
                    base_zero=base_zero,
                    error=error,
                )
            else:
                try:
                    meta = node.capture_to_dir(
                        capture_dir=capture_dir,
                        capture_id=capture_id,
                        action_id=action_id,
                        sequence=sequence,
                        timeout_s=args.capture_timeout_s,
                        depth_scale_m=args.depth_scale_m,
                    )
                    captures.append(meta)
                    risk_point = compute_mock_risk_point(
                        capture_dir=capture_dir,
                        bbox_raw=args.bbox,
                        output_path=capture_dir / "risk_point.json",
                    )
                    risk_points.append(risk_point)
                    evidence_paths = dict(meta.paths)
                    evidence_paths["risk_point"] = str(capture_dir / "risk_point.json")
                    result = ActionResult(
                        action_id=action_id,
                        action_type=ACTION_HOLD_CAPTURE,
                        status=STATUS_SUCCEEDED,
                        started_at=action_started_at,
                        ended_at=now_iso(),
                        base_zero_ok_before=True,
                        published_cmd_vel=False,
                        capture_id=capture_id,
                        capture_meta_path=str(capture_dir / "capture_meta.json"),
                        risk_point_path=str(capture_dir / "risk_point.json"),
                        evidence_paths=evidence_paths,
                        base_zero=base_zero,
                        details={
                            "depth_median_m": risk_point.depth_median_m,
                            "camera_point_xyz_m": risk_point.camera_point_xyz_m,
                        },
                    )
                except Exception as exc:  # noqa: BLE001 - continue with failed_safe result.
                    error = str(exc)
                    append_error(
                        errors,
                        "run_p4x_hold_capture_validation",
                        action_id,
                        capture_id,
                        error,
                        "capture_or_mock_risk",
                    )
                    write_json(
                        capture_dir / "errors.json",
                        [err for err in errors if err.get("capture_id") == capture_id],
                    )
                    result = ActionResult(
                        action_id=action_id,
                        action_type=ACTION_HOLD_CAPTURE,
                        status=STATUS_FAILED_SAFE,
                        started_at=action_started_at,
                        ended_at=now_iso(),
                        base_zero_ok_before=True,
                        published_cmd_vel=False,
                        capture_id=capture_id,
                        capture_meta_path=str(capture_dir / "capture_meta.json")
                        if (capture_dir / "capture_meta.json").exists()
                        else None,
                        risk_point_path=str(capture_dir / "risk_point.json")
                        if (capture_dir / "risk_point.json").exists()
                        else None,
                        base_zero=base_zero,
                        error=error,
                    )

            results.append(result)
            write_json(capture_dir / "action_result.json", result)
            status_rows.append(
                {
                    "sequence": sequence,
                    "action_id": action_id,
                    "capture_id": capture_id,
                    "status": result.status,
                    "base_zero_ok_before": result.base_zero_ok_before,
                    "capture_meta_path": result.capture_meta_path,
                    "risk_point_path": result.risk_point_path,
                    "error": result.error,
                }
            )
            write_status_csv(status_path, status_rows)
            write_json(errors_path, errors)
            write_episode_report(
                episode_path,
                episode_id,
                started_at,
                output_root,
                latest_state,
                actions,
                results,
                captures,
                risk_points,
                errors,
                args.count,
                min_successes,
            )
            if sequence < args.count and args.inter_capture_delay_s > 0.0:
                node.spin_for(args.inter_capture_delay_s)

        summary = episode_summary(args.count, results, min_successes)
        write_output_readme(output_root / "README.md", episode_id, summary, args)
        write_episode_report(
            episode_path,
            episode_id,
            started_at,
            output_root,
            latest_state,
            actions,
            results,
            captures,
            risk_points,
            errors,
            args.count,
            min_successes,
        )
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0 if summary["succeeded"] >= min_successes else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
