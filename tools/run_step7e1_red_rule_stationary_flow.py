#!/usr/bin/env python3
"""Step7-E1 stationary D435 red-rule trigger to arm no-load dry-run flow."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
DEFAULT_OUTPUT_ROOT = ROOT / "outputs" / "step7e1_red_rule_stationary_flow_v1"
PROTOCOL_VERSION = "step7e1_red_rule_stationary_flow_v1"

RISK_TRIGGER_SOURCE = "D435_red_color_rule"
DETECTION_MODE = "hsv_rule_based_red_color"
SELECTED_SEQUENCE = "arm_b3_8_step_safety_adjusted_no_load_sample"

ACTION_BASE_ZERO = "STEP7E1_LIVE_BASE_ZERO_GATE"
ACTION_D435_CAPTURE = "STEP7E1_D435_LIVE_CAPTURE"
ACTION_RED_RULE = "STEP7E1_D435_RED_RULE_TRIGGER"
ACTION_MAP_PROJECT = "STEP7E1_MAP_A0_PROJECTION"
ACTION_ARM_C0 = "STEP7E1_ARM_C0_CANDIDATE"
ACTION_ARM_C1 = "STEP7E1_ARM_C1_NO_LOAD_ONCE"


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def slug_time() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def next_available_dir(root: Path, prefix: str) -> Path:
    for index in range(1, 1000):
        candidate = root / f"{prefix}_{index:03d}"
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"no available output directory under {root} for prefix {prefix}")


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def run_command(
    name: str,
    command: Sequence[str],
    cwd: Path,
    output_dir: Path,
    errors: List[Dict[str, Any]],
) -> Dict[str, Any]:
    result = {
        "name": name,
        "command": list(command),
        "started_at": now_iso(),
        "ended_at": None,
        "returncode": None,
        "stdout_path": str(output_dir / f"{name}.stdout.txt"),
        "stderr_path": str(output_dir / f"{name}.stderr.txt"),
        "ok": False,
    }
    proc = subprocess.run(
        list(command),
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    (output_dir / f"{name}.stdout.txt").write_text(proc.stdout, encoding="utf-8")
    (output_dir / f"{name}.stderr.txt").write_text(proc.stderr, encoding="utf-8")
    result["ended_at"] = now_iso()
    result["returncode"] = proc.returncode
    result["ok"] = proc.returncode == 0
    if proc.returncode != 0:
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": name,
                "error": f"command failed with returncode={proc.returncode}",
                "command": list(command),
                "stdout_path": result["stdout_path"],
                "stderr_path": result["stderr_path"],
            }
        )
    return result


def read_json_or_error(path: Path, stage: str, errors: List[Dict[str, Any]]) -> Dict[str, Any]:
    try:
        data = load_json(path)
        if isinstance(data, dict):
            return data
        raise ValueError("JSON root is not an object")
    except Exception as exc:  # noqa: BLE001 - evidence parser must not crash caller.
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": stage,
                "error": str(exc),
                "path": str(path),
            }
        )
        return {}


def int_or_zero(value: Any) -> int:
    try:
        if value is None:
            return 0
        return int(value)
    except (TypeError, ValueError):
        return 0


def summarize_map(package: Dict[str, Any]) -> Dict[str, Any]:
    points = package.get("risk_map_points") or []
    return {
        "risk_map_points": len(points),
        "projected": sum(1 for point in points if point.get("projection_status") == "projected"),
        "projection_mode": package.get("projection_mode"),
        "tf_validated": package.get("tf_validated"),
        "slam_used": package.get("slam_used"),
        "navigation_used": package.get("navigation_used"),
    }


def summarize_arm_c0(package: Dict[str, Any]) -> Dict[str, Any]:
    summary = package.get("summary") or {}
    safety = package.get("safety_boundary") or {}
    return {
        "candidates": summary.get("candidates"),
        "succeeded_dry_run": summary.get("succeeded_dry_run"),
        "blocked": summary.get("blocked"),
        "hardware_executed": safety.get("hardware_executed"),
        "serial_port_opened": safety.get("serial_port_opened"),
        "serial_bytes_written": safety.get("serial_bytes_written"),
        "published_cmd_vel": safety.get("cmd_vel_published"),
        "contact_allowed": safety.get("contact_allowed"),
        "obstacle_removed": safety.get("obstacle_removed"),
    }


def summarize_arm_execution(report: Dict[str, Any]) -> Dict[str, Any]:
    summary = report.get("summary") or {}
    details = {}
    action_results = report.get("action_results") or []
    if action_results and isinstance(action_results[0], dict):
        details = action_results[0].get("details") or {}
    return {
        "status": summary.get("status"),
        "dry_run": summary.get("dry_run"),
        "base_zero_ok_before_arm": summary.get("base_zero_ok_before_arm"),
        "selected_action": details.get("selected_action"),
        "selected_sequence": details.get("selected_sequence"),
        "hardware_executed": bool(summary.get("hardware_executed") is True),
        "serial_port_opened": bool(summary.get("serial_port_opened") is True),
        "serial_bytes_written": int(summary.get("serial_bytes_written") or 0),
        "published_cmd_vel": bool(summary.get("published_cmd_vel") is True),
        "contact_allowed": bool(summary.get("contact_allowed") is True),
        "obstacle_removed": bool(summary.get("obstacle_removed") is True),
        "candidate_gate_passed": summary.get("candidate_gate_passed"),
        "confirmation_gate_passed": summary.get("confirmation_gate_passed"),
        "step_count": summary.get("step_count"),
        "step_success_count": summary.get("step_success_count"),
    }


def hardware_flags_requested(args: argparse.Namespace) -> bool:
    return any(
        (
            args.enable_hardware_write,
            args.confirm_map_gated_no_load,
            args.confirm_no_contact,
            args.confirm_base_zero_live,
            args.confirm_no_cmd_vel,
        )
    )


def hardware_flags_complete(args: argparse.Namespace) -> bool:
    return all(
        (
            args.enable_hardware_write,
            args.confirm_map_gated_no_load,
            args.confirm_no_contact,
            args.confirm_base_zero_live,
            args.confirm_no_cmd_vel,
        )
    )


def hardware_gate_errors(args: argparse.Namespace) -> List[Dict[str, Any]]:
    if not hardware_flags_requested(args):
        return []
    required = {
        "--enable-hardware-write": args.enable_hardware_write,
        "--confirm-map-gated-no-load": args.confirm_map_gated_no_load,
        "--confirm-no-contact": args.confirm_no_contact,
        "--confirm-base-zero-live": args.confirm_base_zero_live,
        "--confirm-no-cmd-vel": args.confirm_no_cmd_vel,
    }
    errors: List[Dict[str, Any]] = []
    missing = [name for name, enabled in required.items() if not enabled]
    if args.dry_run_arm:
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": "arm_hardware_gate",
                "error": "--dry-run-arm cannot be combined with hardware confirmation flags",
            }
        )
    if missing:
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": "arm_hardware_gate",
                "error": "hardware execution requires all confirmation flags",
                "missing_flags": missing,
            }
        )
    return errors


def action_record(
    action_id: str,
    action_type: str,
    requires_base_zero: bool,
    params: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "action_id": action_id,
        "action_type": action_type,
        "requested_at": now_iso(),
        "requires_base_zero": requires_base_zero,
        "publishes_cmd_vel": False,
        "reason": "Step7-E1 stationary red-rule trigger flow",
        "params": params,
    }


def result_record(
    action_id: str,
    action_type: str,
    status: str,
    base_zero_ok_before: Optional[bool],
    evidence_paths: Dict[str, Any],
    details: Dict[str, Any],
    error: Optional[str] = None,
) -> Dict[str, Any]:
    return {
        "action_id": action_id,
        "action_type": action_type,
        "status": status,
        "started_at": now_iso(),
        "ended_at": now_iso(),
        "base_zero_ok_before": base_zero_ok_before,
        "published_cmd_vel": False,
        "evidence_paths": evidence_paths,
        "details": details,
        "error": error,
    }


def build_red_capture_episode(
    output_dir: Path,
    run_id: str,
    capture_dir: Path,
    capture_meta: Dict[str, Any],
    risk_point: Dict[str, Any],
    detection: Dict[str, Any],
    expect_no_red: bool,
) -> Dict[str, Any]:
    capture_id = capture_meta.get("capture_id") or capture_dir.name
    action_id = f"{run_id}_capture"
    action = action_record(
        action_id,
        ACTION_D435_CAPTURE,
        True,
        {"capture_id": capture_id, "risk_trigger_source": RISK_TRIGGER_SOURCE},
    )
    red_detected = detection.get("red_object_detected") is True
    risk_point_present = bool(risk_point and red_detected)
    result = result_record(
        action_id,
        ACTION_D435_CAPTURE,
        "succeeded",
        True,
        {
            "capture_meta": str(capture_dir / "capture_meta.json"),
            "risk_point": str(capture_dir / "risk_point.json") if risk_point_present else None,
            "red_detection": str(capture_dir / "red_object_rule_detection.json"),
        },
        {
            "capture_id": capture_id,
            "red_object_detected": detection.get("red_object_detected"),
            "negative_control_expected": expect_no_red,
            "negative_control_pass": bool(expect_no_red and not red_detected),
            "risk_trigger_source": RISK_TRIGGER_SOURCE,
            "detection_mode": DETECTION_MODE,
            "model_used": False,
            "accuracy_claimed": False,
        },
    )
    report = {
        "episode_id": f"{run_id}_red_rule_capture",
        "episode_kind": "step7e1_red_rule_capture",
        "started_at": now_iso(),
        "ended_at": now_iso(),
        "protocol_version": "step7e1_red_rule_capture_v1",
        "policy_state": {
            "state_id": f"{run_id}_red_rule_capture_state",
            "timestamp": now_iso(),
            "base_zero_ok": True,
            "odom": capture_meta.get("odom"),
            "source": "run_step7e1_red_rule_stationary_flow",
        },
        "actions": [action],
        "action_results": [result],
        "captures": [
            {
                "capture_id": capture_id,
                "action_id": action_id,
                "timestamp": capture_meta.get("timestamp"),
                "topics": capture_meta.get("topics"),
                "paths": capture_meta.get("paths"),
                "rgb": capture_meta.get("rgb"),
                "depth": capture_meta.get("depth"),
                "camera_info": capture_meta.get("camera_info"),
                "odom": capture_meta.get("odom"),
            }
        ],
        "risk_points": [risk_point] if risk_point_present else [],
        "summary": {
            "status": "succeeded",
            "base_zero_ok_before_capture": True,
            "d435_live_capture_executed": True,
            "risk_point_generated": risk_point_present,
            "red_object_detected": detection.get("red_object_detected"),
            "negative_control_expected": expect_no_red,
            "negative_control_pass": bool(expect_no_red and not red_detected),
            "risk_trigger_source": RISK_TRIGGER_SOURCE,
            "detection_mode": DETECTION_MODE,
            "model_used": False,
            "accuracy_claimed": False,
            "bbox_xywh": detection.get("bbox_xywh"),
            "depth_median_m": detection.get("depth_median_m"),
            "bbox_valid_depth_ratio": detection.get("bbox_valid_depth_ratio"),
            "camera_point_xyz_m": detection.get("camera_point_xyz_m"),
            "published_cmd_vel": False,
        },
        "errors": [],
    }
    write_json(output_dir / "episode_report.json", report)
    return report


def build_episode_report(
    output_dir: Path,
    run_id: str,
    commands: List[Dict[str, Any]],
    base_zero: Dict[str, Any],
    red_capture_report: Dict[str, Any],
    detection: Dict[str, Any],
    risk_point: Dict[str, Any],
    map_package: Dict[str, Any],
    arm_c0_package: Dict[str, Any],
    arm_execution_report: Dict[str, Any],
    arm_mode: str,
    expect_no_red: bool,
    errors: List[Dict[str, Any]],
) -> Dict[str, Any]:
    base_zero_ok = (
        base_zero.get("valid_for_arm_c1_hardware") is True
        and base_zero.get("base_zero_ok_before_arm") is True
        and base_zero.get("published_cmd_vel") is False
    )
    red_detected = detection.get("red_object_detected") is True
    negative_control_pass = bool(expect_no_red and not red_detected and bool(red_capture_report))
    map_summary = summarize_map(map_package)
    arm_c0_summary = summarize_arm_c0(arm_c0_package)
    arm_execution_summary = summarize_arm_execution(arm_execution_report)
    map_ok = int(map_summary.get("risk_map_points") or 0) >= 1 and int(map_summary.get("projected") or 0) >= 1
    arm_c0_ok = (
        int_or_zero(arm_c0_summary.get("candidates")) >= 1
        and int_or_zero(arm_c0_summary.get("succeeded_dry_run")) >= 1
        and int_or_zero(arm_c0_summary.get("blocked")) == 0
    )
    arm_ok = arm_execution_summary.get("status") in ("succeeded", "succeeded_dry_run")
    if arm_mode == "hardware_once":
        arm_safety_ok = (
            arm_execution_summary.get("hardware_executed") is True
            and int(arm_execution_summary.get("serial_bytes_written") or 0) > 0
            and arm_execution_summary.get("published_cmd_vel") is False
            and arm_execution_summary.get("contact_allowed") is False
            and arm_execution_summary.get("obstacle_removed") is False
        )
    else:
        arm_safety_ok = (
            arm_execution_summary.get("hardware_executed") is False
            and arm_execution_summary.get("serial_bytes_written") == 0
            and arm_execution_summary.get("published_cmd_vel") is False
            and arm_execution_summary.get("contact_allowed") is False
            and arm_execution_summary.get("obstacle_removed") is False
        )
    positive_control_pass = bool(red_detected and map_ok and arm_c0_ok and arm_ok and arm_safety_ok)
    top_ok = bool(
        (negative_control_pass if expect_no_red else positive_control_pass)
        and base_zero_ok
        and not errors
    )

    actions: List[Dict[str, Any]] = []
    results: List[Dict[str, Any]] = []
    for action_id, action_type, requires_base_zero, status, details, evidence in (
        (
            f"{run_id}_base_zero",
            ACTION_BASE_ZERO,
            False,
            "succeeded" if base_zero_ok else "failed_safe",
            base_zero,
            {"base_zero_evidence": str(output_dir / "base_zero_live" / "base_zero_evidence.json")},
        ),
        (
            f"{run_id}_d435_capture",
            ACTION_D435_CAPTURE,
            True,
            "succeeded" if red_capture_report else "failed_safe",
            (red_capture_report.get("summary") or {}),
            {"red_capture_episode_report": str(output_dir / "d435_red_rule_capture" / "episode_report.json")},
        ),
        (
            f"{run_id}_red_rule",
            ACTION_RED_RULE,
            True,
            "succeeded" if (negative_control_pass if expect_no_red else red_detected) else "failed_safe",
            detection,
            {"red_detection": str(output_dir / "d435_red_rule_capture" / "captures" / f"{run_id}_red_capture" / "red_object_rule_detection.json")},
        ),
        (
            f"{run_id}_map_a0",
            ACTION_MAP_PROJECT,
            False,
            "succeeded" if map_ok else "blocked",
            map_summary,
            {"risk_map_points": str(output_dir / "map_projection" / "risk_map_points.json")},
        ),
        (
            f"{run_id}_arm_c0",
            ACTION_ARM_C0,
            True,
            "succeeded_dry_run" if arm_c0_ok else "blocked",
            arm_c0_summary,
            {"arm_c0_episode_report": str(output_dir / "arm_candidate" / "episode_report.json")},
        ),
        (
            f"{run_id}_arm_c1_no_load",
            ACTION_ARM_C1,
            True,
            arm_execution_summary.get("status") if arm_ok else "blocked",
            arm_execution_summary,
            {"arm_execution_episode_report": str(output_dir / "arm_execution" / "episode_report.json")},
        ),
    ):
        actions.append(action_record(action_id, action_type, requires_base_zero, {"risk_trigger_source": RISK_TRIGGER_SOURCE}))
        results.append(
            result_record(
                action_id,
                action_type,
                status,
                base_zero_ok if requires_base_zero else True,
                evidence,
                details,
                None if status in ("succeeded", "succeeded_dry_run", "blocked") else "Step7-E1 gate failed",
            )
        )

    capture_meta = {}
    raw_captures = red_capture_report.get("captures") or []
    captures = [dict(item) for item in raw_captures if isinstance(item, dict)]
    top_capture_action_id = f"{run_id}_d435_capture"
    for capture in captures:
        capture["action_id"] = top_capture_action_id
    if captures:
        capture_meta = captures[0]

    return {
        "episode_id": run_id,
        "episode_kind": "step7e1_red_rule_stationary_flow",
        "started_at": now_iso(),
        "ended_at": now_iso(),
        "protocol_version": PROTOCOL_VERSION,
        "policy_state": {
            "state_id": f"{run_id}_state",
            "timestamp": now_iso(),
            "base_zero_ok": base_zero_ok,
            "base_zero": base_zero,
            "odom": capture_meta.get("odom") or ((base_zero.get("base_zero") or {}).get("odom")),
            "front_min_range_m": ((base_zero.get("base_zero") or {}).get("front_min_range_m")),
            "front_p10_range_m": ((base_zero.get("base_zero") or {}).get("front_p10_range_m")),
            "source": "run_step7e1_red_rule_stationary_flow",
        },
        "actions": actions,
        "action_results": results,
        "captures": captures,
        "risk_points": [risk_point] if risk_point and red_detected else [],
        "step7e1_flow": {
            "commands": commands,
            "base_zero_evidence": base_zero,
            "red_detection": detection,
            "map_a0_summary": map_summary,
            "arm_c0_summary": arm_c0_summary,
            "arm_execution_summary": arm_execution_summary,
        },
            "summary": {
            "status": "succeeded" if top_ok else "failed_safe",
            "stationary": True,
            "arm_mode": arm_mode,
            "event_source": "D435_color_image",
            "risk_trigger_source": RISK_TRIGGER_SOURCE,
            "red_object_detected": red_detected,
            "negative_control_expected": expect_no_red,
            "negative_control_pass": negative_control_pass,
            "demo_fast_reuse_policy_base_zero": bool(
                base_zero.get("source_mode") == "guarded_policy_runner_final_base_zero_reuse"
                or (base_zero.get("checks") or {}).get("demo_fast_reuse_policy_base_zero") is True
            ),
            "detection_mode": DETECTION_MODE,
            "model_used": False,
            "accuracy_claimed": False,
            "bbox_xywh": detection.get("bbox_xywh"),
            "red_mask_path": ((detection.get("evidence_paths") or {}).get("red_mask")),
            "overlay_path": ((detection.get("evidence_paths") or {}).get("red_overlay")),
            "depth_median_m": detection.get("depth_median_m"),
            "bbox_valid_depth_ratio": detection.get("bbox_valid_depth_ratio"),
            "camera_point_xyz_m": detection.get("camera_point_xyz_m"),
            "base_zero_ok_before_capture": base_zero_ok,
            "base_zero_ok_before_arm": arm_execution_summary.get("base_zero_ok_before_arm"),
            "d435_live_capture_executed": bool(red_capture_report),
            "risk_point_generated": bool(risk_point and red_detected),
            "mock_risk_triggered": False,
            "risk_map_points": map_summary.get("risk_map_points"),
            "projected": map_summary.get("projected"),
            "arm_candidate_selected": arm_c0_ok,
            "arm_c0_candidates": arm_c0_summary.get("candidates"),
            "arm_c0_succeeded_dry_run": arm_c0_summary.get("succeeded_dry_run"),
            "arm_c0_blocked": arm_c0_summary.get("blocked"),
            "selected_sequence": arm_execution_summary.get("selected_sequence"),
            "arm_execution_status": arm_execution_summary.get("status") or ("skipped_negative_control" if negative_control_pass else None),
            "hardware_executed": arm_execution_summary.get("hardware_executed"),
            "serial_port_opened": arm_execution_summary.get("serial_port_opened"),
            "serial_bytes_written": arm_execution_summary.get("serial_bytes_written"),
            "published_cmd_vel": False,
            "published_cmd_vel_during_capture": False,
            "published_cmd_vel_during_arm": False,
            "contact_allowed": False,
            "obstacle_removed": False,
            "planned_final_pose": "6b",
            "final_pose_observed": None,
            "physical_issue_observed": None,
            "llm_used": False,
            "online_api_used": False,
            "local_model_used": False,
        },
        "errors": errors,
        "output_root": str(output_dir),
    }


def render_report(report: Dict[str, Any]) -> str:
    summary = report["summary"]
    title = "Step7-E1-B D435 Red-Rule Arm-C1 No-Load Flow" if summary.get("arm_mode") == "hardware_once" else "Step7-E1-A D435 Red-Rule Stationary Flow"
    return "\n".join(
        [
            f"# {title}",
            "",
            "## Summary",
            "",
            f"- episode_id: `{report.get('episode_id')}`",
            f"- status: `{summary.get('status')}`",
            f"- arm_mode: `{summary.get('arm_mode')}`",
            f"- risk_trigger_source: `{summary.get('risk_trigger_source')}`",
            f"- red_object_detected: `{summary.get('red_object_detected')}`",
            f"- detection_mode: `{summary.get('detection_mode')}`",
            f"- model_used: `{summary.get('model_used')}`",
            f"- accuracy_claimed: `{summary.get('accuracy_claimed')}`",
            f"- bbox_xywh: `{summary.get('bbox_xywh')}`",
            f"- depth_median_m: `{summary.get('depth_median_m')}`",
            f"- bbox_valid_depth_ratio: `{summary.get('bbox_valid_depth_ratio')}`",
            f"- camera_point_xyz_m: `{summary.get('camera_point_xyz_m')}`",
            f"- base_zero_ok_before_capture: `{summary.get('base_zero_ok_before_capture')}`",
            f"- risk_map_points/projected: `{summary.get('risk_map_points')}/{summary.get('projected')}`",
            f"- arm_candidate_selected: `{summary.get('arm_candidate_selected')}`",
            f"- arm_execution_status: `{summary.get('arm_execution_status')}`",
            f"- hardware_executed: `{summary.get('hardware_executed')}`",
            f"- serial_bytes_written: `{summary.get('serial_bytes_written')}`",
            f"- published_cmd_vel_during_capture: `{summary.get('published_cmd_vel_during_capture')}`",
            f"- published_cmd_vel_during_arm: `{summary.get('published_cmd_vel_during_arm')}`",
            f"- contact_allowed: `{summary.get('contact_allowed')}`",
            f"- obstacle_removed: `{summary.get('obstacle_removed')}`",
            "",
            "## Evidence",
            "",
            "- `base_zero_live/base_zero_evidence.json`",
            "- `d435_red_rule_capture/episode_report.json`",
            "- `d435_red_rule_capture/captures/*/red_object_rule_detection.json`",
            "- `d435_red_rule_capture/captures/*/risk_point.json`",
            "- `map_projection/risk_map_points.json`",
            "- `arm_candidate/episode_report.json`",
            "- `arm_execution/episode_report.json`",
            "- `llm_a_report/risk_report.md`",
            "- `episode_report.json`",
            "- `errors.json`",
            "",
            "## Claim Boundary",
            "",
            "- allowed: stationary D435 red-color rule trigger",
            "- allowed: depth median and approximate camera-frame risk point",
            "- allowed: approximate Map-A0 projection",
            "- allowed: Arm-C0/Arm-C1 dry-run no-load response, or one explicit Arm-C1 no-load hardware response when `hardware_executed=true`",
            "- disallowed: trained model inference or visual accuracy claim",
            "- disallowed: chassis motion, autonomous navigation, or path planning claim",
            "- disallowed: repeated arm hardware execution, grasping, contact, payload handling, or clearing",
            "- disallowed: LLM control of the robot",
            "",
        ]
    )


def render_readme(report: Dict[str, Any]) -> str:
    summary = report.get("summary") or {}
    title = "Step7-E1-B D435 Red-Rule Arm-C1 No-Load Flow" if summary.get("arm_mode") == "hardware_once" else "Step7-E1-A D435 Red-Rule Stationary Flow"
    return (
        f"# {title}\n\n"
        "Stationary live D435 capture, deterministic HSV red rule trigger, Map-A0 projection, "
        "Arm-C0 candidate generation, Arm-C1 no-load gate, and LLM-A deterministic report.\n\n"
        f"- status: `{summary.get('status')}`\n"
        f"- arm_mode: `{summary.get('arm_mode')}`\n"
        f"- red_object_detected: `{summary.get('red_object_detected')}`\n"
        f"- risk_trigger_source: `{summary.get('risk_trigger_source')}`\n"
        f"- hardware_executed: `{summary.get('hardware_executed')}`\n"
    )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--candidate-id", default="arm_c0_candidate_001")
    parser.add_argument(
        "--base-zero-evidence",
        default=None,
        help=(
            "Optional pre-generated live base-zero evidence JSON. "
            "When provided, Step7-E1 reuses this explicit evidence instead of "
            "creating a second ROS base-zero observer."
        ),
    )
    parser.add_argument("--base-zero-max-age-s", type=float, default=60.0)
    parser.add_argument("--serial-port", default="/dev/arm_bus")
    parser.add_argument("--capture-timeout-s", type=float, default=8.0)
    parser.add_argument("--min-red-area-px", type=int, default=80)
    parser.add_argument("--depth-scale-m", type=float, default=0.001)
    parser.add_argument("--expect-no-red", action="store_true")
    parser.add_argument("--dry-run-arm", action="store_true")
    parser.add_argument("--enable-hardware-write", action="store_true")
    parser.add_argument("--confirm-map-gated-no-load", action="store_true")
    parser.add_argument("--confirm-no-contact", action="store_true")
    parser.add_argument("--confirm-base-zero-live", action="store_true")
    parser.add_argument("--confirm-no-cmd-vel", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    run_id = f"step7e1_red_rule_{slug_time()}"
    gate_errors = hardware_gate_errors(args)
    hardware_enabled = hardware_flags_complete(args) and not gate_errors
    arm_mode = "hardware_once" if hardware_enabled else "dry_run"
    output_dir = (
        Path(str(args.output_dir).strip())
        if args.output_dir
        else next_available_dir(DEFAULT_OUTPUT_ROOT, "e1b_red_rule_arm_hw" if hardware_enabled else "e1a_red_rule_dryrun")
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    errors: List[Dict[str, Any]] = list(gate_errors)
    commands: List[Dict[str, Any]] = []
    python = args.python

    base_zero_dir = output_dir / "base_zero_live"
    provided_base_zero = Path(args.base_zero_evidence) if args.base_zero_evidence else None
    if provided_base_zero:
        base_zero_dir.mkdir(parents=True, exist_ok=True)
        target_base_zero = base_zero_dir / "base_zero_evidence.json"
        command_record = {
            "name": "01_base_zero_reused_evidence",
            "command": ["reuse-base-zero-evidence", str(provided_base_zero)],
            "started_at": now_iso(),
            "ended_at": None,
            "returncode": None,
            "stdout_path": str(output_dir / "01_base_zero_reused_evidence.stdout.txt"),
            "stderr_path": str(output_dir / "01_base_zero_reused_evidence.stderr.txt"),
            "ok": False,
        }
        try:
            target_base_zero.write_text(provided_base_zero.read_text(encoding="utf-8"), encoding="utf-8")
            (output_dir / "01_base_zero_reused_evidence.stdout.txt").write_text(
                f"reused_base_zero_evidence={provided_base_zero}\n", encoding="utf-8"
            )
            (output_dir / "01_base_zero_reused_evidence.stderr.txt").write_text("", encoding="utf-8")
            command_record["returncode"] = 0
            command_record["ok"] = True
        except Exception as exc:  # noqa: BLE001 - evidence copy failure must be reported.
            (output_dir / "01_base_zero_reused_evidence.stderr.txt").write_text(str(exc), encoding="utf-8")
            command_record["returncode"] = 1
            errors.append(
                {
                    "timestamp": now_iso(),
                    "stage": "base_zero_reused_evidence",
                    "error": str(exc),
                    "path": str(provided_base_zero),
                }
            )
        command_record["ended_at"] = now_iso()
        commands.append(command_record)
    else:
        commands.append(
            run_command(
                "01_base_zero_live",
                [
                    python,
                    str(TOOLS / "generate_arm_c1_base_zero_evidence.py"),
                    "--ros-live",
                    "--output-dir",
                    str(base_zero_dir),
                ],
                ROOT,
                output_dir,
                errors,
            )
        )
    base_zero = read_json_or_error(base_zero_dir / "base_zero_evidence.json", "base_zero_live", errors)
    base_zero_ok = (
        base_zero.get("valid_for_arm_c1_hardware") is True
        and base_zero.get("base_zero_ok_before_arm") is True
        and base_zero.get("published_cmd_vel") is False
    )

    capture_root = output_dir / "d435_red_rule_capture"
    capture_id = f"{run_id}_red_capture"
    if base_zero_ok and not errors:
        commands.append(
            run_command(
                "02_d435_live_capture_once",
                [
                    python,
                    str(TOOLS / "d435_capture_once.py"),
                    "--output-dir",
                    str(capture_root),
                    "--capture-id",
                    capture_id,
                    "--timeout-s",
                    str(args.capture_timeout_s),
                ],
                ROOT,
                output_dir,
                errors,
            )
        )
    else:
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": "base_zero_gate",
                "error": "base-zero evidence failed; skipping D435 red-rule capture",
            }
        )

    capture_dir = capture_root / "captures" / capture_id
    if capture_dir.exists() and not errors:
        red_detector_command = [
            python,
            str(TOOLS / "d435_red_rule_detector.py"),
            "--capture-dir",
            str(capture_dir),
            "--capture-id",
            capture_id,
            "--depth-scale-m",
            str(args.depth_scale_m),
            "--min-area-px",
            str(args.min_red_area_px),
        ]
        if args.expect_no_red:
            red_detector_command.append("--allow-no-detection")
        commands.append(
            run_command(
                "03_d435_red_rule_detector",
                red_detector_command,
                ROOT,
                output_dir,
                errors,
            )
        )

    capture_meta = read_json_or_error(capture_dir / "capture_meta.json", "capture_meta", errors)
    detection = read_json_or_error(capture_dir / "red_object_rule_detection.json", "red_rule_detection", errors)
    risk_point = read_json_or_error(capture_dir / "risk_point.json", "red_rule_risk_point", errors)
    red_capture_report: Dict[str, Any] = {}
    if capture_meta and detection and risk_point and not errors:
        red_capture_report = build_red_capture_episode(
            output_dir=capture_root,
            run_id=run_id,
            capture_dir=capture_dir,
            capture_meta=capture_meta,
            risk_point=risk_point,
            detection=detection,
            expect_no_red=args.expect_no_red,
        )

    map_dir = output_dir / "map_projection"
    red_detected = detection.get("red_object_detected") is True
    negative_control_pass = bool(args.expect_no_red and red_capture_report and not red_detected)
    if red_capture_report and red_detected and not args.expect_no_red and not errors:
        commands.append(
            run_command(
                "04_map_a0_projection",
                [
                    python,
                    str(TOOLS / "project_risk_point_to_map.py"),
                    "--episode-report",
                    str(capture_root / "episode_report.json"),
                    "--output-dir",
                    str(map_dir),
                ],
                ROOT,
                output_dir,
                errors,
            )
        )
    map_package = (
        read_json_or_error(map_dir / "risk_map_points.json", "map_a0_projection", errors)
        if red_detected and not args.expect_no_red
        else {
            "risk_map_points": [],
            "projection_mode": None,
            "tf_validated": False,
            "slam_used": False,
            "navigation_used": False,
        }
    )

    arm_c0_dir = output_dir / "arm_candidate"
    if map_package and red_detected and not args.expect_no_red and not errors:
        commands.append(
            run_command(
                "05_arm_c0_dryrun",
                [
                    python,
                    str(TOOLS / "generate_arm_c0_map_to_arm_dryrun.py"),
                    "--risk-map-points",
                    str(map_dir / "risk_map_points.json"),
                    "--output-dir",
                    str(arm_c0_dir),
                ],
                ROOT,
                output_dir,
                errors,
            )
        )
    arm_c0_package = (
        read_json_or_error(arm_c0_dir / "map_gated_arm_candidates.json", "arm_c0_dryrun", errors)
        if red_detected and not args.expect_no_red
        else {
            "summary": {"candidates": 0, "succeeded_dry_run": 0, "blocked": 0},
            "safety_boundary": {
                "hardware_executed": False,
                "serial_port_opened": False,
                "serial_bytes_written": 0,
                "cmd_vel_published": False,
                "contact_allowed": False,
                "obstacle_removed": False,
            },
        }
    )

    arm_execution_dir = output_dir / "arm_execution"
    arm_execution_report: Dict[str, Any] = {}
    if arm_c0_package and red_detected and not args.expect_no_red and not errors:
        arm_command = [
            python,
            str(TOOLS / "run_arm_c1_map_gated_no_load_once.py"),
            "--arm-c0-episode-report",
            str(arm_c0_dir / "episode_report.json"),
            "--candidate-id",
            args.candidate_id,
            "--base-zero-evidence",
            str(base_zero_dir / "base_zero_evidence.json"),
            "--base-zero-max-age-s",
            str(args.base_zero_max_age_s),
            "--serial-port",
            args.serial_port,
            "--output-dir",
            str(arm_execution_dir),
        ]
        if hardware_enabled:
            arm_command.extend(
                [
                    "--enable-hardware-write",
                    "--confirm-map-gated-no-load",
                    "--confirm-no-contact",
                    "--confirm-base-zero-live",
                    "--confirm-no-cmd-vel",
                ]
            )
        commands.append(
            run_command(
                "06_arm_c1_no_load_gate",
                arm_command,
                ROOT,
                output_dir,
                errors,
            )
        )
        arm_execution_report = read_json_or_error(
            arm_execution_dir / "episode_report.json", "arm_c1_dryrun_gate", errors
        )
    elif negative_control_pass:
        arm_execution_report = {
            "summary": {
                "status": "skipped_negative_control",
                "dry_run": True,
                "base_zero_ok_before_arm": True,
                "hardware_executed": False,
                "serial_port_opened": False,
                "serial_bytes_written": 0,
                "published_cmd_vel": False,
                "contact_allowed": False,
                "obstacle_removed": False,
            },
            "action_results": [
                {
                    "details": {
                        "selected_action": None,
                        "selected_sequence": None,
                    }
                }
            ],
        }

    report = build_episode_report(
        output_dir=output_dir,
        run_id=run_id,
        commands=commands,
        base_zero=base_zero,
        red_capture_report=red_capture_report,
        detection=detection,
        risk_point=risk_point,
        map_package=map_package,
        arm_c0_package=arm_c0_package,
        arm_execution_report=arm_execution_report,
        arm_mode=arm_mode,
        expect_no_red=args.expect_no_red,
        errors=errors,
    )
    write_json(output_dir / "episode_report.json", report)
    write_json(output_dir / "errors.json", errors)
    write_text(output_dir / "step7e1_report.md", render_report(report))
    write_text(output_dir / "README.md", render_readme(report))

    llm_dir = output_dir / "llm_a_report"
    commands.append(
        run_command(
            "07_llm_a_report",
            [
                python,
                str(TOOLS / "generate_llm_a_risk_report.py"),
                "--episode-report",
                str(output_dir / "episode_report.json"),
                "--output-dir",
                str(llm_dir),
            ],
            ROOT,
            output_dir,
            errors,
        )
    )
    if errors:
        report["errors"] = errors
        report["summary"]["status"] = "failed_safe"
        write_json(output_dir / "episode_report.json", report)
        write_json(output_dir / "errors.json", errors)

    result = {
        "ok": report.get("summary", {}).get("status") == "succeeded" and not errors,
        "episode_id": report.get("episode_id"),
        "status": report.get("summary", {}).get("status"),
        "risk_trigger_source": report.get("summary", {}).get("risk_trigger_source"),
        "red_object_detected": report.get("summary", {}).get("red_object_detected"),
        "negative_control_expected": report.get("summary", {}).get("negative_control_expected"),
        "negative_control_pass": report.get("summary", {}).get("negative_control_pass"),
        "arm_mode": report.get("summary", {}).get("arm_mode"),
        "detection_mode": report.get("summary", {}).get("detection_mode"),
        "bbox_xywh": report.get("summary", {}).get("bbox_xywh"),
        "depth_median_m": report.get("summary", {}).get("depth_median_m"),
        "bbox_valid_depth_ratio": report.get("summary", {}).get("bbox_valid_depth_ratio"),
        "risk_map_points": report.get("summary", {}).get("risk_map_points"),
        "projected": report.get("summary", {}).get("projected"),
        "arm_candidate_selected": report.get("summary", {}).get("arm_candidate_selected"),
        "arm_execution_status": report.get("summary", {}).get("arm_execution_status"),
        "hardware_executed": report.get("summary", {}).get("hardware_executed"),
        "serial_bytes_written": report.get("summary", {}).get("serial_bytes_written"),
        "published_cmd_vel": report.get("summary", {}).get("published_cmd_vel"),
        "contact_allowed": report.get("summary", {}).get("contact_allowed"),
        "obstacle_removed": report.get("summary", {}).get("obstacle_removed"),
        "errors": len(errors),
        "output_dir": str(output_dir),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
