#!/usr/bin/env python3
"""Step7-C guarded D435 mock-risk to arm no-load integration runner.

This runner assumes the guarded stack and D435 ROS topics are available. It
does not publish cmd_vel. By default it does not open any serial port and does
not control the mechanical arm.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
DEFAULT_OUTPUT_ROOT = ROOT / "outputs" / "step7c_guarded_d435_mockrisk_arm_noload_v1"
PROTOCOL_VERSION = "step7c_guarded_d435_mockrisk_arm_noload_v1"
ACTION_BASE_ZERO = "STEP7C_LIVE_BASE_ZERO_GATE"
ACTION_HOLD_CAPTURE = "STEP7C_LIVE_D435_HOLD_CAPTURE"
ACTION_MOCK_RISK = "STEP7C_MOCK_RISK_TRIGGER"
ACTION_MAP_PROJECT = "STEP7C_MAP_A0_LIVE_PROJECTION"
ACTION_ARM_C0 = "STEP7C_ARM_C0_CANDIDATE"
ACTION_ARM_EXECUTION = "STEP7C_ARM_C1_NO_LOAD_ONCE"
SELECTED_SEQUENCE = "arm_b3_8_step_safety_adjusted_no_load_sample"


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
    started_at = now_iso()
    result = {
        "name": name,
        "command": list(command),
        "started_at": started_at,
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


def summarize_p4x(report: Dict[str, Any]) -> Dict[str, Any]:
    summary = report.get("summary") or {}
    action_results = report.get("action_results") or []
    return {
        "episode_id": report.get("episode_id"),
        "status": summary.get("status") or report.get("status"),
        "requested_captures": summary.get("requested_captures"),
        "succeeded": summary.get("succeeded"),
        "failed_safe": summary.get("failed_safe"),
        "published_cmd_vel": summary.get("published_cmd_vel"),
        "base_zero_ok_before_all": bool(action_results)
        and all(result.get("base_zero_ok_before") is True for result in action_results),
        "captures": len(report.get("captures") or []),
        "risk_points": len(report.get("risk_points") or []),
        "errors": len(report.get("errors") or []),
    }


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
        "missing_required_field": sum(
            1 for point in points if point.get("projection_status") == "missing_required_field"
        ),
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
        "base_zero_required": safety.get("base_zero_required"),
        "base_zero_checked": safety.get("base_zero_checked"),
    }


def summarize_arm_execution(report: Dict[str, Any]) -> Dict[str, Any]:
    summary = report.get("summary") or {}
    action_results = report.get("action_results") or []
    details = {}
    if action_results and isinstance(action_results[0], dict):
        details = action_results[0].get("details") or {}
    return {
        "status": summary.get("status"),
        "dry_run": summary.get("dry_run"),
        "candidate_id": summary.get("candidate_id"),
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


def first_risk_point(report: Dict[str, Any]) -> Dict[str, Any]:
    risk_points = report.get("risk_points") or []
    if risk_points and isinstance(risk_points[0], dict):
        return risk_points[0]
    for result in report.get("action_results") or []:
        if isinstance(result, dict):
            details = result.get("details") or {}
            if isinstance(details.get("risk_point"), dict):
                return details["risk_point"]
    return {}


def write_mock_risk_summary(output_dir: Path, p4x_report: Dict[str, Any]) -> Dict[str, Any]:
    risk = first_risk_point(p4x_report)
    summary = {
        "generated_at": now_iso(),
        "mock_risk_triggered": bool(risk),
        "risk_point_generated": bool(risk),
        "risk_point_id": risk.get("risk_point_id"),
        "capture_id": risk.get("capture_id"),
        "label": risk.get("label"),
        "risk_category": risk.get("category") or risk.get("risk_category"),
        "depth_median_m": risk.get("depth_median_m"),
        "camera_point_xyz_m": risk.get("camera_point_xyz_m"),
        "depth_scale_m": risk.get("depth_scale_m"),
        "bbox_valid_depth_ratio": risk.get("bbox_valid_depth_ratio"),
        "evidence_paths": risk.get("evidence_paths") or {},
        "source": "run_p4x_hold_capture_validation.py mock risk detector output",
    }
    write_json(output_dir / "mock_risk_summary.json", summary)
    lines = [
        "# Mock Risk Trigger",
        "",
        f"- mock_risk_triggered: `{summary['mock_risk_triggered']}`",
        f"- risk_point_id: `{summary.get('risk_point_id')}`",
        f"- capture_id: `{summary.get('capture_id')}`",
        f"- label/category: `{summary.get('label') or summary.get('risk_category')}`",
        f"- depth_median_m: `{summary.get('depth_median_m')}`",
        "",
        "This stage records a deterministic mock risk trigger from the live D435 HOLD_CAPTURE evidence.",
        "It does not claim real visual recognition accuracy.",
        "",
    ]
    write_text(output_dir / "README.md", "\n".join(lines))
    return summary


def action_record(
    action_id: str,
    action_type: str,
    requested_at: str,
    requires_base_zero: bool,
    params: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "action_id": action_id,
        "action_type": action_type,
        "requested_at": requested_at,
        "requires_base_zero": requires_base_zero,
        "publishes_cmd_vel": False,
        "reason": "Step7-C guarded D435 mock-risk arm no-load integration",
        "params": params,
    }


def result_record(
    action_id: str,
    action_type: str,
    status: str,
    started_at: str,
    base_zero_ok_before: Optional[bool],
    evidence_paths: Dict[str, Any],
    details: Dict[str, Any],
    error: Optional[str] = None,
) -> Dict[str, Any]:
    return {
        "action_id": action_id,
        "action_type": action_type,
        "status": status,
        "started_at": started_at,
        "ended_at": now_iso(),
        "base_zero_ok_before": base_zero_ok_before,
        "published_cmd_vel": False,
        "evidence_paths": evidence_paths,
        "details": details,
        "error": error,
    }


def build_episode_report(
    output_dir: Path,
    run_id: str,
    commands: List[Dict[str, Any]],
    base_zero: Dict[str, Any],
    p4x_report: Dict[str, Any],
    mock_risk_summary: Dict[str, Any],
    map_package: Dict[str, Any],
    arm_c0_package: Dict[str, Any],
    arm_execution_report: Dict[str, Any],
    arm_mode: str,
    errors: List[Dict[str, Any]],
) -> Dict[str, Any]:
    started_at = now_iso()
    base_zero_ok = base_zero.get("base_zero_ok_before_arm") is True
    p4x_summary = summarize_p4x(p4x_report)
    risk_point_generated = mock_risk_summary.get("risk_point_generated") is True
    map_summary = summarize_map(map_package)
    arm_c0_summary = summarize_arm_c0(arm_c0_package)
    arm_execution_summary = summarize_arm_execution(arm_execution_report)

    actions: List[Dict[str, Any]] = []
    results: List[Dict[str, Any]] = []

    base_action_id = f"{run_id}_base_zero"
    base_started_at = now_iso()
    actions.append(
        action_record(
            base_action_id,
            ACTION_BASE_ZERO,
            base_started_at,
            False,
            {"source_mode": "ros_live_readonly"},
        )
    )
    results.append(
        result_record(
            base_action_id,
            ACTION_BASE_ZERO,
            "succeeded" if base_zero_ok else "failed_safe",
            base_started_at,
            base_zero_ok,
            {"base_zero_evidence": str(output_dir / "base_zero_live" / "base_zero_evidence.json")},
            base_zero,
            None if base_zero_ok else "live base_zero evidence did not pass",
        )
    )

    p4x_action_id = f"{run_id}_hold_capture"
    p4x_started_at = now_iso()
    p4x_ok = (
        p4x_summary.get("succeeded") == 1
        and p4x_summary.get("failed_safe") in (0, None)
        and p4x_summary.get("base_zero_ok_before_all") is True
        and p4x_summary.get("published_cmd_vel") is False
    )
    actions.append(
        action_record(
            p4x_action_id,
            ACTION_HOLD_CAPTURE,
            p4x_started_at,
            True,
            {"count": 1, "source": "run_p4x_hold_capture_validation"},
        )
    )
    results.append(
        result_record(
            p4x_action_id,
            ACTION_HOLD_CAPTURE,
            "succeeded" if p4x_ok else "failed_safe",
            p4x_started_at,
            p4x_summary.get("base_zero_ok_before_all"),
            {"p4x_episode_report": str(output_dir / "d435_hold_capture" / "episode_report.json")},
            p4x_summary,
            None if p4x_ok else "live P4-X HOLD_CAPTURE did not meet Step7-C gate",
        )
    )

    risk_action_id = f"{run_id}_mock_risk"
    risk_started_at = now_iso()
    actions.append(
        action_record(
            risk_action_id,
            ACTION_MOCK_RISK,
            risk_started_at,
            True,
            {"source": "mock_risk_detector", "real_visual_recognition_claimed": False},
        )
    )
    results.append(
        result_record(
            risk_action_id,
            ACTION_MOCK_RISK,
            "succeeded" if risk_point_generated else "failed_safe",
            risk_started_at,
            base_zero_ok,
            {
                "mock_risk_summary": str(output_dir / "mock_risk" / "mock_risk_summary.json"),
                "source_p4x_episode_report": str(output_dir / "d435_hold_capture" / "episode_report.json"),
            },
            mock_risk_summary,
            None if risk_point_generated else "mock risk point was not generated",
        )
    )

    map_action_id = f"{run_id}_map_a0"
    map_started_at = now_iso()
    map_ok = map_summary.get("risk_map_points", 0) >= 1 and map_summary.get("projected", 0) >= 1
    actions.append(
        action_record(
            map_action_id,
            ACTION_MAP_PROJECT,
            map_started_at,
            False,
            {"projection_mode": map_summary.get("projection_mode")},
        )
    )
    results.append(
        result_record(
            map_action_id,
            ACTION_MAP_PROJECT,
            "succeeded" if map_ok else "failed_safe",
            map_started_at,
            base_zero_ok,
            {"risk_map_points": str(output_dir / "map_projection" / "risk_map_points.json")},
            map_summary,
            None if map_ok else "Map-A0 live projection produced no projected risk points",
        )
    )

    arm_c0_action_id = f"{run_id}_arm_c0"
    arm_c0_started_at = now_iso()
    arm_c0_ok = (
        int_or_zero(arm_c0_summary.get("candidates")) >= 1
        and int_or_zero(arm_c0_summary.get("succeeded_dry_run")) >= 1
        and int_or_zero(arm_c0_summary.get("blocked")) == 0
    )
    actions.append(
        action_record(
            arm_c0_action_id,
            ACTION_ARM_C0,
            arm_c0_started_at,
            True,
            {"selected_action_policy": "ARM_SAMPLE_NO_LOAD", "hardware_execution_allowed": False},
        )
    )
    results.append(
        result_record(
            arm_c0_action_id,
            ACTION_ARM_C0,
            "succeeded_dry_run" if arm_c0_ok else "blocked",
            arm_c0_started_at,
            base_zero_ok,
            {"arm_c0_episode_report": str(output_dir / "arm_candidate" / "episode_report.json")},
            arm_c0_summary,
            None if arm_c0_ok else "Arm-C0 live dry-run candidate generation blocked",
        )
    )

    arm_action_id = f"{run_id}_arm_execution"
    arm_started_at = now_iso()
    arm_status = arm_execution_summary.get("status")
    arm_ok = arm_status in ("succeeded", "succeeded_dry_run")
    actions.append(
        action_record(
            arm_action_id,
            ACTION_ARM_EXECUTION,
            arm_started_at,
            True,
            {
                "candidate_id": arm_execution_summary.get("candidate_id"),
                "arm_mode": arm_mode,
                "selected_action": arm_execution_summary.get("selected_action"),
                "selected_sequence": arm_execution_summary.get("selected_sequence"),
                "hardware_execution_allowed": arm_mode == "hardware_once",
            },
        )
    )
    results.append(
        result_record(
            arm_action_id,
            ACTION_ARM_EXECUTION,
            arm_status if arm_ok else "failed_safe",
            arm_started_at,
            arm_execution_summary.get("base_zero_ok_before_arm"),
            {"arm_execution_episode_report": str(output_dir / "arm_execution" / "episode_report.json")},
            arm_execution_summary,
            None if arm_ok else "Arm-C1 no-load gate did not pass",
        )
    )

    arm_safety_ok = (
        arm_execution_summary.get("published_cmd_vel") is False
        and arm_execution_summary.get("contact_allowed") is False
        and arm_execution_summary.get("obstacle_removed") is False
        and arm_execution_summary.get("selected_sequence") == SELECTED_SEQUENCE
    )
    top_ok = bool(
        base_zero_ok
        and p4x_ok
        and risk_point_generated
        and map_ok
        and arm_c0_ok
        and arm_ok
        and arm_safety_ok
        and not errors
    )

    return {
        "episode_id": run_id,
        "episode_kind": "step7c_guarded_d435_mockrisk_arm_noload",
        "started_at": started_at,
        "ended_at": now_iso(),
        "protocol_version": PROTOCOL_VERSION,
        "policy_state": {
            "state_id": f"{run_id}_state_live",
            "timestamp": now_iso(),
            "base_zero_ok": base_zero_ok,
            "base_zero": base_zero,
            "odom": ((base_zero.get("base_zero") or {}).get("odom")),
            "front_min_range_m": ((base_zero.get("base_zero") or {}).get("front_min_range_m")),
            "front_p10_range_m": ((base_zero.get("base_zero") or {}).get("front_p10_range_m")),
            "source": "run_step7c_guarded_d435_mockrisk_arm_noload",
            "notes": [
                "live stationary integration",
                "guarded stack must be running before this script starts",
                "this script does not publish cmd_vel",
                "Arm-C1 dry-run is executed by default",
                "Arm-C1 hardware is disabled unless all explicit confirmation flags are provided",
            ],
        },
        "actions": actions,
        "action_results": results,
        "step7c_flow": {
            "commands": commands,
            "base_zero_evidence": base_zero,
            "p4x_summary": p4x_summary,
            "mock_risk_summary": mock_risk_summary,
            "map_a0_summary": map_summary,
            "arm_c0_summary": arm_c0_summary,
            "arm_execution_summary": arm_execution_summary,
        },
        "summary": {
            "status": "succeeded" if top_ok else "failed_safe",
            "live_stationary": True,
            "arm_mode": arm_mode,
            "base_zero_ok_before_capture": base_zero_ok,
            "base_zero_ok_before_arm": arm_execution_summary.get("base_zero_ok_before_arm"),
            "d435_live_capture_executed": p4x_ok,
            "risk_point_generated": risk_point_generated,
            "mock_risk_triggered": mock_risk_summary.get("mock_risk_triggered") is True,
            "risk_map_points": map_summary.get("risk_map_points"),
            "projected": map_summary.get("projected"),
            "arm_candidate_selected": arm_c0_ok,
            "arm_c0_candidates": arm_c0_summary.get("candidates"),
            "arm_c0_succeeded_dry_run": arm_c0_summary.get("succeeded_dry_run"),
            "arm_c0_blocked": arm_c0_summary.get("blocked"),
            "selected_sequence": arm_execution_summary.get("selected_sequence"),
            "arm_execution_status": arm_execution_summary.get("status"),
            "arm_c1_hardware_executed": bool(arm_execution_summary.get("hardware_executed") is True),
            "hardware_executed": bool(arm_execution_summary.get("hardware_executed") is True),
            "serial_port_opened": bool(arm_execution_summary.get("serial_port_opened") is True),
            "serial_bytes_written": int(arm_execution_summary.get("serial_bytes_written") or 0),
            "published_cmd_vel": bool(arm_execution_summary.get("published_cmd_vel") is True),
            "contact_allowed": False,
            "obstacle_removed": False,
            "planned_final_pose": "6b",
            "final_pose_observed": None,
            "physical_issue_observed": None,
            "base_zero_required": True,
            "base_zero_checked": True,
            "llm_used": False,
            "online_api_used": False,
            "local_model_used": False,
        },
        "errors": errors,
        "output_root": str(output_dir),
    }


def render_report(report: Dict[str, Any]) -> str:
    summary = report["summary"]
    lines = [
        "# Step7-C Guarded D435 Mock-Risk Arm No-Load Flow",
        "",
        "## Summary",
        "",
        f"- episode_id: `{report.get('episode_id')}`",
        f"- status: `{summary.get('status')}`",
        f"- arm_mode: `{summary.get('arm_mode')}`",
        f"- base_zero_ok_before_capture: `{summary.get('base_zero_ok_before_capture')}`",
        f"- base_zero_ok_before_arm: `{summary.get('base_zero_ok_before_arm')}`",
        f"- d435_live_capture_executed: `{summary.get('d435_live_capture_executed')}`",
        f"- risk_point_generated: `{summary.get('risk_point_generated')}`",
        f"- mock_risk_triggered: `{summary.get('mock_risk_triggered')}`",
        f"- risk_map_points/projected: `{summary.get('risk_map_points')}/{summary.get('projected')}`",
        f"- arm_candidate_selected: `{summary.get('arm_candidate_selected')}`",
        f"- arm_c0 candidates/succeeded/blocked: `{summary.get('arm_c0_candidates')}/{summary.get('arm_c0_succeeded_dry_run')}/{summary.get('arm_c0_blocked')}`",
        f"- selected_sequence: `{summary.get('selected_sequence')}`",
        f"- arm_execution_status: `{summary.get('arm_execution_status')}`",
        f"- hardware_executed: `{summary.get('hardware_executed')}`",
        f"- published_cmd_vel: `{summary.get('published_cmd_vel')}`",
        f"- serial_port_opened: `{summary.get('serial_port_opened')}`",
        f"- serial_bytes_written: `{summary.get('serial_bytes_written')}`",
        f"- contact_allowed: `{summary.get('contact_allowed')}`",
        f"- obstacle_removed: `{summary.get('obstacle_removed')}`",
        f"- planned_final_pose: `{summary.get('planned_final_pose')}`",
        "",
        "## Evidence",
        "",
        "- `base_zero_live/base_zero_evidence.json`",
        "- `d435_hold_capture/episode_report.json`",
        "- `mock_risk/mock_risk_summary.json`",
        "- `map_projection/risk_map_points.json`",
        "- `arm_candidate/episode_report.json`",
        "- `arm_execution/episode_report.json`",
        "- `llm_a_report/risk_report.md`",
        "- `episode_report.json`",
        "- `step7c_report.md`",
        "- `errors.json`",
        "",
        "## Claim Boundary",
        "",
        "- allowed: live stationary base-zero gate and D435 HOLD_CAPTURE evidence chain",
        "- allowed: mock anomaly trigger generated from live D435 evidence",
        "- allowed: Map-A0 live projection from the newly captured risk_point",
        "- allowed: map-gated no-load arm response in dry-run mode, or one explicit hardware no-load execution only when `hardware_executed=true`",
        "- disallowed: chassis motion during Step7-C",
        "- disallowed: autonomous navigation or path planning success",
        "- disallowed: real visual detection accuracy claims",
        "- disallowed: grasping, contact, payload handling, or obstacle removal",
        "- disallowed: LLM control of the robot",
        "",
    ]
    return "\n".join(lines)


def render_readme(report: Dict[str, Any]) -> str:
    return f"""# Step7-C Guarded D435 Mock-Risk Arm No-Load Flow

This directory contains one guarded live stationary integration run.

The runner does not publish `cmd_vel`. By default it does not open serial ports
or control the arm. Hardware no-load execution requires all explicit confirmation
flags and remains no-contact/no-load only.

Status: `{report.get('summary', {}).get('status')}`
Arm mode: `{report.get('summary', {}).get('arm_mode')}`
"""


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--candidate-id", default="arm_c0_candidate_001")
    parser.add_argument("--base-zero-max-age-s", type=float, default=60.0)
    parser.add_argument("--serial-port", default="/dev/arm_bus")
    parser.add_argument(
        "--dry-run-arm",
        action="store_true",
        help="Keep the arm in dry-run mode. This is also the default when no hardware flags are present.",
    )
    parser.add_argument("--enable-hardware-write", action="store_true")
    parser.add_argument("--confirm-map-gated-no-load", action="store_true")
    parser.add_argument("--confirm-no-contact", action="store_true")
    parser.add_argument("--confirm-base-zero-live", action="store_true")
    parser.add_argument("--confirm-no-cmd-vel", action="store_true")
    return parser.parse_args(argv)


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
    missing = [name for name, value in required.items() if not value]
    errors: List[Dict[str, Any]] = []
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


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    run_id = f"step7c_guarded_mockrisk_{slug_time()}"
    gate_errors = hardware_gate_errors(args)
    hardware_enabled = hardware_flags_complete(args) and not gate_errors
    arm_mode = "hardware_once" if hardware_enabled else "dry_run"
    output_dir = (
        Path(str(args.output_dir).strip())
        if args.output_dir
        else next_available_dir(DEFAULT_OUTPUT_ROOT, "hw" if hardware_enabled else "dryrun")
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    errors: List[Dict[str, Any]] = list(gate_errors)
    commands: List[Dict[str, Any]] = []
    python = args.python

    base_zero_dir = output_dir / "base_zero_live"
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

    p4x_dir = output_dir / "d435_hold_capture"
    if base_zero_ok and not errors:
        commands.append(
            run_command(
                "02_p4x_live_hold_capture",
                [
                    python,
                    str(TOOLS / "run_p4x_hold_capture_validation.py"),
                    "--count",
                    "1",
                    "--min-successes",
                    "1",
                    "--episode-id",
                    f"{run_id}_hold_capture",
                    "--output-dir",
                    str(p4x_dir),
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
                "error": "base_zero evidence failed or previous gate failed; skipping live HOLD_CAPTURE",
            }
        )
    p4x_report = read_json_or_error(p4x_dir / "episode_report.json", "p4x_live_hold_capture", errors)
    mock_risk_summary: Dict[str, Any] = {}
    if p4x_report:
        mock_risk_summary = write_mock_risk_summary(output_dir / "mock_risk", p4x_report)

    map_dir = output_dir / "map_projection"
    if p4x_report and not errors:
        commands.append(
            run_command(
                "03_map_a0_live_projection",
                [
                    python,
                    str(TOOLS / "project_risk_point_to_map.py"),
                    "--episode-report",
                    str(p4x_dir / "episode_report.json"),
                    "--output-dir",
                    str(map_dir),
                ],
                ROOT,
                output_dir,
                errors,
            )
        )
    map_package = read_json_or_error(map_dir / "risk_map_points.json", "map_a0_live_projection", errors)

    arm_c0_dir = output_dir / "arm_candidate"
    if map_package and not errors:
        commands.append(
            run_command(
                "04_arm_c0_live_dryrun",
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
    arm_c0_package = read_json_or_error(arm_c0_dir / "map_gated_arm_candidates.json", "arm_c0_live_dryrun", errors)

    arm_execution_report: Dict[str, Any] = {}
    arm_execution_dir = output_dir / "arm_execution"
    if arm_c0_package and not errors:
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
            str(args.serial_port),
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
                "05_arm_c1_no_load_gate",
                arm_command,
                ROOT,
                output_dir,
                errors,
            )
        )
        arm_execution_report = read_json_or_error(
            arm_execution_dir / "episode_report.json", "arm_c1_no_load_gate", errors
        )

    report = build_episode_report(
        output_dir=output_dir,
        run_id=run_id,
        commands=commands,
        base_zero=base_zero,
        p4x_report=p4x_report,
        mock_risk_summary=mock_risk_summary,
        map_package=map_package,
        arm_c0_package=arm_c0_package,
        arm_execution_report=arm_execution_report,
        arm_mode=arm_mode,
        errors=errors,
    )
    write_json(output_dir / "episode_report.json", report)
    write_json(output_dir / "errors.json", errors)
    write_text(output_dir / "step7c_report.md", render_report(report))
    write_text(output_dir / "README.md", render_readme(report))

    llm_dir = output_dir / "llm_a_report"
    commands.append(
        run_command(
            "06_llm_a_report",
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
        write_json(output_dir / "errors.json", errors)
        report["errors"] = errors
        report["summary"]["status"] = "failed_safe"
        write_json(output_dir / "episode_report.json", report)

    result = {
        "ok": report.get("summary", {}).get("status") == "succeeded" and not errors,
        "episode_id": report.get("episode_id"),
        "status": report.get("summary", {}).get("status"),
        "arm_mode": report.get("summary", {}).get("arm_mode"),
        "base_zero_ok_before_capture": report.get("summary", {}).get("base_zero_ok_before_capture"),
        "base_zero_ok_before_arm": report.get("summary", {}).get("base_zero_ok_before_arm"),
        "d435_live_capture_executed": report.get("summary", {}).get("d435_live_capture_executed"),
        "risk_point_generated": report.get("summary", {}).get("risk_point_generated"),
        "mock_risk_triggered": report.get("summary", {}).get("mock_risk_triggered"),
        "risk_map_points": report.get("summary", {}).get("risk_map_points"),
        "projected": report.get("summary", {}).get("projected"),
        "arm_candidate_selected": report.get("summary", {}).get("arm_candidate_selected"),
        "arm_c0_candidates": report.get("summary", {}).get("arm_c0_candidates"),
        "arm_c0_succeeded_dry_run": report.get("summary", {}).get("arm_c0_succeeded_dry_run"),
        "arm_execution_status": report.get("summary", {}).get("arm_execution_status"),
        "selected_sequence": report.get("summary", {}).get("selected_sequence"),
        "hardware_executed": report.get("summary", {}).get("hardware_executed"),
        "published_cmd_vel": report.get("summary", {}).get("published_cmd_vel"),
        "serial_port_opened": report.get("summary", {}).get("serial_port_opened"),
        "serial_bytes_written": report.get("summary", {}).get("serial_bytes_written"),
        "contact_allowed": report.get("summary", {}).get("contact_allowed"),
        "obstacle_removed": report.get("summary", {}).get("obstacle_removed"),
        "errors": len(errors),
        "output_dir": str(output_dir),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
