#!/usr/bin/env python3
"""Step7-E2 guarded motion followed by D435 red-rule trigger flow.

This runner composes existing validated pieces:

1. P4-W/P4-Y guarded policy micro-motion.
2. Base-zero gate after motion.
3. Step7-E1 stationary D435 red-rule trigger and map/arm no-load chain.

It does not publish directly to /cmd_vel_guarded, does not write chassis serial,
and defaults the arm to dry-run.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
DEFAULT_OUTPUT_ROOT = ROOT / "outputs" / "step7e2_guarded_motion_red_rule_flow_v1"
PROTOCOL_VERSION = "step7e2_guarded_motion_red_rule_flow_v1"

ACTION_GUARDED_MOTION = "STEP7E2_GUARDED_MICRO_MOTION"
ACTION_BASE_ZERO_AFTER_MOTION = "STEP7E2_BASE_ZERO_AFTER_MOTION"
ACTION_RED_RULE_FLOW = "STEP7E2_D435_RED_RULE_MAP_ARM_FLOW"

RISK_TRIGGER_SOURCE = "D435_red_color_rule"
SELECTED_SEQUENCE = "arm_b3_8_step_safety_adjusted_no_load_sample"


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def slug_time() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def next_available_dir(root: Path, prefix: str) -> Path:
    for index in range(1, 1000):
        candidate = root / f"{prefix}_{index:03d}"
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"no available output directory under {root} for prefix {prefix}")


def run_command(
    name: str,
    command: Sequence[str],
    cwd: Path,
    output_dir: Path,
    errors: List[Dict[str, Any]],
    env: Optional[Dict[str, str]] = None,
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
        env=env,
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
    except Exception as exc:  # noqa: BLE001 - evidence parser must not crash the runner.
        errors.append({"timestamp": now_iso(), "stage": stage, "error": str(exc), "path": str(path)})
        return {}


def extract_path_from_stdout(stdout_path: Path, key: str) -> Optional[Path]:
    if not stdout_path.exists():
        return None
    pattern = re.compile(rf"^{re.escape(key)}=(.+)$")
    for line in stdout_path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = pattern.match(line.strip())
        if match:
            return Path(match.group(1).strip())
    return None


def copy_if_exists(source: Optional[Path], target_dir: Path) -> Optional[str]:
    if source is None or not source.exists():
        return None
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / source.name
    shutil.copy2(source, target)
    return str(target)


def any_true(values: Sequence[bool]) -> bool:
    return any(bool(value) for value in values)


def guarded_motion_requested(args: argparse.Namespace) -> bool:
    return any_true(
        (
            args.enable_guarded_motion,
            args.confirm_guarded_micro_motion,
            args.confirm_n10p_safety,
            args.confirm_no_direct_cmd_vel,
        )
    )


def guarded_motion_enabled(args: argparse.Namespace) -> bool:
    return all(
        (
            args.enable_guarded_motion,
            args.confirm_guarded_micro_motion,
            args.confirm_n10p_safety,
            args.confirm_no_direct_cmd_vel,
        )
    )


def arm_hardware_requested(args: argparse.Namespace) -> bool:
    return any_true(
        (
            args.enable_arm_hardware,
            args.confirm_map_gated_no_load,
            args.confirm_no_contact,
            args.confirm_base_zero_live,
            args.confirm_no_cmd_vel,
        )
    )


def arm_hardware_enabled(args: argparse.Namespace) -> bool:
    return all(
        (
            args.enable_arm_hardware,
            args.confirm_map_gated_no_load,
            args.confirm_no_contact,
            args.confirm_base_zero_live,
            args.confirm_no_cmd_vel,
        )
    )


def confirmation_errors(args: argparse.Namespace) -> List[Dict[str, Any]]:
    errors: List[Dict[str, Any]] = []
    if guarded_motion_requested(args) and not guarded_motion_enabled(args):
        required = {
            "--enable-guarded-motion": args.enable_guarded_motion,
            "--confirm-guarded-micro-motion": args.confirm_guarded_micro_motion,
            "--confirm-n10p-safety": args.confirm_n10p_safety,
            "--confirm-no-direct-cmd-vel": args.confirm_no_direct_cmd_vel,
        }
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": "guarded_motion_gate",
                "error": "guarded motion requires all chassis confirmation flags",
                "missing_flags": [name for name, enabled in required.items() if not enabled],
            }
        )
    if arm_hardware_requested(args) and not arm_hardware_enabled(args):
        required = {
            "--enable-arm-hardware": args.enable_arm_hardware,
            "--confirm-map-gated-no-load": args.confirm_map_gated_no_load,
            "--confirm-no-contact": args.confirm_no_contact,
            "--confirm-base-zero-live": args.confirm_base_zero_live,
            "--confirm-no-cmd-vel": args.confirm_no_cmd_vel,
        }
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": "arm_hardware_gate",
                "error": "arm hardware no-load requires all explicit arm confirmation flags",
                "missing_flags": [name for name, enabled in required.items() if not enabled],
            }
        )
    if args.dry_run_arm and arm_hardware_requested(args):
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": "arm_hardware_gate",
                "error": "--dry-run-arm cannot be combined with arm hardware confirmation flags",
            }
        )
    return errors


def policy_env(args: argparse.Namespace) -> Dict[str, str]:
    env = os.environ.copy()
    env.update(
        {
            "POLICY_MAX_STEPS": str(args.policy_steps),
            "POLICY_MAX_RUNTIME_S": str(args.policy_max_runtime_s),
            "POLICY_MAX_TOTAL_FORWARD_M": str(args.policy_max_total_forward_m),
            "POLICY_ARC_MODE": args.policy_arc_mode,
            "POLICY_MAX_CONSECUTIVE_FAST_ARC": str(args.policy_max_consecutive_fast_arc),
            "POLICY_ARC_FAST_LINEAR": str(args.policy_arc_fast_linear),
            "POLICY_ARC_FAST_ANGULAR": str(args.policy_arc_fast_angular),
            "POLICY_ARC_FAST_DURATION_S": str(args.policy_arc_fast_duration_s),
            "POLICY_CLOSE_ACTION": args.policy_close_action,
            "POLICY_MID_ACTION": args.policy_mid_action,
            "POLICY_NORMAL_ACTION": args.policy_normal_action,
            "SAVE_POLICY": args.save_policy,
            "SAVE_EVERY_N": str(args.save_every_n),
            "ZERO_HOLD_S": str(args.zero_hold_s),
            "ZERO_MIN_HOLD_S": str(args.zero_min_hold_s),
            "ZERO_POLL_S": str(args.zero_poll_s),
            "ZERO_CONFIRM_SAMPLES": str(args.zero_confirm_samples),
            "CONSOLE_MODE": args.console_mode,
            "REPO": str(ROOT),
        }
    )
    return env


def summarize_policy(report: Dict[str, Any], motion_enabled: bool) -> Dict[str, Any]:
    result = report.get("result") or {}
    records = result.get("records") or []
    front_p10_values = [
        record.get("front_p10")
        for record in records
        if isinstance(record, dict) and isinstance(record.get("front_p10"), (int, float))
    ]
    front_min_values = [
        record.get("front_min")
        for record in records
        if isinstance(record, dict) and isinstance(record.get("front_min"), (int, float))
    ]
    executed_count = int(result.get("executed_count") or 0)
    stop_reason = result.get("sequence_stop_reason")
    return {
        "policy_report_mode": result.get("mode"),
        "guarded_motion_requested": motion_enabled,
        "guarded_motion_executed": bool(motion_enabled and executed_count > 0),
        "step_count": result.get("step_count"),
        "executed_count": executed_count,
        "sequence_stop_reason": stop_reason,
        "base_zero_ok_after_motion": result.get("base_zero_ok"),
        "final_map_saved": result.get("final_map_saved"),
        "cumulative_positive_forward_m": result.get("cumulative_positive_forward_m"),
        "odom_delta": result.get("odom_delta"),
        "front_p10_min_m": min(front_p10_values) if front_p10_values else None,
        "front_p10_max_m": max(front_p10_values) if front_p10_values else None,
        "front_min_min_m": min(front_min_values) if front_min_values else None,
        "front_min_max_m": max(front_min_values) if front_min_values else None,
        "record_count": len(records),
        "direct_cmd_vel_bypass": False,
        "motion_command_path": "/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded",
    }


def latest_executed_record(report: Dict[str, Any]) -> Dict[str, Any]:
    result = report.get("result") or {}
    records = result.get("records") or []
    if not isinstance(records, list):
        return {}
    for record in reversed(records):
        if isinstance(record, dict) and record.get("executed") is True:
            return record
    for record in reversed(records):
        if isinstance(record, dict):
            return record
    return {}


def write_policy_base_zero_evidence(
    *,
    output_dir: Path,
    policy_report: Dict[str, Any],
    policy_report_copy: Optional[str],
    policy_summary: Dict[str, Any],
) -> Optional[Path]:
    result = policy_report.get("result") or {}
    base_zero_after = result.get("base_zero_after") or {}
    record = latest_executed_record(policy_report)
    base_zero_ok = (
        policy_summary.get("base_zero_ok_after_motion") is True
        and isinstance(base_zero_after, dict)
        and base_zero_after.get("base_zero_ok") is True
    )
    evidence = {
        "schema_version": "arm_c1_base_zero_evidence_v1",
        "generated_at": now_utc_iso(),
        "generator": "tools/run_step7e2_guarded_motion_red_rule_flow.py",
        "evidence_type": "live_base_zero_observation",
        "source_mode": "guarded_policy_runner_final_base_zero_reuse",
        "valid_for_arm_c1_hardware": bool(base_zero_ok),
        "read_only": True,
        "ros_node_created_by_this_script": False,
        "cmd_vel_publisher_created": False,
        "cmd_vel_published_by_this_script": False,
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "base_zero_checked_live": True,
        "base_zero_ok_before_arm": bool(base_zero_ok),
        "published_cmd_vel": False,
        "source_episode_report": policy_report_copy,
        "source_episode_id": policy_report.get("episode_id"),
        "source_protocol_version": policy_report.get("protocol_version"),
        "source_policy_timestamp": policy_report.get("started_at"),
        "base_zero": base_zero_after,
        "odom": record.get("odom_after"),
        "checks": {
            "source": "P4-W/P4-Y guarded policy runner final base-zero result",
            "demo_fast_reuse_policy_base_zero": True,
            "base_zero_ok_after_motion": policy_summary.get("base_zero_ok_after_motion"),
            "direct_cmd_vel_bypass": False,
            "motion_command_path": "/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded",
            "guarded_cmd_zero_ok": base_zero_after.get("guarded_cmd_zero_ok") if isinstance(base_zero_after, dict) else None,
            "robot_vel_zero_ok": base_zero_after.get("robot_vel_zero_ok") if isinstance(base_zero_after, dict) else None,
            "diag_zero_ok": base_zero_after.get("diag_zero_ok") if isinstance(base_zero_after, dict) else None,
            "latest_executed_step_index": record.get("step_index"),
        },
        "claim_boundary": [
            "This evidence reuses the final base-zero observation produced by the immediately preceding guarded policy runner.",
            "It is only emitted when Step7-E2 explicitly enables demo-fast policy base-zero reuse.",
            "It does not bypass the chassis safety guard and does not publish cmd_vel.",
        ],
    }
    evidence_path = output_dir / "demo_fast_base_zero" / "base_zero_evidence.json"
    write_json(evidence_path, evidence)
    return evidence_path


def summarize_red_rule(report: Dict[str, Any]) -> Dict[str, Any]:
    return report.get("summary") or {}


def action_record(action_id: str, action_type: str, requires_base_zero: bool, params: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "action_id": action_id,
        "action_type": action_type,
        "requested_at": now_iso(),
        "requires_base_zero": requires_base_zero,
        "publishes_cmd_vel": False,
        "reason": "Step7-E2 guarded motion followed by D435 red-rule trigger",
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


def build_episode_report(
    output_dir: Path,
    run_id: str,
    commands: List[Dict[str, Any]],
    policy_report_path: Optional[Path],
    policy_run_log_path: Optional[Path],
    policy_report_copy: Optional[str],
    policy_log_copy: Optional[str],
    policy_report: Dict[str, Any],
    red_rule_report: Dict[str, Any],
    motion_enabled: bool,
    arm_hw_enabled: bool,
    errors: List[Dict[str, Any]],
) -> Dict[str, Any]:
    policy_summary = summarize_policy(policy_report, motion_enabled)
    red_summary = summarize_red_rule(red_rule_report)
    base_zero_after_motion = policy_summary.get("base_zero_ok_after_motion") is True
    red_detected = red_summary.get("red_object_detected") is True
    negative_control_expected = bool(red_summary.get("negative_control_expected") is True)
    negative_control_pass = bool(red_summary.get("negative_control_pass") is True)
    d435_ok = red_summary.get("d435_live_capture_executed") is True
    map_ok = int(red_summary.get("risk_map_points") or 0) >= 1 and int(red_summary.get("projected") or 0) >= 1
    arm_ok = red_summary.get("arm_execution_status") in ("succeeded", "succeeded_dry_run")
    motion_ok = (
        policy_summary.get("policy_report_mode")
        in ("guarded-policy-run", "guarded-policy-dry-run")
        and (not motion_enabled or policy_summary.get("guarded_motion_executed") is True)
        and base_zero_after_motion
    )
    positive_control_pass = bool(red_detected and map_ok and arm_ok)
    top_ok = bool(
        motion_ok
        and d435_ok
        and not errors
        and (negative_control_pass if negative_control_expected else positive_control_pass)
    )

    actions: List[Dict[str, Any]] = []
    results: List[Dict[str, Any]] = []
    motion_action_id = f"{run_id}_guarded_motion"
    actions.append(
        action_record(
            motion_action_id,
            ACTION_GUARDED_MOTION,
            False,
            {"motion_enabled": motion_enabled, "command_path": policy_summary.get("motion_command_path")},
        )
    )
    results.append(
        result_record(
            motion_action_id,
            ACTION_GUARDED_MOTION,
            "succeeded" if motion_ok else "failed_safe",
            True,
            {
                "policy_report_original": str(policy_report_path) if policy_report_path else None,
                "policy_run_log_original": str(policy_run_log_path) if policy_run_log_path else None,
                "policy_report_copy": policy_report_copy,
                "policy_run_log_copy": policy_log_copy,
            },
            policy_summary,
            None if motion_ok else "guarded policy motion did not meet Step7-E2 gate",
        )
    )

    base_action_id = f"{run_id}_base_zero_after_motion"
    actions.append(
        action_record(base_action_id, ACTION_BASE_ZERO_AFTER_MOTION, False, {"source": "guarded_policy_report"})
    )
    results.append(
        result_record(
            base_action_id,
            ACTION_BASE_ZERO_AFTER_MOTION,
            "succeeded" if base_zero_after_motion else "failed_safe",
            base_zero_after_motion,
            {"policy_report": policy_report_copy or str(policy_report_path)},
            {"base_zero_ok_after_motion": base_zero_after_motion},
            None if base_zero_after_motion else "base_zero_ok_after_motion=false",
        )
    )

    red_action_id = f"{run_id}_red_rule_flow"
    actions.append(
        action_record(
            red_action_id,
            ACTION_RED_RULE_FLOW,
            True,
            {"risk_trigger_source": RISK_TRIGGER_SOURCE, "arm_hardware_requested": arm_hw_enabled},
        )
    )
    results.append(
        result_record(
            red_action_id,
            ACTION_RED_RULE_FLOW,
            "succeeded" if (negative_control_pass if negative_control_expected else positive_control_pass) else "failed_safe",
            red_summary.get("base_zero_ok_before_capture"),
            {"red_rule_flow_episode_report": str(output_dir / "red_rule_after_motion" / "episode_report.json")},
            red_summary,
            None if (negative_control_pass if negative_control_expected else positive_control_pass) else "red-rule subflow failed after guarded motion",
        )
    )
    captures = [dict(item) for item in red_rule_report.get("captures") or [] if isinstance(item, dict)]
    for capture in captures:
        capture["action_id"] = red_action_id
    risk_points = [
        dict(item) for item in red_rule_report.get("risk_points") or [] if isinstance(item, dict)
    ]

    return {
        "episode_id": run_id,
        "episode_kind": "step7e2_guarded_motion_red_rule_flow",
        "started_at": now_iso(),
        "ended_at": now_iso(),
        "protocol_version": PROTOCOL_VERSION,
        "policy_state": {
            "state_id": f"{run_id}_state",
            "timestamp": now_iso(),
            "base_zero_ok": base_zero_after_motion,
            "base_zero": {"source": "guarded_policy_report", "base_zero_ok_after_motion": base_zero_after_motion},
            "odom": (policy_report.get("result") or {}).get("odom_end"),
            "front_min_range_m": policy_summary.get("front_min_min_m"),
            "front_p10_range_m": policy_summary.get("front_p10_min_m"),
            "source": "run_step7e2_guarded_motion_red_rule_flow",
            "notes": [
                "guarded micro-motion is delegated to existing P4 policy tooling",
                "D435 red-rule capture is executed only after base-zero after motion",
                "this runner does not bypass scan_safety_guard",
                "arm hardware is disabled unless all explicit confirmation flags are provided",
            ],
        },
        "actions": actions,
        "action_results": results,
        "captures": captures,
        "risk_points": risk_points,
        "step7e2_flow": {
            "commands": commands,
            "policy_summary": policy_summary,
            "red_rule_summary": red_summary,
        },
        "summary": {
            "status": "succeeded" if top_ok else "failed_safe",
            "guarded_motion_enabled": motion_enabled,
            "guarded_motion_executed": policy_summary.get("guarded_motion_executed"),
            "motion_command_path": policy_summary.get("motion_command_path"),
            "direct_cmd_vel_bypass": False,
            "policy_step_count": policy_summary.get("step_count"),
            "policy_executed_count": policy_summary.get("executed_count"),
            "policy_sequence_stop_reason": policy_summary.get("sequence_stop_reason"),
            "base_zero_ok_after_motion": base_zero_after_motion,
            "final_map_saved": policy_summary.get("final_map_saved"),
            "cumulative_positive_forward_m": policy_summary.get("cumulative_positive_forward_m"),
            "event_source": "D435_color_image_after_guarded_motion",
            "risk_trigger_source": RISK_TRIGGER_SOURCE,
            "red_object_detected": red_detected,
            "negative_control_expected": negative_control_expected,
            "negative_control_pass": negative_control_pass,
            "demo_fast_reuse_policy_base_zero": red_summary.get("demo_fast_reuse_policy_base_zero"),
            "detection_mode": red_summary.get("detection_mode"),
            "model_used": False,
            "accuracy_claimed": False,
            "bbox_xywh": red_summary.get("bbox_xywh"),
            "depth_median_m": red_summary.get("depth_median_m"),
            "bbox_valid_depth_ratio": red_summary.get("bbox_valid_depth_ratio"),
            "camera_point_xyz_m": red_summary.get("camera_point_xyz_m"),
            "base_zero_ok_before_capture": red_summary.get("base_zero_ok_before_capture"),
            "base_zero_ok_before_arm": red_summary.get("base_zero_ok_before_arm"),
            "d435_live_capture_executed": d435_ok,
            "risk_point_generated": red_summary.get("risk_point_generated"),
            "mock_risk_triggered": False,
            "risk_map_points": red_summary.get("risk_map_points"),
            "projected": red_summary.get("projected"),
            "arm_candidate_selected": red_summary.get("arm_candidate_selected"),
            "arm_execution_status": red_summary.get("arm_execution_status"),
            "selected_sequence": red_summary.get("selected_sequence"),
            "hardware_executed": red_summary.get("hardware_executed"),
            "serial_port_opened": red_summary.get("serial_port_opened"),
            "serial_bytes_written": red_summary.get("serial_bytes_written"),
            "published_cmd_vel": False,
            "published_cmd_vel_during_capture": False,
            "published_cmd_vel_during_arm": False,
            "contact_allowed": False,
            "obstacle_removed": False,
            "llm_used": False,
            "online_api_used": False,
            "local_model_used": False,
        },
        "errors": errors,
        "output_root": str(output_dir),
    }


def render_report(report: Dict[str, Any]) -> str:
    summary = report["summary"]
    return "\n".join(
        [
            "# Step7-E2 Guarded Motion Red-Rule Flow",
            "",
            "## Summary",
            "",
            f"- episode_id: `{report.get('episode_id')}`",
            f"- status: `{summary.get('status')}`",
            f"- guarded_motion_executed: `{summary.get('guarded_motion_executed')}`",
            f"- motion_command_path: `{summary.get('motion_command_path')}`",
            f"- direct_cmd_vel_bypass: `{summary.get('direct_cmd_vel_bypass')}`",
            f"- base_zero_ok_after_motion: `{summary.get('base_zero_ok_after_motion')}`",
            f"- red_object_detected: `{summary.get('red_object_detected')}`",
            f"- risk_trigger_source: `{summary.get('risk_trigger_source')}`",
            f"- bbox_xywh: `{summary.get('bbox_xywh')}`",
            f"- depth_median_m: `{summary.get('depth_median_m')}`",
            f"- bbox_valid_depth_ratio: `{summary.get('bbox_valid_depth_ratio')}`",
            f"- risk_map_points/projected: `{summary.get('risk_map_points')}/{summary.get('projected')}`",
            f"- arm_execution_status: `{summary.get('arm_execution_status')}`",
            f"- selected_sequence: `{summary.get('selected_sequence')}`",
            f"- hardware_executed: `{summary.get('hardware_executed')}`",
            f"- serial_bytes_written: `{summary.get('serial_bytes_written')}`",
            f"- published_cmd_vel_during_capture: `{summary.get('published_cmd_vel_during_capture')}`",
            f"- published_cmd_vel_during_arm: `{summary.get('published_cmd_vel_during_arm')}`",
            f"- contact_allowed: `{summary.get('contact_allowed')}`",
            f"- obstacle_removed: `{summary.get('obstacle_removed')}`",
            "",
            "## Claim Boundary",
            "",
            "- allowed: guarded micro-motion through the existing P4/N10P safety chain",
            "- allowed: D435 red-color rule trigger after base-zero",
            "- allowed: approximate Map-A0 projection",
            "- allowed: Arm-C0 dry-run by default",
            "- disallowed: direct `/cmd_vel_guarded` publish or chassis serial bypass",
            "- disallowed: autonomous navigation or path planning success claim",
            "- disallowed: trained visual model or visual detection accuracy claim",
            "- disallowed: grasping, contact, payload handling, or obstacle removal",
            "- disallowed: LLM control of the robot",
            "",
        ]
    )


def render_readme(report: Dict[str, Any]) -> str:
    summary = report.get("summary") or {}
    return (
        "# Step7-E2 Guarded Motion Red-Rule Flow\n\n"
        "Guarded micro-motion followed by live D435 red-rule trigger, Map-A0 projection, "
        "Arm-C0/Arm-C1 no-load gate, and deterministic LLM-A report.\n\n"
        f"- status: `{summary.get('status')}`\n"
        f"- red_object_detected: `{summary.get('red_object_detected')}`\n"
        f"- hardware_executed: `{summary.get('hardware_executed')}`\n"
    )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--policy-steps", type=int, default=5, choices=(5, 6, 7))
    parser.add_argument("--policy-max-runtime-s", type=float, default=120.0)
    parser.add_argument("--policy-max-total-forward-m", type=float, default=1.0)
    parser.add_argument("--policy-arc-mode", choices=("precise", "fast"), default="fast")
    parser.add_argument("--policy-max-consecutive-fast-arc", type=int, default=2)
    parser.add_argument("--policy-arc-fast-linear", type=float, default=0.12)
    parser.add_argument("--policy-arc-fast-angular", type=float, default=0.80)
    parser.add_argument("--policy-arc-fast-duration-s", type=float, default=1.0)
    parser.add_argument("--policy-close-action", choices=("arc30", "forward"), default="arc30")
    parser.add_argument("--policy-mid-action", choices=("arc30", "forward"), default="arc30")
    parser.add_argument("--policy-normal-action", choices=("forward", "arc30"), default="forward")
    parser.add_argument("--save-policy", default="pipelined_critical")
    parser.add_argument("--save-every-n", type=int, default=2)
    parser.add_argument("--zero-hold-s", type=float, default=4.0)
    parser.add_argument("--zero-min-hold-s", type=float, default=0.8)
    parser.add_argument("--zero-poll-s", type=float, default=0.1)
    parser.add_argument("--zero-confirm-samples", type=int, default=3)
    parser.add_argument("--console-mode", choices=("full", "compact"), default="compact")
    parser.add_argument("--candidate-id", default="arm_c0_candidate_001")
    parser.add_argument("--base-zero-max-age-s", type=float, default=60.0)
    parser.add_argument("--serial-port", default="/dev/arm_bus")
    parser.add_argument("--capture-timeout-s", type=float, default=8.0)
    parser.add_argument("--min-red-area-px", type=int, default=80)
    parser.add_argument("--expect-no-red", action="store_true")
    parser.add_argument(
        "--demo-fast-reuse-policy-base-zero",
        action="store_true",
        help=(
            "Demo-only latency reduction: reuse the immediately preceding guarded "
            "policy final base-zero evidence instead of launching a second base-zero observer."
        ),
    )
    parser.add_argument("--enable-guarded-motion", action="store_true")
    parser.add_argument("--confirm-guarded-micro-motion", action="store_true")
    parser.add_argument("--confirm-n10p-safety", action="store_true")
    parser.add_argument("--confirm-no-direct-cmd-vel", action="store_true")
    parser.add_argument("--dry-run-arm", action="store_true")
    parser.add_argument("--enable-arm-hardware", action="store_true")
    parser.add_argument("--confirm-map-gated-no-load", action="store_true")
    parser.add_argument("--confirm-no-contact", action="store_true")
    parser.add_argument("--confirm-base-zero-live", action="store_true")
    parser.add_argument("--confirm-no-cmd-vel", action="store_true")
    return parser.parse_args(argv)


def output_prefix(motion_enabled: bool, arm_hw_enabled: bool) -> str:
    if motion_enabled and arm_hw_enabled:
        return "e2_guarded_red_rule_arm_hw"
    if motion_enabled:
        return "e2_guarded_red_rule_dryrun"
    return "e2_policy_dryrun"


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    run_id = f"step7e2_guarded_red_rule_{slug_time()}"
    gate_errors = confirmation_errors(args)
    motion_enabled = guarded_motion_enabled(args) and not gate_errors
    arm_hw_enabled = arm_hardware_enabled(args) and not gate_errors
    output_dir = (
        Path(str(args.output_dir).strip())
        if args.output_dir
        else next_available_dir(DEFAULT_OUTPUT_ROOT, output_prefix(motion_enabled, arm_hw_enabled))
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    errors: List[Dict[str, Any]] = list(gate_errors)
    commands: List[Dict[str, Any]] = []

    policy_mode = "run" if motion_enabled else "dry-run"
    policy_command = ["bash", str(TOOLS / "p4w_guarded_policy_branch_mixed.sh"), policy_mode]
    if not errors:
        commands.append(
            run_command(
                "01_guarded_policy_motion",
                policy_command,
                ROOT,
                output_dir,
                errors,
                env=policy_env(args),
            )
        )

    motion_dir = output_dir / "guarded_motion"
    policy_stdout = output_dir / "01_guarded_policy_motion.stdout.txt"
    policy_report_path = extract_path_from_stdout(policy_stdout, "P4W_REPORT")
    policy_run_log_path = extract_path_from_stdout(policy_stdout, "P4W_RUN_LOG")
    policy_report_copy = copy_if_exists(policy_report_path, motion_dir)
    policy_log_copy = copy_if_exists(policy_run_log_path, motion_dir)
    policy_report = read_json_or_error(
        Path(policy_report_copy) if policy_report_copy else policy_report_path or Path("missing_policy_report.json"),
        "guarded_policy_motion_report",
        errors,
    )
    policy_summary = summarize_policy(policy_report, motion_enabled) if policy_report else {}
    base_zero_after_motion = policy_summary.get("base_zero_ok_after_motion") is True
    reused_base_zero_evidence_path: Optional[Path] = None
    if (
        args.demo_fast_reuse_policy_base_zero
        and policy_report
        and base_zero_after_motion
        and not errors
    ):
        reused_base_zero_evidence_path = write_policy_base_zero_evidence(
            output_dir=output_dir,
            policy_report=policy_report,
            policy_report_copy=policy_report_copy,
            policy_summary=policy_summary,
        )

    red_rule_report: Dict[str, Any] = {}
    if policy_report and base_zero_after_motion and not errors:
        red_rule_dir = output_dir / "red_rule_after_motion"
        red_command = [
            args.python,
            str(TOOLS / "run_step7e1_red_rule_stationary_flow.py"),
            "--candidate-id",
            args.candidate_id,
            "--base-zero-max-age-s",
            str(args.base_zero_max_age_s),
            "--serial-port",
            args.serial_port,
            "--capture-timeout-s",
            str(args.capture_timeout_s),
            "--min-red-area-px",
            str(args.min_red_area_px),
            "--output-dir",
            str(red_rule_dir),
        ]
        if reused_base_zero_evidence_path:
            red_command.extend(["--base-zero-evidence", str(reused_base_zero_evidence_path)])
        if args.expect_no_red:
            red_command.append("--expect-no-red")
        if arm_hw_enabled:
            red_command.extend(
                [
                    "--enable-hardware-write",
                    "--confirm-map-gated-no-load",
                    "--confirm-no-contact",
                    "--confirm-base-zero-live",
                    "--confirm-no-cmd-vel",
                ]
            )
        else:
            red_command.append("--dry-run-arm")
        commands.append(run_command("02_red_rule_after_motion", red_command, ROOT, output_dir, errors))
        red_rule_report = read_json_or_error(
            red_rule_dir / "episode_report.json", "red_rule_after_motion", errors
        )
    elif not base_zero_after_motion:
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": "base_zero_after_motion",
                "error": "base_zero_ok_after_motion is not true; skipping D435 red-rule flow",
            }
        )

    report = build_episode_report(
        output_dir=output_dir,
        run_id=run_id,
        commands=commands,
        policy_report_path=policy_report_path,
        policy_run_log_path=policy_run_log_path,
        policy_report_copy=policy_report_copy,
        policy_log_copy=policy_log_copy,
        policy_report=policy_report,
        red_rule_report=red_rule_report,
        motion_enabled=motion_enabled,
        arm_hw_enabled=arm_hw_enabled,
        errors=errors,
    )
    write_json(output_dir / "episode_report.json", report)
    write_json(output_dir / "errors.json", errors)
    write_text(output_dir / "step7e2_report.md", render_report(report))
    write_text(output_dir / "README.md", render_readme(report))

    llm_dir = output_dir / "llm_a_report"
    commands.append(
        run_command(
            "03_llm_a_report",
            [
                args.python,
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
        "guarded_motion_executed": report.get("summary", {}).get("guarded_motion_executed"),
        "motion_command_path": report.get("summary", {}).get("motion_command_path"),
        "base_zero_ok_after_motion": report.get("summary", {}).get("base_zero_ok_after_motion"),
        "risk_trigger_source": report.get("summary", {}).get("risk_trigger_source"),
        "red_object_detected": report.get("summary", {}).get("red_object_detected"),
        "negative_control_expected": report.get("summary", {}).get("negative_control_expected"),
        "negative_control_pass": report.get("summary", {}).get("negative_control_pass"),
        "depth_median_m": report.get("summary", {}).get("depth_median_m"),
        "risk_map_points": report.get("summary", {}).get("risk_map_points"),
        "projected": report.get("summary", {}).get("projected"),
        "arm_execution_status": report.get("summary", {}).get("arm_execution_status"),
        "hardware_executed": report.get("summary", {}).get("hardware_executed"),
        "serial_bytes_written": report.get("summary", {}).get("serial_bytes_written"),
        "published_cmd_vel_during_capture": False,
        "published_cmd_vel_during_arm": False,
        "direct_cmd_vel_bypass": False,
        "contact_allowed": False,
        "obstacle_removed": False,
        "errors": len(errors),
        "output_dir": str(output_dir),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
