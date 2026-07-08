#!/usr/bin/env python3
"""Step7-E0 N10P event-triggered D435 capture to arm no-load flow.

This runner does not implement a new chassis controller. Guarded motion is
delegated to the existing P4-W/P4-Y policy runner, which routes commands through:

    /input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded

The D435 HOLD_CAPTURE/map/arm chain is executed only after a N10P event is found
in the guarded policy report. By default the arm remains dry-run.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
DEFAULT_OUTPUT_ROOT = ROOT / "outputs" / "step7e_event_triggered_capture_arm_flow_v1"
PROTOCOL_VERSION = "step7e_event_triggered_capture_arm_flow_v1"

ACTION_GUARDED_MOTION = "STEP7E_GUARDED_MICRO_MOTION"
ACTION_N10P_EVENT_GATE = "STEP7E_N10P_FRONT_EVENT_GATE"
ACTION_EVENT_FLOW = "STEP7E_EVENT_TRIGGERED_D435_MAP_ARM_FLOW"

EVENT_SOURCE = "N10P_front_p10"
RISK_TRIGGER_SOURCE = "N10P_front_p10_event"
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


def as_float(value: Any) -> Optional[float]:
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


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
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": stage,
                "error": str(exc),
                "path": str(path),
            }
        )
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
                "error": "guarded micro-motion requires all chassis confirmation flags",
                "missing_flags": [name for name, value in required.items() if not value],
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
                "error": "Arm-C1 no-load hardware requires all explicit arm confirmation flags",
                "missing_flags": [name for name, value in required.items() if not value],
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


def front_values(record: Dict[str, Any]) -> Tuple[Optional[float], Optional[float]]:
    front_p10 = as_float(record.get("front_p10"))
    front_min = as_float(record.get("front_min"))
    for key in ("precheck", "postcheck"):
        state = record.get(key) or {}
        if front_p10 is None:
            front_p10 = as_float(state.get("front_p10_range_m"))
        if front_min is None:
            front_min = as_float(state.get("front_min_range_m"))
        sectors = state.get("scan_sectors") or {}
        front = sectors.get("front") or {}
        if front_p10 is None:
            front_p10 = as_float(front.get("p10"))
        if front_min is None:
            front_min = as_float(front.get("min"))
    return front_p10, front_min


def summarize_policy(report: Dict[str, Any], motion_enabled: bool) -> Dict[str, Any]:
    result = report.get("result") or {}
    records = result.get("records") or []
    p10_values: List[float] = []
    min_values: List[float] = []
    for record in records:
        if not isinstance(record, dict):
            continue
        p10, fmin = front_values(record)
        if p10 is not None:
            p10_values.append(p10)
        if fmin is not None:
            min_values.append(fmin)
    stop_reason = result.get("sequence_stop_reason")
    n10p_safety_stop = any(
        token in str(stop_reason or "")
        for token in ("front_blocked", "HARD_STOP", "hard stop", "front_min", "front_p10")
    )
    executed_count = int(result.get("executed_count") or 0)
    return {
        "policy_report_mode": result.get("mode"),
        "guarded_motion_requested": motion_enabled,
        "guarded_motion_executed": bool(motion_enabled and executed_count > 0),
        "step_count": result.get("step_count"),
        "executed_count": executed_count,
        "sequence_stop_reason": stop_reason,
        "n10p_safety_stop_observed": bool(n10p_safety_stop),
        "base_zero_ok_after_motion": result.get("base_zero_ok"),
        "final_map_saved": result.get("final_map_saved"),
        "cumulative_positive_forward_m": result.get("cumulative_positive_forward_m"),
        "odom_delta": result.get("odom_delta"),
        "front_p10_min_m": min(p10_values) if p10_values else None,
        "front_p10_max_m": max(p10_values) if p10_values else None,
        "front_min_min_m": min(min_values) if min_values else None,
        "front_min_max_m": max(min_values) if min_values else None,
        "record_count": len(records),
        "direct_cmd_vel_bypass": False,
        "motion_command_path": "/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded",
    }


def event_for_record(
    record: Dict[str, Any],
    front_p10_threshold_m: float,
    front_min_threshold_m: float,
) -> Dict[str, Any]:
    front_p10, front_min = front_values(record)
    p10_hit = front_p10 is not None and front_p10 < front_p10_threshold_m
    min_hit = front_min is not None and front_min < front_min_threshold_m
    reasons: List[str] = []
    if p10_hit:
        reasons.append(f"front_p10 {front_p10:.3f}m < {front_p10_threshold_m:.3f}m")
    if min_hit:
        reasons.append(f"front_min {front_min:.3f}m < {front_min_threshold_m:.3f}m")
    if p10_hit:
        reason_code = "front_p10_below_threshold"
    elif min_hit:
        reason_code = "front_min_below_threshold"
    else:
        reason_code = "threshold_not_met"
    return {
        "matched": bool(p10_hit or min_hit),
        "reason": reason_code,
        "reason_detail": "; ".join(reasons) if reasons else "threshold not met",
        "front_p10_m": front_p10,
        "front_min_m": front_min,
        "step_id": record.get("step_index"),
        "execution_action": record.get("execution_action"),
        "selected_action": record.get("selected_action"),
        "base_zero_ok": record.get("base_zero_ok"),
    }


def extract_n10p_event(report: Dict[str, Any], args: argparse.Namespace) -> Dict[str, Any]:
    records = (report.get("result") or {}).get("records") or []
    matched_run: List[Dict[str, Any]] = []
    all_matches: List[Dict[str, Any]] = []
    for index, record in enumerate(records, start=1):
        if not isinstance(record, dict):
            matched_run = []
            continue
        event = event_for_record(record, args.trigger_front_p10_m, args.trigger_front_min_m)
        event["record_index"] = index
        if event["matched"]:
            matched_run.append(event)
            all_matches.append(event)
            if len(matched_run) >= args.trigger_min_consecutive_records:
                trigger = matched_run[-1]
                return {
                    "event_triggered": True,
                    "event_source": EVENT_SOURCE,
                    "risk_trigger_source": RISK_TRIGGER_SOURCE,
                    "trigger_reason": trigger["reason"],
                    "trigger_reason_detail": trigger["reason_detail"],
                    "trigger_front_p10_m": trigger["front_p10_m"],
                    "trigger_front_min_m": trigger["front_min_m"],
                    "trigger_step_id": trigger["step_id"],
                    "trigger_record_index": trigger["record_index"],
                    "trigger_execution_action": trigger["execution_action"],
                    "trigger_selected_action": trigger["selected_action"],
                    "trigger_base_zero_ok": trigger["base_zero_ok"],
                    "trigger_front_p10_threshold_m": args.trigger_front_p10_m,
                    "trigger_front_min_threshold_m": args.trigger_front_min_m,
                    "trigger_min_consecutive_records": args.trigger_min_consecutive_records,
                    "trigger_consecutive_observed": len(matched_run),
                    "matched_records": all_matches,
                }
        else:
            matched_run = []
    policy_summary = summarize_policy(report, motion_enabled=True)
    return {
        "event_triggered": False,
        "event_source": EVENT_SOURCE,
        "risk_trigger_source": RISK_TRIGGER_SOURCE,
        "trigger_reason": "threshold_not_met",
        "trigger_reason_detail": "no policy record met front_p10/front_min event threshold",
        "trigger_front_p10_m": None,
        "trigger_front_min_m": None,
        "trigger_step_id": None,
        "trigger_record_index": None,
        "trigger_front_p10_threshold_m": args.trigger_front_p10_m,
        "trigger_front_min_threshold_m": args.trigger_front_min_m,
        "trigger_min_consecutive_records": args.trigger_min_consecutive_records,
        "trigger_consecutive_observed": 0,
        "matched_records": all_matches,
        "policy_front_p10_min_m": policy_summary.get("front_p10_min_m"),
        "policy_front_min_min_m": policy_summary.get("front_min_min_m"),
    }


def summarize_step7c(report: Dict[str, Any]) -> Dict[str, Any]:
    return report.get("summary") or {}


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
        "reason": "Step7-E0 event-triggered guarded capture/arm flow",
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
    event: Dict[str, Any],
    step7c_report: Dict[str, Any],
    motion_enabled: bool,
    arm_hw_enabled: bool,
    errors: List[Dict[str, Any]],
) -> Dict[str, Any]:
    policy_summary = summarize_policy(policy_report, motion_enabled)
    step7c_summary = summarize_step7c(step7c_report)
    base_zero_after_motion = policy_summary.get("base_zero_ok_after_motion") is True
    event_triggered = event.get("event_triggered") is True
    d435_ok = step7c_summary.get("d435_live_capture_executed") is True
    risk_ok = step7c_summary.get("risk_point_generated") is True
    map_ok = int(step7c_summary.get("risk_map_points") or 0) >= 1 and int(step7c_summary.get("projected") or 0) >= 1
    arm_candidate_ok = step7c_summary.get("arm_candidate_selected") is True
    arm_ok = step7c_summary.get("arm_execution_status") in ("succeeded", "succeeded_dry_run")
    motion_ok = (
        policy_summary.get("policy_report_mode")
        in ("guarded-policy-run", "guarded-policy-dry-run")
        and (not motion_enabled or policy_summary.get("guarded_motion_executed") is True)
        and base_zero_after_motion
    )
    top_ok = bool(
        motion_ok
        and event_triggered
        and d435_ok
        and risk_ok
        and map_ok
        and arm_candidate_ok
        and arm_ok
        and not errors
    )

    actions: List[Dict[str, Any]] = []
    results: List[Dict[str, Any]] = []

    motion_action_id = f"{run_id}_guarded_motion"
    actions.append(
        action_record(
            motion_action_id,
            ACTION_GUARDED_MOTION,
            False,
            {
                "motion_enabled": motion_enabled,
                "policy_steps": policy_summary.get("step_count"),
                "command_path": policy_summary.get("motion_command_path"),
            },
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
            None if motion_ok else "guarded policy motion did not meet Step7-E0 gate",
        )
    )

    event_action_id = f"{run_id}_n10p_event"
    actions.append(
        action_record(
            event_action_id,
            ACTION_N10P_EVENT_GATE,
            False,
            {
                "event_source": EVENT_SOURCE,
                "front_p10_threshold_m": event.get("trigger_front_p10_threshold_m"),
                "front_min_threshold_m": event.get("trigger_front_min_threshold_m"),
                "min_consecutive_records": event.get("trigger_min_consecutive_records"),
            },
        )
    )
    results.append(
        result_record(
            event_action_id,
            ACTION_N10P_EVENT_GATE,
            "succeeded" if event_triggered else "failed_safe",
            base_zero_after_motion,
            {"event_trigger": str(output_dir / "event_trigger.json")},
            event,
            None if event_triggered else "N10P event threshold was not met; HOLD_CAPTURE skipped",
        )
    )

    flow_action_id = f"{run_id}_event_capture_map_arm"
    actions.append(
        action_record(
            flow_action_id,
            ACTION_EVENT_FLOW,
            True,
            {
                "risk_trigger_source": RISK_TRIGGER_SOURCE,
                "arm_mode": "hardware_once" if arm_hw_enabled else "dry_run",
                "selected_sequence": SELECTED_SEQUENCE,
            },
        )
    )
    results.append(
        result_record(
            flow_action_id,
            ACTION_EVENT_FLOW,
            "succeeded" if d435_ok and risk_ok and map_ok and arm_candidate_ok and arm_ok else "failed_safe",
            step7c_summary.get("base_zero_ok_before_capture"),
            {"step7c_episode_report": str(output_dir / "event_triggered_step7c_flow" / "episode_report.json")},
            step7c_summary,
            None if d435_ok and risk_ok and map_ok and arm_candidate_ok and arm_ok else "event-triggered Step7-C subflow failed or was skipped",
        )
    )

    return {
        "episode_id": run_id,
        "episode_kind": "step7e_event_triggered_capture_arm_flow",
        "started_at": now_iso(),
        "ended_at": now_iso(),
        "protocol_version": PROTOCOL_VERSION,
        "policy_state": {
            "state_id": f"{run_id}_state",
            "timestamp": now_iso(),
            "base_zero_ok": base_zero_after_motion,
            "base_zero": {
                "source": "guarded_policy_report",
                "base_zero_ok_after_motion": base_zero_after_motion,
            },
            "odom": (policy_report.get("result") or {}).get("odom_end"),
            "front_min_range_m": event.get("trigger_front_min_m") or policy_summary.get("front_min_min_m"),
            "front_p10_range_m": event.get("trigger_front_p10_m") or policy_summary.get("front_p10_min_m"),
            "source": "run_step7e_event_triggered_capture_arm_flow",
            "notes": [
                "guarded micro-motion is delegated to existing P4 policy tooling",
                "D435 HOLD_CAPTURE is gated by N10P front_p10/front_min event evidence",
                "this runner does not bypass scan_safety_guard",
                "arm hardware is disabled unless all explicit confirmation flags are provided",
            ],
        },
        "actions": actions,
        "action_results": results,
        "step7e_flow": {
            "commands": commands,
            "policy_summary": policy_summary,
            "event_trigger": event,
            "step7c_summary": step7c_summary,
        },
        "summary": {
            "status": "succeeded" if top_ok else "failed_safe",
            "event_triggered": event_triggered,
            "event_source": EVENT_SOURCE,
            "trigger_reason": event.get("trigger_reason"),
            "trigger_reason_detail": event.get("trigger_reason_detail"),
            "trigger_front_p10_m": event.get("trigger_front_p10_m"),
            "trigger_front_min_m": event.get("trigger_front_min_m"),
            "trigger_step_id": event.get("trigger_step_id"),
            "trigger_record_index": event.get("trigger_record_index"),
            "guarded_motion_enabled": motion_enabled,
            "guarded_motion_executed": policy_summary.get("guarded_motion_executed"),
            "motion_command_path": policy_summary.get("motion_command_path"),
            "direct_cmd_vel_bypass": False,
            "policy_step_count": policy_summary.get("step_count"),
            "policy_executed_count": policy_summary.get("executed_count"),
            "policy_sequence_stop_reason": policy_summary.get("sequence_stop_reason"),
            "n10p_safety_stop_observed": policy_summary.get("n10p_safety_stop_observed"),
            "base_zero_ok_after_motion": base_zero_after_motion,
            "base_zero_ok_before_capture": step7c_summary.get("base_zero_ok_before_capture"),
            "base_zero_ok_before_arm": step7c_summary.get("base_zero_ok_before_arm"),
            "final_map_saved": policy_summary.get("final_map_saved"),
            "d435_live_capture_executed": d435_ok,
            "risk_point_generated": risk_ok,
            "mock_risk_triggered": step7c_summary.get("mock_risk_triggered") is True,
            "risk_trigger_source": RISK_TRIGGER_SOURCE,
            "risk_map_points": step7c_summary.get("risk_map_points"),
            "projected": step7c_summary.get("projected"),
            "arm_candidate_selected": arm_candidate_ok,
            "arm_execution_status": step7c_summary.get("arm_execution_status"),
            "selected_sequence": step7c_summary.get("selected_sequence"),
            "hardware_executed": step7c_summary.get("hardware_executed"),
            "serial_port_opened": step7c_summary.get("serial_port_opened"),
            "serial_bytes_written": step7c_summary.get("serial_bytes_written"),
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
    lines = [
        "# Step7-E0 Event-Triggered Capture Arm Flow",
        "",
        "## Summary",
        "",
        f"- episode_id: `{report.get('episode_id')}`",
        f"- status: `{summary.get('status')}`",
        f"- event_triggered: `{summary.get('event_triggered')}`",
        f"- event_source: `{summary.get('event_source')}`",
        f"- trigger_reason: `{summary.get('trigger_reason')}`",
        f"- trigger_reason_detail: `{summary.get('trigger_reason_detail')}`",
        f"- trigger_front_p10_m: `{summary.get('trigger_front_p10_m')}`",
        f"- trigger_front_min_m: `{summary.get('trigger_front_min_m')}`",
        f"- trigger_step_id: `{summary.get('trigger_step_id')}`",
        f"- guarded_motion_executed: `{summary.get('guarded_motion_executed')}`",
        f"- motion_command_path: `{summary.get('motion_command_path')}`",
        f"- direct_cmd_vel_bypass: `{summary.get('direct_cmd_vel_bypass')}`",
        f"- base_zero_ok_after_motion: `{summary.get('base_zero_ok_after_motion')}`",
        f"- base_zero_ok_before_capture: `{summary.get('base_zero_ok_before_capture')}`",
        f"- d435_live_capture_executed: `{summary.get('d435_live_capture_executed')}`",
        f"- risk_point_generated: `{summary.get('risk_point_generated')}`",
        f"- mock_risk_triggered: `{summary.get('mock_risk_triggered')}`",
        f"- risk_trigger_source: `{summary.get('risk_trigger_source')}`",
        f"- risk_map_points/projected: `{summary.get('risk_map_points')}/{summary.get('projected')}`",
        f"- arm_candidate_selected: `{summary.get('arm_candidate_selected')}`",
        f"- arm_execution_status: `{summary.get('arm_execution_status')}`",
        f"- selected_sequence: `{summary.get('selected_sequence')}`",
        f"- hardware_executed: `{summary.get('hardware_executed')}`",
        f"- serial_bytes_written: `{summary.get('serial_bytes_written')}`",
        f"- published_cmd_vel_during_capture: `{summary.get('published_cmd_vel_during_capture')}`",
        f"- published_cmd_vel_during_arm: `{summary.get('published_cmd_vel_during_arm')}`",
        f"- contact_allowed: `{summary.get('contact_allowed')}`",
        f"- obstacle_removed: `{summary.get('obstacle_removed')}`",
        "",
        "## Event Rule",
        "",
        "- Trigger when any guarded policy step record satisfies:",
        "  - `front_p10 < 0.80m`, or",
        "  - `front_min < 0.70m`",
        "- The trigger source is N10P front range evidence, not D435 visual recognition.",
        "",
        "## Evidence",
        "",
        "- `guarded_motion/`",
        "- `event_trigger.json`",
        "- `event_triggered_step7c_flow/episode_report.json`",
        "- `event_triggered_step7c_flow/d435_hold_capture/episode_report.json`",
        "- `event_triggered_step7c_flow/map_projection/risk_map_points.json`",
        "- `event_triggered_step7c_flow/arm_candidate/episode_report.json`",
        "- `event_triggered_step7c_flow/arm_execution/episode_report.json`",
        "- `llm_a_report/risk_report.md`",
        "- `episode_report.json`",
        "- `errors.json`",
        "",
        "## Claim Boundary",
        "",
        "- allowed: N10P/front_p10 event-triggered D435 HOLD_CAPTURE",
        "- allowed: deterministic mock risk output with `risk_trigger_source=N10P_front_p10_event`",
        "- allowed: approximate Map-A0 risk point projection",
        "- allowed: Arm-C0 dry-run no-load response by default",
        "- allowed: one Arm-C1 no-load hardware response only when explicit hardware flags are provided and source evidence reports it",
        "- disallowed: direct `/cmd_vel_guarded` publish or chassis serial bypass",
        "- disallowed: autonomous navigation or path planning success claim",
        "- disallowed: high-precision SLAM claim",
        "- disallowed: real visual detection accuracy claim",
        "- disallowed: grasping, contact, payload handling, or obstacle removal",
        "- disallowed: LLM control of the robot",
        "",
    ]
    return "\n".join(lines)


def render_readme(report: Dict[str, Any]) -> str:
    summary = report.get("summary") or {}
    return f"""# Step7-E0 Event-Triggered Capture Arm Flow

This evidence directory contains one Step7-E0 run.

The runner gates D435 HOLD_CAPTURE on N10P front range evidence from the existing
guarded policy report. The risk trigger is deterministic/mock and sourced from
`N10P_front_p10_event`; it is not a D435 visual-recognition accuracy claim.

Status: `{summary.get('status')}`
event_triggered: `{summary.get('event_triggered')}`
arm hardware executed: `{summary.get('hardware_executed')}`
"""


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
    parser.add_argument("--trigger-front-p10-m", type=float, default=0.80)
    parser.add_argument("--trigger-front-min-m", type=float, default=0.70)
    parser.add_argument("--trigger-min-consecutive-records", type=int, default=1)
    parser.add_argument("--candidate-id", default="arm_c0_candidate_001")
    parser.add_argument("--base-zero-max-age-s", type=float, default=60.0)
    parser.add_argument("--serial-port", default="/dev/arm_bus")
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
        return "e0_n10p_trigger_arm_hw"
    if motion_enabled:
        return "e0_n10p_trigger_dryrun"
    return "e0_policy_dryrun"


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    run_id = f"step7e_event_triggered_{slug_time()}"
    gate_errors = confirmation_errors(args)
    motion_enabled = guarded_motion_enabled(args) and not gate_errors
    arm_hw_enabled = arm_hardware_enabled(args) and not gate_errors
    if args.trigger_min_consecutive_records < 1:
        gate_errors.append(
            {
                "timestamp": now_iso(),
                "stage": "argument_validation",
                "error": "trigger_min_consecutive_records must be >= 1",
            }
        )
    if args.policy_max_consecutive_fast_arc < 1 or args.policy_max_consecutive_fast_arc > 3:
        gate_errors.append(
            {
                "timestamp": now_iso(),
                "stage": "argument_validation",
                "error": "policy_max_consecutive_fast_arc must be in [1, 3]",
            }
        )

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
    event = extract_n10p_event(policy_report, args) if policy_report else {
        "event_triggered": False,
        "event_source": EVENT_SOURCE,
        "risk_trigger_source": RISK_TRIGGER_SOURCE,
        "trigger_reason": "missing guarded policy report",
        "trigger_reason_detail": "missing guarded policy report",
    }
    write_json(output_dir / "event_trigger.json", event)

    step7c_report: Dict[str, Any] = {}
    if policy_report and base_zero_after_motion and event.get("event_triggered") is True and not errors:
        step7c_dir = output_dir / "event_triggered_step7c_flow"
        step7c_command = [
            args.python,
            str(TOOLS / "run_step7c_guarded_d435_mockrisk_arm_noload.py"),
            "--candidate-id",
            args.candidate_id,
            "--base-zero-max-age-s",
            str(args.base_zero_max_age_s),
            "--serial-port",
            args.serial_port,
            "--output-dir",
            str(step7c_dir),
        ]
        if arm_hw_enabled:
            step7c_command.extend(
                [
                    "--enable-hardware-write",
                    "--confirm-map-gated-no-load",
                    "--confirm-no-contact",
                    "--confirm-base-zero-live",
                    "--confirm-no-cmd-vel",
                ]
            )
        else:
            step7c_command.append("--dry-run-arm")
        commands.append(
            run_command(
                "02_event_triggered_step7c_flow",
                step7c_command,
                ROOT,
                output_dir,
                errors,
            )
        )
        step7c_report = read_json_or_error(
            step7c_dir / "episode_report.json", "event_triggered_step7c_flow", errors
        )
    else:
        if not base_zero_after_motion:
            errors.append(
                {
                    "timestamp": now_iso(),
                    "stage": "base_zero_after_motion",
                    "error": "base_zero_ok_after_motion is not true; skipping event-triggered HOLD_CAPTURE",
                }
            )
        if event.get("event_triggered") is not True:
            errors.append(
                {
                    "timestamp": now_iso(),
                    "stage": "n10p_event_gate",
                    "error": "N10P front event was not triggered; skipping HOLD_CAPTURE",
                    "event_trigger": event,
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
        event=event,
        step7c_report=step7c_report,
        motion_enabled=motion_enabled,
        arm_hw_enabled=arm_hw_enabled,
        errors=errors,
    )
    write_json(output_dir / "episode_report.json", report)
    write_json(output_dir / "errors.json", errors)
    write_text(output_dir / "step7e_report.md", render_report(report))
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
        "event_triggered": report.get("summary", {}).get("event_triggered"),
        "event_source": report.get("summary", {}).get("event_source"),
        "trigger_reason": report.get("summary", {}).get("trigger_reason"),
        "trigger_reason_detail": report.get("summary", {}).get("trigger_reason_detail"),
        "trigger_front_p10_m": report.get("summary", {}).get("trigger_front_p10_m"),
        "trigger_front_min_m": report.get("summary", {}).get("trigger_front_min_m"),
        "trigger_step_id": report.get("summary", {}).get("trigger_step_id"),
        "base_zero_ok_before_capture": report.get("summary", {}).get("base_zero_ok_before_capture"),
        "d435_live_capture_executed": report.get("summary", {}).get("d435_live_capture_executed"),
        "risk_point_generated": report.get("summary", {}).get("risk_point_generated"),
        "mock_risk_triggered": report.get("summary", {}).get("mock_risk_triggered"),
        "risk_trigger_source": report.get("summary", {}).get("risk_trigger_source"),
        "risk_map_points": report.get("summary", {}).get("risk_map_points"),
        "projected": report.get("summary", {}).get("projected"),
        "arm_candidate_selected": report.get("summary", {}).get("arm_candidate_selected"),
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
