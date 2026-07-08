#!/usr/bin/env python3
"""Step7-D guarded micro-motion to D435/map/arm no-load integration.

This runner composes the existing guarded policy runner with the Step7-C
D435/mock-risk/map/arm flow. It does not bypass the P4 guard chain. Real
chassis motion requires explicit confirmation flags and is routed through the
existing P4-W/P4-Y guarded policy script.
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
DEFAULT_OUTPUT_ROOT = ROOT / "outputs" / "step7d_guarded_motion_d435_arm_flow_v1"
PROTOCOL_VERSION = "step7d_guarded_motion_d435_arm_flow_v1"

ACTION_GUARDED_MOTION = "STEP7D_GUARDED_MICRO_MOTION"
ACTION_BASE_ZERO_AFTER_MOTION = "STEP7D_BASE_ZERO_AFTER_MOTION"
ACTION_STEP7C_FLOW = "STEP7D_D435_MAP_ARM_FLOW"


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
    env: Optional[Dict[str, str]] = None,
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


def bool_any_flags(values: Sequence[bool]) -> bool:
    return any(bool(value) for value in values)


def guarded_motion_requested(args: argparse.Namespace) -> bool:
    return bool_any_flags(
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
    return bool_any_flags(
        (
            args.enable_arm_hardware_once,
            args.confirm_map_gated_no_load,
            args.confirm_no_contact,
            args.confirm_base_zero_live,
            args.confirm_no_cmd_vel,
        )
    )


def arm_hardware_enabled(args: argparse.Namespace) -> bool:
    return all(
        (
            args.enable_arm_hardware_once,
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
                "error": "guarded motion requires all motion confirmation flags",
                "missing_flags": [name for name, value in required.items() if not value],
            }
        )
    if arm_hardware_requested(args) and not arm_hardware_enabled(args):
        required = {
            "--enable-arm-hardware-once": args.enable_arm_hardware_once,
            "--confirm-map-gated-no-load": args.confirm_map_gated_no_load,
            "--confirm-no-contact": args.confirm_no_contact,
            "--confirm-base-zero-live": args.confirm_base_zero_live,
            "--confirm-no-cmd-vel": args.confirm_no_cmd_vel,
        }
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": "arm_hardware_gate",
                "error": "arm hardware no-load requires all arm confirmation flags",
                "missing_flags": [name for name, value in required.items() if not value],
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
        if isinstance(record.get("front_p10"), (int, float))
    ]
    front_min_values = [
        record.get("front_min")
        for record in records
        if isinstance(record.get("front_min"), (int, float))
    ]
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
        "front_p10_min_m": min(front_p10_values) if front_p10_values else None,
        "front_p10_max_m": max(front_p10_values) if front_p10_values else None,
        "front_min_min_m": min(front_min_values) if front_min_values else None,
        "front_min_max_m": max(front_min_values) if front_min_values else None,
        "record_count": len(records),
        "direct_cmd_vel_bypass": False,
        "motion_command_path": "/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded",
        "claim_boundary": [
            "Motion is delegated to existing P4-W/P4-Y guarded policy tooling.",
            "This runner does not publish directly to /cmd_vel_guarded or chassis serial.",
            "N10P safety state and base_zero are taken from the guarded policy report.",
        ],
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
        "reason": "Step7-D guarded micro-motion integration",
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
    step7c_report: Dict[str, Any],
    motion_enabled: bool,
    arm_hw_enabled: bool,
    errors: List[Dict[str, Any]],
) -> Dict[str, Any]:
    policy_summary = summarize_policy(policy_report, motion_enabled)
    step7c_summary = summarize_step7c(step7c_report)
    base_zero_after_motion = policy_summary.get("base_zero_ok_after_motion") is True
    d435_ok = step7c_summary.get("d435_live_capture_executed") is True
    map_ok = int(step7c_summary.get("risk_map_points") or 0) >= 1
    arm_ok = step7c_summary.get("arm_execution_status") in ("succeeded", "succeeded_dry_run")
    motion_ok = (
        policy_summary.get("policy_report_mode")
        in ("guarded-policy-run", "guarded-policy-dry-run")
        and (not motion_enabled or policy_summary.get("guarded_motion_executed") is True)
        and base_zero_after_motion
    )
    top_ok = bool(motion_ok and d435_ok and map_ok and arm_ok and not errors)

    actions = []
    results = []

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
            None if motion_ok else "guarded policy motion did not meet Step7-D gate",
        )
    )

    base_action_id = f"{run_id}_base_zero_after_motion"
    actions.append(
        action_record(
            base_action_id,
            ACTION_BASE_ZERO_AFTER_MOTION,
            False,
            {"source": "guarded policy report base_zero_after"},
        )
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

    step7c_action_id = f"{run_id}_step7c_flow"
    actions.append(
        action_record(
            step7c_action_id,
            ACTION_STEP7C_FLOW,
            True,
            {
                "arm_mode": step7c_summary.get("arm_mode"),
                "arm_hardware_requested": arm_hw_enabled,
            },
        )
    )
    results.append(
        result_record(
            step7c_action_id,
            ACTION_STEP7C_FLOW,
            "succeeded" if d435_ok and map_ok and arm_ok else "failed_safe",
            step7c_summary.get("base_zero_ok_before_capture"),
            {"step7c_episode_report": str(output_dir / "step7c_after_motion" / "episode_report.json")},
            step7c_summary,
            None if d435_ok and map_ok and arm_ok else "Step7-C subflow failed after guarded motion",
        )
    )

    return {
        "episode_id": run_id,
        "episode_kind": "step7d_guarded_motion_d435_arm_flow",
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
            "front_min_range_m": policy_summary.get("front_min_min_m"),
            "front_p10_range_m": policy_summary.get("front_p10_min_m"),
            "source": "run_step7d_guarded_motion_d435_arm_flow",
            "notes": [
                "guarded micro-motion is delegated to existing P4 policy tooling",
                "D435/map/arm response is delegated to Step7-C tooling",
                "this runner does not bypass scan_safety_guard",
            ],
        },
        "actions": actions,
        "action_results": results,
        "step7d_flow": {
            "commands": commands,
            "policy_summary": policy_summary,
            "step7c_summary": step7c_summary,
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
            "n10p_safety_stop_observed": policy_summary.get("n10p_safety_stop_observed"),
            "base_zero_ok_after_motion": base_zero_after_motion,
            "final_map_saved": policy_summary.get("final_map_saved"),
            "cumulative_positive_forward_m": policy_summary.get("cumulative_positive_forward_m"),
            "base_zero_ok_before_capture": step7c_summary.get("base_zero_ok_before_capture"),
            "base_zero_ok_before_arm": step7c_summary.get("base_zero_ok_before_arm"),
            "d435_live_capture_executed": step7c_summary.get("d435_live_capture_executed"),
            "risk_point_generated": step7c_summary.get("risk_point_generated"),
            "mock_risk_triggered": step7c_summary.get("mock_risk_triggered"),
            "risk_map_points": step7c_summary.get("risk_map_points"),
            "projected": step7c_summary.get("projected"),
            "arm_candidate_selected": step7c_summary.get("arm_candidate_selected"),
            "arm_execution_status": step7c_summary.get("arm_execution_status"),
            "selected_sequence": step7c_summary.get("selected_sequence"),
            "hardware_executed": step7c_summary.get("hardware_executed"),
            "serial_port_opened": step7c_summary.get("serial_port_opened"),
            "serial_bytes_written": step7c_summary.get("serial_bytes_written"),
            "published_cmd_vel": False,
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
        "# Step7-D Guarded Motion D435 Arm Flow",
        "",
        "## Summary",
        "",
        f"- episode_id: `{report.get('episode_id')}`",
        f"- status: `{summary.get('status')}`",
        f"- guarded_motion_enabled: `{summary.get('guarded_motion_enabled')}`",
        f"- guarded_motion_executed: `{summary.get('guarded_motion_executed')}`",
        f"- motion_command_path: `{summary.get('motion_command_path')}`",
        f"- direct_cmd_vel_bypass: `{summary.get('direct_cmd_vel_bypass')}`",
        f"- policy step/executed count: `{summary.get('policy_step_count')}/{summary.get('policy_executed_count')}`",
        f"- policy_sequence_stop_reason: `{summary.get('policy_sequence_stop_reason')}`",
        f"- n10p_safety_stop_observed: `{summary.get('n10p_safety_stop_observed')}`",
        f"- base_zero_ok_after_motion: `{summary.get('base_zero_ok_after_motion')}`",
        f"- final_map_saved: `{summary.get('final_map_saved')}`",
        f"- cumulative_positive_forward_m: `{summary.get('cumulative_positive_forward_m')}`",
        f"- d435_live_capture_executed: `{summary.get('d435_live_capture_executed')}`",
        f"- mock_risk_triggered: `{summary.get('mock_risk_triggered')}`",
        f"- risk_map_points/projected: `{summary.get('risk_map_points')}/{summary.get('projected')}`",
        f"- arm_execution_status: `{summary.get('arm_execution_status')}`",
        f"- selected_sequence: `{summary.get('selected_sequence')}`",
        f"- hardware_executed: `{summary.get('hardware_executed')}`",
        f"- serial_bytes_written: `{summary.get('serial_bytes_written')}`",
        f"- contact_allowed: `{summary.get('contact_allowed')}`",
        f"- obstacle_removed: `{summary.get('obstacle_removed')}`",
        "",
        "## Evidence",
        "",
        "- `guarded_motion/`",
        "- `step7c_after_motion/episode_report.json`",
        "- `episode_report.json`",
        "- `step7d_report.md`",
        "- `llm_a_report/risk_report.md`",
        "- `errors.json`",
        "",
        "## Claim Boundary",
        "",
        "- allowed: guarded micro-motion through the existing P4 policy and N10P safety guard chain",
        "- allowed: base-zero verification after guarded motion",
        "- allowed: D435 HOLD_CAPTURE after base-zero",
        "- allowed: deterministic mock risk trigger and approximate map projection",
        "- allowed: Arm-C0 dry-run or one explicit no-load Arm-C1 hardware response when source evidence says so",
        "- disallowed: direct cmd_vel bypass",
        "- disallowed: autonomous navigation/path-planning success claim",
        "- disallowed: SLAM/high-precision map claim",
        "- disallowed: visual detection accuracy claim",
        "- disallowed: grasping, contact, payload handling, or obstacle removal",
        "- disallowed: LLM control of the robot",
        "",
    ]
    return "\n".join(lines)


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
    parser.add_argument("--enable-guarded-motion", action="store_true")
    parser.add_argument("--confirm-guarded-micro-motion", action="store_true")
    parser.add_argument("--confirm-n10p-safety", action="store_true")
    parser.add_argument("--confirm-no-direct-cmd-vel", action="store_true")
    parser.add_argument("--dry-run-arm", action="store_true")
    parser.add_argument("--enable-arm-hardware-once", action="store_true")
    parser.add_argument("--confirm-map-gated-no-load", action="store_true")
    parser.add_argument("--confirm-no-contact", action="store_true")
    parser.add_argument("--confirm-base-zero-live", action="store_true")
    parser.add_argument("--confirm-no-cmd-vel", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    run_id = f"step7d_guarded_motion_{slug_time()}"
    gate_errors = confirmation_errors(args)
    motion_enabled = guarded_motion_enabled(args) and not gate_errors
    arm_hw_enabled = arm_hardware_enabled(args) and not gate_errors
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
        else next_available_dir(
            DEFAULT_OUTPUT_ROOT,
            "motion_arm_hw" if motion_enabled and arm_hw_enabled else "motion_arm_dryrun" if motion_enabled else "dryrun",
        )
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

    step7c_report: Dict[str, Any] = {}
    if policy_report and base_zero_after_motion and not errors:
        step7c_dir = output_dir / "step7c_after_motion"
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
                "02_step7c_after_motion",
                step7c_command,
                ROOT,
                output_dir,
                errors,
            )
        )
        step7c_report = read_json_or_error(
            step7c_dir / "episode_report.json", "step7c_after_motion", errors
        )
    elif not base_zero_after_motion:
        errors.append(
            {
                "timestamp": now_iso(),
                "stage": "base_zero_after_motion",
                "error": "base_zero_ok_after_motion is not true; skipping D435/map/arm flow",
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
        step7c_report=step7c_report,
        motion_enabled=motion_enabled,
        arm_hw_enabled=arm_hw_enabled,
        errors=errors,
    )
    write_json(output_dir / "episode_report.json", report)
    write_json(output_dir / "errors.json", errors)
    write_text(output_dir / "step7d_report.md", render_report(report))
    write_text(
        output_dir / "README.md",
        "# Step7-D Guarded Motion D435 Arm Flow\n\n"
        f"Status: `{report.get('summary', {}).get('status')}`\n",
    )

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
        "guarded_motion_enabled": report.get("summary", {}).get("guarded_motion_enabled"),
        "guarded_motion_executed": report.get("summary", {}).get("guarded_motion_executed"),
        "policy_step_count": report.get("summary", {}).get("policy_step_count"),
        "policy_executed_count": report.get("summary", {}).get("policy_executed_count"),
        "policy_sequence_stop_reason": report.get("summary", {}).get("policy_sequence_stop_reason"),
        "n10p_safety_stop_observed": report.get("summary", {}).get("n10p_safety_stop_observed"),
        "base_zero_ok_after_motion": report.get("summary", {}).get("base_zero_ok_after_motion"),
        "d435_live_capture_executed": report.get("summary", {}).get("d435_live_capture_executed"),
        "risk_map_points": report.get("summary", {}).get("risk_map_points"),
        "arm_execution_status": report.get("summary", {}).get("arm_execution_status"),
        "hardware_executed": report.get("summary", {}).get("hardware_executed"),
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
