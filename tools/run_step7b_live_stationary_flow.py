#!/usr/bin/env python3
"""Step7-B0 live stationary integration runner.

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
DEFAULT_OUTPUT_ROOT = ROOT / "outputs" / "step7b_live_stationary_flow_v1"
PROTOCOL_VERSION = "step7b_live_stationary_flow_v1"
ACTION_BASE_ZERO = "STEP7B_LIVE_BASE_ZERO_GATE"
ACTION_HOLD_CAPTURE = "STEP7B_LIVE_D435_HOLD_CAPTURE"
ACTION_MAP_PROJECT = "STEP7B_MAP_A0_LIVE_PROJECTION"
ACTION_ARM_C0 = "STEP7B_ARM_C0_LIVE_DRYRUN"
ACTION_ARM_C1_OPTIONAL = "STEP7B_ARM_C1_OPTIONAL_NO_LOAD_ONCE"


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


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
        "reason": "Step7-B0 live stationary integration",
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
    map_package: Dict[str, Any],
    arm_c0_package: Dict[str, Any],
    arm_c1_report: Optional[Dict[str, Any]],
    errors: List[Dict[str, Any]],
) -> Dict[str, Any]:
    started_at = now_iso()
    base_zero_ok = base_zero.get("base_zero_ok_before_arm") is True
    p4x_summary = summarize_p4x(p4x_report)
    map_summary = summarize_map(map_package)
    arm_c0_summary = summarize_arm_c0(arm_c0_package)
    arm_c1_summary = (arm_c1_report or {}).get("summary") or {}

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
            {"p4x_episode_report": str(output_dir / "p4x_live_hold_capture" / "episode_report.json")},
            p4x_summary,
            None if p4x_ok else "live P4-X HOLD_CAPTURE did not meet Step7-B0 gate",
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
            {"risk_map_points": str(output_dir / "map_a0_live_projection" / "risk_map_points.json")},
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
            None,
            {"arm_c0_episode_report": str(output_dir / "arm_c0_live_dryrun" / "episode_report.json")},
            arm_c0_summary,
            None if arm_c0_ok else "Arm-C0 live dry-run candidate generation blocked",
        )
    )

    if arm_c1_report is not None:
        arm_c1_action_id = f"{run_id}_arm_c1_optional"
        arm_c1_started_at = now_iso()
        arm_c1_ok = arm_c1_summary.get("status") == "succeeded"
        actions.append(
            action_record(
                arm_c1_action_id,
                ACTION_ARM_C1_OPTIONAL,
                arm_c1_started_at,
                True,
                {"candidate_id": arm_c1_summary.get("candidate_id"), "hardware_execution_allowed": True},
            )
        )
        results.append(
            result_record(
                arm_c1_action_id,
                ACTION_ARM_C1_OPTIONAL,
                "succeeded" if arm_c1_ok else "failed_safe",
                arm_c1_started_at,
                arm_c1_summary.get("base_zero_ok_before_arm"),
                {"arm_c1_episode_report": str(output_dir / "arm_c1_optional_hw_once" / "episode_report.json")},
                arm_c1_summary,
                None if arm_c1_ok else "optional Arm-C1-H no-load execution failed safe",
            )
        )

    top_ok = bool(base_zero_ok and p4x_ok and map_ok and arm_c0_ok and not errors)
    if arm_c1_report is not None:
        top_ok = top_ok and arm_c1_summary.get("status") == "succeeded"

    return {
        "episode_id": run_id,
        "episode_kind": "step7b_live_stationary_flow",
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
            "source": "run_step7b_live_stationary_flow",
            "notes": [
                "live stationary integration",
                "guarded stack must be running before this script starts",
                "this script does not publish cmd_vel",
                "Arm-C1-H is optional and disabled unless explicit flags are provided",
            ],
        },
        "actions": actions,
        "action_results": results,
        "step7b_flow": {
            "commands": commands,
            "base_zero_evidence": base_zero,
            "p4x_summary": p4x_summary,
            "map_a0_summary": map_summary,
            "arm_c0_summary": arm_c0_summary,
            "arm_c1_optional_summary": arm_c1_summary if arm_c1_report else None,
        },
        "summary": {
            "status": "succeeded" if top_ok else "failed_safe",
            "live_stationary": True,
            "base_zero_ok_before_capture": base_zero_ok,
            "d435_live_capture_executed": p4x_ok,
            "risk_map_points": map_summary.get("risk_map_points"),
            "projected": map_summary.get("projected"),
            "arm_c0_candidates": arm_c0_summary.get("candidates"),
            "arm_c0_succeeded_dry_run": arm_c0_summary.get("succeeded_dry_run"),
            "arm_c0_blocked": arm_c0_summary.get("blocked"),
            "arm_c1_hardware_executed": bool(arm_c1_summary.get("hardware_executed") is True),
            "hardware_executed": bool(arm_c1_summary.get("hardware_executed") is True),
            "serial_port_opened": bool(arm_c1_summary.get("serial_port_opened") is True),
            "serial_bytes_written": int(arm_c1_summary.get("serial_bytes_written") or 0),
            "published_cmd_vel": False,
            "contact_allowed": False,
            "obstacle_removed": False,
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
        "# Step7-B0 Live Stationary Flow",
        "",
        "## Summary",
        "",
        f"- episode_id: `{report.get('episode_id')}`",
        f"- status: `{summary.get('status')}`",
        f"- base_zero_ok_before_capture: `{summary.get('base_zero_ok_before_capture')}`",
        f"- d435_live_capture_executed: `{summary.get('d435_live_capture_executed')}`",
        f"- risk_map_points/projected: `{summary.get('risk_map_points')}/{summary.get('projected')}`",
        f"- arm_c0 candidates/succeeded/blocked: `{summary.get('arm_c0_candidates')}/{summary.get('arm_c0_succeeded_dry_run')}/{summary.get('arm_c0_blocked')}`",
        f"- arm_c1_hardware_executed: `{summary.get('arm_c1_hardware_executed')}`",
        f"- published_cmd_vel: `{summary.get('published_cmd_vel')}`",
        f"- serial_port_opened: `{summary.get('serial_port_opened')}`",
        f"- serial_bytes_written: `{summary.get('serial_bytes_written')}`",
        f"- contact_allowed: `{summary.get('contact_allowed')}`",
        f"- obstacle_removed: `{summary.get('obstacle_removed')}`",
        "",
        "## Evidence",
        "",
        "- `base_zero_live/base_zero_evidence.json`",
        "- `p4x_live_hold_capture/episode_report.json`",
        "- `map_a0_live_projection/risk_map_points.json`",
        "- `arm_c0_live_dryrun/episode_report.json`",
        "- `episode_report.json`",
        "- `step7b_live_report.md`",
        "- `errors.json`",
        "",
        "## Claim Boundary",
        "",
        "- allowed: live stationary base-zero gate and D435 HOLD_CAPTURE evidence chain",
        "- allowed: Map-A0 live projection from the newly captured risk_point",
        "- allowed: Arm-C0 map-gated no-load candidate dry-run",
        "- disallowed: chassis motion during Step7-B0",
        "- disallowed: autonomous navigation or path planning success",
        "- disallowed: grasping, contact, payload handling, or obstacle removal",
        "- disallowed: LLM control of the robot",
        "",
    ]
    return "\n".join(lines)


def render_readme(report: Dict[str, Any]) -> str:
    return f"""# Step7-B0 Live Stationary Flow

This directory contains one live stationary integration run.

The runner does not publish `cmd_vel`. By default it does not open serial ports
or control the arm.

Status: `{report.get('summary', {}).get('status')}`
"""


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--candidate-id", default="arm_c0_candidate_001")
    parser.add_argument("--base-zero-max-age-s", type=float, default=60.0)
    parser.add_argument("--enable-arm-c1-hardware-once", action="store_true")
    parser.add_argument("--confirm-arm-c1-hardware-once", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    run_id = f"step7b_live_stationary_{slug_time()}"
    output_dir = Path(str(args.output_dir).strip()) if args.output_dir else DEFAULT_OUTPUT_ROOT / run_id
    output_dir.mkdir(parents=True, exist_ok=True)
    errors: List[Dict[str, Any]] = []
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

    p4x_dir = output_dir / "p4x_live_hold_capture"
    if base_zero_ok:
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
                "error": "base_zero evidence failed; skipping live HOLD_CAPTURE",
            }
        )
    p4x_report = read_json_or_error(p4x_dir / "episode_report.json", "p4x_live_hold_capture", errors)

    map_dir = output_dir / "map_a0_live_projection"
    if p4x_report:
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

    arm_c0_dir = output_dir / "arm_c0_live_dryrun"
    if map_package:
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

    arm_c1_report: Optional[Dict[str, Any]] = None
    if args.enable_arm_c1_hardware_once or args.confirm_arm_c1_hardware_once:
        if not (args.enable_arm_c1_hardware_once and args.confirm_arm_c1_hardware_once):
            errors.append(
                {
                    "timestamp": now_iso(),
                    "stage": "arm_c1_optional_gate",
                    "error": "optional Arm-C1-H requires both --enable-arm-c1-hardware-once and --confirm-arm-c1-hardware-once",
                }
            )
        else:
            arm_c1_dir = output_dir / "arm_c1_optional_hw_once"
            commands.append(
                run_command(
                    "05_arm_c1_optional_hw_once",
                    [
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
                        "--output-dir",
                        str(arm_c1_dir),
                        "--enable-hardware-write",
                        "--confirm-map-gated-no-load",
                        "--confirm-no-contact",
                        "--confirm-base-zero-live",
                        "--confirm-no-cmd-vel",
                    ],
                    ROOT,
                    output_dir,
                    errors,
                )
            )
            arm_c1_report = read_json_or_error(arm_c1_dir / "episode_report.json", "arm_c1_optional_hw_once", errors)

    report = build_episode_report(
        output_dir=output_dir,
        run_id=run_id,
        commands=commands,
        base_zero=base_zero,
        p4x_report=p4x_report,
        map_package=map_package,
        arm_c0_package=arm_c0_package,
        arm_c1_report=arm_c1_report,
        errors=errors,
    )
    write_json(output_dir / "episode_report.json", report)
    write_json(output_dir / "errors.json", errors)
    write_text(output_dir / "step7b_live_report.md", render_report(report))
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
        "base_zero_ok_before_capture": report.get("summary", {}).get("base_zero_ok_before_capture"),
        "d435_live_capture_executed": report.get("summary", {}).get("d435_live_capture_executed"),
        "risk_map_points": report.get("summary", {}).get("risk_map_points"),
        "projected": report.get("summary", {}).get("projected"),
        "arm_c0_candidates": report.get("summary", {}).get("arm_c0_candidates"),
        "arm_c0_succeeded_dry_run": report.get("summary", {}).get("arm_c0_succeeded_dry_run"),
        "arm_c1_hardware_executed": report.get("summary", {}).get("arm_c1_hardware_executed"),
        "published_cmd_vel": False,
        "serial_port_opened": report.get("summary", {}).get("serial_port_opened"),
        "serial_bytes_written": report.get("summary", {}).get("serial_bytes_written"),
        "errors": len(errors),
        "output_dir": str(output_dir),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
