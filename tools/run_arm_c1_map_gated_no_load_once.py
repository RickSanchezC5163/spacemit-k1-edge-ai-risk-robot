#!/usr/bin/env python3
"""Arm-C1: map-gated no-load arm execution gate.

Default mode is dry-run evidence generation. The script opens the serial port
and writes no-load motion frames only when all explicit hardware confirmation
flags are present and the live base-zero evidence gate passes.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))
sys.path.insert(0, str(PROJECT_ROOT / "tools"))

from arm_safety import ArmSafety, MultiServoCommand  # noqa: E402
from run_arm_b3_no_load_sample_sequence import (  # noqa: E402
    SAMPLE_SEQUENCE,
    audit_serial_port,
    enable_runtime_hardware_gates,
    query_battery_voltage,
    write_frame,
)


ACTION_TYPE = "ARM_C1_MAP_GATED_NO_LOAD_ONCE"
SELECTED_ACTION = "ARM_SAMPLE_NO_LOAD"
SELECTED_SEQUENCE = "arm_b3_8_step_safety_adjusted_no_load_sample"
TARGET_POSE_NAME = "safe_idle_home_like_6b"
PHASE = "arm_c_no_load_home_cycle"
LIVE_BASE_ZERO_EVIDENCE_TYPE = "live_base_zero_observation"
DEFAULT_BASE_ZERO_MAX_AGE_S = 60.0
DEFAULT_SERIAL_PORT = "/dev/arm_bus"
DEFAULT_OUTPUT_BASE = PROJECT_ROOT / "outputs" / "arm_c1_map_gated_no_load_once_v1"


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def run_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def find_candidate(arm_c0_episode: Dict[str, Any], candidate_id: str) -> Optional[Dict[str, Any]]:
    for candidate in arm_c0_episode.get("map_gated_arm_candidates") or []:
        if candidate.get("candidate_id") == candidate_id:
            return candidate
    for result in arm_c0_episode.get("action_results") or []:
        details = result.get("details") or {}
        if details.get("candidate_id") == candidate_id:
            return details
    return None


def candidate_gate(candidate: Optional[Dict[str, Any]]) -> Tuple[bool, List[str]]:
    if candidate is None:
        return False, ["candidate not found"]
    failures: List[str] = []
    required_values = {
        "status": "succeeded_dry_run",
        "selected_action": SELECTED_ACTION,
        "selected_sequence": SELECTED_SEQUENCE,
        "validated_no_load_action": True,
        "map_projection_valid": True,
        "contact_allowed": False,
        "obstacle_removed": False,
        "hardware_executed": False,
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "published_cmd_vel": False,
    }
    for key, expected in required_values.items():
        if candidate.get(key) != expected:
            failures.append(f"candidate.{key}={candidate.get(key)!r}, expected {expected!r}")
    if candidate.get("block_reasons"):
        failures.append(f"candidate has block_reasons={candidate.get('block_reasons')}")
    return not failures, failures


def read_home_pose(config: Dict[str, Any]) -> Dict[int, int]:
    pose = config["poses"][TARGET_POSE_NAME]
    servos = {int(k): int(v) for k, v in pose["servos"].items()}
    if set(servos) != {1, 2, 3, 4, 5}:
        raise ValueError(f"{TARGET_POSE_NAME} must contain servo IDs 1-5")
    return servos


def extract_base_zero_ok(raw: Dict[str, Any]) -> Optional[bool]:
    for key in ("base_zero_ok_before_arm", "base_zero_ok"):
        if key in raw:
            return raw.get(key) is True
    base_zero = raw.get("base_zero") or {}
    if "base_zero_ok" in base_zero:
        return base_zero.get("base_zero_ok") is True
    policy_state = raw.get("policy_state") or {}
    if "base_zero_ok" in policy_state:
        return policy_state.get("base_zero_ok") is True
    return None


def extract_published_cmd_vel(raw: Dict[str, Any]) -> Optional[bool]:
    for key in ("published_cmd_vel", "published_cmd_vel_before", "published_cmd_vel_during_arm"):
        if key in raw:
            return raw.get(key) is True
    base_zero = raw.get("base_zero") or {}
    if "published_cmd_vel" in base_zero:
        return base_zero.get("published_cmd_vel") is True
    summary = raw.get("summary") or {}
    if "published_cmd_vel" in summary:
        return summary.get("published_cmd_vel") is True
    return None


def evidence_age_s(generated_at: Any) -> Optional[float]:
    if not isinstance(generated_at, str) or not generated_at:
        return None
    try:
        parsed = datetime.fromisoformat(generated_at.replace("Z", "+00:00"))
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        return max(0.0, (datetime.now(timezone.utc) - parsed).total_seconds())
    except ValueError:
        return None


def evaluate_base_zero_evidence(path: Optional[Path]) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "provided": path is not None,
        "path": str(path) if path else None,
        "loaded": False,
        "evidence_type": None,
        "valid_for_arm_c1_hardware": None,
        "generated_at": None,
        "age_s": None,
        "source_mode": None,
        "base_zero_checked_live": None,
        "base_zero_ok_before_arm": None,
        "published_cmd_vel": None,
        "raw": None,
        "error": None,
    }
    if path is None:
        return result
    try:
        raw = load_json(path)
        if not isinstance(raw, dict):
            raise ValueError("base-zero evidence must be a JSON object")
        result["loaded"] = True
        result["raw"] = raw
        result["evidence_type"] = raw.get("evidence_type")
        result["valid_for_arm_c1_hardware"] = raw.get("valid_for_arm_c1_hardware")
        result["generated_at"] = raw.get("generated_at")
        result["age_s"] = evidence_age_s(raw.get("generated_at"))
        result["source_mode"] = raw.get("source_mode")
        result["base_zero_checked_live"] = raw.get("base_zero_checked_live")
        result["base_zero_ok_before_arm"] = extract_base_zero_ok(raw)
        result["published_cmd_vel"] = extract_published_cmd_vel(raw)
    except Exception as exc:  # noqa: BLE001 - evidence parser must record failures.
        result["error"] = str(exc)
    return result


def hardware_requested(args: argparse.Namespace) -> bool:
    return any(
        (
            args.enable_hardware_write,
            args.confirm_map_gated_no_load,
            args.confirm_no_contact,
            args.confirm_base_zero_live,
            args.confirm_no_cmd_vel,
        )
    )


def hardware_enabled(args: argparse.Namespace) -> bool:
    return all(
        (
            args.enable_hardware_write,
            args.confirm_map_gated_no_load,
            args.confirm_no_contact,
            args.confirm_base_zero_live,
            args.confirm_no_cmd_vel,
        )
    )


def confirmation_gate(args: argparse.Namespace) -> Tuple[bool, List[str]]:
    required = {
        "--enable-hardware-write": args.enable_hardware_write,
        "--confirm-map-gated-no-load": args.confirm_map_gated_no_load,
        "--confirm-no-contact": args.confirm_no_contact,
        "--confirm-base-zero-live": args.confirm_base_zero_live,
        "--confirm-no-cmd-vel": args.confirm_no_cmd_vel,
    }
    missing = [name for name, enabled in required.items() if not enabled]
    if missing:
        return False, [f"hardware path missing confirmation flags: {', '.join(missing)}"]
    return True, []


def build_sequence(
    config_path: Path,
    hardware_enabled_for_frames: bool,
    base_zero_ok: bool,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], List[str], List[str]]:
    safety = ArmSafety(str(config_path))
    phase_result = safety.set_phase(PHASE)
    errors: List[str] = []
    warnings: List[str] = []
    if not phase_result.allowed:
        errors.append(phase_result.reason)
    safety.update_base_zero(base_zero_ok)
    if hardware_enabled_for_frames:
        enable_runtime_hardware_gates(safety)

    actions: List[Dict[str, Any]] = []
    frames: List[Dict[str, Any]] = []
    for index, (label, duration_ms, servos) in enumerate(SAMPLE_SEQUENCE, start=1):
        command = MultiServoCommand(servos=servos, time_ms=duration_ms, label=label)
        validation = safety.validate_all(command)
        if not validation.allowed:
            errors.append(f"{label} validation failed: {validation.reason}")
        if validation.warnings:
            warnings.extend(f"{label}: {warning}" for warning in validation.warnings)
        frame_info = safety.build_move_frame(command)
        frames.append(frame_info)
        safety.record_multi(command)
        actions.append(
            {
                "step_index": index,
                "step_name": label,
                "duration_ms": duration_ms,
                "servos": servos,
                "frame_hex": frame_info.get("frame_hex"),
                "serial_write_allowed_effective": frame_info.get("serial_write_allowed_effective"),
            }
        )
    return actions, frames, errors, warnings


def write_readme(output_dir: Path, status_doc: Dict[str, Any]) -> None:
    text = f"""# Arm-C1 Map-Gated No-Load Once

This directory contains Arm-C1 gate-script evidence.

## Result

- status: `{status_doc.get('status')}`
- dry_run: `{str(status_doc.get('dry_run')).lower()}`
- hardware_requested: `{str(status_doc.get('hardware_requested')).lower()}`
- hardware_executed: `{str(status_doc.get('hardware_executed')).lower()}`
- selected_candidate_id: `{status_doc.get('selected_candidate_id')}`
- base_zero_ok_before_arm: `{status_doc.get('base_zero_ok_before_arm')}`
- serial_port_opened: `{str(status_doc.get('serial_port_opened')).lower()}`
- serial_bytes_written: `{status_doc.get('serial_bytes_written')}`
- published_cmd_vel: `{str(status_doc.get('published_cmd_vel')).lower()}`
- contact_allowed: `false`
- obstacle_removed: `false`

## Boundary

- Default mode is dry-run.
- Hardware writes require all explicit confirmation flags.
- Real arm execution is not claimed unless `hardware_executed=true` and the
  physical confirmation file is completed by the operator.
- Contact, grasping, payload handling, and obstacle removal are not allowed.
- LLM output and map output do not directly control hardware.
"""
    write_text(output_dir / "README.md", text)


def physical_confirmation_template(hardware_executed: bool) -> Dict[str, Any]:
    return {
        "required_for_hardware_claim": True,
        "hardware_executed": hardware_executed,
        "physical_actuation_observed": None if hardware_executed else False,
        "returned_to_6b_observed": None if hardware_executed else False,
        "physical_issue_observed": None if hardware_executed else False,
        "contact_observed": None if hardware_executed else False,
        "abnormal_sound_observed": None if hardware_executed else False,
        "binding_or_stall_observed": None if hardware_executed else False,
        "visible_overheating_observed": None if hardware_executed else False,
        "operator_notes": "",
    }


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arm-c0-episode-report",
        required=True,
        help="Arm-C0 episode_report.json containing map-gated candidates.",
    )
    parser.add_argument("--candidate-id", required=True)
    parser.add_argument("--base-zero-evidence", default=None)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs" / "arm_safety_config.json"))
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--base-zero-max-age-s", type=float, default=DEFAULT_BASE_ZERO_MAX_AGE_S)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--enable-hardware-write", action="store_true")
    parser.add_argument("--confirm-map-gated-no-load", action="store_true")
    parser.add_argument("--confirm-no-contact", action="store_true")
    parser.add_argument("--confirm-base-zero-live", action="store_true")
    parser.add_argument("--confirm-no-cmd-vel", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    run_id = f"arm_c1_map_gated_no_load_once_{run_stamp()}"
    output_dir = Path(args.output_dir) if args.output_dir else DEFAULT_OUTPUT_BASE / run_id
    output_dir.mkdir(parents=True, exist_ok=True)

    started_at = now_iso()
    errors: List[str] = []
    warnings: List[str] = []
    arm_c0_path = Path(args.arm_c0_episode_report)
    config_path = Path(args.config)
    base_zero_path = Path(args.base_zero_evidence) if args.base_zero_evidence else None
    requested_hw = hardware_requested(args)
    enabled_hw = hardware_enabled(args)
    dry_run = not enabled_hw

    arm_c0_report = load_json(arm_c0_path)
    candidate = find_candidate(arm_c0_report, args.candidate_id)
    candidate_ok, candidate_errors = candidate_gate(candidate)
    if not candidate_ok:
        errors.extend(candidate_errors)

    confirm_ok = True
    if requested_hw:
        confirm_ok, confirm_errors = confirmation_gate(args)
        if not confirm_ok:
            errors.extend(confirm_errors)

    base_zero_evidence = evaluate_base_zero_evidence(base_zero_path)
    base_zero_ok_before_arm = None
    published_cmd_vel = False
    if base_zero_evidence.get("provided") and not base_zero_evidence.get("loaded"):
        errors.append(f"base-zero evidence failed to load: {base_zero_evidence.get('error')}")
    elif base_zero_evidence.get("loaded"):
        base_zero_ok_before_arm = base_zero_evidence.get("base_zero_ok_before_arm")
        if base_zero_evidence.get("published_cmd_vel") is not None:
            published_cmd_vel = bool(base_zero_evidence.get("published_cmd_vel"))

    if enabled_hw:
        if not base_zero_evidence.get("provided"):
            errors.append("--base-zero-evidence is required for Arm-C1 hardware execution")
        elif not base_zero_evidence.get("loaded"):
            pass
        elif base_zero_evidence.get("evidence_type") != LIVE_BASE_ZERO_EVIDENCE_TYPE:
            errors.append(
                "Arm-C1 hardware requires live base-zero evidence; "
                f"got evidence_type={base_zero_evidence.get('evidence_type')!r}"
            )
        elif base_zero_evidence.get("valid_for_arm_c1_hardware") is not True:
            errors.append("base-zero evidence is not marked valid_for_arm_c1_hardware=true")
        elif base_zero_evidence.get("age_s") is None:
            errors.append("base-zero evidence generated_at is missing or unparsable")
        elif float(base_zero_evidence.get("age_s")) > float(args.base_zero_max_age_s):
            errors.append(
                "base-zero evidence is stale: "
                f"age_s={base_zero_evidence.get('age_s'):.1f}, "
                f"max_age_s={args.base_zero_max_age_s:.1f}"
            )
        elif base_zero_evidence.get("base_zero_ok_before_arm") is not True:
            errors.append("base_zero_ok_before_arm is not true in base-zero evidence")
        elif base_zero_evidence.get("published_cmd_vel") is not False:
            errors.append("base-zero evidence does not prove published_cmd_vel=false")
        else:
            base_zero_ok_before_arm = True
        published_cmd_vel = bool(base_zero_evidence.get("published_cmd_vel") is True)

    config = load_json(config_path)
    home_pose = read_home_pose(config)
    if SAMPLE_SEQUENCE[-1][2] != home_pose:
        errors.append("Arm-C1 sequence must end at safe_idle_home_like_6b")
    if any(step[1] < 1000 for step in SAMPLE_SEQUENCE):
        errors.append("all Arm-C1 sequence durations must be >= 1000 ms")

    port_audit = audit_serial_port(args.serial_port)
    if enabled_hw and not port_audit.get("exists"):
        errors.append(f"serial port does not exist: {args.serial_port}")

    sequence_actions, frame_infos, validation_errors, validation_warnings = build_sequence(
        config_path=config_path,
        hardware_enabled_for_frames=False,
        base_zero_ok=True if not enabled_hw else base_zero_ok_before_arm is True,
    )
    errors.extend(validation_errors)
    warnings.extend(validation_warnings)

    if enabled_hw and not errors:
        sequence_actions, frame_infos, validation_errors, validation_warnings = build_sequence(
            config_path=config_path,
            hardware_enabled_for_frames=True,
            base_zero_ok=True,
        )
        errors.extend(validation_errors)
        warnings.extend(validation_warnings)

    voltage_query = None
    writes: List[Optional[Dict[str, Any]]] = [None for _ in sequence_actions]
    if enabled_hw and not errors:
        voltage_query = query_battery_voltage(args.serial_port, args.baudrate)
        if not voltage_query.get("controller_response_observed", False):
            errors.append("controller voltage query did not receive a valid response")

    if enabled_hw and not errors:
        for index, (step, frame_info) in enumerate(zip(sequence_actions, frame_infos)):
            if not frame_info.get("serial_write_allowed_effective", False):
                errors.append(f"{step['step_name']} serial_write_allowed_effective=false")
                break
            frame = frame_info.get("frame_bytes")
            if frame is None:
                errors.append(f"{step['step_name']} frame_bytes is None")
                break
            writes[index] = write_frame(args.serial_port, args.baudrate, frame)
            if not writes[index].get("write_ok"):
                errors.append(f"{step['step_name']} serial write failed: {writes[index].get('error')}")
                break
            time.sleep(float(step["duration_ms"]) / 1000.0)

    step_ok = [bool(write and write.get("write_ok")) for write in writes]
    hardware_executed = bool(enabled_hw and not errors and all(step_ok))
    if errors:
        status = "failed_safe"
    elif dry_run:
        status = "succeeded_dry_run"
    else:
        status = "succeeded"

    serial_port_opened = any(bool(write and write.get("serial_port_opened")) for write in writes)
    serial_bytes_written = sum(int(write.get("serial_bytes_written", 0)) for write in writes if write)
    if voltage_query:
        serial_port_opened = serial_port_opened or bool(voltage_query.get("serial_port_opened"))
        serial_bytes_written += int(voltage_query.get("serial_bytes_written", 0) or 0)

    action_id = f"{run_id}_action_01"
    action = {
        "action_id": action_id,
        "action_type": ACTION_TYPE,
        "requested_at": started_at,
        "requires_base_zero": True,
        "publishes_cmd_vel": False,
        "reason": "Arm-C1 map-gated no-load hardware gate",
        "params": {
            "candidate_id": args.candidate_id,
            "selected_action": candidate.get("selected_action") if candidate else None,
            "selected_sequence": candidate.get("selected_sequence") if candidate else None,
            "hardware_execution_allowed": enabled_hw,
            "no_contact_confirmed": bool(args.confirm_no_contact),
            "no_cmd_vel_confirmed": bool(args.confirm_no_cmd_vel),
            "base_zero_max_age_s": float(args.base_zero_max_age_s),
        },
    }
    action_result = {
        "action_id": action_id,
        "action_type": ACTION_TYPE,
        "status": status,
        "started_at": started_at,
        "ended_at": now_iso(),
        "base_zero_ok_before": base_zero_ok_before_arm,
        "base_zero_ok_before_arm": base_zero_ok_before_arm,
        "published_cmd_vel": published_cmd_vel,
        "evidence_paths": {
            "arm_c0_episode_report": str(arm_c0_path),
            "base_zero_evidence": str(base_zero_path) if base_zero_path else None,
            "selected_candidate": str(output_dir / "selected_candidate.json"),
            "sent_frame_hex": str(output_dir / "sent_frame_hex.txt"),
            "arm_c1_status": str(output_dir / "arm_c1_status.json"),
        },
        "details": {
            "candidate_id": args.candidate_id,
            "selected_action": candidate.get("selected_action") if candidate else None,
            "selected_sequence": candidate.get("selected_sequence") if candidate else None,
            "dry_run": dry_run,
            "hardware_requested": requested_hw,
            "hardware_executed": hardware_executed,
            "serial_port_opened": serial_port_opened,
            "serial_bytes_written": serial_bytes_written,
            "contact_allowed": False,
            "obstacle_removed": False,
            "base_zero_evidence": {
                key: value for key, value in base_zero_evidence.items() if key != "raw"
            },
            "live_base_zero_evidence_required_for_hardware": True,
            "base_zero_max_age_s": float(args.base_zero_max_age_s),
            "candidate_gate_passed": candidate_ok,
            "confirmation_gate_passed": confirm_ok,
            "step_ok": step_ok,
            "phase": PHASE,
            "projection_precision": candidate.get("projection_precision") if candidate else None,
        },
        "error": "; ".join(errors) if errors else None,
    }
    episode_report = {
        "episode_id": run_id,
        "started_at": started_at,
        "ended_at": now_iso(),
        "protocol_version": "arm_c1_map_gated_no_load_once_v1",
        "policy_state": {
            "state_id": f"{run_id}_state",
            "timestamp": now_iso(),
            "base_zero_ok": base_zero_ok_before_arm,
            "base_zero": {
                "required_for_real_arm_execution": True,
                "checked": bool(
                    base_zero_evidence.get("loaded")
                    and base_zero_evidence.get("evidence_type") == LIVE_BASE_ZERO_EVIDENCE_TYPE
                ),
                "base_zero_ok_before_arm": base_zero_ok_before_arm,
                "evidence_path": str(base_zero_path) if base_zero_path else None,
                "evidence_type": base_zero_evidence.get("evidence_type"),
                "valid_for_arm_c1_hardware": base_zero_evidence.get("valid_for_arm_c1_hardware"),
                "evidence_age_s": base_zero_evidence.get("age_s"),
                "max_age_s": float(args.base_zero_max_age_s),
                "check_mode": "external_live_evidence_required",
            },
            "odom": None,
            "front_min_range_m": None,
            "front_p10_range_m": None,
            "source": "run_arm_c1_map_gated_no_load_once",
            "notes": [
                "Arm-C1 gate script",
                "no ROS process is started by this script",
                "no cmd_vel publisher is created by this script",
                "contact and obstacle removal are not allowed",
            ],
        },
        "actions": [action],
        "action_results": [action_result],
        "summary": {
            "candidate_id": args.candidate_id,
            "dry_run": dry_run,
            "hardware_requested": requested_hw,
            "hardware_executed": hardware_executed,
            "serial_port_opened": serial_port_opened,
            "serial_bytes_written": serial_bytes_written,
            "published_cmd_vel": published_cmd_vel,
            "contact_allowed": False,
            "obstacle_removed": False,
            "base_zero_required": True,
            "base_zero_ok_before_arm": base_zero_ok_before_arm,
            "candidate_gate_passed": candidate_ok,
            "confirmation_gate_passed": confirm_ok,
            "step_count": len(sequence_actions),
            "step_success_count": sum(1 for ok in step_ok if ok),
            "status": status,
        },
        "errors": [
            {"timestamp": now_iso(), "stage": "arm_c1_gate", "error": error}
            for error in errors
        ],
        "output_root": str(output_dir),
    }
    status_doc = {
        "generated_at": now_iso(),
        "status": status,
        "dry_run": dry_run,
        "hardware_requested": requested_hw,
        "hardware_enabled": enabled_hw,
        "hardware_executed": hardware_executed,
        "selected_candidate_id": args.candidate_id,
        "base_zero_ok_before_arm": base_zero_ok_before_arm,
        "published_cmd_vel": published_cmd_vel,
        "serial_port": args.serial_port,
        "serial_port_opened": serial_port_opened,
        "serial_bytes_written": serial_bytes_written,
        "candidate_gate_passed": candidate_ok,
        "candidate_gate_errors": candidate_errors,
        "base_zero_evidence": {
            key: value for key, value in base_zero_evidence.items() if key != "raw"
        },
        "serial_audit": port_audit,
        "voltage_query": voltage_query,
        "base_zero_max_age_s": float(args.base_zero_max_age_s),
        "writes": writes,
        "frame_infos": [{k: v for k, v in info.items() if k != "frame_bytes"} for info in frame_infos],
        "sequence": sequence_actions,
        "errors": errors,
        "warnings": warnings,
    }

    frame_lines = [
        f"{step['step_index']:02d} {step['step_name']} {step['duration_ms']}ms {step['frame_hex']}"
        for step in sequence_actions
    ]
    write_json(output_dir / "selected_candidate.json", candidate or {})
    write_json(output_dir / "action_result.json", action_result)
    write_json(output_dir / "episode_report.json", episode_report)
    write_json(output_dir / "arm_c1_status.json", status_doc)
    write_json(output_dir / "errors.json", episode_report["errors"])
    write_json(output_dir / "physical_actuation_confirmation.json", physical_confirmation_template(hardware_executed))
    write_text(output_dir / "sent_frame_hex.txt", "\n".join(frame_lines) + "\n")
    write_readme(output_dir, status_doc)

    print(
        json.dumps(
            {
                "ok": status in ("succeeded", "succeeded_dry_run"),
                "status": status,
                "dry_run": dry_run,
                "candidate_id": args.candidate_id,
                "base_zero_ok_before_arm": base_zero_ok_before_arm,
                "hardware_executed": hardware_executed,
                "serial_port_opened": serial_port_opened,
                "serial_bytes_written": serial_bytes_written,
                "published_cmd_vel": published_cmd_vel,
                "contact_allowed": False,
                "obstacle_removed": False,
                "output_dir": str(output_dir),
                "errors": errors,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if status in ("succeeded", "succeeded_dry_run") else 1


if __name__ == "__main__":
    raise SystemExit(main())
