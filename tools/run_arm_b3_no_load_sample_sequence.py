#!/usr/bin/env python3
"""Arm-B3: run the guarded full no-load sample action sequence.

Default mode is dry-run evidence generation. Hardware writes require both
--enable-hardware-write and --confirm-no-load-sample-sequence.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional


PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from arm_safety import ArmSafety, MultiServoCommand  # noqa: E402


PHASE = "arm_b3_full_no_load_sequence"
TARGET_POSE_NAME = "safe_idle_home_like_6b"
DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_OUTPUT_BASE = PROJECT_ROOT / "outputs" / "arm_b3_no_load_sample_sequence_v1"

SAMPLE_SEQUENCE = [
    ("step_1_safe_flat_start", 1500, {1: 499, 2: 770, 3: 457, 4: 500, 5: 494}),
    ("step_2_mid_retract", 1500, {1: 498, 2: 600, 3: 540, 4: 470, 5: 498}),
    ("step_3a_pre_reach", 1500, {1: 498, 2: 400, 3: 590, 4: 470, 5: 496}),
    ("step_3b_reach_no_load", 2000, {1: 498, 2: 250, 3: 646, 4: 470, 5: 494}),
    ("step_4_pre_gripper", 1500, {1: 498, 2: 291, 3: 644, 4: 470, 5: 495}),
    ("step_5_gripper_open_no_object", 1500, {1: 498, 2: 290, 3: 642, 4: 470, 5: 220}),
    ("step_6a_return_mid", 1500, {1: 498, 2: 500, 3: 540, 4: 470, 5: 360}),
    ("step_6b_return_home_like", 2000, {1: 510, 2: 771, 3: 426, 4: 503, 5: 497}),
]


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def run_command(args: List[str]) -> Dict[str, Any]:
    try:
        completed = subprocess.run(
            args,
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
        return {
            "cmd": args,
            "returncode": completed.returncode,
            "stdout": completed.stdout.strip(),
            "stderr": completed.stderr.strip(),
        }
    except Exception as exc:
        return {"cmd": args, "error": str(exc)}


def audit_serial_port(port: str) -> Dict[str, Any]:
    audit: Dict[str, Any] = {
        "port": port,
        "exists": os.path.exists(port),
        "stat": None,
        "udevadm": None,
        "holders": [],
    }
    if audit["exists"]:
        audit["stat"] = run_command(["stat", "-c", "%n %F %a %U %G %t:%T", port])
        audit["udevadm"] = run_command(["udevadm", "info", "-q", "property", "-n", port])
        holders = []
        for cmd in (["fuser", "-v", port], ["lsof", port]):
            result = run_command(cmd)
            if result.get("stdout") or result.get("stderr"):
                holders.append(result)
        audit["holders"] = holders
    return audit


def write_json(path: Path, data: Any) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def load_config(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_home_pose(config: Dict[str, Any]) -> Dict[int, int]:
    pose = config["poses"][TARGET_POSE_NAME]
    servos = {int(k): int(v) for k, v in pose["servos"].items()}
    if set(servos) != {1, 2, 3, 4, 5}:
        raise ValueError(f"{TARGET_POSE_NAME} must contain servo IDs 1-5")
    return servos


def enable_runtime_hardware_gates(safety: ArmSafety) -> None:
    gates = safety.config.setdefault("safety_gates", {})
    gates["arm_enabled"] = True
    gates["hardware_access_allowed"] = True
    gates["serial_write_allowed"] = True
    gates["dry_run"] = False
    gates["contact_allowed"] = False
    gates["obstacle_removal_allowed"] = False


def query_battery_voltage(port: str, baudrate: int) -> Dict[str, Any]:
    frame = bytes([0x55, 0x55, 0x02, 0x0F])
    result: Dict[str, Any] = {
        "query_frame_hex": frame.hex(),
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "response_hex": "",
        "response_len": 0,
        "controller_response_observed": False,
        "battery_mv": None,
        "error": None,
    }
    try:
        import serial  # type: ignore

        with serial.Serial(port, baudrate, timeout=0.8) as handle:
            result["serial_port_opened"] = True
            handle.reset_input_buffer()
            result["serial_bytes_written"] = handle.write(frame)
            handle.flush()
            time.sleep(0.3)
            data = handle.read(64)
        result["response_hex"] = data.hex()
        result["response_len"] = len(data)
        if len(data) >= 6 and data[0] == 0x55 and data[1] == 0x55 and data[3] == 0x0F:
            result["controller_response_observed"] = True
            result["battery_mv"] = data[4] | (data[5] << 8)
    except Exception as exc:
        result["error"] = str(exc)
    return result


def write_frame(port: str, baudrate: int, frame: Optional[bytes]) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "expected_bytes": len(frame) if frame else 0,
        "write_ok": False,
        "error": None,
    }
    if frame is None:
        result["error"] = "frame_bytes is None"
        return result
    try:
        import serial  # type: ignore

        with serial.Serial(port, baudrate, timeout=0.5) as handle:
            result["serial_port_opened"] = True
            result["serial_bytes_written"] = handle.write(frame)
            handle.flush()
        result["write_ok"] = result["serial_bytes_written"] == result["expected_bytes"]
        if not result["write_ok"]:
            result["error"] = (
                f"serial write length mismatch: {result['serial_bytes_written']} != "
                f"{result['expected_bytes']}"
            )
    except Exception as exc:
        result["error"] = str(exc)
    return result


def make_safety(config_path: Path, hardware_enabled: bool) -> ArmSafety:
    safety = ArmSafety(str(config_path))
    phase_result = safety.set_phase(PHASE)
    if not phase_result.allowed:
        raise RuntimeError(phase_result.reason)
    if hardware_enabled:
        enable_runtime_hardware_gates(safety)
    return safety


def build_sequence(config_path: Path, hardware_enabled: bool) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]], List[str], List[str]]:
    safety = make_safety(config_path, hardware_enabled)
    actions: List[Dict[str, Any]] = []
    frame_infos: List[Dict[str, Any]] = []
    errors: List[str] = []
    warnings: List[str] = []

    for index, (label, duration_ms, servos) in enumerate(SAMPLE_SEQUENCE, start=1):
        command = MultiServoCommand(servos=servos, time_ms=duration_ms, label=label)
        validation = safety.validate_all(command)
        if not validation.allowed:
            errors.append(f"{label} validation failed: {validation.reason}")
        if validation.warnings:
            warnings.extend(f"{label}: {warning}" for warning in validation.warnings)
        frame_infos.append(safety.build_move_frame(command))
        safety.record_multi(command)
        actions.append(
            {
                "action_id": f"arm_b3_step_{index}_{label}",
                "action_type": "ARM_B3_NO_LOAD_SAMPLE_STEP",
                "step_index": index,
                "step_name": label,
                "requires_base_zero": True,
                "publishes_cmd_vel": False,
                "duration_ms": duration_ms,
                "servos": servos,
                "no_load_only": True,
                "contact_allowed": False,
                "obstacle_removal_allowed": False,
            }
        )
    return actions, frame_infos, errors, warnings


def write_readme(output_dir: Path, status_doc: Dict[str, Any]) -> None:
    lines = [
        "# Arm-B3 No-Load Sample Sequence",
        "",
        "This directory contains evidence for one guarded full no-load sample sequence.",
        "",
        "## Boundary",
        "",
        "- Runs only the safety-adjusted no-load sample sequence.",
        "- Does not start ROS.",
        "- Does not publish `cmd_vel`.",
        "- Contact and obstacle removal are not allowed.",
        "- This does not validate grasping, payload handling, or real obstacle removal.",
        "",
        "## Result",
        "",
        f"- status: `{status_doc.get('status')}`",
        f"- dry_run: `{str(status_doc.get('dry_run')).lower()}`",
        f"- hardware_executed: `{str(status_doc.get('hardware_executed')).lower()}`",
        f"- step_count: `{status_doc.get('step_count')}`",
        f"- step_success_count: `{status_doc.get('step_success_count')}`",
        f"- errors: `{len(status_doc.get('errors', []))}`",
        "",
    ]
    (output_dir / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs" / "arm_safety_config.json"))
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--output-dir")
    parser.add_argument("--enable-hardware-write", action="store_true")
    parser.add_argument("--confirm-no-load-sample-sequence", action="store_true")
    args = parser.parse_args()

    run_id = f"arm_b3_no_load_sample_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    output_dir = Path(args.output_dir) if args.output_dir else DEFAULT_OUTPUT_BASE / run_id
    output_dir.mkdir(parents=True, exist_ok=True)

    config_path = Path(args.config)
    config = load_config(config_path)
    home_servos = read_home_pose(config)
    errors: List[str] = []
    warnings: List[str] = []
    dry_run = not (args.enable_hardware_write and args.confirm_no_load_sample_sequence)

    if args.enable_hardware_write != args.confirm_no_load_sample_sequence:
        errors.append(
            "hardware write requires both --enable-hardware-write and "
            "--confirm-no-load-sample-sequence"
        )
    if SAMPLE_SEQUENCE[-1][2] != home_servos:
        errors.append("last sample sequence step must return to safe_idle_home_like_6b")
    if any(step[1] < 1000 for step in SAMPLE_SEQUENCE):
        errors.append("all sample sequence durations must be >= 1000 ms")

    port_audit = audit_serial_port(args.serial_port)
    if args.enable_hardware_write and args.confirm_no_load_sample_sequence and not port_audit["exists"]:
        errors.append(f"serial port does not exist: {args.serial_port}")

    actions, dry_frame_infos, validation_errors, validation_warnings = build_sequence(
        config_path=config_path,
        hardware_enabled=False,
    )
    errors.extend(validation_errors)
    warnings.extend(validation_warnings)

    frame_infos = dry_frame_infos
    if args.enable_hardware_write and args.confirm_no_load_sample_sequence and not errors:
        actions, frame_infos, validation_errors, validation_warnings = build_sequence(
            config_path=config_path,
            hardware_enabled=True,
        )
        errors.extend(validation_errors)
        warnings.extend(validation_warnings)

    voltage_query = None
    writes: List[Optional[Dict[str, Any]]] = [None for _ in SAMPLE_SEQUENCE]
    if args.enable_hardware_write and args.confirm_no_load_sample_sequence and not errors:
        voltage_query = query_battery_voltage(args.serial_port, args.baudrate)
        if not voltage_query.get("controller_response_observed", False):
            errors.append("controller voltage query did not receive a valid response")

    if args.enable_hardware_write and args.confirm_no_load_sample_sequence and not errors:
        for index, ((label, duration_ms, _servos), frame_info) in enumerate(zip(SAMPLE_SEQUENCE, frame_infos)):
            if not frame_info.get("serial_write_allowed_effective", False):
                errors.append(f"{label} serial_write_allowed_effective=false")
                break
            if frame_info.get("frame_bytes") is None:
                errors.append(f"{label} frame_bytes is None")
                break
            writes[index] = write_frame(args.serial_port, args.baudrate, frame_info["frame_bytes"])
            if not writes[index].get("write_ok"):
                errors.append(f"{label} serial write failed: {writes[index].get('error')}")
                break
            time.sleep(duration_ms / 1000.0)

    step_ok = [bool(write and write.get("write_ok")) for write in writes]
    hardware_executed = all(step_ok) if not dry_run else False
    episode_status = "succeeded" if not errors and (dry_run or hardware_executed) else "failed_safe"

    action_results: List[Dict[str, Any]] = []
    for index, action in enumerate(actions):
        write = writes[index]
        action_results.append(
            {
                "action_id": action["action_id"],
                "action_type": action["action_type"],
                "step_index": action["step_index"],
                "step_name": action["step_name"],
                "status": "succeeded" if not errors and (dry_run or step_ok[index]) else "failed_safe",
                "base_zero_ok_before": True,
                "published_cmd_vel": False,
                "hardware_executed": step_ok[index],
                "dry_run": dry_run,
                "mock": False,
                "contact_allowed": False,
                "obstacle_removed": False,
                "serial_port_opened": bool(write and write.get("serial_port_opened")),
                "serial_bytes_written": int(write.get("serial_bytes_written", 0)) if write else 0,
                "frame_hex": frame_infos[index].get("frame_hex"),
                "serial_write_allowed_effective": frame_infos[index].get("serial_write_allowed_effective"),
                "errors": errors if episode_status == "failed_safe" else [],
                "warnings": warnings,
            }
        )

    episode_report = {
        "episode_id": run_id,
        "created_at": now_iso(),
        "status": episode_status,
        "policy_state": {
            "state_id": "arm_b3_no_load_prechecked_stationary",
            "base_zero_ok": True,
            "source": "arm_b3_no_load_sample_sequence",
        },
        "actions": actions,
        "action_results": action_results,
        "claim_boundary": {
            "claims": [
                "Validates one safety-adjusted full no-load sample sequence and return to 6b home.",
            ],
            "non_claims": [
                "Does not validate grasping, payload handling, contact, or obstacle removal.",
                "Does not start ROS or publish cmd_vel.",
                "Does not validate autonomous execution.",
            ],
        },
    }
    status_doc = {
        "generated_at": now_iso(),
        "status": episode_status,
        "dry_run": dry_run,
        "hardware_executed": hardware_executed,
        "step_count": len(SAMPLE_SEQUENCE),
        "step_success_count": sum(step_ok) if not dry_run else 0,
        "step_ok": step_ok,
        "serial_audit": port_audit,
        "voltage_query": voltage_query,
        "writes": writes,
        "frame_infos": [{k: v for k, v in info.items() if k != "frame_bytes"} for info in frame_infos],
        "sample_sequence": [
            {"step_name": label, "duration_ms": duration_ms, "servos": servos}
            for label, duration_ms, servos in SAMPLE_SEQUENCE
        ],
        "errors": errors,
        "warnings": warnings,
    }

    write_json(output_dir / "episode_report.json", episode_report)
    write_json(output_dir / "action_results.json", action_results)
    write_json(output_dir / "arm_b3_status.json", status_doc)
    write_json(output_dir / "sent_frames.json", {
        frame["step_name"]: frame_infos[index].get("frame_hex")
        for index, frame in enumerate(status_doc["sample_sequence"])
    })
    write_json(output_dir / "errors.json", errors)
    write_readme(output_dir, status_doc)

    print(f"wrote {output_dir}")
    print(f"status={episode_status}")
    print(f"dry_run={dry_run}")
    print(f"hardware_executed={hardware_executed}")
    print(f"step_ok={step_ok}")
    if voltage_query is not None:
        print(f"controller_response_observed={voltage_query.get('controller_response_observed')}")
        print(f"battery_mv={voltage_query.get('battery_mv')}")
    if errors:
        for error in errors:
            print(f"error: {error}")
    return 0 if episode_status == "succeeded" else 1


if __name__ == "__main__":
    raise SystemExit(main())
