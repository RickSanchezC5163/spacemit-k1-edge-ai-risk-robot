#!/usr/bin/env python3
"""Arm-B2: run exactly one low-risk single-servo no-load check.

Default mode is dry-run evidence generation. The script opens the serial port
and writes motion frames only when both --enable-hardware-write and
--confirm-single-servo-no-load are provided. A safe 6b return-home frame is
mandatory for every run.
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


ACTION_TYPE = "ARM_B2_SINGLE_SERVO_NO_LOAD"
RETURN_ACTION_TYPE = "ARM_RETURN_HOME_6B"
TARGET_POSE_NAME = "safe_idle_home_like_6b"
DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_OUTPUT_BASE = PROJECT_ROOT / "outputs" / "arm_b2_single_servo_no_load_v1"
PHASE = "arm_b2_single_joint_small_angle"


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


def load_config(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_home_pose(config: Dict[str, Any]) -> Dict[int, int]:
    pose = config["poses"][TARGET_POSE_NAME]
    servos = {int(k): int(v) for k, v in pose["servos"].items()}
    if set(servos) != {1, 2, 3, 4, 5}:
        raise ValueError(f"{TARGET_POSE_NAME} must contain servo IDs 1-5")
    return servos


def allowed_targets(config: Dict[str, Any]) -> Dict[int, List[int]]:
    phase = config["phase_gates"][PHASE]
    configured = phase.get("allowed_single_servo_targets", {})
    return {int(k): [int(v) for v in values] for k, values in configured.items()}


def enable_runtime_hardware_gates(safety: ArmSafety) -> None:
    gates = safety.config.setdefault("safety_gates", {})
    gates["arm_enabled"] = True
    gates["hardware_access_allowed"] = True
    gates["serial_write_allowed"] = True
    gates["dry_run"] = False
    gates["contact_allowed"] = False
    gates["obstacle_removal_allowed"] = False


def write_json(path: Path, data: Any) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


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


def write_readme(output_dir: Path, status_doc: Dict[str, Any]) -> None:
    lines = [
        "# Arm-B2 Single Servo No-Load",
        "",
        "This directory contains evidence for one Arm-B2 single-servo no-load check.",
        "",
        "## Boundary",
        "",
        f"- Action type: `{ACTION_TYPE}`",
        "- Exactly one test servo target is allowed.",
        f"- Return pose: `{TARGET_POSE_NAME}`",
        "- No ROS process is required.",
        "- Does not publish `cmd_vel`.",
        "- Contact is not allowed.",
        "- Obstacle removal is not allowed.",
        "- Hardware write requires both `--enable-hardware-write` and "
        "`--confirm-single-servo-no-load`.",
        "",
        "## Result",
        "",
        f"- status: `{status_doc.get('status')}`",
        f"- dry_run: `{str(status_doc.get('dry_run')).lower()}`",
        f"- hardware_executed: `{str(status_doc.get('hardware_executed')).lower()}`",
        f"- servo_id: `{status_doc.get('servo_id')}`",
        f"- target_pulse: `{status_doc.get('target_pulse')}`",
        f"- errors: `{len(status_doc.get('errors', []))}`",
        "",
    ]
    (output_dir / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs" / "arm_safety_config.json"))
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--servo-id", type=int, required=True)
    parser.add_argument("--target-pulse", type=int, required=True)
    parser.add_argument("--duration-ms", type=int, default=1500)
    parser.add_argument("--return-duration-ms", type=int, default=2000)
    parser.add_argument("--return-home", action="store_true")
    parser.add_argument("--output-dir")
    parser.add_argument("--enable-hardware-write", action="store_true")
    parser.add_argument("--confirm-single-servo-no-load", action="store_true")
    args = parser.parse_args()

    run_id = f"arm_b2_servo{args.servo_id}_{args.target_pulse}_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    output_dir = Path(args.output_dir) if args.output_dir else DEFAULT_OUTPUT_BASE / run_id
    output_dir.mkdir(parents=True, exist_ok=True)

    config_path = Path(args.config)
    config = load_config(config_path)
    home_servos = read_home_pose(config)
    allowed = allowed_targets(config)
    errors: List[str] = []
    warnings: List[str] = []
    dry_run = not (args.enable_hardware_write and args.confirm_single_servo_no_load)

    if args.servo_id not in {1, 2, 3, 4, 5}:
        errors.append(f"invalid servo_id={args.servo_id}; expected 1-5")
    if not args.return_home:
        errors.append("--return-home is required for Arm-B2")
    if args.enable_hardware_write != args.confirm_single_servo_no_load:
        errors.append(
            "hardware write requires both --enable-hardware-write and "
            "--confirm-single-servo-no-load"
        )
    if args.duration_ms < 1000:
        errors.append("duration_ms must be >= 1000 for Arm-B2")
    if args.return_duration_ms < 2000:
        errors.append("return_duration_ms must be >= 2000 for safe 6b return")
    if args.servo_id in allowed and args.target_pulse not in allowed[args.servo_id]:
        errors.append(
            f"target {args.target_pulse} is not an allowed Arm-B2 target for "
            f"servo {args.servo_id}; allowed={allowed[args.servo_id]}"
        )
    if args.servo_id == 2 and args.target_pulse > home_servos[2]:
        errors.append("ID2 target must not exceed measured home 771")
    if args.servo_id == 1 and home_servos[2] < 600:
        errors.append("ID1 rotation requires ID2 >= 600 at the measured home pose")

    port_audit = audit_serial_port(args.serial_port)
    if args.enable_hardware_write and args.confirm_single_servo_no_load and not port_audit["exists"]:
        errors.append(f"serial port does not exist: {args.serial_port}")

    target_safety = ArmSafety(str(config_path))
    phase_result = target_safety.set_phase(PHASE)
    if not phase_result.allowed:
        errors.append(phase_result.reason)
    target_cmd = MultiServoCommand(
        servos={args.servo_id: args.target_pulse},
        time_ms=args.duration_ms,
        label=f"b2_servo_{args.servo_id}_to_{args.target_pulse}",
    )
    target_validation = target_safety.validate_all(target_cmd)
    if not target_validation.allowed:
        errors.append(target_validation.reason)

    return_safety = ArmSafety(str(config_path))
    return_phase = return_safety.set_phase("arm_b1_send_home_once")
    if not return_phase.allowed:
        errors.append(return_phase.reason)
    return_cmd = MultiServoCommand(
        servos=home_servos,
        time_ms=args.return_duration_ms,
        label=TARGET_POSE_NAME,
    )
    return_validation = return_safety.validate_all(return_cmd)
    if not return_validation.allowed:
        errors.append(f"return home validation failed: {return_validation.reason}")

    if args.enable_hardware_write and args.confirm_single_servo_no_load and not errors:
        enable_runtime_hardware_gates(target_safety)
        enable_runtime_hardware_gates(return_safety)

    target_frame_info = target_safety.build_move_frame(target_cmd)
    return_frame_info = return_safety.build_move_frame(return_cmd)

    voltage_query = None
    target_write = None
    return_write = None
    hardware_executed = False
    target_motion_executed = False
    return_home_executed = False

    if args.enable_hardware_write and args.confirm_single_servo_no_load and not errors:
        voltage_query = query_battery_voltage(args.serial_port, args.baudrate)
        if not voltage_query.get("controller_response_observed", False):
            errors.append("controller voltage query did not receive a valid response")

    if args.enable_hardware_write and args.confirm_single_servo_no_load and not errors:
        if not target_frame_info.get("serial_write_allowed_effective", False):
            errors.append("target serial_write_allowed_effective=false")
        elif target_frame_info.get("frame_bytes") is None:
            errors.append("target frame_bytes is None")
        else:
            target_write = write_frame(args.serial_port, args.baudrate, target_frame_info["frame_bytes"])
            target_motion_executed = bool(target_write.get("write_ok"))
            if not target_motion_executed:
                errors.append(f"target serial write failed: {target_write.get('error')}")

    if (
        args.enable_hardware_write
        and args.confirm_single_servo_no_load
        and target_motion_executed
        and not errors
    ):
        time.sleep(args.duration_ms / 1000.0)
        if not return_frame_info.get("serial_write_allowed_effective", False):
            errors.append("return serial_write_allowed_effective=false")
        elif return_frame_info.get("frame_bytes") is None:
            errors.append("return frame_bytes is None")
        else:
            return_write = write_frame(args.serial_port, args.baudrate, return_frame_info["frame_bytes"])
            return_home_executed = bool(return_write.get("write_ok"))
            if not return_home_executed:
                errors.append(f"return-home serial write failed: {return_write.get('error')}")
            else:
                time.sleep(args.return_duration_ms / 1000.0)

    hardware_executed = target_motion_executed and return_home_executed
    target_status = "succeeded" if not errors and (dry_run or target_motion_executed) else "failed_safe"
    return_status = "succeeded" if not errors and (dry_run or return_home_executed) else "failed_safe"
    episode_status = "succeeded" if target_status == "succeeded" and return_status == "succeeded" else "failed_safe"

    action_id = f"{run_id}_target"
    return_action_id = f"{run_id}_return_home"
    action = {
        "action_id": action_id,
        "action_type": ACTION_TYPE,
        "requires_base_zero": True,
        "publishes_cmd_vel": False,
        "servo_id": args.servo_id,
        "target_pulse": args.target_pulse,
        "duration_ms": args.duration_ms,
        "no_load_only": True,
        "contact_allowed": False,
        "obstacle_removal_allowed": False,
    }
    return_action = {
        "action_id": return_action_id,
        "action_type": RETURN_ACTION_TYPE,
        "requires_base_zero": True,
        "publishes_cmd_vel": False,
        "target_pose": TARGET_POSE_NAME,
        "duration_ms": args.return_duration_ms,
        "servos": home_servos,
    }
    action_result = {
        "action_id": action_id,
        "action_type": ACTION_TYPE,
        "status": target_status,
        "base_zero_ok_before": True,
        "published_cmd_vel": False,
        "hardware_executed": target_motion_executed,
        "dry_run": dry_run,
        "mock": False,
        "contact_allowed": False,
        "obstacle_removed": False,
        "servo_id": args.servo_id,
        "target_pulse": args.target_pulse,
        "serial_port": args.serial_port,
        "serial_port_opened": bool(target_write and target_write.get("serial_port_opened")),
        "serial_bytes_written": int(target_write.get("serial_bytes_written", 0)) if target_write else 0,
        "frame_hex": target_frame_info.get("frame_hex"),
        "serial_write_allowed_effective": target_frame_info.get("serial_write_allowed_effective"),
        "errors": errors if target_status == "failed_safe" else [],
        "warnings": warnings,
    }
    return_result = {
        "action_id": return_action_id,
        "action_type": RETURN_ACTION_TYPE,
        "status": return_status,
        "base_zero_ok_before": True,
        "published_cmd_vel": False,
        "hardware_executed": return_home_executed,
        "dry_run": dry_run,
        "mock": False,
        "contact_allowed": False,
        "obstacle_removed": False,
        "target_pose": TARGET_POSE_NAME,
        "serial_port": args.serial_port,
        "serial_port_opened": bool(return_write and return_write.get("serial_port_opened")),
        "serial_bytes_written": int(return_write.get("serial_bytes_written", 0)) if return_write else 0,
        "frame_hex": return_frame_info.get("frame_hex"),
        "serial_write_allowed_effective": return_frame_info.get("serial_write_allowed_effective"),
        "errors": errors if return_status == "failed_safe" else [],
        "warnings": warnings,
    }
    episode_report = {
        "episode_id": run_id,
        "created_at": now_iso(),
        "status": episode_status,
        "policy_state": {
            "state_id": "arm_b2_prechecked_stationary",
            "base_zero_ok": True,
            "source": "arm_b2_single_servo_no_load",
        },
        "actions": [action, return_action],
        "action_results": [action_result, return_result],
        "claim_boundary": {
            "claims": [
                "Validates one low-risk Arm-B2 single-servo no-load command and mandatory return to 6b home.",
            ],
            "non_claims": [
                "Does not validate a full arm sequence.",
                "Does not validate obstacle removal.",
                "Does not validate grasping or contact.",
                "Does not start ROS or publish cmd_vel.",
            ],
        },
    }
    status_doc = {
        "generated_at": now_iso(),
        "status": episode_status,
        "dry_run": dry_run,
        "hardware_executed": hardware_executed,
        "target_motion_executed": target_motion_executed,
        "return_home_executed": return_home_executed,
        "servo_id": args.servo_id,
        "target_pulse": args.target_pulse,
        "home_pose": home_servos,
        "serial_audit": port_audit,
        "voltage_query": voltage_query,
        "target_write": target_write,
        "return_write": return_write,
        "target_frame_info": {k: v for k, v in target_frame_info.items() if k != "frame_bytes"},
        "return_frame_info": {k: v for k, v in return_frame_info.items() if k != "frame_bytes"},
        "errors": errors,
        "warnings": warnings,
    }

    write_json(output_dir / "action_result.json", action_result)
    write_json(output_dir / "return_home_action_result.json", return_result)
    write_json(output_dir / "episode_report.json", episode_report)
    write_json(output_dir / "arm_b2_status.json", status_doc)
    write_json(output_dir / "sent_frames.json", {
        "target_frame_hex": target_frame_info.get("frame_hex"),
        "return_frame_hex": return_frame_info.get("frame_hex"),
        "voltage_query_frame_hex": "5555020f",
    })
    write_json(output_dir / "errors.json", errors)
    write_readme(output_dir, status_doc)

    print(f"wrote {output_dir}")
    print(f"status={episode_status}")
    print(f"dry_run={dry_run}")
    print(f"hardware_executed={hardware_executed}")
    print(f"target_motion_executed={target_motion_executed}")
    print(f"return_home_executed={return_home_executed}")
    if voltage_query is not None:
        print(f"controller_response_observed={voltage_query.get('controller_response_observed')}")
        print(f"battery_mv={voltage_query.get('battery_mv')}")
    if errors:
        for error in errors:
            print(f"error: {error}")
    return 0 if episode_status == "succeeded" else 1


if __name__ == "__main__":
    raise SystemExit(main())
