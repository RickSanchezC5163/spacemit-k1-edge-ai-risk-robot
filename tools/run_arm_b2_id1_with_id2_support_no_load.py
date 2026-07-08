#!/usr/bin/env python3
"""Arm-B2-ID1: run ID1 yaw only after moving ID2 to a validated support pose.

This is a narrow no-load diagnostic, not a general arm sequence runner.
Default mode is dry-run. Hardware writes require both --enable-hardware-write
and --confirm-id1-with-id2-support-no-load.
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


PHASE = "arm_b2_single_joint_small_angle"
TARGET_POSE_NAME = "safe_idle_home_like_6b"
DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_OUTPUT_BASE = PROJECT_ROOT / "outputs" / "arm_b2_single_servo_no_load_v1"

ACTION_SUPPORT = "ARM_B2_ID2_SUPPORT_FOR_ID1"
ACTION_ID1_TARGET = "ARM_B2_ID1_SINGLE_SERVO_NO_LOAD"
ACTION_ID1_HOME = "ARM_B2_ID1_RETURN_HOME"
ACTION_RETURN_6B = "ARM_RETURN_HOME_6B"


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


def build_action(action_id: str, action_type: str, servo_id: int, pulse: int, duration_ms: int) -> Dict[str, Any]:
    return {
        "action_id": action_id,
        "action_type": action_type,
        "requires_base_zero": True,
        "publishes_cmd_vel": False,
        "servo_id": servo_id,
        "target_pulse": pulse,
        "duration_ms": duration_ms,
        "no_load_only": True,
        "contact_allowed": False,
        "obstacle_removal_allowed": False,
    }


def build_result(
    action: Dict[str, Any],
    status: str,
    dry_run: bool,
    write_result: Optional[Dict[str, Any]],
    frame_info: Dict[str, Any],
    errors: List[str],
    warnings: List[str],
) -> Dict[str, Any]:
    return {
        "action_id": action["action_id"],
        "action_type": action["action_type"],
        "status": status,
        "base_zero_ok_before": True,
        "published_cmd_vel": False,
        "hardware_executed": bool(write_result and write_result.get("write_ok")),
        "dry_run": dry_run,
        "mock": False,
        "contact_allowed": False,
        "obstacle_removed": False,
        "servo_id": action.get("servo_id"),
        "target_pulse": action.get("target_pulse"),
        "serial_port_opened": bool(write_result and write_result.get("serial_port_opened")),
        "serial_bytes_written": int(write_result.get("serial_bytes_written", 0)) if write_result else 0,
        "frame_hex": frame_info.get("frame_hex"),
        "serial_write_allowed_effective": frame_info.get("serial_write_allowed_effective"),
        "errors": errors if status == "failed_safe" else [],
        "warnings": warnings,
    }


def write_readme(output_dir: Path, status_doc: Dict[str, Any]) -> None:
    lines = [
        "# Arm-B2-ID1 With ID2 Support No-Load",
        "",
        "This directory contains evidence for the narrow ID1 no-load check.",
        "",
        "## Boundary",
        "",
        "- Moves ID2 to the validated support pulse before ID1 yaw.",
        "- Rotates only ID1 after ID2 support is established.",
        "- Returns ID1 home, then returns all joints to `safe_idle_home_like_6b`.",
        "- Does not run a full arm action sequence.",
        "- Does not start ROS or publish `cmd_vel`.",
        "- Contact and obstacle removal are not allowed.",
        "",
        "## Result",
        "",
        f"- status: `{status_doc.get('status')}`",
        f"- dry_run: `{str(status_doc.get('dry_run')).lower()}`",
        f"- hardware_executed: `{str(status_doc.get('hardware_executed')).lower()}`",
        f"- support_id2_pulse: `{status_doc.get('support_id2_pulse')}`",
        f"- id1_target_pulse: `{status_doc.get('id1_target_pulse')}`",
        f"- errors: `{len(status_doc.get('errors', []))}`",
        "",
    ]
    (output_dir / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs" / "arm_safety_config.json"))
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--support-id2-pulse", type=int, default=671)
    parser.add_argument("--id1-target-pulse", type=int, default=610)
    parser.add_argument("--duration-ms", type=int, default=1500)
    parser.add_argument("--return-duration-ms", type=int, default=2000)
    parser.add_argument("--return-home", action="store_true")
    parser.add_argument("--output-dir")
    parser.add_argument("--enable-hardware-write", action="store_true")
    parser.add_argument("--confirm-id1-with-id2-support-no-load", action="store_true")
    args = parser.parse_args()

    run_id = f"arm_b2_id1_{args.id1_target_pulse}_id2_support_{args.support_id2_pulse}_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    output_dir = Path(args.output_dir) if args.output_dir else DEFAULT_OUTPUT_BASE / run_id
    output_dir.mkdir(parents=True, exist_ok=True)

    config_path = Path(args.config)
    config = load_config(config_path)
    home_servos = read_home_pose(config)
    errors: List[str] = []
    warnings: List[str] = []
    dry_run = not (args.enable_hardware_write and args.confirm_id1_with_id2_support_no_load)

    if args.support_id2_pulse != 671:
        errors.append("support-id2-pulse must be 671; this is the validated support pulse near 670")
    if args.id1_target_pulse != 610:
        errors.append("id1-target-pulse must be 610 for the current Arm-B2-ID1 check")
    if args.support_id2_pulse < 600:
        errors.append("ID1 rotation requires ID2 >= 600")
    if args.support_id2_pulse > home_servos[2]:
        errors.append("ID2 support pulse must not exceed measured home 771")
    if not args.return_home:
        errors.append("--return-home is required")
    if args.enable_hardware_write != args.confirm_id1_with_id2_support_no_load:
        errors.append(
            "hardware write requires both --enable-hardware-write and "
            "--confirm-id1-with-id2-support-no-load"
        )
    if args.duration_ms < 1000:
        errors.append("duration_ms must be >= 1000")
    if args.return_duration_ms < 2000:
        errors.append("return_duration_ms must be >= 2000")

    port_audit = audit_serial_port(args.serial_port)
    if args.enable_hardware_write and args.confirm_id1_with_id2_support_no_load and not port_audit["exists"]:
        errors.append(f"serial port does not exist: {args.serial_port}")

    safety = ArmSafety(str(config_path))
    phase_result = safety.set_phase(PHASE)
    if not phase_result.allowed:
        errors.append(phase_result.reason)

    commands = [
        {
            "action_type": ACTION_SUPPORT,
            "servo_id": 2,
            "pulse": args.support_id2_pulse,
            "duration_ms": args.duration_ms,
            "label": "id2_support_for_id1",
        },
        {
            "action_type": ACTION_ID1_TARGET,
            "servo_id": 1,
            "pulse": args.id1_target_pulse,
            "duration_ms": args.duration_ms,
            "label": "id1_yaw_small_pos",
        },
        {
            "action_type": ACTION_ID1_HOME,
            "servo_id": 1,
            "pulse": home_servos[1],
            "duration_ms": args.duration_ms,
            "label": "id1_back_to_home",
        },
    ]

    frame_infos: List[Dict[str, Any]] = []
    actions: List[Dict[str, Any]] = []
    for index, item in enumerate(commands, start=1):
        cmd = MultiServoCommand(
            servos={item["servo_id"]: item["pulse"]},
            time_ms=item["duration_ms"],
            label=item["label"],
        )
        validation = safety.validate_all(cmd)
        if not validation.allowed:
            errors.append(f"{item['label']} validation failed: {validation.reason}")
        frame_infos.append(safety.build_move_frame(cmd))
        safety.record_multi(cmd)
        actions.append(
            build_action(
                f"{run_id}_step{index}_{item['label']}",
                item["action_type"],
                item["servo_id"],
                item["pulse"],
                item["duration_ms"],
            )
        )

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
    return_frame_info = return_safety.build_move_frame(return_cmd)
    return_action = {
        "action_id": f"{run_id}_step4_return_6b",
        "action_type": ACTION_RETURN_6B,
        "requires_base_zero": True,
        "publishes_cmd_vel": False,
        "target_pose": TARGET_POSE_NAME,
        "duration_ms": args.return_duration_ms,
        "servos": home_servos,
        "contact_allowed": False,
        "obstacle_removal_allowed": False,
    }

    if args.enable_hardware_write and args.confirm_id1_with_id2_support_no_load and not errors:
        enable_runtime_hardware_gates(safety)
        enable_runtime_hardware_gates(return_safety)
        frame_infos = []
        replay = ArmSafety(str(config_path))
        replay.set_phase(PHASE)
        enable_runtime_hardware_gates(replay)
        for item in commands:
            cmd = MultiServoCommand(
                servos={item["servo_id"]: item["pulse"]},
                time_ms=item["duration_ms"],
                label=item["label"],
            )
            frame_infos.append(replay.build_move_frame(cmd))
            replay.record_multi(cmd)
        return_frame_info = return_safety.build_move_frame(return_cmd)

    voltage_query = None
    writes: List[Optional[Dict[str, Any]]] = [None, None, None]
    return_write = None
    if args.enable_hardware_write and args.confirm_id1_with_id2_support_no_load and not errors:
        voltage_query = query_battery_voltage(args.serial_port, args.baudrate)
        if not voltage_query.get("controller_response_observed", False):
            errors.append("controller voltage query did not receive a valid response")

    if args.enable_hardware_write and args.confirm_id1_with_id2_support_no_load and not errors:
        for idx, frame_info in enumerate(frame_infos):
            if not frame_info.get("serial_write_allowed_effective", False):
                errors.append(f"step {idx + 1} serial_write_allowed_effective=false")
                break
            if frame_info.get("frame_bytes") is None:
                errors.append(f"step {idx + 1} frame_bytes is None")
                break
            writes[idx] = write_frame(args.serial_port, args.baudrate, frame_info["frame_bytes"])
            if not writes[idx].get("write_ok"):
                errors.append(f"step {idx + 1} serial write failed: {writes[idx].get('error')}")
                break
            time.sleep(commands[idx]["duration_ms"] / 1000.0)

    if args.enable_hardware_write and args.confirm_id1_with_id2_support_no_load and not errors:
        if not return_frame_info.get("serial_write_allowed_effective", False):
            errors.append("return serial_write_allowed_effective=false")
        elif return_frame_info.get("frame_bytes") is None:
            errors.append("return frame_bytes is None")
        else:
            return_write = write_frame(args.serial_port, args.baudrate, return_frame_info["frame_bytes"])
            if not return_write.get("write_ok"):
                errors.append(f"return-home serial write failed: {return_write.get('error')}")
            else:
                time.sleep(args.return_duration_ms / 1000.0)

    step_ok = [bool(write and write.get("write_ok")) for write in writes]
    return_ok = bool(return_write and return_write.get("write_ok"))
    hardware_executed = all(step_ok) and return_ok
    episode_status = "succeeded" if not errors and (dry_run or hardware_executed) else "failed_safe"

    action_results = [
        build_result(
            action,
            "succeeded" if not errors and (dry_run or step_ok[idx]) else "failed_safe",
            dry_run,
            writes[idx],
            frame_infos[idx],
            errors,
            warnings,
        )
        for idx, action in enumerate(actions)
    ]
    action_results.append(
        {
            "action_id": return_action["action_id"],
            "action_type": return_action["action_type"],
            "status": "succeeded" if not errors and (dry_run or return_ok) else "failed_safe",
            "base_zero_ok_before": True,
            "published_cmd_vel": False,
            "hardware_executed": return_ok,
            "dry_run": dry_run,
            "mock": False,
            "contact_allowed": False,
            "obstacle_removed": False,
            "target_pose": TARGET_POSE_NAME,
            "serial_port_opened": bool(return_write and return_write.get("serial_port_opened")),
            "serial_bytes_written": int(return_write.get("serial_bytes_written", 0)) if return_write else 0,
            "frame_hex": return_frame_info.get("frame_hex"),
            "serial_write_allowed_effective": return_frame_info.get("serial_write_allowed_effective"),
            "errors": errors if episode_status == "failed_safe" else [],
            "warnings": warnings,
        }
    )

    episode_report = {
        "episode_id": run_id,
        "created_at": now_iso(),
        "status": episode_status,
        "policy_state": {
            "state_id": "arm_b2_id1_with_id2_support_prechecked_stationary",
            "base_zero_ok": True,
            "source": "arm_b2_id1_with_id2_support_no_load",
        },
        "actions": actions + [return_action],
        "action_results": action_results,
        "claim_boundary": {
            "claims": [
                "Validates a narrow no-load ID1 yaw check only after ID2 reaches the validated 671 support pose.",
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
        "support_id2_pulse": args.support_id2_pulse,
        "id1_target_pulse": args.id1_target_pulse,
        "step_ok": step_ok,
        "return_home_executed": return_ok,
        "home_pose": home_servos,
        "serial_audit": port_audit,
        "voltage_query": voltage_query,
        "writes": writes,
        "return_write": return_write,
        "frame_infos": [{k: v for k, v in info.items() if k != "frame_bytes"} for info in frame_infos],
        "return_frame_info": {k: v for k, v in return_frame_info.items() if k != "frame_bytes"},
        "errors": errors,
        "warnings": warnings,
    }

    write_json(output_dir / "episode_report.json", episode_report)
    write_json(output_dir / "action_results.json", action_results)
    write_json(output_dir / "arm_b2_id1_status.json", status_doc)
    write_json(output_dir / "sent_frames.json", {
        "support_id2_frame_hex": frame_infos[0].get("frame_hex") if frame_infos else None,
        "id1_target_frame_hex": frame_infos[1].get("frame_hex") if len(frame_infos) > 1 else None,
        "id1_home_frame_hex": frame_infos[2].get("frame_hex") if len(frame_infos) > 2 else None,
        "return_6b_frame_hex": return_frame_info.get("frame_hex"),
        "voltage_query_frame_hex": "5555020f",
    })
    write_json(output_dir / "errors.json", errors)
    write_readme(output_dir, status_doc)

    print(f"wrote {output_dir}")
    print(f"status={episode_status}")
    print(f"dry_run={dry_run}")
    print(f"hardware_executed={hardware_executed}")
    print(f"step_ok={step_ok}")
    print(f"return_home_executed={return_ok}")
    if voltage_query is not None:
        print(f"controller_response_observed={voltage_query.get('controller_response_observed')}")
        print(f"battery_mv={voltage_query.get('battery_mv')}")
    if errors:
        for error in errors:
            print(f"error: {error}")
    return 0 if episode_status == "succeeded" else 1


if __name__ == "__main__":
    raise SystemExit(main())
