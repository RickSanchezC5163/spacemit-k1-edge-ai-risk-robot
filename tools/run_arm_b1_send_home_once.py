#!/usr/bin/env python3
"""Arm-B1: send the measured safe idle home pose once.

Default mode is dry-run evidence generation. The script only opens the serial
port and writes one frame when both --enable-hardware-write and
--confirm-send-home-6b are provided.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List


PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from arm_safety import ArmSafety, MultiServoCommand  # noqa: E402


ACTION_TYPE = "ARM_SEND_HOME_ONCE"
TARGET_POSE_NAME = "safe_idle_home_like_6b"
DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "outputs" / "arm_b1_send_home_once_v1"


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
        udev = run_command(["udevadm", "info", "-q", "property", "-n", port])
        audit["udevadm"] = udev
        holders = []
        for cmd in (["fuser", "-v", port], ["lsof", port]):
            result = run_command(cmd)
            if result.get("stdout") or result.get("stderr"):
                holders.append(result)
        audit["holders"] = holders
    return audit


def read_pose(config: dict) -> Dict[int, int]:
    pose = config["poses"][TARGET_POSE_NAME]
    servos = {int(k): int(v) for k, v in pose["servos"].items()}
    if set(servos) != {1, 2, 3, 4, 5}:
        raise ValueError(f"{TARGET_POSE_NAME} must contain servo IDs 1-5")
    return servos


def write_json(path: Path, data: Any) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def write_readme(output_dir: Path, status: dict) -> None:
    lines = [
        "# Arm-B1 Send Home Once",
        "",
        "This directory contains evidence for the Arm-B1 safe-idle-home test.",
        "",
        "## Boundary",
        "",
        "- Action type: `ARM_SEND_HOME_ONCE`",
        f"- Target pose: `{TARGET_POSE_NAME}`",
        "- Sequence length: 1",
        "- No ROS process required.",
        "- Does not publish `cmd_vel`.",
        "- Contact is not allowed.",
        "- Obstacle removal is not allowed.",
        "- Hardware write requires both `--enable-hardware-write` and `--confirm-send-home-6b`.",
        "",
        "## Result",
        "",
        f"- status: `{status.get('status')}`",
        f"- dry_run: `{str(status.get('dry_run')).lower()}`",
        f"- serial_port_opened: `{str(status.get('serial_port_opened')).lower()}`",
        f"- serial_bytes_written: `{status.get('serial_bytes_written')}`",
        f"- hardware_executed: `{str(status.get('hardware_executed')).lower()}`",
        f"- errors: `{len(status.get('errors', []))}`",
        "",
    ]
    (output_dir / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(PROJECT_ROOT / "configs" / "arm_safety_config.json"))
    parser.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--duration-ms", type=int, default=2000)
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--enable-hardware-write", action="store_true")
    parser.add_argument("--confirm-send-home-6b", action="store_true")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    errors: List[str] = []
    warnings: List[str] = []
    serial_port_opened = False
    serial_bytes_written = 0
    hardware_executed = False
    dry_run = not (args.enable_hardware_write and args.confirm_send_home_6b)
    status = "succeeded"

    config_path = Path(args.config)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    servos = read_pose(config)

    if args.duration_ms < 2000:
        errors.append("duration_ms must be >= 2000 for ARM_SEND_HOME_ONCE")
    if args.enable_hardware_write != args.confirm_send_home_6b:
        errors.append("hardware write requires both --enable-hardware-write and --confirm-send-home-6b")

    port_audit = audit_serial_port(args.serial_port)
    if args.enable_hardware_write and args.confirm_send_home_6b and not port_audit["exists"]:
        errors.append(f"serial port does not exist: {args.serial_port}")

    safety = ArmSafety(str(config_path))
    phase_result = safety.set_phase("arm_b1_send_home_once")
    if not phase_result.allowed:
        errors.append(phase_result.reason)

    command = MultiServoCommand(servos=servos, time_ms=args.duration_ms, label=TARGET_POSE_NAME)
    validation = safety.validate_all(command)
    if not validation.allowed:
        errors.append(validation.reason)

    if args.enable_hardware_write and args.confirm_send_home_6b and not errors:
        # Runtime-only gate opening. The config file remains safe by default.
        gates = safety.config.setdefault("safety_gates", {})
        gates["arm_enabled"] = True
        gates["hardware_access_allowed"] = True
        gates["serial_write_allowed"] = True
        gates["dry_run"] = False
        gates["contact_allowed"] = False
        gates["obstacle_removal_allowed"] = False

    frame_info = safety.build_move_frame(command)
    frame_hex = frame_info["frame_hex"]

    if args.enable_hardware_write and args.confirm_send_home_6b and not errors:
        if not frame_info.get("serial_write_allowed_effective", False):
            errors.append("serial_write_allowed_effective=false after explicit hardware enable")
        elif frame_info.get("frame_bytes") is None:
            errors.append("frame_bytes is None after explicit hardware enable")
        else:
            try:
                import serial  # type: ignore

                with serial.Serial(args.serial_port, args.baudrate, timeout=0.5) as handle:
                    serial_port_opened = True
                    serial_bytes_written = handle.write(frame_info["frame_bytes"])
                    handle.flush()
                hardware_executed = serial_bytes_written == len(frame_info["frame_bytes"])
                if not hardware_executed:
                    errors.append(
                        f"serial write length mismatch: {serial_bytes_written} != "
                        f"{len(frame_info['frame_bytes'])}"
                    )
            except Exception as exc:
                errors.append(f"serial write failed: {exc}")

    if errors:
        status = "failed_safe"

    action_id = f"arm_b1_send_home_once_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}"
    action = {
        "action_id": action_id,
        "action_type": ACTION_TYPE,
        "target_pose": TARGET_POSE_NAME,
        "requires_base_zero": True,
        "publishes_cmd_vel": False,
        "duration_ms": args.duration_ms,
        "servos": servos,
    }
    action_result = {
        "action_id": action_id,
        "action_type": ACTION_TYPE,
        "status": status,
        "base_zero_ok_before": True,
        "published_cmd_vel": False,
        "hardware_executed": hardware_executed,
        "dry_run": dry_run,
        "mock": False,
        "contact_allowed": False,
        "obstacle_removed": False,
        "serial_port": args.serial_port,
        "serial_port_opened": serial_port_opened,
        "serial_bytes_written": serial_bytes_written,
        "frame_hex": frame_hex,
        "serial_write_allowed_effective": frame_info.get("serial_write_allowed_effective"),
        "serial_write_allowed_global": frame_info.get("serial_write_allowed_global"),
        "serial_write_allowed_phase": frame_info.get("serial_write_allowed_phase"),
        "errors": errors,
        "warnings": warnings,
    }
    episode_report = {
        "episode_id": f"arm_b1_send_home_once_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}",
        "created_at": now_iso(),
        "status": status,
        "policy_state": {
            "state_id": "arm_b1_prechecked_stationary",
            "base_zero_ok": True,
            "source": "arm_b1_send_home_once",
        },
        "actions": [action],
        "action_results": [action_result],
        "claim_boundary": {
            "claims": [
                "Only validates sending the measured safe idle 6b home pose once when hardware write is explicitly enabled.",
            ],
            "non_claims": [
                "Does not validate obstacle removal.",
                "Does not validate grasping or contact.",
                "Does not start ROS or publish cmd_vel.",
            ],
        },
    }
    status_doc = {
        "generated_at": now_iso(),
        "status": status,
        "dry_run": dry_run,
        "hardware_executed": hardware_executed,
        "serial_port_opened": serial_port_opened,
        "serial_bytes_written": serial_bytes_written,
        "target_pose": TARGET_POSE_NAME,
        "action_type": ACTION_TYPE,
        "serial_audit": port_audit,
        "frame_info": {
            k: v for k, v in frame_info.items() if k != "frame_bytes"
        },
        "errors": errors,
        "warnings": warnings,
    }

    write_json(output_dir / "action_result.json", action_result)
    write_json(output_dir / "episode_report.json", episode_report)
    write_json(output_dir / "arm_b1_status.json", status_doc)
    write_json(output_dir / "errors.json", errors)
    (output_dir / "sent_frame_hex.txt").write_text(frame_hex + "\n", encoding="utf-8")
    write_readme(output_dir, status_doc)

    print(f"wrote {output_dir}")
    print(f"status={status}")
    print(f"dry_run={dry_run}")
    print(f"serial_port_opened={serial_port_opened}")
    print(f"serial_bytes_written={serial_bytes_written}")
    print(f"hardware_executed={hardware_executed}")
    if errors:
        for error in errors:
            print(f"error: {error}")
    return 0 if status == "succeeded" else 1


if __name__ == "__main__":
    raise SystemExit(main())
