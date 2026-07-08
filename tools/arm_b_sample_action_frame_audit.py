#!/usr/bin/env python3
"""Dry-run audit for the Arm-B sample no-load action.

This script does not import serial, does not open a device, and does not write
bytes to hardware. It only validates the sample pulses and builds the Lobot bus
servo frames that a later ROS/hardware node would send after safety gates pass.
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List


PROJECT_ROOT = Path(__file__).resolve().parent.parent

FRAME_HEADER = bytes([0x55, 0x55])
CMD_SERVO_MOVE = 3
PULSE_MIN = 0
PULSE_MAX = 1000
TIME_MIN_MS = 0
TIME_MAX_MS = 30000
VALID_SERVO_IDS = {1, 2, 3, 4, 5}
MAX_SINGLE_STEP_DELTA = 300


SAMPLE_ACTION = [
    ("step_1_safe_flat_start", 1500, {1: 499, 2: 770, 3: 457, 4: 500, 5: 494}),
    ("step_2_mid_retract", 1500, {1: 498, 2: 600, 3: 540, 4: 470, 5: 498}),
    ("step_3a_pre_reach", 1500, {1: 498, 2: 400, 3: 590, 4: 470, 5: 496}),
    ("step_3b_reach_no_load", 2000, {1: 498, 2: 250, 3: 646, 4: 470, 5: 494}),
    ("step_4_pre_gripper", 1500, {1: 498, 2: 291, 3: 644, 4: 470, 5: 495}),
    ("step_5_gripper_close_no_object", 1500, {1: 498, 2: 290, 3: 642, 4: 470, 5: 220}),
    ("step_6a_return_mid", 1500, {1: 498, 2: 500, 3: 540, 4: 470, 5: 360}),
    ("step_6b_return_home_like", 2000, {1: 510, 2: 771, 3: 426, 4: 503, 5: 497}),
]

SAFE_IDLE_POSE = {1: 510, 2: 771, 3: 426, 4: 503, 5: 497}


@dataclass
class StepAudit:
    index: int
    label: str
    time_ms: int
    servos: Dict[int, int]
    frame_hex: str
    max_delta_from_previous: int
    per_servo_delta: Dict[int, int]
    valid: bool
    errors: List[str]
    warnings: List[str]


def build_lobot_move_frame(servos: Dict[int, int], time_ms: int) -> bytes:
    count = len(servos)
    buf = bytearray(FRAME_HEADER)
    buf.append(count * 3 + 5)
    buf.append(CMD_SERVO_MOVE)
    buf.append(count)
    buf.extend(time_ms.to_bytes(2, "little"))
    for servo_id in sorted(servos):
        pulse = servos[servo_id]
        buf.append(servo_id)
        buf.extend(pulse.to_bytes(2, "little"))
    return bytes(buf)


def check_coupled_safety(
    servos: Dict[int, int],
    previous: Dict[int, int] | None,
    config: dict,
) -> List[str]:
    errors: List[str] = []
    rules = config.get("coupled_safety_rules", {})

    id1_rule = rules.get("id1_rotation_requires", {})
    if id1_rule.get("enabled", False) and previous is not None:
        id1_servo = int(id1_rule.get("id1_servo", 1))
        support_servo = int(id1_rule.get("support_servo", 2))
        support_min = int(id1_rule.get("support_min_pulse", 600))
        if servos.get(id1_servo) != previous.get(id1_servo):
            support_pulse = servos.get(support_servo)
            if support_pulse is None or support_pulse < support_min:
                errors.append(
                    f"ID{id1_servo} rotation requires ID{support_servo} >= "
                    f"{support_min}; step has ID{support_servo}={support_pulse}"
                )

    for conditional in rules.get("conditional_joint_ranges", []):
        when = conditional.get("when", {})
        when_servo = int(when.get("servo_id"))
        when_pulse = servos.get(when_servo)
        if when_pulse is None:
            continue
        active = True
        if "pulse_gt" in when:
            active = active and when_pulse > int(when["pulse_gt"])
        if "pulse_gte" in when:
            active = active and when_pulse >= int(when["pulse_gte"])
        if "pulse_lt" in when:
            active = active and when_pulse < int(when["pulse_lt"])
        if "pulse_lte" in when:
            active = active and when_pulse <= int(when["pulse_lte"])
        if not active:
            continue
        for limited_servo_str, limits in conditional.get("limits", {}).items():
            limited_servo = int(limited_servo_str)
            pulse = servos.get(limited_servo)
            if pulse is None:
                continue
            lo, hi = int(limits[0]), int(limits[1])
            if not (lo <= pulse <= hi):
                errors.append(
                    f"Conditional safety active at ID{when_servo}={when_pulse}: "
                    f"ID{limited_servo} pulse {pulse} outside [{lo}, {hi}]"
                )
    return errors


def audit_step(
    index: int,
    label: str,
    time_ms: int,
    servos: Dict[int, int],
    previous: Dict[int, int] | None,
    config: dict,
) -> StepAudit:
    errors: List[str] = []
    warnings: List[str] = []

    if not (TIME_MIN_MS <= time_ms <= TIME_MAX_MS):
        errors.append(f"time_ms {time_ms} outside [{TIME_MIN_MS}, {TIME_MAX_MS}]")

    if set(servos) != VALID_SERVO_IDS:
        errors.append(f"servo set {sorted(servos)} does not match {sorted(VALID_SERVO_IDS)}")

    for servo_id, pulse in servos.items():
        if servo_id not in VALID_SERVO_IDS:
            errors.append(f"invalid servo id {servo_id}")
        if not (PULSE_MIN <= pulse <= PULSE_MAX):
            errors.append(f"servo {servo_id} pulse {pulse} outside [{PULSE_MIN}, {PULSE_MAX}]")

    deltas: Dict[int, int] = {}
    if previous is not None:
        for servo_id, pulse in servos.items():
            deltas[servo_id] = abs(pulse - previous.get(servo_id, pulse))
            if deltas[servo_id] > MAX_SINGLE_STEP_DELTA:
                errors.append(
                    f"servo {servo_id} step delta {deltas[servo_id]} exceeds {MAX_SINGLE_STEP_DELTA}"
                )
    else:
        deltas = {servo_id: 0 for servo_id in servos}

    max_delta = max(deltas.values()) if deltas else 0
    if max_delta > 250:
        warnings.append(f"large but allowed step delta {max_delta}; execute with no-load supervision")

    errors.extend(check_coupled_safety(servos, previous, config))

    frame = build_lobot_move_frame(servos, time_ms) if not errors else b""
    return StepAudit(
        index=index,
        label=label,
        time_ms=time_ms,
        servos=servos,
        frame_hex=frame.hex(" "),
        max_delta_from_previous=max_delta,
        per_servo_delta=deltas,
        valid=not errors,
        errors=errors,
        warnings=warnings,
    )


def write_markdown(report: dict, output_dir: Path) -> None:
    lines = [
        "# Arm-B Sample Action Frame Audit",
        "",
        f"Generated: {report['generated_at']}",
        "",
        "## Boundary",
        "",
        "- Dry-run only.",
        "- No serial port opened.",
        "- No bytes written to hardware.",
        "- No ROS process started.",
        "- Intended next use: supervised no-load Arm-B validation after global safety gates are fixed.",
        "",
        "## Summary",
        "",
        f"- steps: {report['step_count']}",
        f"- all_valid: {str(report['all_valid']).lower()}",
        f"- max_step_delta: {report['max_step_delta']}",
        f"- safe_idle_pose: {report['safe_idle_pose']}",
        f"- requires_base_zero: {str(report['requires_base_zero']).lower()}",
        f"- publishes_cmd_vel: {str(report['publishes_cmd_vel']).lower()}",
        "",
        "## Step Audit",
        "",
        "| index | label | time_ms | max_delta | valid | warnings |",
        "| --- | --- | ---: | ---: | --- | --- |",
    ]
    for step in report["steps"]:
        warnings = "; ".join(step["warnings"])
        lines.append(
            f"| {step['index']} | {step['label']} | {step['time_ms']} | "
            f"{step['max_delta_from_previous']} | {str(step['valid']).lower()} | {warnings} |"
        )
    lines.extend([
        "",
        "## Lobot Frame Format",
        "",
        "`55 55 <len> 03 <count> <time_lo> <time_hi> [<id> <pulse_lo> <pulse_hi>]...`",
        "",
        "## Frame Hex",
        "",
    ])
    for step in report["steps"]:
        lines.append(f"### {step['index']} {step['label']}")
        lines.append("")
        lines.append(f"`{step['frame_hex']}`")
        lines.append("")
    (output_dir / "arm_b_sample_action_frame_audit.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "outputs" / "arm_b_sample_action_frame_audit_v1"),
        help="Directory for dry-run audit outputs.",
    )
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "configs" / "arm_safety_config.json"),
        help="Arm safety config containing coupled safety rules.",
    )
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    config = json.loads(Path(args.config).read_text(encoding="utf-8"))

    step_audits: List[StepAudit] = []
    previous: Dict[int, int] | None = dict(SAFE_IDLE_POSE)
    for index, (label, time_ms, servos) in enumerate(SAMPLE_ACTION, start=1):
        audit = audit_step(index, label, time_ms, servos, previous, config)
        step_audits.append(audit)
        previous = dict(servos)

    report = {
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": "arm_b_sample_action_frame_audit",
        "hardware_executed": False,
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "ros_process_started": False,
        "requires_base_zero": True,
        "publishes_cmd_vel": False,
        "safe_idle_pose_name": "safe_idle_home_like_6b",
        "safe_idle_pose": SAFE_IDLE_POSE,
        "step_count": len(step_audits),
        "all_valid": all(step.valid for step in step_audits),
        "max_step_delta": max(step.max_delta_from_previous for step in step_audits),
        "steps": [asdict(step) for step in step_audits],
        "errors": [err for step in step_audits for err in step.errors],
    }

    (output_dir / "arm_b_sample_action_frame_audit.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (output_dir / "errors.json").write_text(
        json.dumps(report["errors"], ensure_ascii=False, indent=2), encoding="utf-8"
    )

    with (output_dir / "arm_b_sample_action_frame_audit.csv").open(
        "w", encoding="utf-8", newline=""
    ) as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "index",
                "label",
                "time_ms",
                "max_delta_from_previous",
                "valid",
                "frame_hex",
            ],
        )
        writer.writeheader()
        for step in step_audits:
            writer.writerow({
                "index": step.index,
                "label": step.label,
                "time_ms": step.time_ms,
                "max_delta_from_previous": step.max_delta_from_previous,
                "valid": step.valid,
                "frame_hex": step.frame_hex,
            })

    write_markdown(report, output_dir)
    print(f"wrote {output_dir}")
    print(f"all_valid={report['all_valid']} max_step_delta={report['max_step_delta']}")
    return 0 if report["all_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
