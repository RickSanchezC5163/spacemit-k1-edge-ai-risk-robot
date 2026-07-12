#!/usr/bin/env python3
"""
Generate a MoveIt-side arm plan plus a simulated gripper event schedule.

This is intentionally not a full gripper URDF implementation.  MoveIt is
expected to plan only the 4 arm joints (j1-j4).  Gripper open/close is modeled
as discrete state events attached to arm waypoints so the risk pipeline can
rehearse pick-place behavior before the real gripper model is available.
"""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from arm_safety import ArmSafety, MultiServoCommand  # noqa: E402

GripperState = Literal["open", "closed"]


@dataclass(frozen=True)
class Waypoint:
    name: str
    duration_ms: int
    arm_targets: dict[int, int]
    gripper_state: GripperState
    gripper_event: str
    task_phase: str
    note: str


def default_waypoints() -> list[Waypoint]:
    """Conservative no-load pick-place skeleton based on current pulse config."""
    return [
        Waypoint(
            name="home_closed_start",
            duration_ms=1200,
            arm_targets={1: 510, 2: 771, 3: 426, 4: 503},
            gripper_state="closed",
            gripper_event="hold_closed",
            task_phase="home",
            note="Start at safe idle. Gripper is visually closed.",
        ),
        Waypoint(
            name="open_before_motion",
            duration_ms=500,
            arm_targets={1: 510, 2: 771, 3: 426, 4: 503},
            gripper_state="open",
            gripper_event="open",
            task_phase="prepare",
            note="Open simulated gripper before approach motion starts.",
        ),
        Waypoint(
            name="pre_grasp_above_ground",
            duration_ms=1400,
            arm_targets={1: 510, 2: 560, 3: 520, 4: 465},
            gripper_state="open",
            gripper_event="hold_open",
            task_phase="approach",
            note="Move above the risk/object point, no contact.",
        ),
        Waypoint(
            name="lower_to_ground_pick",
            duration_ms=1600,
            arm_targets={1: 510, 2: 380, 3: 650, 4: 470},
            gripper_state="open",
            gripper_event="hold_open",
            task_phase="descend",
            note="Lower toward ground-level simulated pickup pose.",
        ),
        Waypoint(
            name="close_on_ground",
            duration_ms=700,
            arm_targets={1: 510, 2: 380, 3: 650, 4: 470},
            gripper_state="closed",
            gripper_event="close",
            task_phase="grasp",
            note="Close simulated gripper at the ground pose.",
        ),
        Waypoint(
            name="lift_with_closed_gripper",
            duration_ms=1500,
            arm_targets={1: 510, 2: 650, 3: 500, 4: 488},
            gripper_state="closed",
            gripper_event="hold_closed",
            task_phase="lift",
            note="Lift while keeping simulated gripper closed.",
        ),
        Waypoint(
            name="transfer_to_place_side",
            duration_ms=1500,
            arm_targets={1: 650, 2: 650, 3: 500, 4: 488},
            gripper_state="closed",
            gripper_event="hold_closed",
            task_phase="transfer",
            note="Yaw to another placement area.",
        ),
        Waypoint(
            name="lower_to_place",
            duration_ms=1400,
            arm_targets={1: 650, 2: 430, 3: 610, 4: 465},
            gripper_state="closed",
            gripper_event="hold_closed",
            task_phase="place_descend",
            note="Lower to simulated place pose.",
        ),
        Waypoint(
            name="release_at_place",
            duration_ms=700,
            arm_targets={1: 650, 2: 430, 3: 610, 4: 465},
            gripper_state="open",
            gripper_event="open",
            task_phase="release",
            note="Open simulated gripper to release at the place pose.",
        ),
        Waypoint(
            name="retreat_open",
            duration_ms=1200,
            arm_targets={1: 650, 2: 650, 3: 500, 4: 488},
            gripper_state="open",
            gripper_event="hold_open",
            task_phase="retreat",
            note="Retreat from place pose before returning home.",
        ),
        Waypoint(
            name="home_closed_end",
            duration_ms=1800,
            arm_targets={1: 510, 2: 771, 3: 426, 4: 503},
            gripper_state="closed",
            gripper_event="close",
            task_phase="home",
            note="Return to safe idle and close simulated gripper.",
        ),
    ]


def validate_waypoints(waypoints: list[Waypoint], safety: ArmSafety) -> list[dict]:
    safety.set_phase("arm_b3_full_no_load_sequence")
    records = []
    for index, waypoint in enumerate(waypoints):
        cmd = MultiServoCommand(
            servos=waypoint.arm_targets,
            time_ms=waypoint.duration_ms,
            label=waypoint.name,
        )
        results = safety.validate_multi(cmd)
        allowed = all(result.allowed for result in results)
        if allowed:
            safety.record_multi(cmd)
        records.append(
            {
                "index": index,
                "name": waypoint.name,
                "task_phase": waypoint.task_phase,
                "duration_ms": waypoint.duration_ms,
                "moveit_arm_group": "arm",
                "moveit_joint_targets": {
                    f"j{joint_id}": pulse for joint_id, pulse in waypoint.arm_targets.items()
                },
                "servo_pulse_targets": {
                    str(joint_id): pulse for joint_id, pulse in waypoint.arm_targets.items()
                },
                "simulated_gripper": {
                    "state": waypoint.gripper_state,
                    "event": waypoint.gripper_event,
                    "mode": "discrete_event_not_urdf_joint",
                },
                "allowed_by_arm_safety": allowed,
                "per_joint": [
                    {
                        "servo_id": result.servo_id,
                        "allowed": result.allowed,
                        "reason": result.reason,
                        "warnings": result.warnings,
                        "rule_checks": result.rule_checks,
                    }
                    for result in results
                ],
                "note": waypoint.note,
            }
        )
    return records


def write_markdown(report: dict, path: Path) -> None:
    lines = [
        "# MoveIt Arm + Simulated Gripper Pick-Place Plan",
        "",
        f"Generated: `{report['generated_at']}`",
        "",
        "## Boundary",
        "",
        "- MoveIt plans only arm joints `j1`-`j4`.",
        "- The gripper is a simulated discrete state event, not a URDF joint.",
        "- No hardware, serial port, chassis command, or real contact is used.",
        "- This is a no-load planning candidate for leakage/risk response rehearsal.",
        "",
        "## Sequence",
        "",
        "| # | Waypoint | Arm Target Pulses | Gripper | Safety |",
        "|---|---|---|---|---|",
    ]
    for step in report["waypoints"]:
        pulses = ", ".join(f"{k}:{v}" for k, v in step["servo_pulse_targets"].items())
        grip = f"{step['simulated_gripper']['event']} -> {step['simulated_gripper']['state']}"
        safety = "PASS" if step["allowed_by_arm_safety"] else "FAIL"
        lines.append(f"| {step['index']} | `{step['name']}` | {pulses} | {grip} | {safety} |")

    lines.extend(
        [
            "",
            "## Intended Behavior",
            "",
            "1. Start at home with the simulated gripper closed.",
            "2. Open the simulated gripper before arm motion starts.",
            "3. Approach and lower to the ground-level pickup pose.",
            "4. Close the simulated gripper at the ground pose.",
            "5. Lift, transfer to another place, lower, then open to release.",
            "6. Retreat and return home with the simulated gripper closed.",
            "",
            "## MoveIt Integration Notes",
            "",
            "- Use these waypoints as joint-space targets for the `arm` planning group.",
            "- Attach gripper events to trajectory boundaries in the executor.",
            "- Do not add a fake gripper collision body to MoveIt until a mechanical gripper export exists.",
            "- Before real execution, replace pulse placeholders with calibrated joint angles and rerun collision checks.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    output_dir = PROJECT_ROOT / "outputs" / "moveit_gripper_sim_pick_place_plan_v1"
    output_dir.mkdir(parents=True, exist_ok=True)

    safety = ArmSafety(str(PROJECT_ROOT / "configs" / "arm_safety_config.json"))
    waypoints = default_waypoints()
    waypoint_records = validate_waypoints(waypoints, safety)

    report = {
        "schema_version": "moveit_gripper_sim_pick_place_plan_v1",
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "planning_boundary": {
            "moveit_plans_arm_joints_only": True,
            "simulated_gripper_not_urdf_joint": True,
            "hardware_executed": False,
            "serial_bytes_written": 0,
            "cmd_vel_published": False,
            "contact_allowed": False,
        },
        "moveit": {
            "description_package": "mechanical_arm_1_description",
            "planning_group": "arm",
            "joint_names": ["j1", "j2", "j3", "j4"],
            "end_effector_model": "simulated_discrete_gripper_event",
        },
        "summary": {
            "waypoint_count": len(waypoint_records),
            "all_arm_waypoints_pass_safety": all(
                step["allowed_by_arm_safety"] for step in waypoint_records
            ),
            "final_gripper_state": waypoint_records[-1]["simulated_gripper"]["state"],
            "final_waypoint": waypoint_records[-1]["name"],
        },
        "waypoints": waypoint_records,
    }

    json_path = output_dir / "moveit_gripper_sim_pick_place_plan.json"
    md_path = output_dir / "moveit_gripper_sim_pick_place_plan.md"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_markdown(report, md_path)

    print(f"wrote {json_path}")
    print(f"wrote {md_path}")
    print(json.dumps(report["summary"], ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
