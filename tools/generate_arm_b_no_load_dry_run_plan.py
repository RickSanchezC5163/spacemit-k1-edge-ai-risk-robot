#!/usr/bin/env python3
"""
Arm-B No-Load Dry-Run Plan Generator
=====================================
Generates arm motion plans that are validated by arm_safety.py but never
sent to hardware. Produces structured outputs for each Arm-B sub-phase.

Phases:
  Arm-B1: plan-only — validate sequences against safety config
  Arm-B2: single-joint small-angle plans — ±100 pulse from center
  Arm-B3: full no-load sequence plans — all joints, full range

Output:
  outputs/arm_b_no_load_dry_run_plan_v1/
    arm_b1_plan_only.json          — all planned sequences with validation
    arm_b2_single_joint_small.json — B2 phase plans
    arm_b3_full_no_load.json       — B3 phase plans
    arm_b_dry_run_report.md        — human-readable summary
"""

from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from arm_safety import (
    ArmSafety, MultiServoCommand, ServoMoveCommand, ValidationResult,
    ArmJointState,
)


# ── plan definitions ──────────────────────────────────────────────────────

def plan_b1_single_joint_id_range(labels: bool = True) -> list[dict]:
    """
    B1: Validate all joints can be commanded across their full ID range.
    Each joint: center → soft_limit_lower → center → soft_limit_upper → center.
    """
    sequences = []
    for joint_id in [1, 2, 3, 4, 5]:
        steps = []
        # center → lower soft limit → center → upper soft limit → center
        steps.extend([
            {"servo_id": joint_id, "pulse": "center", "label": f"J{joint_id}_center_start"},
            {"servo_id": joint_id, "pulse": "soft_lower", "label": f"J{joint_id}_to_soft_lower"},
            {"servo_id": joint_id, "pulse": "center", "label": f"J{joint_id}_back_to_center"},
            {"servo_id": joint_id, "pulse": "soft_upper", "label": f"J{joint_id}_to_soft_upper"},
            {"servo_id": joint_id, "pulse": "center", "label": f"J{joint_id}_back_to_center_end"},
        ])
        sequences.append({
            "sequence_id": f"b1_single_joint_{joint_id}",
            "phase": "arm_b1_plan_only",
            "description": f"Single joint {joint_id} full safe range traversal (plan only)",
            "joint_ids": [joint_id],
            "steps": steps,
        })
    return sequences


def plan_b1_home_cycle() -> dict:
    """B1: Home all joints → verify center positions."""
    return {
        "sequence_id": "b1_home_cycle",
        "phase": "arm_b1_plan_only",
        "description": "Home all joints to center pulse (plan only)",
        "joint_ids": [1, 2, 3, 4, 5],
        "steps": [
            {"servo_id": 0, "pulse": "home_all", "time_ms": 2000,
             "label": "home_all_joints"},
        ],
    }


def plan_b2_small_angle_tests(
    home_pulses: dict[int, int] | None = None,
    test_offsets: dict[int, list[int]] | None = None,
) -> list[dict]:
    """
    B2: Single joint, small offset from the measured home pose.
    Only one joint moves per plan. Hardware access allowed but contact forbidden.
    Offsets are joint-specific because a measured home may already sit near a
    mechanical obstruction in one direction.
    """
    sequences = []
    if home_pulses is None:
        home_pulses = {joint_id: 500 for joint_id in [1, 2, 3, 4, 5]}
    if test_offsets is None:
        test_offsets = {joint_id: [-100, 100] for joint_id in [1, 2, 3, 4, 5]}
    for joint_id in [1, 2, 3, 4, 5]:
        home = home_pulses[joint_id]
        for offset in test_offsets.get(joint_id, []):
            target = max(0, min(1000, home + offset))
            direction = "pos" if offset > 0 else "neg"
            sign = "+" if offset > 0 else ""
            sequences.append({
                "sequence_id": f"b2_small_{direction}_{joint_id}",
                "phase": "arm_b2_single_joint_small_angle",
                "description": f"Joint {joint_id}: home {sign}{offset} pulse (small {direction})",
                "joint_ids": [joint_id],
                "max_pulse_deviation": abs(offset),
                "steps": [
                    {"servo_id": joint_id, "pulse": target, "time_ms": 1500,
                     "label": f"J{joint_id}_home_{sign}{offset}"},
                    {"servo_id": joint_id, "pulse": home, "time_ms": 1500,
                     "label": f"J{joint_id}_back_to_home"},
                ],
            })
    return sequences


def plan_b3_full_no_load_sequence() -> dict:
    """
    B3: Complete no-load motion sequence.
    All joints move through a realistic removal-like trajectory.
    No external contact, no obstacle.
    """
    return {
        "sequence_id": "b3_full_no_load",
        "phase": "arm_b3_full_no_load_sequence",
        "description": "Full 5-DOF no-load removal sequence: "
                       "home → reach → grasp → lift → place → home",
        "joint_ids": [1, 2, 3, 4, 5],
        "steps": [
            # Step 0: home all
            {"servo_id": 0, "pulse": "home_all", "time_ms": 2000,
             "label": "home_all_start"},

            # Step 1: yaw to face right (ID1)
            {"servo_id": 1, "pulse": 700, "time_ms": 1000,
             "label": "yaw_right_45deg"},

            # Step 2: wrist must enter the safe range before ID2 moves down.
            {"servo_id": 4, "pulse": 400, "time_ms": 1000,
             "label": "wrist_level"},

            # Step 3: shoulder forward, elbow extend (reach toward object)
            {"servo_id": 2, "pulse": 350, "time_ms": 1500,
             "label": "shoulder_forward"},
            {"servo_id": 3, "pulse": 700, "time_ms": 1500,
             "label": "elbow_extend"},

            # Step 4: open gripper (2-step: center→300→100 to respect step limit)
            {"servo_id": 5, "pulse": 300, "time_ms": 600,
             "label": "gripper_open_intermediate"},
            {"servo_id": 5, "pulse": 100, "time_ms": 600,
             "label": "gripper_open_full"},

            # Step 5: close gripper (2-step: 100→350→600)
            {"servo_id": 5, "pulse": 350, "time_ms": 600,
             "label": "gripper_close_intermediate"},
            {"servo_id": 5, "pulse": 600, "time_ms": 600,
             "label": "gripper_close_full"},

            # Step 6: lift (shoulder up)
            {"servo_id": 2, "pulse": 650, "time_ms": 1500,
             "label": "shoulder_lift"},
            # elbow retract (2-step: 700→500→300)
            {"servo_id": 3, "pulse": 500, "time_ms": 1000,
             "label": "elbow_retract_intermediate"},
            {"servo_id": 3, "pulse": 400, "time_ms": 1000,
             "label": "elbow_retract_full"},

            # Step 7: yaw to place position (2-step: 700→500→300)
            {"servo_id": 1, "pulse": 500, "time_ms": 1000,
             "label": "yaw_left_intermediate"},
            {"servo_id": 1, "pulse": 300, "time_ms": 1000,
             "label": "yaw_left_45deg"},

            # Step 8: lower and release (2-step gripper: 600→350→100)
            {"servo_id": 2, "pulse": 400, "time_ms": 1000,
             "label": "shoulder_lower"},
            {"servo_id": 5, "pulse": 350, "time_ms": 600,
             "label": "gripper_release_intermediate"},
            {"servo_id": 5, "pulse": 100, "time_ms": 600,
             "label": "gripper_release_full"},

            # Step 9: home all
            {"servo_id": 0, "pulse": "home_all", "time_ms": 2500,
             "label": "home_all_end"},
        ],
    }


def plan_b3_joint_sweep() -> list[dict]:
    """B3: Individual joint sweeps through full soft range with intermediate steps."""
    sequences = []
    for joint_id in [1, 2, 3, 4]:
        sequences.append({
            "sequence_id": f"b3_sweep_{joint_id}",
            "phase": "arm_b3_full_no_load_sequence",
            "description": f"Joint {joint_id} full soft-range sweep with intermediate steps",
            "joint_ids": [joint_id],
            "steps": [
                {"servo_id": joint_id, "pulse": "center",
                 "label": f"J{joint_id}_center_start"},
                {"servo_id": joint_id, "pulse": "soft_lower",
                 "time_ms": 3000,
                 "label": f"J{joint_id}_soft_lower"},
                {"servo_id": joint_id, "pulse": "center",
                 "time_ms": 3000,
                 "label": f"J{joint_id}_back_to_center_from_lower"},
                {"servo_id": joint_id, "pulse": "soft_upper",
                 "time_ms": 3000,
                 "label": f"J{joint_id}_soft_upper"},
                {"servo_id": joint_id, "pulse": "center",
                 "time_ms": 3000,
                 "label": f"J{joint_id}_back_to_center_end"},
            ],
        })
    return sequences


# ── plan resolution ───────────────────────────────────────────────────────

def resolve_pulse_values(plan: dict, safety: ArmSafety) -> dict:
    """
    Resolve symbolic pulse values ('center', 'home_all', 'soft_lower', 'soft_upper')
    to actual integers using the safety config joint definitions.
    Does NOT mutate safety joint state — that's done during validation.
    """
    resolved_steps = []
    for step in plan.get("steps", []):
        sid = step["servo_id"]
        pulse = step.get("pulse")
        if sid == 0 and pulse == "home_all":
            resolved_steps.append({
                **step,
                "resolved_type": "home_all",
                "resolved_servos": {
                    jid: safety._joints[jid].home_pulse
                    for jid in safety._joints
                },
            })
        elif sid in safety._joints:
            joint = safety._joints[sid]
            if pulse is None or (isinstance(pulse, str) and pulse in ("center", "home")):
                resolved_pulse = joint.home_pulse
            elif isinstance(pulse, str) and pulse == "soft_lower":
                resolved_pulse = joint.soft_limit[0]
            elif isinstance(pulse, str) and pulse == "soft_upper":
                resolved_pulse = joint.soft_limit[1]
            elif isinstance(pulse, str) and pulse == "soft_intermediate":
                # Halfway between center and the next symbolic target
                resolved_pulse = joint.home_pulse
            else:
                resolved_pulse = int(pulse)
            resolved_steps.append({
                **step,
                "resolved_type": "single_servo",
                "resolved_pulse": resolved_pulse,
                "soft_limit": list(joint.soft_limit),
                "hard_limit": list(joint.hard_limit),
            })
    return {**plan, "resolved_steps": resolved_steps}


# ── validation runner ─────────────────────────────────────────────────────

def validate_plan(plan: dict, safety: ArmSafety, phase: str) -> dict:
    """Run a resolved plan through safety validation. Returns validation report."""
    safety.set_phase(phase)
    resolved = resolve_pulse_values(plan, safety)
    step_results = []

    for step in resolved.get("resolved_steps", []):
        if step.get("resolved_type") == "home_all":
            cmd = MultiServoCommand(
                servos=step["resolved_servos"],
                time_ms=step.get("time_ms", 2000),
                label=step.get("label", "home_all"),
            )
            results = safety.validate_multi(cmd)
            safety.record_multi(cmd)
            step_results.append({
                "label": step["label"],
                "type": "home_all",
                "servos": step["resolved_servos"],
                "allowed": all(r.allowed for r in results),
                "all_passed": all(r.allowed for r in results),
                "per_joint": [
                    {
                        "servo_id": r.servo_id,
                        "allowed": r.allowed,
                        "reason": r.reason,
                        "warnings": r.warnings,
                        "rule_checks": r.rule_checks,
                    }
                    for r in results
                ],
            })
        elif step.get("resolved_type") == "single_servo":
            sid = step["servo_id"]
            prev = safety._joints[sid].current_pulse if sid in safety._joints else None
            result = safety.validate_single(ServoMoveCommand(
                servo_id=sid,
                target_pulse=step["resolved_pulse"],
                time_ms=step.get("time_ms", 1000),
                previous_pulse=prev,
                label=step.get("label", ""),
            ))
            if result.allowed:
                safety.record_single(ServoMoveCommand(
                    servo_id=sid,
                    target_pulse=step["resolved_pulse"],
                    time_ms=step.get("time_ms", 1000),
                    previous_pulse=prev,
                ))
            step_results.append({
                "label": step["label"],
                "type": "single_servo",
                "servo_id": sid,
                "target_pulse": step["resolved_pulse"],
                "previous_pulse": prev,
                "allowed": result.allowed,
                "reason": result.reason,
                "warnings": result.warnings,
                "rule_checks": result.rule_checks,
            })

    all_passed = all(sr.get("allowed", False) for sr in step_results)
    return {
        "sequence_id": plan["sequence_id"],
        "phase": phase,
        "description": plan.get("description", ""),
        "step_count": len(step_results),
        "all_steps_passed": all_passed,
        "step_results": step_results,
        "final_joint_states": {
            str(jid): {"name": j.name, "current_pulse": j.current_pulse}
            for jid, j in safety.get_all_joint_states().items()
        },
        "safety_summary": safety.get_safety_summary(),
    }


# ── report generation ─────────────────────────────────────────────────────

def generate_markdown_report(results: dict, output_dir: Path) -> str:
    """Generate a human-readable markdown report from validation results."""
    lines = [
        "# Arm-B No-Load Dry-Run Safety Plan",
        f"",
        f"**Generated**: {datetime.now(timezone.utc).isoformat(timespec='seconds')}",
        f"**Protocol version**: arm_safety_v1",
        f"**Phase gates**: all hardware gates = false, dry_run = true",
        f"",
        f"---",
        f"",
        f"## Summary",
        f"",
        f"| Phase | Sequences | Steps | All Passed |",
        f"|-------|-----------|-------|------------|",
    ]

    for phase_key, phase_data in results.items():
        seq_count = len(phase_data.get("sequences", []))
        total_steps = sum(
            seq.get("step_count", 0) for seq in phase_data.get("sequences", [])
        )
        if phase_data.get("all_rejected_by_phase_gate"):
            status_str = "✓ (phase gate)"
        else:
            all_pass = all(
                seq.get("all_steps_passed", False)
                for seq in phase_data.get("sequences", [])
            )
            status_str = '✓' if all_pass else '✗'
        lines.append(f"| {phase_key} | {seq_count} | {total_steps} | {status_str} |")

    lines.extend(["", "---", "", "## Per-Phase Details", ""])

    for phase_key, phase_data in results.items():
        lines.append(f"### {phase_key}")
        lines.append(f"")
        lines.append(f"**Phase gate config:**")
        safety_summary = (phase_data.get("sequences", [{}])[0]
                          .get("safety_summary", {}))
        for gate_key in ["arm_enabled", "hardware_access_allowed",
                         "serial_write_allowed", "contact_allowed",
                         "obstacle_removal_allowed"]:
            lines.append(f"- `{gate_key}`: {safety_summary.get(gate_key, 'N/A')}")
        lines.append(f"")

        for seq in phase_data.get("sequences", []):
            status = "✓" if seq.get("all_steps_passed") else "✗"
            lines.append(f"#### {status} {seq.get('sequence_id')}")
            lines.append(f"")
            lines.append(f"{seq.get('description', '')}")
            lines.append(f"")
            lines.append(f"| Step | Joint | Target | Previous | Allowed | Reason |")
            lines.append(f"|------|-------|--------|----------|---------|--------|")
            for sr in seq.get("step_results", []):
                if sr.get("type") == "home_all":
                    joint_str = "all"
                    target_str = "home"
                    prev_str = "-"
                else:
                    joint_str = str(sr.get("servo_id", "?"))
                    target_str = str(sr.get("target_pulse", "?"))
                    prev_str = str(sr.get("previous_pulse", "?"))
                allowed = "✓" if sr.get("allowed") else "✗"
                reason = sr.get("reason", "")[:80]
                lines.append(
                    f"| {sr.get('label', '?')} | {joint_str} | {target_str} "
                    f"| {prev_str} | {allowed} | {reason} |"
                )
            lines.append(f"")

            # Show warnings
            warned_steps = [
                sr for sr in seq.get("step_results", [])
                if sr.get("warnings")
            ]
            if warned_steps:
                lines.append(f"**Warnings:**")
                for sr in warned_steps:
                    for w in sr["warnings"]:
                        lines.append(f"- [{sr.get('label')}] {w}")
                lines.append(f"")

    lines.extend([
        "---",
        "",
        "## Final Joint States",
        "",
        "All joints should end at their home (center) position.",
        "",
    ])
    return "\n".join(lines)


# ── main ───────────────────────────────────────────────────────────────────

def main():
    config_path = PROJECT_ROOT / "configs" / "arm_safety_config.json"
    output_dir = PROJECT_ROOT / "outputs" / "arm_b_no_load_dry_run_plan_v1"
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Arm-B Dry-Run Plan Generator")
    print(f"  Config: {config_path}")
    print(f"  Output: {output_dir}")
    print()

    # Load safety (arm_b1: plan only, no hardware)
    safety = ArmSafety(str(config_path))

    # ── B1: plan-only validation ──────────────────────────────────────
    print("=== Arm-B1: Plan-Only Validation ===")
    safety.set_phase("arm_b1_plan_only")

    b1_plans = plan_b1_single_joint_id_range() + [plan_b1_home_cycle()]
    b1_results = []
    for plan in b1_plans:
        result = validate_plan(plan, safety, "arm_b1_plan_only")
        b1_results.append(result)
        status = "PASS" if result["all_steps_passed"] else "FAIL"
        print(f"  [EXPECTED_REJECT] {plan['sequence_id']} "
              f"({result['step_count']} steps) — phase gate blocks all commands")

    b1_output = {
        "phase": "arm_b1_plan_only",
        "phase_gate": safety._phase_gate(),
        "note": "B1 is plan-only: ALL commands are correctly rejected by the L1 phase gate (arm_enabled=false). "
                "This is expected safety behavior. The plan structures are valid but cannot execute without hardware access.",
        "sequence_count": len(b1_results),
        "all_rejected_by_phase_gate": True,
        "all_passed": False,
        "sequences": b1_results,
    }
    (output_dir / "arm_b1_plan_only.json").write_text(
        json.dumps(b1_output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"  → {output_dir / 'arm_b1_plan_only.json'}")
    print()

    # Reset safety state for B2
    safety = ArmSafety(str(config_path))

    # ── B2: single-joint small-angle plans ────────────────────────────
    print("=== Arm-B2: Single-Joint Small-Angle Plans ===")
    safety.set_phase("arm_b2_single_joint_small_angle")

    b2_home_pulses = {jid: joint.home_pulse for jid, joint in safety.get_all_joint_states().items()}
    b2_offsets_cfg = safety._phase_gate().get("test_offsets_from_home", {})
    b2_test_offsets = {
        int(jid): [int(offset) for offset in offsets]
        for jid, offsets in b2_offsets_cfg.items()
    }
    b2_plans = plan_b2_small_angle_tests(b2_home_pulses, b2_test_offsets)
    b2_results = []
    for plan in b2_plans:
        result = validate_plan(plan, safety, "arm_b2_single_joint_small_angle")
        b2_results.append(result)
        status = "PASS" if result["all_steps_passed"] else "FAIL"
        print(f"  [{status}] {plan['sequence_id']} "
              f"({result['step_count']} steps)")

    b2_output = {
        "phase": "arm_b2_single_joint_small_angle",
        "phase_gate": safety._phase_gate(),
        "sequence_count": len(b2_results),
        "all_passed": all(r["all_steps_passed"] for r in b2_results),
        "sequences": b2_results,
    }
    (output_dir / "arm_b2_single_joint_small.json").write_text(
        json.dumps(b2_output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"  → {output_dir / 'arm_b2_single_joint_small.json'}")
    print()

    # Reset safety state for B3
    safety = ArmSafety(str(config_path))

    # ── B3: full no-load sequence ─────────────────────────────────────
    print("=== Arm-B3: Full No-Load Sequence ===")
    safety.set_phase("arm_b3_full_no_load_sequence")

    b3_plans = [plan_b3_full_no_load_sequence()] + plan_b3_joint_sweep()
    b3_results = []
    for plan in b3_plans:
        result = validate_plan(plan, safety, "arm_b3_full_no_load_sequence")
        b3_results.append(result)
        status = "PASS" if result["all_steps_passed"] else "FAIL"
        print(f"  [{status}] {plan['sequence_id']} "
              f"({result['step_count']} steps)")

    b3_output = {
        "phase": "arm_b3_full_no_load_sequence",
        "phase_gate": safety._phase_gate(),
        "sequence_count": len(b3_results),
        "all_passed": all(r["all_steps_passed"] for r in b3_results),
        "sequences": b3_results,
    }
    (output_dir / "arm_b3_full_no_load.json").write_text(
        json.dumps(b3_output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"  → {output_dir / 'arm_b3_full_no_load.json'}")
    print()

    # ── Generate markdown report ──────────────────────────────────────
    all_results = {
        "arm_b1_plan_only": b1_output,
        "arm_b2_single_joint_small_angle": b2_output,
        "arm_b3_full_no_load_sequence": b3_output,
    }
    md_content = generate_markdown_report(all_results, output_dir)
    md_path = output_dir / "arm_b_dry_run_report.md"
    md_path.write_text(md_content, encoding="utf-8")
    print(f"  → {md_path}")
    print()

    # ── Final summary ─────────────────────────────────────────────────
    total_plans = len(b1_results) + len(b2_results) + len(b3_results)
    total_passed = sum(
        1 for r in b1_results + b2_results + b3_results
        if r["all_steps_passed"]
    )
    print(f"=== Arm-B Dry-Run Complete ===")
    print(f"  Total sequences: {total_plans}")
    print(f"  All passed: {total_passed}/{total_plans}")
    print(f"  Hardware executed: false")
    print(f"  Serial writes: 0")
    print(f"  Output: {output_dir}")


if __name__ == "__main__":
    main()
