"""Arm primitive dry-run and Arm-D staging helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

from .safety_gate import evaluate_safety_gate
from .schemas import now_iso, write_json, write_text


def clearance_candidate_dryrun(
    risk_map_summary: Dict[str, Any] | None,
    output_dir: str | Path | None = None,
) -> Dict[str, Any]:
    risks = (risk_map_summary or {}).get("risk_points") or []
    selected = risks[0] if risks else None
    observation = {"base_zero": True, "risk_detected": bool(selected)}
    gate = evaluate_safety_gate("ARM_CLEAR_CANDIDATE_DRYRUN", observation, execution_mode="dry_run")
    candidate = {
        "candidate_id": "arm_d0_clearance_candidate_001",
        "primitive": "ARM_CLEAR_CANDIDATE_DRYRUN",
        "stage": "Arm-D0",
        "status": "succeeded_dry_run" if gate["allowed"] else "blocked",
        "risk_point": selected,
        "selected_sequence": "arm_b3_8_step_safety_adjusted_no_load_sample",
        "requires_base_zero": True,
        "hardware_executed": False,
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "contact_allowed": False,
        "obstacle_removal_allowed": False,
        "published_cmd_vel_during_arm": False,
        "final_pose_expected": "6b",
        "safety_gate": gate,
        "claim_boundary": [
            "Arm-D0 is dry-run only.",
            "No contact, grasping, payload handling, or obstacle removal is claimed.",
        ],
    }
    if output_dir:
        out = Path(output_dir)
        write_json(out / "clearance_candidate.json", candidate)
        write_json(out / "errors.json", [] if gate["allowed"] else [{"code": "candidate_blocked", "reason": gate["reason"]}])
        write_text(
            out / "arm_d0_report.md",
            "# Arm-D0 Clearance Candidate Dry-Run\n\n"
            f"- status: `{candidate['status']}`\n"
            "- hardware_executed: `false`\n"
            "- contact_allowed: `false`\n"
            "- obstacle_removal_allowed: `false`\n"
            "- final_pose_expected: `6b`\n",
        )
    return candidate


def arm_no_load_response_result(action_id: str = "arm_no_load_response_dryrun") -> Dict[str, Any]:
    return {
        "action_id": action_id,
        "primitive": "ARM_NO_LOAD_RESPONSE",
        "status": "succeeded_dry_run",
        "started_at": now_iso(),
        "ended_at": now_iso(),
        "hardware_executed": False,
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "contact_allowed": False,
        "obstacle_removal_allowed": False,
        "published_cmd_vel": False,
        "base_zero_ok_before": True,
        "final_pose_expected": "6b",
    }
