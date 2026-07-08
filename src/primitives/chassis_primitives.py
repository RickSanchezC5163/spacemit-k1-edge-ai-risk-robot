"""Dry-run chassis primitive records.

Real chassis motion must remain in existing P4/Step7 runners. This module only
creates semantic action/result records for sim/RL/dry-run interfaces.
"""

from __future__ import annotations

from typing import Any, Dict

from .safety_gate import evaluate_safety_gate
from .schemas import now_iso


def chassis_action_candidate(primitive: str, observation: Dict[str, Any] | None = None) -> Dict[str, Any]:
    gate = evaluate_safety_gate(primitive, observation or {}, execution_mode="dry_run")
    return {
        "candidate_id": f"candidate_{primitive.lower()}",
        "policy_name": "semantic_guarded_nav",
        "rl_action": primitive,
        "confidence": 1.0 if gate["allowed"] else 0.0,
        "observation": observation or {},
        "safety_gate_required": primitive not in {"HOLD"},
        "allowed_by_safety_gate": gate["allowed"],
        "execution_mode": "dry_run_only",
        "safety_gate": gate,
    }


def dry_run_chassis_primitive(primitive: str, observation: Dict[str, Any] | None = None) -> Dict[str, Any]:
    gate = evaluate_safety_gate(primitive, observation or {}, execution_mode="dry_run")
    started = now_iso()
    return {
        "action_id": f"dryrun_{primitive.lower()}",
        "primitive": primitive,
        "status": "succeeded_dry_run" if gate["allowed"] else "blocked",
        "started_at": started,
        "ended_at": now_iso(),
        "hardware_executed": False,
        "published_cmd_vel": False,
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "base_zero_ok_before": (observation or {}).get("base_zero"),
        "details": {
            "real_command_path": "/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded",
            "direct_cmd_vel_bypass": False,
            "dry_run_only": True,
        },
        "safety_gate": gate,
    }
