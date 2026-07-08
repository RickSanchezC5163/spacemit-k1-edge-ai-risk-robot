"""Dry-run safety gate helpers for semantic primitives."""

from __future__ import annotations

from typing import Any, Dict

from .schemas import now_iso


ARM_PREFIX = "ARM_"
CAPTURE_PRIMITIVES = {"HOLD_CAPTURE", "D435_CAPTURE"}
CHASSIS_MOTION = {"FORWARD_0P15", "ARC_FAST_LEFT", "ARC_FAST_RIGHT", "STOP_SAFE"}


def evaluate_safety_gate(
    primitive: str,
    observation: Dict[str, Any] | None = None,
    execution_mode: str = "dry_run",
    requested_direct_cmd_vel: bool = False,
    requested_servo_pulse: bool = False,
) -> Dict[str, Any]:
    obs = observation or {}
    base_zero_required = primitive.startswith(ARM_PREFIX) or primitive in CAPTURE_PRIMITIVES or primitive == "SAVE_MAP"
    base_zero_ok = obs.get("base_zero")
    no_direct_cmd_vel = not requested_direct_cmd_vel
    no_direct_servo_pulse = not requested_servo_pulse
    hardware_executed = execution_mode == "hardware_gated"
    allowed = True
    reasons: list[str] = []

    if not no_direct_cmd_vel:
        allowed = False
        reasons.append("direct_cmd_vel_forbidden")
    if not no_direct_servo_pulse:
        allowed = False
        reasons.append("direct_servo_pulse_forbidden")
    if base_zero_required and base_zero_ok is not True:
        allowed = False
        reasons.append("base_zero_required_not_satisfied")
    if execution_mode == "hardware_gated" and primitive == "ARM_CONTROLLED_CLEAR":
        allowed = False
        reasons.append("ARM_CONTROLLED_CLEAR_not_enabled_in_current_stage")

    return {
        "gate_id": f"safety_gate_{primitive.lower()}",
        "timestamp": now_iso(),
        "primitive": primitive,
        "allowed": allowed,
        "reason": "ok" if allowed else ",".join(reasons),
        "base_zero_required": base_zero_required,
        "base_zero_ok": base_zero_ok if base_zero_required else None,
        "no_direct_cmd_vel": no_direct_cmd_vel,
        "no_direct_servo_pulse": no_direct_servo_pulse,
        "hardware_executed": hardware_executed and allowed,
        "execution_mode": execution_mode,
    }
