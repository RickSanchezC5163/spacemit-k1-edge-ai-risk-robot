#!/usr/bin/env python3
"""
K1 Arm Safety Module
====================
Validates all 5-DOF bus servo commands against safety boundaries before
any serial transmission. Designed for phased deployment: Arm-B dry-run
through Arm-E real obstacle removal.

Protocol: Lobot Bus Servo UART
  Frame: 0x55 0x55 <len> <cmd> <params...>
  CMD 3 (SERVO_MOVE): <count> <time_lo> <time_hi> [<id> <pulse_lo> <pulse_hi>]...

Usage:
  from arm_safety import ArmSafety

  safety = ArmSafety("configs/arm_safety_config.json")
  safety.set_phase("arm_b1_plan_only")

  cmd = ServoMoveCommand(servo_id=2, target_pulse=600, time_ms=1000)
  result = safety.validate_command(cmd)
  if result.allowed:
      safety.record_command(cmd)  # dry-run: no serial write
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# ── protocol constants (from Lobot bus servo SDK) ────────────────────────

FRAME_HEADER = bytes([0x55, 0x55])
CMD_SERVO_MOVE = 3
CMD_ACTION_GROUP_RUN = 6
CMD_ACTION_GROUP_STOP = 7
CMD_ACTION_GROUP_SPEED = 11
CMD_GET_BATTERY_VOLTAGE = 15

BUS_PULSE_MIN = 0
BUS_PULSE_MAX = 1000
BUS_PULSE_CENTER = 500
TIME_MIN_MS = 0
TIME_MAX_MS = 30000
VALID_SERVO_IDS = {1, 2, 3, 4, 5}


# ── data classes ──────────────────────────────────────────────────────────

@dataclass
class ServoMoveCommand:
    """Single servo move command. All fields validated before serial write."""
    servo_id: int
    target_pulse: int
    time_ms: int
    previous_pulse: Optional[int] = None
    label: str = ""


@dataclass
class MultiServoCommand:
    """Multi-servo synchronized move. All servos share the same move time."""
    servos: Dict[int, int]  # {servo_id: target_pulse}
    time_ms: int
    label: str = ""


@dataclass
class ValidationResult:
    """Result of a safety validation check."""
    allowed: bool
    servo_id: int
    target_pulse: int
    reason: str = ""
    warnings: List[str] = field(default_factory=list)
    rule_checks: Dict[str, bool] = field(default_factory=dict)


@dataclass
class ArmJointState:
    """Tracked state of a single joint."""
    servo_id: int
    name: str
    current_pulse: int
    home_pulse: int
    soft_limit: Tuple[int, int]
    hard_limit: Tuple[int, int]
    angle_range_deg: Tuple[float, float]


# ── simplified forward kinematics (for workspace checking) ───────────────

class ArmKinematics:
    """
    Simplified FK for the K1 5-DOF arm. Uses joint pulses to estimate
    end-effector position for workspace boundary checks.

    Coordinate frame: origin at ID1 rotation axis.
      +x: forward (chassis front)
      +y: left
      +z: up

    Arm structure:
      ID1 (yaw, z-axis) → L1 (19cm vertical, z) → ID2 (shoulder, y-axis)
        → L2 (4cm horizontal, x) → ID3 (elbow, y-axis)
        → L3 (19cm vertical, z) → ID4 (wrist, y-axis)
        → L4 (5.5cm wrist, x) → gripper
    """

    # Link lengths in meters (from MODEL_MEASUREMENTS.md)
    L1 = 0.19   # ID1→ID2 vertical arm
    L2 = 0.04   # ID2→ID3 short link
    L3 = 0.19   # ID3→ID4 vertical arm
    L4 = 0.055  # wrist length
    L5 = 0.060  # finger length

    def __init__(self, arm_base_x: float = -0.005, arm_base_y: float = 0.0,
                 arm_base_z: float = 0.13):
        self.base_x = arm_base_x
        self.base_y = arm_base_y
        self.base_z = arm_base_z

    @staticmethod
    def pulse_to_angle_rad(pulse: int, angle_range_deg: Tuple[float, float]) -> float:
        """Convert bus servo pulse (0-1000) to joint angle in radians."""
        lo_deg, hi_deg = angle_range_deg
        t = pulse / 1000.0  # 0.0 to 1.0
        angle_deg = lo_deg + t * (hi_deg - lo_deg)
        return math.radians(angle_deg)

    def end_effector_position(self, pulses: Dict[int, int],
                               angle_ranges: Dict[int, Tuple[float, float]]
                               ) -> Tuple[float, float, float]:
        """
        Estimate end-effector (gripper center) position given joint pulses.
        Returns (x, y, z) in meters relative to chassis center.
        """
        # Get joint angles
        yaw = self.pulse_to_angle_rad(pulses.get(1, 500), angle_ranges.get(1, (-180, 180)))
        shoulder = self.pulse_to_angle_rad(pulses.get(2, 500), angle_ranges.get(2, (-90, 90)))
        elbow = self.pulse_to_angle_rad(pulses.get(3, 500), angle_ranges.get(3, (-105, 105)))
        wrist = self.pulse_to_angle_rad(pulses.get(4, 500), angle_ranges.get(4, (-90, 90)))

        # Simplified FK: treat arm as planar 3-DOF in the vertical plane,
        # then rotate the whole plane by yaw.

        # Vertical plane positions (in arm's local frame, before yaw rotation)
        # ID1 → ID2: vertical segment (L1 along z)
        z1 = self.L1

        # ID2 → ID3: shoulder pitch rotates L2 in xz plane
        x2 = self.L2 * math.cos(shoulder)
        z2 = self.L2 * math.sin(shoulder)

        # ID3 → ID4: elbow pitch, relative to shoulder output orientation
        elbow_abs = shoulder + elbow  # absolute angle from horizontal
        x3 = self.L3 * math.cos(elbow_abs)
        z3 = self.L3 * math.sin(elbow_abs)

        # ID4 → gripper: wrist pitch
        wrist_abs = elbow_abs + wrist
        x4 = (self.L4 + self.L5) * math.cos(wrist_abs)
        z4 = (self.L4 + self.L5) * math.sin(wrist_abs)

        # Total in arm-local frame
        x_local = x2 + x3 + x4
        y_local = 0.0
        z_local = z1 + z2 + z3 + z4

        # Rotate by yaw around z-axis
        x_global = self.base_x + x_local * math.cos(yaw)
        y_global = self.base_y + x_local * math.sin(yaw)
        z_global = self.base_z + z_local

        return (x_global, y_global, z_global)

    def is_in_workspace(self, pulses: Dict[int, int],
                        angle_ranges: Dict[int, Tuple[float, float]],
                        workspace_safe_zone: Dict[str, float]) -> bool:
        """Check if end-effector is within the safe operating zone."""
        x, y, z = self.end_effector_position(pulses, angle_ranges)
        return (
            -workspace_safe_zone.get("rearward_m", 0.0) <= x <= workspace_safe_zone.get("forward_m", 0.35)
            and -workspace_safe_zone.get("right_m", 0.30) <= y <= workspace_safe_zone.get("left_m", 0.30)
            and workspace_safe_zone.get("down_m", 0.01) <= z <= workspace_safe_zone.get("up_m", 0.45)
        )


# ── main safety class ─────────────────────────────────────────────────────

class ArmSafety:
    """
    K1 5-DOF bus servo arm safety validator.

    Safety architecture (layered):
      Layer 1 — Phase gates: which phases allow hardware access
      Layer 2 — Protocol validation: servo ID, pulse range, time range
      Layer 3 — Per-joint soft/hard limits
      Layer 4 — Step-size limits (prevent sudden large moves)
      Layer 5 — Workspace collision (keep-out zones, chassis clearance)
      Layer 6 — Robot-level safety (base zero required, driving prohibited)
      Layer 7 — Heartbeat / emergency stop
    """

    def __init__(self, config_path: str = "configs/arm_safety_config.json"):
        config_file = Path(config_path)
        if not config_file.exists():
            raise FileNotFoundError(f"Arm safety config not found: {config_path}")
        self.config = json.loads(config_file.read_text(encoding="utf-8"))
        self._joints: Dict[int, ArmJointState] = {}
        self._current_phase: str = "arm_b1_plan_only"
        self._command_history: List[MultiServoCommand] = []
        self._last_heartbeat: float = 0.0
        self._estop_active: bool = False
        self._base_zero_ok: bool = True
        self._robot_driving: bool = False
        self._error_count: int = 0
        self._kinematics = ArmKinematics(
            arm_base_x=self.config["workspace"]["arm_base"]["x"],
            arm_base_y=self.config["workspace"]["arm_base"]["y"],
            arm_base_z=self.config["workspace"]["arm_base"]["z"],
        )
        self._init_joint_states()

    def _init_joint_states(self) -> None:
        """Initialize joint state tracking from config."""
        for joint_id_str, joint_cfg in self.config["joints"].items():
            jid = int(joint_id_str)
            self._joints[jid] = ArmJointState(
                servo_id=jid,
                name=joint_cfg["name"],
                current_pulse=joint_cfg["home_pulse"],
                home_pulse=joint_cfg["home_pulse"],
                soft_limit=(
                    joint_cfg["soft_limit_lower_pulse"],
                    joint_cfg["soft_limit_upper_pulse"],
                ),
                hard_limit=(
                    joint_cfg["hard_limit_lower_pulse"],
                    joint_cfg["hard_limit_upper_pulse"],
                ),
                angle_range_deg=tuple(joint_cfg["angle_range_deg"]),
            )

    # ── phase management ───────────────────────────────────────────────

    @property
    def current_phase(self) -> str:
        return self._current_phase

    def set_phase(self, phase: str) -> ValidationResult:
        """Set the current safety phase. Returns validation result."""
        if phase not in self.config["phase_gates"]:
            return ValidationResult(
                allowed=False, servo_id=0, target_pulse=0,
                reason=f"Unknown phase: {phase}. "
                        f"Valid phases: {list(self.config['phase_gates'].keys())}"
            )
        self._current_phase = phase
        return ValidationResult(
            allowed=True, servo_id=0, target_pulse=0,
            reason=f"Phase set to {phase}"
        )

    def _global_gate(self, key: str) -> bool:
        """Read a boolean gate from the global safety_gates block."""
        return bool(self.config.get("safety_gates", {}).get(key, False))

    def _phase_gate(self) -> Dict[str, Any]:
        return self.config["phase_gates"].get(self._current_phase, {})

    def _effective_gate(self, key: str) -> bool:
        """Effective gate = global_gate AND phase_gate.
        A phase CANNOT bypass a global gate that is set to false.
        """
        global_val = self._global_gate(key)
        phase_val = bool(self._phase_gate().get(key, False))
        return global_val and phase_val

    @property
    def arm_enabled(self) -> bool:
        """Logical phase permission for dry-run validation.

        Hardware access and serial writes are still controlled by effective
        global AND phase gates. This property intentionally follows the phase
        gate so B2/B3 plans can be audited while global hardware gates remain
        closed.
        """
        return bool(self._phase_gate().get("arm_enabled", False))

    @property
    def hardware_access_allowed(self) -> bool:
        return self._effective_gate("hardware_access_allowed")

    @property
    def serial_write_allowed(self) -> bool:
        """Effective serial write permission: global AND phase.
        When global serial_write_allowed=false, no phase can write serial."""
        return self._effective_gate("serial_write_allowed")

    @property
    def serial_write_allowed_global(self) -> bool:
        """The global serial_write_allowed gate value (before AND with phase)."""
        return self._global_gate("serial_write_allowed")

    @property
    def serial_write_allowed_phase(self) -> bool:
        """The phase-level serial_write_allowed value (before AND with global)."""
        return bool(self._phase_gate().get("serial_write_allowed", False))

    @property
    def contact_allowed(self) -> bool:
        return self._effective_gate("contact_allowed")

    @property
    def obstacle_removal_allowed(self) -> bool:
        return self._effective_gate("obstacle_removal_allowed")

    # ── robot state updates ────────────────────────────────────────────

    def update_base_zero(self, ok: bool) -> None:
        self._base_zero_ok = ok

    def update_robot_driving(self, driving: bool) -> None:
        self._robot_driving = driving

    def heartbeat(self) -> None:
        self._last_heartbeat = time.monotonic()

    def emergency_stop(self) -> None:
        self._estop_active = True

    def emergency_stop_clear(self) -> None:
        self._estop_active = False

    # ── command validation ─────────────────────────────────────────────

    def validate_single(self, cmd: ServoMoveCommand) -> ValidationResult:
        """Validate a single servo move command against all safety rules."""
        return self._validate(cmd.servo_id, cmd.target_pulse, cmd.time_ms,
                              cmd.previous_pulse)

    def validate_multi(self, cmd: MultiServoCommand) -> List[ValidationResult]:
        """Validate a multi-servo command. Returns per-joint results."""
        results = []
        trial_pulses = {jid: j.current_pulse for jid, j in self._joints.items()}
        trial_pulses.update(cmd.servos)
        for servo_id, target_pulse in cmd.servos.items():
            prev = self._joints[servo_id].current_pulse if servo_id in self._joints else None
            results.append(self._validate(
                servo_id,
                target_pulse,
                cmd.time_ms,
                prev,
                trial_pulses_override=trial_pulses,
            ))
        return results

    def validate_all(self, cmd: MultiServoCommand) -> ValidationResult:
        """Validate multi-servo command. Single result: all joints must pass."""
        per_joint = self.validate_multi(cmd)
        failed = [r for r in per_joint if not r.allowed]
        if failed:
            return ValidationResult(
                allowed=False,
                servo_id=failed[0].servo_id,
                target_pulse=failed[0].target_pulse,
                reason="; ".join(f"{r.servo_id}: {r.reason}" for r in failed),
                rule_checks={f"joint_{r.servo_id}": r.allowed for r in per_joint},
            )
        return ValidationResult(
            allowed=True, servo_id=0, target_pulse=0,
            reason="all joints pass",
            rule_checks={f"joint_{r.servo_id}": True for r in per_joint},
        )

    def _validate(self, servo_id: int, target_pulse: int, time_ms: int,
                  previous_pulse: Optional[int] = None,
                  trial_pulses_override: Optional[Dict[int, int]] = None) -> ValidationResult:
        """Core validation logic. Checks all safety layers."""
        warnings: List[str] = []
        checks: Dict[str, bool] = {}

        phase = self._phase_gate()

        # L1: Phase gate — is arm enabled?
        if not phase.get("arm_enabled", False):
            return ValidationResult(
                allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                reason=f"Phase '{self._current_phase}': arm_enabled=false. "
                       f"Set phase to arm_b2 or higher for hardware access.",
                rule_checks={"L1_arm_enabled": False},
            )

        # L2: Protocol validation
        if servo_id not in VALID_SERVO_IDS:
            return ValidationResult(
                allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                reason=f"Invalid servo ID {servo_id}. Valid: {sorted(VALID_SERVO_IDS)}",
                rule_checks={"L2_valid_servo_id": False},
            )
        checks["L2_valid_servo_id"] = True

        if not (BUS_PULSE_MIN <= target_pulse <= BUS_PULSE_MAX):
            return ValidationResult(
                allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                reason=f"Pulse {target_pulse} out of range [{BUS_PULSE_MIN}, {BUS_PULSE_MAX}]",
                rule_checks={"L2_pulse_range": False},
            )
        checks["L2_pulse_range"] = True

        if not (TIME_MIN_MS <= time_ms <= TIME_MAX_MS):
            return ValidationResult(
                allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                reason=f"Move time {time_ms}ms out of range [{TIME_MIN_MS}, {TIME_MAX_MS}]",
                rule_checks={"L2_time_range": False},
            )
        checks["L2_time_range"] = True

        # L3: Per-joint soft/hard limits
        joint = self._joints.get(servo_id)
        if joint is None:
            return ValidationResult(
                allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                reason=f"Joint {servo_id} not configured",
                rule_checks={"L3_joint_configured": False},
            )
        checks["L3_joint_configured"] = True

        hard_lo, hard_hi = joint.hard_limit
        if not (hard_lo <= target_pulse <= hard_hi):
            return ValidationResult(
                allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                reason=f"Joint {servo_id} pulse {target_pulse} exceeds "
                       f"hard limit [{hard_lo}, {hard_hi}]",
                rule_checks={"L3_hard_limit": False},
            )
        checks["L3_hard_limit"] = True

        soft_lo, soft_hi = joint.soft_limit
        if not (soft_lo <= target_pulse <= soft_hi):
            warnings.append(
                f"Joint {servo_id} pulse {target_pulse} exceeds "
                f"soft limit [{soft_lo}, {soft_hi}]"
            )
        checks["L3_soft_limit"] = soft_lo <= target_pulse <= soft_hi

        # L4: Step size limit (time-scaled: longer moves allow larger steps)
        max_step = self.config["pulse_safety_rules"]["max_single_step_pulse_change"]
        if previous_pulse is not None:
            step = abs(target_pulse - previous_pulse)
            # Scale allowed step with move time: 3000ms → 3× larger step allowed
            time_scale = max(1.0, time_ms / 1000.0)
            effective_max_step = int(max_step * time_scale)
            if step > effective_max_step:
                return ValidationResult(
                    allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                    reason=f"Joint {servo_id} step {step} exceeds effective max {effective_max_step} "
                           f"(base {max_step} × time_scale {time_scale:.1f}). "
                           f"Previous: {previous_pulse}, target: {target_pulse}",
                    rule_checks={"L4_step_size": False},
                )
            checks["L4_step_size"] = True
        else:
            checks["L4_step_size"] = True  # no previous state to check

        # L4b: Phase-specific max pulse deviation from the measured home pose.
        max_dev_by_servo = phase.get("max_pulse_deviation_from_home_by_servo", {})
        max_dev = max_dev_by_servo.get(str(servo_id), phase.get("max_pulse_deviation_from_center"))
        if max_dev is not None:
            dev = abs(target_pulse - joint.home_pulse)
            if dev > int(max_dev):
                return ValidationResult(
                    allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                    reason=f"Phase '{self._current_phase}': pulse deviation {dev} "
                           f"exceeds max {max_dev} from home {joint.home_pulse}",
                    rule_checks={"L4_phase_max_deviation": False},
                )
            checks["L4_phase_max_deviation"] = True

        # L4c: Phase-specific max single joint count (Arm-B2)
        max_joints = phase.get("max_single_joint_count")
        if max_joints is not None:
            # This check is done in validate_multi, flag here for awareness
            checks["L4_phase_max_joints"] = True

        # L6: Robot-level safety
        if self.config["robot_level_safety"]["require_base_zero_before_arm"]:
            if not self._base_zero_ok:
                return ValidationResult(
                    allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                    reason="Robot base is not zero. Cannot move arm.",
                    rule_checks={"L6_base_zero": False},
                )
            checks["L6_base_zero"] = True

        if self.config["robot_level_safety"]["arm_cannot_move_while_driving"]:
            if self._robot_driving:
                return ValidationResult(
                    allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                    reason="Robot is driving. Cannot move arm while driving.",
                    rule_checks={"L6_not_driving": False},
                )
            checks["L6_not_driving"] = True

        # L6b: Coupled mechanical interlocks.
        trial_pulses = (
            dict(trial_pulses_override)
            if trial_pulses_override is not None
            else {jid: j.current_pulse for jid, j in self._joints.items()}
        )
        trial_pulses[servo_id] = target_pulse
        coupled_error = self._check_coupled_safety(
            servo_id=servo_id,
            target_pulse=target_pulse,
            previous_pulse=previous_pulse,
            trial_pulses=trial_pulses,
        )
        if coupled_error:
            return ValidationResult(
                allowed=False,
                servo_id=servo_id,
                target_pulse=target_pulse,
                reason=coupled_error,
                rule_checks={"L6b_coupled_safety": False},
            )
        checks["L6b_coupled_safety"] = True

        # L7: Emergency stop
        if self._estop_active:
            return ValidationResult(
                allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                reason="Emergency stop is active",
                rule_checks={"L7_estop": False},
            )
        checks["L7_estop"] = True

        # L7b: Heartbeat
        heartbeat_timeout = self.config["robot_level_safety"]["heartbeat_timeout_s"]
        if self._last_heartbeat > 0:
            since = time.monotonic() - self._last_heartbeat
            if since > heartbeat_timeout:
                return ValidationResult(
                    allowed=False, servo_id=servo_id, target_pulse=target_pulse,
                    reason=f"Heartbeat lost ({since:.1f}s > {heartbeat_timeout}s)",
                    rule_checks={"L7_heartbeat": False},
                )
        checks["L7_heartbeat"] = True

        # L5: Workspace check (optional warning for now, not blocking in early phases)
        # Build hypothetical pulse state
        angle_ranges = {jid: j.angle_range_deg for jid, j in self._joints.items()}
        safe_zone = self.config["workspace"]["safe_zone"]
        wk_valid = self._kinematics.is_in_workspace(trial_pulses, angle_ranges, safe_zone)
        if not wk_valid:
            warnings.append("End-effector may be outside safe workspace zone")
        checks["L5_workspace"] = wk_valid

        return ValidationResult(
            allowed=True,
            servo_id=servo_id,
            target_pulse=target_pulse,
            warnings=warnings,
            rule_checks=checks,
        )

    def _check_coupled_safety(
        self,
        servo_id: int,
        target_pulse: int,
        previous_pulse: Optional[int],
        trial_pulses: Dict[int, int],
    ) -> str:
        """Validate measured inter-joint mechanical constraints."""
        rules = self.config.get("coupled_safety_rules", {})

        id1_rule = rules.get("id1_rotation_requires", {})
        if id1_rule.get("enabled", False):
            id1_servo = int(id1_rule.get("id1_servo", 1))
            support_servo = int(id1_rule.get("support_servo", 2))
            support_min = int(id1_rule.get("support_min_pulse", 600))
            if servo_id == id1_servo and previous_pulse is not None:
                if target_pulse != previous_pulse:
                    support_pulse = trial_pulses.get(support_servo)
                    if support_pulse is None or support_pulse < support_min:
                        return (
                            f"ID{id1_servo} rotation requires ID{support_servo} >= "
                            f"{support_min}; trial ID{support_servo}={support_pulse}"
                        )

        for conditional in rules.get("conditional_joint_ranges", []):
            when = conditional.get("when", {})
            when_servo = int(when.get("servo_id"))
            when_pulse = trial_pulses.get(when_servo)
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
                pulse = trial_pulses.get(limited_servo)
                if pulse is None:
                    continue
                lo, hi = int(limits[0]), int(limits[1])
                if not (lo <= pulse <= hi):
                    return (
                        f"Conditional safety active at ID{when_servo}={when_pulse}: "
                        f"ID{limited_servo} pulse {pulse} outside [{lo}, {hi}]"
                    )

        return ""

    # ── command recording (dry-run safe) ──────────────────────────────────

    def record_single(self, cmd: ServoMoveCommand) -> None:
        """Record a single servo command. Updates joint state tracking.
        Does NOT send serial data. Safe for dry-run."""
        joint = self._joints.get(cmd.servo_id)
        if joint is None:
            return
        joint.current_pulse = cmd.target_pulse

    def record_multi(self, cmd: MultiServoCommand) -> None:
        """Record a multi-servo command. Updates all joint states."""
        for servo_id, target_pulse in cmd.servos.items():
            joint = self._joints.get(servo_id)
            if joint is not None:
                joint.current_pulse = target_pulse
        self._command_history.append(cmd)
        if len(self._command_history) > 1000:
            self._command_history = self._command_history[-500:]

    # ── serial frame building (safe: honors effective gate) ─────────────────

    def build_move_frame(self, cmd: MultiServoCommand) -> Dict[str, Any]:
        """Build Lobot bus servo move frame. Always builds the frame bytes
        for review/dry-run purposes, but only marks it as executable when
        serial_write_allowed_effective is true.

        Returns dict with keys:
          frame_bytes: bytes or None
          frame_hex: hex string for review
          frame_built_for_review_only: bool
          serial_write_allowed_effective: bool
          serial_write_allowed_global: bool
          serial_write_allowed_phase: bool
          hardware_executed: bool
        """
        global_ok = self.serial_write_allowed_global
        phase_ok = self.serial_write_allowed_phase
        effective = self.serial_write_allowed

        servos = cmd.servos
        count = len(servos)
        buf = bytearray(FRAME_HEADER)
        buf.append(count * 3 + 5)
        buf.append(CMD_SERVO_MOVE)
        buf.append(count)
        time_ms = max(TIME_MIN_MS, min(TIME_MAX_MS, cmd.time_ms))
        buf.extend(time_ms.to_bytes(2, 'little'))
        for servo_id in sorted(servos.keys()):
            pulse = servos[servo_id]
            pulse = max(BUS_PULSE_MIN, min(BUS_PULSE_MAX, pulse))
            buf.append(servo_id)
            buf.extend(pulse.to_bytes(2, 'little'))
        frame_bytes = bytes(buf)
        return {
            "frame_bytes": frame_bytes if effective else None,
            "frame_hex": frame_bytes.hex(),
            "frame_built_for_review_only": not effective,
            "serial_write_allowed_effective": effective,
            "serial_write_allowed_global": global_ok,
            "serial_write_allowed_phase": phase_ok,
            "hardware_executed": False,
            "cmd": {
                "servos": cmd.servos,
                "time_ms": time_ms,
                "label": cmd.label,
            },
        }

    def build_stop_frame(self) -> Dict[str, Any]:
        """Build action group stop frame (review-only if serial blocked)."""
        effective = self.serial_write_allowed
        buf = bytearray(FRAME_HEADER)
        buf.append(2)
        buf.append(CMD_ACTION_GROUP_STOP)
        frame_bytes = bytes(buf)
        return {
            "frame_bytes": frame_bytes if effective else None,
            "frame_hex": frame_bytes.hex(),
            "frame_built_for_review_only": not effective,
            "serial_write_allowed_effective": effective,
            "hardware_executed": False,
        }

    def build_home_frame(self, time_ms: int = 2000) -> Dict[str, Any]:
        """Build a home-all frame using the configured safe_idle_home pose.
        Returns the same dict structure as build_move_frame."""
        poses = self.config.get("poses", {})
        home_pose = poses.get("safe_idle_home_like_6b", {})
        servos = home_pose.get("servos", {})
        if not servos:
            # Fallback: use joint home_pulse values
            servos = {str(jid): j.home_pulse for jid, j in self._joints.items()}
        cmd = MultiServoCommand(
            servos={int(k): int(v) for k, v in servos.items()},
            time_ms=time_ms,
            label="home_all",
        )
        return self.build_move_frame(cmd)

    # ── state queries ─────────────────────────────────────────────────

    def get_joint_state(self, servo_id: int) -> Optional[ArmJointState]:
        return self._joints.get(servo_id)

    def get_all_joint_states(self) -> Dict[int, ArmJointState]:
        return dict(self._joints)

    def get_end_effector_position(self) -> Tuple[float, float, float]:
        pulses = {jid: j.current_pulse for jid, j in self._joints.items()}
        angle_ranges = {jid: j.angle_range_deg for jid, j in self._joints.items()}
        return self._kinematics.end_effector_position(pulses, angle_ranges)

    def get_safety_summary(self) -> Dict[str, Any]:
        """Return a summary of current safety state with effective gate status."""
        return {
            "phase": self._current_phase,
            "arm_enabled": self.arm_enabled,
            "hardware_access_allowed": self.hardware_access_allowed,
            "serial_write_allowed": self.serial_write_allowed,
            "serial_write_allowed_global": self.serial_write_allowed_global,
            "serial_write_allowed_phase": self.serial_write_allowed_phase,
            "contact_allowed": self.contact_allowed,
            "obstacle_removal_allowed": self.obstacle_removal_allowed,
            "base_zero_ok": self._base_zero_ok,
            "robot_driving": self._robot_driving,
            "estop_active": self._estop_active,
            "hardware_executed": False,
            "error_count": self._error_count,
            "command_count": len(self._command_history),
            "joints": {
                jid: {
                    "name": j.name,
                    "current_pulse": j.current_pulse,
                    "home_pulse": j.home_pulse,
                }
                for jid, j in self._joints.items()
            },
            "end_effector_xyz": self.get_end_effector_position(),
        }


# ── convenience function ──────────────────────────────────────────────────

def load_safety(config_path: str = "configs/arm_safety_config.json",
                phase: str = "arm_b1_plan_only") -> ArmSafety:
    """Load arm safety with given config and phase. Returns ArmSafety instance."""
    safety = ArmSafety(config_path)
    safety.set_phase(phase)
    return safety
