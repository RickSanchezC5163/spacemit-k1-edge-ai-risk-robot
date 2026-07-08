"""Semantic guarded navigation mock environment.

This is a dependency-free mock environment for RL-A1 interface bring-up. It
emits and consumes high-level primitives only; it never publishes cmd_vel.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any, Dict, List, Tuple


ACTIONS = [
    "HOLD",
    "FORWARD_0P15",
    "ARC_FAST_LEFT",
    "ARC_FAST_RIGHT",
    "HOLD_CAPTURE",
    "ARM_NO_LOAD_RESPONSE",
    "STOP_SAFE",
]


@dataclass
class SemanticState:
    front_min: float = 0.8
    front_p10: float = 0.76
    left_p10: float = 1.2
    right_p10: float = 0.9
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_yaw: float = 0.0
    map_progress: float = 0.0
    risk_detected: bool = False
    risk_confidence: float = 0.0
    risk_class_id: int = -1
    risk_distance_m: float = 0.0
    base_zero: bool = True
    arm_ready: bool = True
    capture_recent: bool = False
    steps_since_capture: int = 0
    consecutive_fast_arc: int = 0
    total_forward_m: float = 0.0


class SemanticGuardedNavEnv:
    def __init__(self, max_steps: int = 40, max_consecutive_fast_arc: int = 2, max_total_forward_m: float = 1.0) -> None:
        self.max_steps = max_steps
        self.max_consecutive_fast_arc = max_consecutive_fast_arc
        self.max_total_forward_m = max_total_forward_m
        self.step_count = 0
        self.state = SemanticState()

    def reset(self) -> Dict[str, Any]:
        self.step_count = 0
        self.state = SemanticState()
        return asdict(self.state)

    def step(self, action: str) -> Tuple[Dict[str, Any], float, bool, Dict[str, Any]]:
        if action not in ACTIONS:
            raise ValueError(f"unknown semantic action {action}")
        self.step_count += 1
        reward = 0.0
        info: Dict[str, Any] = {"action": action, "cmd_vel_published": False, "servo_pulse_output": False}

        if action == "FORWARD_0P15":
            self.state.consecutive_fast_arc = 0
            if self.state.front_p10 <= 0.30:
                reward -= 3.0
                info["stop_reason"] = "front_p10_too_low"
            elif self.state.total_forward_m + 0.15 > self.max_total_forward_m:
                reward -= 1.2
                info["stop_reason"] = "max_total_forward_reached"
            else:
                self.state.odom_x += 0.15
                self.state.total_forward_m += 0.15
                self.state.map_progress = min(1.0, self.state.map_progress + 0.05)
                self.state.front_p10 = max(0.2, self.state.front_p10 - 0.02)
                reward += 0.2
        elif action in {"ARC_FAST_LEFT", "ARC_FAST_RIGHT"}:
            if self.state.consecutive_fast_arc >= self.max_consecutive_fast_arc:
                reward -= 1.2
                info["stop_reason"] = "max_consecutive_fast_arc_reached"
                self.state.base_zero = True
            else:
                self.state.consecutive_fast_arc += 1
                self.state.odom_yaw += 0.18 if action == "ARC_FAST_LEFT" else -0.18
                self.state.front_p10 = min(1.4, self.state.front_p10 + 0.08)
                reward += 0.15
        elif action == "HOLD_CAPTURE":
            self.state.consecutive_fast_arc = 0
            if self.state.base_zero:
                self.state.capture_recent = True
                self.state.steps_since_capture = 0
                self.state.risk_detected = True
                self.state.risk_confidence = 0.75
                self.state.risk_class_id = 1
                self.state.risk_distance_m = 0.8
                reward += 1.0
            else:
                reward -= 2.0
        elif action == "ARM_NO_LOAD_RESPONSE":
            self.state.consecutive_fast_arc = 0
            if self.state.base_zero and self.state.risk_detected:
                reward += 0.8
            else:
                reward -= 1.5
        elif action == "STOP_SAFE":
            self.state.consecutive_fast_arc = 0
            self.state.base_zero = True
            reward += 0.2
        elif action == "HOLD":
            self.state.consecutive_fast_arc = 0
            reward -= 0.01

        self.state.steps_since_capture += 1
        done = self.step_count >= self.max_steps or self.state.map_progress >= 1.0
        return asdict(self.state), reward, done, info


def default_action_space() -> List[str]:
    return list(ACTIONS)
