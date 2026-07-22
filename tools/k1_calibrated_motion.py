#!/usr/bin/env python3
"""Reusable K1 chassis motion semantics with direct odometry targets."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict, Iterable


LINEAR_SPEED_MPS = 0.18
ANGULAR_SPEED_RADPS = 0.35


@dataclass(frozen=True)
class MotionSemantic:
    code: str
    label: str
    kind: str
    direction: int
    requested_value: float
    odom_cutoff: float
    speed: float
    timeout_s: float
    unit: str

    def public_dict(self) -> dict:
        return asdict(self)


def _drive(code: str, label: str, direction: int, distance_m: float, cutoff_m: float) -> MotionSemantic:
    return MotionSemantic(
        code=code,
        label=label,
        kind="drive",
        direction=direction,
        requested_value=distance_m,
        odom_cutoff=cutoff_m,
        speed=LINEAR_SPEED_MPS,
        timeout_s=max(8.0, distance_m / LINEAR_SPEED_MPS * 3.0),
        unit="m",
    )


def _turn(code: str, label: str, direction: int, angle_deg: float, cutoff_deg: float) -> MotionSemantic:
    return MotionSemantic(
        code=code,
        label=label,
        kind="turn",
        direction=direction,
        requested_value=angle_deg,
        odom_cutoff=cutoff_deg,
        speed=ANGULAR_SPEED_RADPS,
        timeout_s=max(10.0, angle_deg / 20.0 * 2.5),
        unit="deg",
    )


_SEMANTICS: Dict[str, MotionSemantic] = {}


def _register(items: Iterable[MotionSemantic]) -> None:
    for item in items:
        if item.code in _SEMANTICS:
            raise ValueError(f"duplicate motion semantic: {item.code}")
        _SEMANTICS[item.code] = item


_register(
    [
        _drive("FORWARD_5", "前进 5 cm", 1, 0.05, 0.05),
        _drive("FORWARD_10", "前进 10 cm", 1, 0.10, 0.10),
        _drive("FORWARD_20", "前进 20 cm", 1, 0.20, 0.20),
        _drive("FORWARD_25", "前进 25 cm", 1, 0.25, 0.25),
        _drive("FORWARD_30", "前进 30 cm", 1, 0.30, 0.30),
        _drive("FORWARD_40", "前进 40 cm", 1, 0.40, 0.40),
        _drive("FORWARD_50", "前进 50 cm", 1, 0.50, 0.50),
        _drive("FORWARD_60", "前进 60 cm", 1, 0.60, 0.60),
        _drive("FORWARD_75", "前进 75 cm", 1, 0.75, 0.75),
        _drive("FORWARD_80", "前进 80 cm", 1, 0.80, 0.80),
        _drive("FORWARD_100", "前进 100 cm", 1, 1.00, 1.00),
        _drive("REVERSE_10", "后退 10 cm", -1, 0.10, 0.10),
        _drive("REVERSE_20", "后退 20 cm", -1, 0.20, 0.20),
        _drive("REVERSE_25", "后退 25 cm", -1, 0.25, 0.25),
        _drive("REVERSE_30", "后退 30 cm", -1, 0.30, 0.30),
        _drive("REVERSE_40", "后退 40 cm", -1, 0.40, 0.40),
        _drive("REVERSE_50", "后退 50 cm", -1, 0.50, 0.50),
        _drive("REVERSE_60", "后退 60 cm", -1, 0.60, 0.60),
        _drive("REVERSE_75", "后退 75 cm", -1, 0.75, 0.75),
        _drive("REVERSE_80", "后退 80 cm", -1, 0.80, 0.80),
        _drive("REVERSE_100", "后退 100 cm", -1, 1.00, 1.00),
        _turn("LEFT_15", "左转 15°", 1, 15.0, 15.0),
        _turn("LEFT_30", "左转 30°", 1, 30.0, 30.0),
        _turn("LEFT_45", "左转 45°", 1, 45.0, 45.0),
        _turn("LEFT_60", "左转 60°", 1, 60.0, 60.0),
        _turn("LEFT_90", "左转 90°", 1, 90.0, 90.0),
        _turn("LEFT_120", "左转 120°", 1, 120.0, 120.0),
        _turn("LEFT_180", "左转 180°", 1, 180.0, 180.0),
        _turn("RIGHT_15", "右转 15°", -1, 15.0, 15.0),
        _turn("RIGHT_30", "右转 30°", -1, 30.0, 30.0),
        _turn("RIGHT_45", "右转 45°", -1, 45.0, 45.0),
        _turn("RIGHT_60", "右转 60°", -1, 60.0, 60.0),
        _turn("RIGHT_90", "右转 90°", -1, 90.0, 90.0),
        _turn("RIGHT_120", "右转 120°", -1, 120.0, 120.0),
        _turn("RIGHT_180", "右转 180°", -1, 180.0, 180.0),
    ]
)


def get_semantic(code: str) -> MotionSemantic:
    try:
        return _SEMANTICS[code.upper()]
    except KeyError as exc:
        raise ValueError(f"unknown motion semantic: {code}") from exc


def list_semantics() -> list[MotionSemantic]:
    return list(_SEMANTICS.values())


def forward(distance_cm: int) -> MotionSemantic:
    return get_semantic(f"FORWARD_{int(distance_cm)}")


def reverse(distance_cm: int) -> MotionSemantic:
    return get_semantic(f"REVERSE_{int(distance_cm)}")


def turn_left(angle_deg: int) -> MotionSemantic:
    return get_semantic(f"LEFT_{int(angle_deg)}")


def turn_right(angle_deg: int) -> MotionSemantic:
    return get_semantic(f"RIGHT_{int(angle_deg)}")
