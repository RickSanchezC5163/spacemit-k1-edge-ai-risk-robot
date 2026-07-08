#!/usr/bin/env python3
"""Lightweight protocol records for K1 policy actions and evidence reports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, is_dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional


PROTOCOL_VERSION = "p4x_d435_hold_capture_v1"
P4Z_LITE_SCHEMA_VERSION = "p4z_lite_v1"
ACTION_HOLD_CAPTURE = "HOLD_CAPTURE"
ACTION_ARM_REMOVE_OBSTACLE = "ARM_REMOVE_OBSTACLE"
STATUS_SUCCEEDED = "succeeded"
STATUS_FAILED_SAFE = "failed_safe"


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def to_jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return to_jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    return value


@dataclass
class EvidencePaths:
    capture_dir: Optional[str] = None
    rgb: Optional[str] = None
    depth_raw: Optional[str] = None
    depth_vis: Optional[str] = None
    camera_info: Optional[str] = None
    odom: Optional[str] = None
    capture_meta: Optional[str] = None
    risk_point: Optional[str] = None
    action_result: Optional[str] = None
    episode_report: Optional[str] = None
    source_episode_report: Optional[str] = None


@dataclass
class PolicyState:
    state_id: str
    timestamp: str
    base_zero_ok: bool
    base_zero: Dict[str, Any] = field(default_factory=dict)
    odom: Optional[Dict[str, Any]] = None
    front_min_range_m: Optional[float] = None
    front_p10_range_m: Optional[float] = None
    source: str = "p4x_hold_capture_validation"
    notes: List[str] = field(default_factory=list)


@dataclass
class PolicyAction:
    action_id: str
    action_type: str
    requested_at: str
    requires_base_zero: bool = True
    publishes_cmd_vel: bool = False
    reason: str = ""
    params: Dict[str, Any] = field(default_factory=dict)


@dataclass
class CaptureMeta:
    capture_id: str
    action_id: str
    timestamp: str
    topics: Dict[str, str]
    paths: Dict[str, str]
    rgb: Dict[str, Any]
    depth: Dict[str, Any]
    camera_info: Dict[str, Any]
    odom: Optional[Dict[str, Any]] = None
    sequence: Optional[int] = None
    rgb_header_stamp: Optional[Dict[str, int]] = None
    depth_header_stamp: Optional[Dict[str, int]] = None
    rgb_frame_id: Optional[str] = None
    depth_frame_id: Optional[str] = None
    depth_encoding: Optional[str] = None
    depth_scale_m: Optional[float] = None
    valid_depth_ratio: Optional[float] = None


@dataclass
class RiskPoint:
    risk_point_id: str
    capture_id: str
    label: str
    bbox_xywh: Dict[str, int]
    depth_median_m: float
    camera_point_xyz_m: Dict[str, float]
    confidence: float
    evidence_paths: Dict[str, str]
    depth_scale_m: Optional[float] = None
    bbox_valid_depth_samples: Optional[int] = None
    bbox_valid_depth_ratio: Optional[float] = None
    generated_by: str = "mock_risk_detector"
    timestamp: str = field(default_factory=now_iso)
    notes: List[str] = field(default_factory=list)


@dataclass
class ActionResult:
    action_id: str
    action_type: str
    status: str
    started_at: str
    ended_at: str
    base_zero_ok_before: bool
    published_cmd_vel: bool = False
    capture_id: Optional[str] = None
    capture_meta_path: Optional[str] = None
    risk_point_path: Optional[str] = None
    evidence_paths: Dict[str, str] = field(default_factory=dict)
    base_zero: Dict[str, Any] = field(default_factory=dict)
    error: Optional[str] = None
    details: Dict[str, Any] = field(default_factory=dict)
    obstacle_removed: Optional[bool] = None
    mock: bool = False


@dataclass
class EpisodeReport:
    episode_id: str
    started_at: str
    ended_at: str
    protocol_version: str = PROTOCOL_VERSION
    policy_state: Optional[PolicyState] = None
    actions: List[PolicyAction] = field(default_factory=list)
    action_results: List[ActionResult] = field(default_factory=list)
    captures: List[CaptureMeta] = field(default_factory=list)
    risk_points: List[RiskPoint] = field(default_factory=list)
    summary: Dict[str, Any] = field(default_factory=dict)
    errors: List[Dict[str, Any]] = field(default_factory=list)
    output_root: Optional[str] = None
