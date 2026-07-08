#!/usr/bin/env python3
"""Step7 offline integrated flow runner.

This runner does not start ROS, publish cmd_vel, open serial ports, or execute
mechanical-arm hardware. It consumes existing P4-X, Map-A0, and Arm-C0 evidence
and produces a single episode_report.json for deterministic LLM-A reporting.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


PROTOCOL_VERSION = "step7_integrated_offline_flow_v1"
ACTION_PLAN_MAP = "STEP7_PLANNING_MAPPING_RULE_DRY_RUN"
ACTION_D435_TRIGGER = "STEP7_D435_RULE_TRIGGER_HOLD_CAPTURE_SIM"
ACTION_ARM_TRIGGER = "STEP7_ARM_RULE_TRIGGER_NO_LOAD_SIM"
STATUS_SUCCEEDED_DRY_RUN = "succeeded_dry_run"
STATUS_BLOCKED = "blocked"
SELECTED_ARM_ACTION = "ARM_SAMPLE_NO_LOAD"
SELECTED_ARM_SEQUENCE = "arm_b3_8_step_safety_adjusted_no_load_sample"
DEFAULT_DEPTH_TRIGGER_M = 1.5
DEFAULT_P4Y2_POLICY_RUN_REPORT = "logs/policy_p4w_run_branch_mixed_20260629_183731.json"
DEFAULT_P4Y2_FINAL_MARKED_MAP = "maps/policy_p4w_branch_mixed_20260629_183731_final_marked.png"
DEFAULT_P4Y2_DOC = "docs/p4_guarded_policy_executable_modes_20260629.md"
DEFAULT_P4Y2_BUNDLE = "edge-ai-robot-k1-p4-y2-7step-guarded-stress-58399be.bundle"


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def slug_time() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def load_json_if_present(path: Optional[Path]) -> Optional[Any]:
    if path is None or not path.exists():
        return None
    return load_json(path)


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def as_number(value: Any) -> Optional[float]:
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        value = float(value)
        return value if math.isfinite(value) else None
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return None
    return converted if math.isfinite(converted) else None


def flatten(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)


def bool_all(values: Iterable[Any]) -> bool:
    values = list(values)
    return bool(values) and all(value is True for value in values)


def p4x_base_zero_ok_all(report: Dict[str, Any]) -> bool:
    return bool_all(result.get("base_zero_ok_before") for result in report.get("action_results") or [])


def p4x_published_cmd_vel_any(report: Dict[str, Any]) -> bool:
    actions = report.get("actions") or []
    results = report.get("action_results") or []
    summary = report.get("summary") or {}
    return bool(
        any(action.get("publishes_cmd_vel") is True for action in actions)
        or any(result.get("published_cmd_vel") is True for result in results)
        or summary.get("published_cmd_vel") is True
    )


def build_p4y2_policy_evidence(
    policy_run_path: Optional[Path],
    final_marked_map_path: Optional[Path],
    policy_doc_path: Optional[Path],
    bundle_path: Optional[Path],
) -> Dict[str, Any]:
    policy_run = load_json_if_present(policy_run_path)
    result = policy_run.get("result") if isinstance(policy_run, dict) else {}
    args = policy_run.get("args") if isinstance(policy_run, dict) else {}
    records = result.get("records") if isinstance(result, dict) else []
    step_records: List[Dict[str, Any]] = []
    for record in records or []:
        odom_delta = record.get("odom_delta") or {}
        step_records.append(
            {
                "step_index": record.get("step_index"),
                "front_min": record.get("front_min"),
                "front_p10": record.get("front_p10"),
                "selected_action": record.get("selected_action"),
                "execution_action": record.get("execution_action"),
                "arc_mode": record.get("arc_mode"),
                "arc_direction": record.get("arc_direction"),
                "consecutive_fast_arc": record.get("consecutive_fast_arc"),
                "executed": record.get("executed"),
                "base_zero_ok": record.get("base_zero_ok"),
                "delta_yaw_deg": odom_delta.get("delta_yaw_deg"),
                "forward_delta_m": odom_delta.get("forward_delta_m"),
                "step_positive_forward_m": record.get("step_positive_forward_m"),
                "cumulative_positive_forward_m": record.get("cumulative_positive_forward_m"),
                "map_saved": record.get("map_saved"),
                "map_save_status": record.get("map_save_status"),
                "stop_reason": record.get("stop_reason"),
                "sequence_limit_stop_reason": record.get("sequence_limit_stop_reason"),
            }
        )

    final_map_save = result.get("final_map_save") or {}
    saved_maps = result.get("saved_maps") or []
    critical_map_saves = [
        item
        for item in saved_maps
        if isinstance(item, dict) and item.get("reason") == "critical" and item.get("ok") is True
    ]
    policy_loaded = policy_run is not None
    return {
        "evidence_kind": "p4_y2_guarded_policy_stress_stop",
        "policy_run_report": str(policy_run_path) if policy_run_path else None,
        "policy_run_report_exists": bool(policy_run_path and policy_run_path.exists()),
        "final_marked_map": str(final_marked_map_path) if final_marked_map_path else None,
        "final_marked_map_exists": bool(final_marked_map_path and final_marked_map_path.exists()),
        "policy_doc": str(policy_doc_path) if policy_doc_path else None,
        "policy_doc_exists": bool(policy_doc_path and policy_doc_path.exists()),
        "bundle": str(bundle_path) if bundle_path else None,
        "bundle_exists": bool(bundle_path and bundle_path.exists()),
        "policy_loaded": policy_loaded,
        "started_at": policy_run.get("started_at") if isinstance(policy_run, dict) else None,
        "commit": "58399be",
        "mode": result.get("mode"),
        "profile": result.get("profile"),
        "policy_arc_mode": result.get("policy_arc_mode"),
        "policy_max_steps": result.get("max_steps") or args.get("policy_max_steps"),
        "policy_max_consecutive_fast_arc": args.get("policy_max_consecutive_fast_arc"),
        "policy_max_total_forward_m": result.get("policy_max_total_forward_m")
        or args.get("policy_max_total_forward_m"),
        "step_count": result.get("step_count"),
        "executed_count": result.get("executed_count"),
        "sequence_stop_reason": result.get("sequence_stop_reason"),
        "stopped_by_guard": result.get("sequence_stop_reason") == "max_consecutive_fast_arc_reached",
        "hard_stop_triggered": False,
        "base_zero_ok": result.get("base_zero_ok"),
        "final_map_saved": result.get("final_map_saved"),
        "critical_map_saved": bool(critical_map_saves),
        "final_map_save_ok": final_map_save.get("ok"),
        "final_map_save_reason": final_map_save.get("reason"),
        "cumulative_positive_forward_m": result.get("cumulative_positive_forward_m"),
        "odom_delta": result.get("odom_delta"),
        "postcheck_front_state": ((result.get("postcheck") or {}).get("front_state")),
        "postcheck_front_p10_range_m": ((result.get("postcheck") or {}).get("front_p10_range_m")),
        "step_records": step_records,
        "claim_note": (
            "P4-Y2 is used as guarded policy / critical map-save evidence. "
            "It does not claim full autonomous navigation, path planning, or high-precision SLAM."
        ),
    }


def merge_visual_evidence_paths(point: Dict[str, Any]) -> Tuple[Dict[str, Any], List[str]]:
    source = point.get("source_evidence_paths") or {}
    capture = source.get("capture") or {}
    action_result = source.get("action_result") or {}
    risk_source = source.get("risk_point") or {}
    merged = {
        "source_episode_report": source.get("source_episode_report"),
        "rgb": capture.get("rgb") or action_result.get("rgb") or risk_source.get("rgb"),
        "depth_raw": capture.get("depth_raw") or action_result.get("depth_raw") or risk_source.get("depth_raw"),
        "depth_vis": capture.get("depth_vis") or action_result.get("depth_vis") or risk_source.get("depth_vis"),
        "camera_info": capture.get("camera_info") or action_result.get("camera_info") or risk_source.get("camera_info"),
        "odom": capture.get("odom") or action_result.get("odom"),
        "capture_meta": (
            capture.get("capture_meta")
            or action_result.get("capture_meta")
            or source.get("capture_meta_path")
            or risk_source.get("capture_meta")
        ),
        "risk_point": action_result.get("risk_point") or source.get("risk_point_path"),
        "risk_map_points": None,
    }
    required = ("rgb", "depth_raw", "depth_vis", "camera_info", "odom", "capture_meta", "risk_point")
    missing = [key for key in required if not merged.get(key)]
    return merged, missing


def candidates_by_map_point(package: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    indexed: Dict[str, Dict[str, Any]] = {}
    for candidate in package.get("candidates") or []:
        map_point_id = candidate.get("source_map_point_id")
        if map_point_id:
            indexed[str(map_point_id)] = candidate
    return indexed


def build_d435_trigger(
    point: Dict[str, Any],
    risk_map_points_path: Path,
    p4x_report: Dict[str, Any],
    depth_trigger_m: float,
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    errors: List[Dict[str, Any]] = []
    evidence_paths, missing_paths = merge_visual_evidence_paths(point)
    evidence_paths["risk_map_points"] = str(risk_map_points_path)
    depth_median_m = as_number(point.get("depth_median_m"))
    block_reasons: List[str] = []

    if point.get("projection_status") != "projected":
        block_reasons.append(f"projection_status={point.get('projection_status')}")
    if missing_paths:
        block_reasons.append("missing visual evidence paths: " + ", ".join(missing_paths))
    if depth_median_m is None:
        block_reasons.append("missing depth_median_m")
    elif depth_median_m > depth_trigger_m:
        block_reasons.append(f"depth_median_m={depth_median_m:.3f} > trigger_m={depth_trigger_m:.3f}")
    if not p4x_base_zero_ok_all(p4x_report):
        block_reasons.append("source P4-X does not prove all base_zero_ok_before=true")
    if p4x_published_cmd_vel_any(p4x_report):
        block_reasons.append("source P4-X published cmd_vel")

    status = STATUS_BLOCKED if block_reasons else STATUS_SUCCEEDED_DRY_RUN
    if block_reasons:
        errors.append(
            {
                "timestamp": now_iso(),
                "level": "warning",
                "code": "d435_rule_trigger_blocked",
                "map_point_id": point.get("map_point_id"),
                "risk_point_id": point.get("risk_point_id"),
                "block_reasons": block_reasons,
            }
        )

    trigger = {
        "trigger_id": f"d435_trigger_{point.get('map_point_id') or 'unknown'}",
        "status": status,
        "block_reasons": block_reasons,
        "source_map_point_id": point.get("map_point_id"),
        "source_risk_point_id": point.get("risk_point_id"),
        "source_capture_id": point.get("capture_id"),
        "source_hold_capture_action_id": point.get("action_id"),
        "rule": {
            "name": "projected_risk_depth_with_existing_stationary_capture",
            "depth_trigger_m": depth_trigger_m,
            "requires_projection_status": "projected",
            "requires_visual_evidence_paths": True,
            "requires_source_base_zero_all": True,
            "requires_source_cmd_vel_false": True,
        },
        "triggered": status == STATUS_SUCCEEDED_DRY_RUN,
        "simulated_capture": True,
        "new_capture_created": False,
        "requires_base_zero": True,
        "base_zero_source": "upstream_p4x_episode_report",
        "base_zero_ok_before": p4x_base_zero_ok_all(p4x_report),
        "published_cmd_vel": False,
        "risk_label": point.get("risk_label"),
        "risk_category": point.get("risk_category"),
        "depth_median_m": depth_median_m,
        "depth_scale_m": point.get("depth_scale_m"),
        "camera_point_xyz_m": point.get("camera_point_xyz_m"),
        "base_point_xyz_m": point.get("base_point_xyz_m"),
        "odom_point_xy_m": point.get("odom_point_xy_m"),
        "projection_mode": point.get("projection_mode"),
        "projection_precision": "tf_validated" if point.get("tf_validated") is True else "approximate",
        "tf_validated": point.get("tf_validated") is True,
        "slam_used": point.get("slam_used") is True,
        "navigation_used": point.get("navigation_used") is True,
        "evidence_paths": evidence_paths,
        "missing_evidence_paths": missing_paths,
    }
    return trigger, errors


def build_arm_trigger(
    point: Dict[str, Any],
    d435_trigger: Dict[str, Any],
    candidate: Optional[Dict[str, Any]],
    arm_c0_candidates_path: Path,
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    errors: List[Dict[str, Any]] = []
    block_reasons: List[str] = []
    if d435_trigger.get("triggered") is not True:
        block_reasons.append("D435 trigger did not pass")
    if candidate is None:
        block_reasons.append("missing Arm-C0 candidate for map point")
    elif candidate.get("status") != STATUS_SUCCEEDED_DRY_RUN:
        block_reasons.append(f"candidate.status={candidate.get('status')}")
    elif candidate.get("selected_action") != SELECTED_ARM_ACTION:
        block_reasons.append(f"candidate.selected_action={candidate.get('selected_action')}")
    elif candidate.get("selected_sequence") != SELECTED_ARM_SEQUENCE:
        block_reasons.append(f"candidate.selected_sequence={candidate.get('selected_sequence')}")
    elif candidate.get("validated_no_load_action") is not True:
        block_reasons.append("candidate.validated_no_load_action is not true")
    elif candidate.get("contact_allowed") is not False:
        block_reasons.append("candidate.contact_allowed is not false")
    elif candidate.get("obstacle_removed") is not False:
        block_reasons.append("candidate.obstacle_removed is not false")

    status = STATUS_BLOCKED if block_reasons else STATUS_SUCCEEDED_DRY_RUN
    if block_reasons:
        errors.append(
            {
                "timestamp": now_iso(),
                "level": "warning",
                "code": "arm_rule_trigger_blocked",
                "map_point_id": point.get("map_point_id"),
                "risk_point_id": point.get("risk_point_id"),
                "block_reasons": block_reasons,
            }
        )

    trigger = {
        "trigger_id": f"arm_trigger_{point.get('map_point_id') or 'unknown'}",
        "status": status,
        "block_reasons": block_reasons,
        "source_map_point_id": point.get("map_point_id"),
        "source_risk_point_id": point.get("risk_point_id"),
        "source_d435_trigger_id": d435_trigger.get("trigger_id"),
        "source_candidate_id": candidate.get("candidate_id") if candidate else None,
        "rule": {
            "name": "map_gated_no_load_candidate_only",
            "requires_d435_rule_trigger": True,
            "requires_arm_c0_succeeded_dry_run": True,
            "requires_validated_no_load_action": True,
            "allows_contact": False,
            "allows_obstacle_removal": False,
            "allows_hardware_execution": False,
        },
        "triggered": status == STATUS_SUCCEEDED_DRY_RUN,
        "selected_action": candidate.get("selected_action") if candidate else None,
        "selected_sequence": candidate.get("selected_sequence") if candidate else None,
        "validated_no_load_action": candidate.get("validated_no_load_action") if candidate else None,
        "zone": candidate.get("zone") if candidate else None,
        "map_projection_valid": candidate.get("map_projection_valid") if candidate else None,
        "projection_precision": candidate.get("projection_precision") if candidate else None,
        "tf_validated": candidate.get("tf_validated") if candidate else point.get("tf_validated") is True,
        "base_zero_required": True,
        "base_zero_checked": False,
        "base_zero_ok_before": None,
        "hardware_executed": False,
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "published_cmd_vel": False,
        "contact_allowed": False,
        "obstacle_removed": False,
        "source_evidence_paths": {
            "arm_c0_candidates": str(arm_c0_candidates_path),
            "risk_map_points": d435_trigger.get("evidence_paths", {}).get("risk_map_points"),
            "source_visual_evidence": d435_trigger.get("evidence_paths"),
        },
    }
    return trigger, errors


def build_action_result(
    action_id: str,
    action_type: str,
    status: str,
    started_at: str,
    evidence_paths: Dict[str, Any],
    details: Dict[str, Any],
    base_zero_ok_before: Optional[bool],
    error: Optional[str] = None,
) -> Dict[str, Any]:
    return {
        "action_id": action_id,
        "action_type": action_type,
        "status": status,
        "started_at": started_at,
        "ended_at": now_iso(),
        "base_zero_ok_before": base_zero_ok_before,
        "published_cmd_vel": False,
        "evidence_paths": evidence_paths,
        "details": details,
        "error": error,
    }


def build_episode(
    p4x_path: Path,
    risk_map_path: Path,
    arm_c0_candidates_path: Path,
    output_dir: Path,
    p4x_report: Dict[str, Any],
    risk_map_package: Dict[str, Any],
    arm_c0_package: Dict[str, Any],
    p4y2_policy_evidence: Dict[str, Any],
    depth_trigger_m: float,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    started_at = now_iso()
    episode_id = f"step7_integrated_offline_flow_{slug_time()}"
    risk_points = risk_map_package.get("risk_map_points") or []
    candidate_index = candidates_by_map_point(arm_c0_package)
    d435_triggers: List[Dict[str, Any]] = []
    arm_triggers: List[Dict[str, Any]] = []
    errors: List[Dict[str, Any]] = []

    for point in risk_points:
        d435_trigger, trigger_errors = build_d435_trigger(
            point=point,
            risk_map_points_path=risk_map_path,
            p4x_report=p4x_report,
            depth_trigger_m=depth_trigger_m,
        )
        d435_triggers.append(d435_trigger)
        errors.extend(trigger_errors)
        candidate = candidate_index.get(str(point.get("map_point_id")))
        arm_trigger, arm_errors = build_arm_trigger(
            point=point,
            d435_trigger=d435_trigger,
            candidate=candidate,
            arm_c0_candidates_path=arm_c0_candidates_path,
        )
        arm_triggers.append(arm_trigger)
        errors.extend(arm_errors)

    actions: List[Dict[str, Any]] = []
    results: List[Dict[str, Any]] = []
    plan_action_id = f"{episode_id}_plan_map_00"
    plan_started_at = now_iso()
    plan_details = {
        "planning_policy_source": "P4-Y2 guarded policy stress stop",
        "mapping_source": "P4-Y2 final map save plus Map-A0 offline risk point projection",
        "p4y2_policy_evidence": p4y2_policy_evidence,
        "source_risk_map_points": str(risk_map_path),
        "source_p4x_episode_report": str(p4x_path),
        "risk_map_points": len(risk_points),
        "projected_points": sum(1 for point in risk_points if point.get("projection_status") == "projected"),
        "projection_mode": risk_map_package.get("projection_mode"),
        "tf_validated": risk_map_package.get("tf_validated") is True,
        "slam_used": risk_map_package.get("slam_used") is True,
        "navigation_used": risk_map_package.get("navigation_used") is True,
        "autonomous_navigation_executed": False,
        "path_planning_executed": False,
        "guarded_policy_executed_upstream": p4y2_policy_evidence.get("policy_loaded") is True,
        "guarded_policy_stop_reason": p4y2_policy_evidence.get("sequence_stop_reason"),
        "guarded_policy_stopped_by_guard": p4y2_policy_evidence.get("stopped_by_guard"),
        "guarded_policy_base_zero_ok": p4y2_policy_evidence.get("base_zero_ok"),
        "guarded_policy_final_map_saved": p4y2_policy_evidence.get("final_map_saved"),
        "guarded_policy_critical_map_saved": p4y2_policy_evidence.get("critical_map_saved"),
        "cmd_vel_published_by_step7": False,
        "map_built_in_this_step": False,
        "rule_check_only": True,
    }
    actions.append(
        {
            "action_id": plan_action_id,
            "action_type": ACTION_PLAN_MAP,
            "requested_at": plan_started_at,
            "requires_base_zero": False,
            "publishes_cmd_vel": False,
            "reason": "Offline Step7 planning/mapping evidence rule check",
            "params": plan_details,
        }
    )
    results.append(
        build_action_result(
            action_id=plan_action_id,
            action_type=ACTION_PLAN_MAP,
            status=STATUS_SUCCEEDED_DRY_RUN,
            started_at=plan_started_at,
            evidence_paths={
                "p4y2_policy_run_report": p4y2_policy_evidence.get("policy_run_report"),
                "p4y2_final_marked_map": p4y2_policy_evidence.get("final_marked_map"),
                "p4y2_policy_doc": p4y2_policy_evidence.get("policy_doc"),
                "p4y2_bundle": p4y2_policy_evidence.get("bundle"),
                "p4x_episode_report": str(p4x_path),
                "risk_map_points": str(risk_map_path),
                "arm_c0_candidates": str(arm_c0_candidates_path),
            },
            details=plan_details,
            base_zero_ok_before=None,
        )
    )

    for index, d435_trigger in enumerate(d435_triggers, start=1):
        action_id = f"{episode_id}_d435_{index:02d}"
        requested_at = now_iso()
        actions.append(
            {
                "action_id": action_id,
                "action_type": ACTION_D435_TRIGGER,
                "requested_at": requested_at,
                "requires_base_zero": True,
                "publishes_cmd_vel": False,
                "reason": "D435 rule-triggered simulated HOLD_CAPTURE from existing P4-X evidence",
                "params": {
                    "trigger_id": d435_trigger.get("trigger_id"),
                    "source_map_point_id": d435_trigger.get("source_map_point_id"),
                    "source_risk_point_id": d435_trigger.get("source_risk_point_id"),
                    "new_capture_created": False,
                    "depth_trigger_m": depth_trigger_m,
                },
            }
        )
        results.append(
            build_action_result(
                action_id=action_id,
                action_type=ACTION_D435_TRIGGER,
                status=d435_trigger["status"],
                started_at=requested_at,
                evidence_paths=d435_trigger.get("evidence_paths") or {},
                details=d435_trigger,
                base_zero_ok_before=d435_trigger.get("base_zero_ok_before"),
                error="; ".join(d435_trigger.get("block_reasons") or []) or None,
            )
        )

    for index, arm_trigger in enumerate(arm_triggers, start=1):
        action_id = f"{episode_id}_arm_{index:02d}"
        requested_at = now_iso()
        actions.append(
            {
                "action_id": action_id,
                "action_type": ACTION_ARM_TRIGGER,
                "requested_at": requested_at,
                "requires_base_zero": True,
                "publishes_cmd_vel": False,
                "reason": "Mechanical-arm no-load rule trigger simulation from Arm-C0 candidates",
                "params": {
                    "trigger_id": arm_trigger.get("trigger_id"),
                    "source_candidate_id": arm_trigger.get("source_candidate_id"),
                    "selected_action": arm_trigger.get("selected_action"),
                    "selected_sequence": arm_trigger.get("selected_sequence"),
                    "hardware_execution_allowed": False,
                    "base_zero_checked": False,
                },
            }
        )
        results.append(
            build_action_result(
                action_id=action_id,
                action_type=ACTION_ARM_TRIGGER,
                status=arm_trigger["status"],
                started_at=requested_at,
                evidence_paths=arm_trigger.get("source_evidence_paths") or {},
                details=arm_trigger,
                base_zero_ok_before=None,
                error="; ".join(arm_trigger.get("block_reasons") or []) or None,
            )
        )

    d435_passed = sum(1 for trigger in d435_triggers if trigger.get("status") == STATUS_SUCCEEDED_DRY_RUN)
    arm_passed = sum(1 for trigger in arm_triggers if trigger.get("status") == STATUS_SUCCEEDED_DRY_RUN)
    d435_blocked = sum(1 for trigger in d435_triggers if trigger.get("status") == STATUS_BLOCKED)
    arm_blocked = sum(1 for trigger in arm_triggers if trigger.get("status") == STATUS_BLOCKED)

    episode = {
        "episode_id": episode_id,
        "episode_kind": "step7_integrated_offline_flow",
        "started_at": started_at,
        "ended_at": now_iso(),
        "protocol_version": PROTOCOL_VERSION,
        "policy_state": {
            "state_id": f"{episode_id}_state_offline",
            "timestamp": now_iso(),
            "base_zero_ok": None,
            "base_zero": {
                "checked_live": False,
                "source_p4x_base_zero_ok_all": p4x_base_zero_ok_all(p4x_report),
                "required_before_real_arm": True,
                "reason": "Step7 offline simulation consumes previous evidence; no live base-zero gate is checked here.",
            },
            "odom": None,
            "front_min_range_m": None,
            "front_p10_range_m": None,
            "source": "run_step7_integrated_offline_flow",
            "notes": [
                "offline integrated flow only",
                "no ROS process started",
                "no cmd_vel published",
                "no serial port opened",
                "no mechanical-arm hardware executed",
            ],
        },
        "actions": actions,
        "action_results": results,
        "step7_flow": {
            "planning_mapping": plan_details,
            "d435_rule_triggers": d435_triggers,
            "arm_rule_triggers": arm_triggers,
        },
        "summary": {
            "status": STATUS_SUCCEEDED_DRY_RUN if not errors else "review_dry_run",
            "risk_map_points": len(risk_points),
            "d435_triggers": len(d435_triggers),
            "d435_succeeded_dry_run": d435_passed,
            "d435_blocked": d435_blocked,
            "arm_triggers": len(arm_triggers),
            "arm_succeeded_dry_run": arm_passed,
            "arm_blocked": arm_blocked,
            "failed_safe": 0,
            "published_cmd_vel": False,
            "ros_started": False,
            "hardware_executed": False,
            "serial_port_opened": False,
            "serial_bytes_written": 0,
            "contact_allowed": False,
            "obstacle_removed": False,
            "base_zero_required": True,
            "base_zero_checked": False,
            "llm_used": False,
            "online_api_used": False,
            "local_model_used": False,
            "source_p4x_base_zero_ok_all": p4x_base_zero_ok_all(p4x_report),
            "source_p4x_published_cmd_vel_any": p4x_published_cmd_vel_any(p4x_report),
            "projection_mode": risk_map_package.get("projection_mode"),
            "tf_validated": risk_map_package.get("tf_validated") is True,
            "slam_used": risk_map_package.get("slam_used") is True,
            "navigation_used": risk_map_package.get("navigation_used") is True,
            "path_planning_executed": False,
            "autonomous_navigation_executed": False,
            "p4y2_policy_run_report": p4y2_policy_evidence.get("policy_run_report"),
            "p4y2_policy_loaded": p4y2_policy_evidence.get("policy_loaded"),
            "p4y2_step_count": p4y2_policy_evidence.get("step_count"),
            "p4y2_executed_count": p4y2_policy_evidence.get("executed_count"),
            "p4y2_sequence_stop_reason": p4y2_policy_evidence.get("sequence_stop_reason"),
            "p4y2_stopped_by_guard": p4y2_policy_evidence.get("stopped_by_guard"),
            "p4y2_base_zero_ok": p4y2_policy_evidence.get("base_zero_ok"),
            "p4y2_final_map_saved": p4y2_policy_evidence.get("final_map_saved"),
            "p4y2_critical_map_saved": p4y2_policy_evidence.get("critical_map_saved"),
            "p4y2_cumulative_positive_forward_m": p4y2_policy_evidence.get("cumulative_positive_forward_m"),
        },
        "errors": errors,
        "output_root": str(output_dir),
    }
    flow_summary = {
        "schema_version": PROTOCOL_VERSION,
        "generated_at": now_iso(),
        "episode_id": episode_id,
        "source_inputs": {
            "p4x_episode_report": str(p4x_path),
            "p4y2_policy_run_report": p4y2_policy_evidence.get("policy_run_report"),
            "p4y2_final_marked_map": p4y2_policy_evidence.get("final_marked_map"),
            "p4y2_policy_doc": p4y2_policy_evidence.get("policy_doc"),
            "p4y2_bundle": p4y2_policy_evidence.get("bundle"),
            "risk_map_points": str(risk_map_path),
            "arm_c0_candidates": str(arm_c0_candidates_path),
        },
        "summary": episode["summary"],
        "claim_boundary": claim_boundary(),
    }
    return episode, flow_summary


def claim_boundary() -> Dict[str, Any]:
    return {
        "allowed_claims": [
            "Step7-A offline integrated rule flow completed.",
            "Existing P4-Y2 guarded policy stress-stop evidence is consumed as planning/mapping safety evidence.",
            "Existing P4-X stationary D435 evidence was consumed as the visual trigger source.",
            "Existing Map-A0 approximate risk map points were consumed as mapping evidence.",
            "Arm-C0 no-load candidates were consumed as simulated arm trigger outputs.",
            "A deterministic LLM-A report can be generated from the Step7 episode_report.json.",
        ],
        "disallowed_claims": [
            "Do not claim new live D435 capture in this Step7 offline run.",
            "Do not claim ROS was started.",
            "Do not claim cmd_vel was published.",
            "Do not claim real mechanical-arm motion from this Step7 offline run.",
            "Do not claim grasping, contact, payload handling, or obstacle removal.",
            "Do not claim full autonomous navigation, path planning success, or high-precision SLAM.",
            "Do not claim LLM control of the robot.",
        ],
    }


def write_trace_csv(path: Path, d435_triggers: Sequence[Dict[str, Any]], arm_triggers: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "index",
        "map_point_id",
        "risk_point_id",
        "d435_status",
        "d435_triggered",
        "arm_status",
        "arm_triggered",
        "depth_median_m",
        "selected_action",
        "selected_sequence",
        "zone",
        "hardware_executed",
        "serial_port_opened",
        "serial_bytes_written",
        "published_cmd_vel",
        "contact_allowed",
        "obstacle_removed",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for index, (d435, arm) in enumerate(zip(d435_triggers, arm_triggers), start=1):
            writer.writerow(
                {
                    "index": index,
                    "map_point_id": d435.get("source_map_point_id"),
                    "risk_point_id": d435.get("source_risk_point_id"),
                    "d435_status": d435.get("status"),
                    "d435_triggered": d435.get("triggered"),
                    "arm_status": arm.get("status"),
                    "arm_triggered": arm.get("triggered"),
                    "depth_median_m": d435.get("depth_median_m"),
                    "selected_action": arm.get("selected_action"),
                    "selected_sequence": arm.get("selected_sequence"),
                    "zone": flatten(arm.get("zone")),
                    "hardware_executed": arm.get("hardware_executed"),
                    "serial_port_opened": arm.get("serial_port_opened"),
                    "serial_bytes_written": arm.get("serial_bytes_written"),
                    "published_cmd_vel": arm.get("published_cmd_vel"),
                    "contact_allowed": arm.get("contact_allowed"),
                    "obstacle_removed": arm.get("obstacle_removed"),
                }
            )


def render_report(episode: Dict[str, Any], flow_summary: Dict[str, Any]) -> str:
    summary = episode["summary"]
    lines = [
        "# Step7 Integrated Offline Flow Report",
        "",
        "## Summary",
        "",
        f"- episode_id: `{episode.get('episode_id')}`",
        f"- status: `{summary.get('status')}`",
        f"- risk_map_points: `{summary.get('risk_map_points')}`",
        f"- D435 simulated trigger pass/block: `{summary.get('d435_succeeded_dry_run')}/{summary.get('d435_blocked')}`",
        f"- arm simulated trigger pass/block: `{summary.get('arm_succeeded_dry_run')}/{summary.get('arm_blocked')}`",
        f"- ros_started: `{summary.get('ros_started')}`",
        f"- published_cmd_vel: `{summary.get('published_cmd_vel')}`",
        f"- hardware_executed: `{summary.get('hardware_executed')}`",
        f"- serial_port_opened: `{summary.get('serial_port_opened')}`",
        "",
        "## Input Evidence",
        "",
        f"- P4-Y2 policy run report: `{flow_summary['source_inputs'].get('p4y2_policy_run_report')}`",
        f"- P4-Y2 final marked map: `{flow_summary['source_inputs'].get('p4y2_final_marked_map')}`",
        f"- P4-Y2 policy doc: `{flow_summary['source_inputs'].get('p4y2_policy_doc')}`",
        f"- P4-Y2 bundle: `{flow_summary['source_inputs'].get('p4y2_bundle')}`",
        f"- P4-X episode report: `{flow_summary['source_inputs']['p4x_episode_report']}`",
        f"- Map-A0 risk map points: `{flow_summary['source_inputs']['risk_map_points']}`",
        f"- Arm-C0 candidates: `{flow_summary['source_inputs']['arm_c0_candidates']}`",
        "",
        "## Planning And Mapping Rule",
        "",
        "- This stage consumes P4-Y2 guarded policy stress-stop evidence as the upstream planning/mapping safety evidence.",
        "- P4-Y2 stopped early because `max_consecutive_fast_arc_reached`; that is treated as a valid guard outcome, not a failed 7-step completion.",
        "- P4-Y2 produced a final map save and final marked map artifact.",
        "- This Step7 offline runner does not start SLAM, Nav2, Gazebo, or ROS.",
        "- `path_planning_executed=false` and `autonomous_navigation_executed=false` apply to this Step7 offline runner.",
        "- `tf_validated=false`, `slam_used=false`, and `navigation_used=false` are preserved from Map-A0.",
        "",
        "### P4-Y2 Policy Evidence Summary",
        "",
        f"- policy_loaded: `{summary.get('p4y2_policy_loaded')}`",
        f"- step_count / max_steps: `{summary.get('p4y2_step_count')} / {((episode.get('step7_flow') or {}).get('planning_mapping') or {}).get('p4y2_policy_evidence', {}).get('policy_max_steps')}`",
        f"- executed_count: `{summary.get('p4y2_executed_count')}`",
        f"- sequence_stop_reason: `{summary.get('p4y2_sequence_stop_reason')}`",
        f"- stopped_by_guard: `{summary.get('p4y2_stopped_by_guard')}`",
        f"- base_zero_ok: `{summary.get('p4y2_base_zero_ok')}`",
        f"- final_map_saved: `{summary.get('p4y2_final_map_saved')}`",
        f"- critical_map_saved: `{summary.get('p4y2_critical_map_saved')}`",
        f"- cumulative_positive_forward_m: `{summary.get('p4y2_cumulative_positive_forward_m')}`",
        "",
        "## D435 Simulated Trigger Rule",
        "",
        "- Rule: projected risk point + complete existing visual evidence + depth within threshold.",
        "- No new RGB/depth capture is created by this offline runner.",
        "- Upstream P4-X must show `base_zero_ok_before=true` and `published_cmd_vel=false`.",
        "",
        "## Arm Simulated Trigger Rule",
        "",
        "- Rule: D435 trigger passed + Arm-C0 candidate is `succeeded_dry_run`.",
        f"- Selected action is restricted to `{SELECTED_ARM_ACTION}`.",
        f"- Selected sequence is restricted to `{SELECTED_ARM_SEQUENCE}`.",
        "- Hardware execution is disabled; contact and obstacle removal are forbidden.",
        "",
        "## Results",
        "",
        "| item | value |",
        "| --- | --- |",
        f"| D435 trigger count | {summary.get('d435_triggers')} |",
        f"| D435 succeeded_dry_run | {summary.get('d435_succeeded_dry_run')} |",
        f"| D435 blocked | {summary.get('d435_blocked')} |",
        f"| Arm trigger count | {summary.get('arm_triggers')} |",
        f"| Arm succeeded_dry_run | {summary.get('arm_succeeded_dry_run')} |",
        f"| Arm blocked | {summary.get('arm_blocked')} |",
        f"| errors | {len(episode.get('errors') or [])} |",
        "",
        "## Claim Boundary",
        "",
    ]
    for claim in flow_summary["claim_boundary"]["allowed_claims"]:
        lines.append(f"- allowed: {claim}")
    for claim in flow_summary["claim_boundary"]["disallowed_claims"]:
        lines.append(f"- disallowed: {claim}")
    lines.extend(
        [
            "",
            "## Next Recommended Step",
            "",
            "Run a separate Step7-B simulation or K1 live guarded test only after defining live ROS process gates, map/navigation scope, and explicit no-contact arm gates. Do not reuse this offline evidence as permission for hardware motion.",
            "",
        ]
    )
    return "\n".join(lines)


def render_readme(output_dir: Path, episode: Dict[str, Any]) -> str:
    return f"""# Step7 Integrated Offline Flow

This directory contains an offline integrated Step7 rule-flow run.

Files:

- `episode_report.json`
- `step7_flow_summary.json`
- `step7_trigger_trace.csv`
- `step7_integrated_report.md`
- `errors.json`
- `README.md`

Boundary:

- ROS was not started.
- `cmd_vel` was not published.
- No serial port was opened.
- No mechanical-arm hardware was controlled.
- No new D435 capture was created.
- P4-Y2 guarded policy stress-stop evidence is referenced as upstream evidence.
- Full autonomous navigation, path planning success, and high-precision SLAM are not claimed.

Source episode:

```text
{episode.get('episode_id')}
```
"""


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate Step7 offline integrated flow evidence.")
    parser.add_argument(
        "--p4x-episode-report",
        default="outputs/p4x_d435_hold_capture_v1/episode_report.json",
    )
    parser.add_argument(
        "--policy-run-report",
        default=DEFAULT_P4Y2_POLICY_RUN_REPORT,
        help="Existing P4-Y/P4-Y2 guarded policy run report used as upstream planning/mapping evidence.",
    )
    parser.add_argument(
        "--policy-final-marked-map",
        default=DEFAULT_P4Y2_FINAL_MARKED_MAP,
        help="Existing final marked map image from the guarded policy run.",
    )
    parser.add_argument(
        "--policy-doc",
        default=DEFAULT_P4Y2_DOC,
        help="Policy documentation that explains guarded policy executable modes.",
    )
    parser.add_argument(
        "--policy-bundle",
        default=DEFAULT_P4Y2_BUNDLE,
        help="Optional git bundle containing the frozen P4-Y2 evidence commit.",
    )
    parser.add_argument(
        "--risk-map-points",
        default="outputs/map_a_risk_point_projection_v1/offline_p4x/risk_map_points.json",
    )
    parser.add_argument(
        "--arm-c0-candidates",
        default="outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/map_gated_arm_candidates.json",
    )
    parser.add_argument(
        "--output-dir",
        default="outputs/step7_integrated_offline_flow_v1/offline_p4x_arm_c0",
    )
    parser.add_argument("--depth-trigger-m", type=float, default=DEFAULT_DEPTH_TRIGGER_M)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    p4x_path = Path(args.p4x_episode_report)
    policy_run_path = Path(args.policy_run_report) if args.policy_run_report else None
    policy_final_marked_map_path = (
        Path(args.policy_final_marked_map) if args.policy_final_marked_map else None
    )
    policy_doc_path = Path(args.policy_doc) if args.policy_doc else None
    policy_bundle_path = Path(args.policy_bundle) if args.policy_bundle else None
    risk_map_path = Path(args.risk_map_points)
    arm_c0_path = Path(args.arm_c0_candidates)
    output_dir = Path(args.output_dir)

    p4x_report = load_json(p4x_path)
    risk_map_package = load_json(risk_map_path)
    arm_c0_package = load_json(arm_c0_path)
    p4y2_policy_evidence = build_p4y2_policy_evidence(
        policy_run_path=policy_run_path,
        final_marked_map_path=policy_final_marked_map_path,
        policy_doc_path=policy_doc_path,
        bundle_path=policy_bundle_path,
    )

    episode, flow_summary = build_episode(
        p4x_path=p4x_path,
        risk_map_path=risk_map_path,
        arm_c0_candidates_path=arm_c0_path,
        output_dir=output_dir,
        p4x_report=p4x_report,
        risk_map_package=risk_map_package,
        arm_c0_package=arm_c0_package,
        p4y2_policy_evidence=p4y2_policy_evidence,
        depth_trigger_m=float(args.depth_trigger_m),
    )
    step7_flow = episode["step7_flow"]
    write_json(output_dir / "episode_report.json", episode)
    write_json(output_dir / "step7_flow_summary.json", flow_summary)
    write_json(output_dir / "errors.json", episode.get("errors") or [])
    write_trace_csv(
        output_dir / "step7_trigger_trace.csv",
        step7_flow["d435_rule_triggers"],
        step7_flow["arm_rule_triggers"],
    )
    write_text(output_dir / "step7_integrated_report.md", render_report(episode, flow_summary))
    write_text(output_dir / "README.md", render_readme(output_dir, episode))

    print(
        json.dumps(
            {
                "ok": True,
                "episode_id": episode.get("episode_id"),
                "status": episode.get("summary", {}).get("status"),
                "p4y2_sequence_stop_reason": episode.get("summary", {}).get(
                    "p4y2_sequence_stop_reason"
                ),
                "p4y2_final_map_saved": episode.get("summary", {}).get("p4y2_final_map_saved"),
                "risk_map_points": episode.get("summary", {}).get("risk_map_points"),
                "d435_succeeded_dry_run": episode.get("summary", {}).get("d435_succeeded_dry_run"),
                "d435_blocked": episode.get("summary", {}).get("d435_blocked"),
                "arm_succeeded_dry_run": episode.get("summary", {}).get("arm_succeeded_dry_run"),
                "arm_blocked": episode.get("summary", {}).get("arm_blocked"),
                "published_cmd_vel": False,
                "hardware_executed": False,
                "serial_port_opened": False,
                "serial_bytes_written": 0,
                "errors": len(episode.get("errors") or []),
                "output_dir": str(output_dir),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
