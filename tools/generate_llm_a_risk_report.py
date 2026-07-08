#!/usr/bin/env python3
"""Generate a deterministic LLM-A style risk report from episode_report.json."""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


REPORT_VERSION = "llm_a_deterministic_risk_report_v1"
VISUAL_PATH_KEYS = ("rgb", "depth_raw", "depth_vis", "camera_info", "odom", "capture_meta", "risk_point")


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def bool_all(values: Iterable[Any]) -> bool:
    values = list(values)
    return bool(values) and all(value is True for value in values)


def detect_episode_kind(action_types: Iterable[str], protocol_version: str) -> str:
    action_set = set(action_types)
    if (
        "STEP7E2_D435_RED_RULE_MAP_ARM_FLOW" in action_set
        or "STEP7E2_GUARDED_MICRO_MOTION" in action_set
        or "step7e2_guarded_motion_red_rule_flow" in protocol_version
    ):
        return "step7e2_guarded_motion_red_rule_flow"
    if (
        "STEP7E1_D435_RED_RULE_TRIGGER" in action_set
        or "STEP7E1_ARM_C1_DRY_RUN_GATE" in action_set
        or "step7e1_red_rule_stationary_flow" in protocol_version
    ):
        return "step7e1_red_rule_stationary_flow"
    if (
        "STEP7E_N10P_FRONT_EVENT_GATE" in action_set
        or "STEP7E_EVENT_TRIGGERED_D435_MAP_ARM_FLOW" in action_set
        or "step7e_event_triggered_capture_arm_flow" in protocol_version
    ):
        return "step7e_event_triggered_capture_arm_flow"
    if (
        "STEP7D_GUARDED_MICRO_MOTION" in action_set
        or "STEP7D_D435_MAP_ARM_FLOW" in action_set
        or "step7d_guarded_motion_d435_arm_flow" in protocol_version
    ):
        return "step7d_guarded_motion_d435_arm_flow"
    if (
        "STEP7C_LIVE_BASE_ZERO_GATE" in action_set
        or "STEP7C_LIVE_D435_HOLD_CAPTURE" in action_set
        or "STEP7C_MOCK_RISK_TRIGGER" in action_set
        or "STEP7C_ARM_C1_NO_LOAD_ONCE" in action_set
        or "step7c_guarded_d435_mockrisk_arm_noload" in protocol_version
    ):
        return "step7c_guarded_d435_mockrisk_arm_noload"
    if (
        "STEP7B_LIVE_BASE_ZERO_GATE" in action_set
        or "STEP7B_LIVE_D435_HOLD_CAPTURE" in action_set
        or "STEP7B_MAP_A0_LIVE_PROJECTION" in action_set
        or "STEP7B_ARM_C0_LIVE_DRYRUN" in action_set
        or "step7b_live_stationary_flow" in protocol_version
    ):
        return "step7b_live_stationary_flow"
    if (
        "STEP7_PLANNING_MAPPING_RULE_DRY_RUN" in action_set
        or "STEP7_D435_RULE_TRIGGER_HOLD_CAPTURE_SIM" in action_set
        or "STEP7_ARM_RULE_TRIGGER_NO_LOAD_SIM" in action_set
        or "step7_integrated_offline_flow" in protocol_version
    ):
        return "step7_integrated_offline_flow"
    if (
        "ARM_C1_MAP_GATED_NO_LOAD_ONCE" in action_set
        or "arm_c1_map_gated_no_load_once" in protocol_version
    ):
        return "arm_c1_map_gated_no_load_once"
    if "MAP_GATED_ARM_CANDIDATE" in action_set or "arm_c0_map_to_arm_dryrun" in protocol_version:
        return "arm_c0_map_to_arm_dryrun"
    if any(
        action_type and action_type.startswith("ARM_B") and "NO_LOAD" in action_type
        for action_type in action_set
    ):
        return "arm_b_no_load"
    if "ARM_REMOVE_OBSTACLE" in action_set or "arm_a_mock" in protocol_version:
        return "arm_a_mock_remove_obstacle"
    if "HOLD_CAPTURE" in action_set or "p4x_d435" in protocol_version:
        return "p4x_hold_capture"
    return "generic_episode_report"


def status_from_report(report: Dict[str, Any], failed_safe_count: int, published_cmd_vel_any: bool) -> str:
    summary = report.get("summary") or {}
    errors = report.get("errors") or []
    if errors or failed_safe_count > 0 or published_cmd_vel_any:
        return "REVIEW"
    if report.get("status") == "succeeded":
        return "PASS"
    if summary.get("status") == "succeeded":
        return "PASS"
    if summary.get("acceptance_10_runs_9_success") is True:
        return "PASS"
    if summary.get("acceptance_10_runs_10_success") is True:
        return "PASS"
    if summary.get("status") == "succeeded_dry_run":
        return "PASS_DRY_RUN"
    candidates = summary.get("candidates")
    succeeded_dry_run = summary.get("succeeded_dry_run")
    blocked = summary.get("blocked")
    if (
        candidates is not None
        and succeeded_dry_run is not None
        and int(candidates or 0) == int(succeeded_dry_run or 0)
        and int(blocked or 0) == 0
    ):
        return "PASS_DRY_RUN"
    succeeded = summary.get("succeeded")
    completed = summary.get("completed_actions") or summary.get("requested_captures") or summary.get("requested_actions")
    if succeeded is not None and completed is not None and succeeded == completed:
        return "PASS"
    return "REVIEW"


def build_safety_summary(report: Dict[str, Any]) -> Dict[str, Any]:
    actions = report.get("actions") or []
    results = report.get("action_results") or []
    summary = report.get("summary") or {}
    failed_safe_count = sum(1 for result in results if result.get("status") == "failed_safe")
    if summary.get("failed_safe") is not None:
        failed_safe_count = max(failed_safe_count, int(summary.get("failed_safe") or 0))
    published_cmd_vel_any = any(action.get("publishes_cmd_vel") is True for action in actions)
    published_cmd_vel_any = published_cmd_vel_any or any(
        result.get("published_cmd_vel") is True for result in results
    )
    published_cmd_vel_any = published_cmd_vel_any or summary.get("published_cmd_vel") is True
    return {
        "base_zero_ok_before_all": bool_all(
            result.get("base_zero_ok_before") for result in results
        ),
        "requires_base_zero_all": bool_all(
            action.get("requires_base_zero") for action in actions
        ),
        "published_cmd_vel_any": bool(published_cmd_vel_any),
        "published_cmd_vel_summary": summary.get("published_cmd_vel"),
        "failed_safe_count": failed_safe_count,
        "errors_count": len(report.get("errors") or []),
        "hardware_executed": summary.get("hardware_executed"),
        "serial_port_opened": summary.get("serial_port_opened"),
        "serial_bytes_written": summary.get("serial_bytes_written"),
        "contact_allowed": summary.get("contact_allowed"),
        "obstacle_removed": summary.get("obstacle_removed"),
        "base_zero_required": summary.get("base_zero_required"),
        "base_zero_checked": summary.get("base_zero_checked"),
        "succeeded_dry_run": summary.get("succeeded_dry_run"),
        "blocked": summary.get("blocked"),
    }


def build_action_trace(report: Dict[str, Any]) -> List[Dict[str, Any]]:
    actions = report.get("actions") or []
    results_by_action = {
        result.get("action_id"): result for result in report.get("action_results") or []
    }
    trace = []
    for action in actions:
        result = results_by_action.get(action.get("action_id"), {})
        details = result.get("details") or {}
        trace.append(
            {
                "action_id": action.get("action_id"),
                "action_type": action.get("action_type"),
                "status": result.get("status"),
                "requires_base_zero": action.get("requires_base_zero"),
                "publishes_cmd_vel": action.get("publishes_cmd_vel"),
                "base_zero_ok_before": result.get("base_zero_ok_before"),
                "published_cmd_vel": result.get("published_cmd_vel"),
                "mock": result.get("mock") if "mock" in result else details.get("mock"),
                "obstacle_removed": result.get("obstacle_removed")
                if "obstacle_removed" in result
                else details.get("obstacle_removed"),
                "selected_action": details.get("selected_action"),
                "selected_sequence": details.get("selected_sequence"),
                "hardware_executed": details.get("hardware_executed"),
                "serial_port_opened": details.get("serial_port_opened"),
                "serial_bytes_written": details.get("serial_bytes_written"),
                "contact_allowed": details.get("contact_allowed"),
                "base_zero_checked": details.get("base_zero_checked"),
                "projection_precision": details.get("projection_precision"),
                "error": result.get("error"),
            }
        )
    return trace


def first_non_empty(*values: Optional[str]) -> Optional[str]:
    for value in values:
        if value:
            return value
    return None


def build_evidence_summary(report: Dict[str, Any]) -> Dict[str, Any]:
    captures = report.get("captures") or []
    risks_by_capture = {
        risk.get("capture_id"): risk for risk in report.get("risk_points") or []
    }
    results_by_capture = {
        result.get("capture_id"): result
        for result in report.get("action_results") or []
        if result.get("capture_id")
    }
    capture_entries = []
    for capture in captures:
        capture_id = capture.get("capture_id")
        paths = dict(capture.get("paths") or {})
        risk = risks_by_capture.get(capture_id, {})
        result = results_by_capture.get(capture_id, {})
        evidence_paths = dict(result.get("evidence_paths") or {})
        risk_paths = dict(risk.get("evidence_paths") or {})
        merged = {
            "rgb": first_non_empty(paths.get("rgb"), evidence_paths.get("rgb"), risk_paths.get("rgb")),
            "depth_raw": first_non_empty(
                paths.get("depth_raw"), evidence_paths.get("depth_raw"), risk_paths.get("depth_raw")
            ),
            "depth_vis": first_non_empty(
                paths.get("depth_vis"), evidence_paths.get("depth_vis"), risk_paths.get("depth_vis")
            ),
            "camera_info": first_non_empty(
                paths.get("camera_info"),
                evidence_paths.get("camera_info"),
                risk_paths.get("camera_info"),
            ),
            "odom": first_non_empty(paths.get("odom"), evidence_paths.get("odom")),
            "capture_meta": first_non_empty(
                paths.get("capture_meta"),
                result.get("capture_meta_path"),
                evidence_paths.get("capture_meta"),
                risk_paths.get("capture_meta"),
            ),
            "risk_point": first_non_empty(
                result.get("risk_point_path"),
                evidence_paths.get("risk_point"),
                risk_paths.get("risk_point"),
            ),
        }
        missing = [key for key in VISUAL_PATH_KEYS if not merged.get(key)]
        capture_entries.append(
            {
                "capture_id": capture_id,
                "action_id": capture.get("action_id"),
                "paths": merged,
                "missing_paths": missing,
            }
        )

    action_result_entries = []
    for result in report.get("action_results") or []:
        evidence_paths = dict(result.get("evidence_paths") or {})
        action_result_entries.append(
            {
                "action_id": result.get("action_id"),
                "capture_id": result.get("capture_id"),
                "capture_meta_path": result.get("capture_meta_path"),
                "risk_point_path": result.get("risk_point_path"),
                "evidence_paths": evidence_paths,
                "missing_action_evidence_paths": []
                if evidence_paths
                else ["evidence_paths"],
            }
        )

    return {
        "capture_count": len(capture_entries),
        "captures": capture_entries,
        "action_results": action_result_entries,
    }


def build_risk_point_summary(report: Dict[str, Any]) -> List[Dict[str, Any]]:
    entries = []
    for risk in report.get("risk_points") or []:
        missing = []
        for key in (
            "label",
            "category",
            "depth_median_m",
            "camera_point_xyz_m",
            "depth_scale_m",
            "bbox_valid_depth_ratio",
        ):
            if key not in risk and not (key == "category" and "risk_category" in risk):
                missing.append(key)
        entries.append(
            {
                "risk_point_id": risk.get("risk_point_id"),
                "capture_id": risk.get("capture_id"),
                "label": risk.get("label"),
                "category": risk.get("category") or risk.get("risk_category"),
                "depth_median_m": risk.get("depth_median_m"),
                "camera_point_xyz_m": risk.get("camera_point_xyz_m"),
                "depth_scale_m": risk.get("depth_scale_m"),
                "bbox_valid_depth_ratio": risk.get("bbox_valid_depth_ratio"),
                "confidence": risk.get("confidence"),
                "generated_by": risk.get("generated_by"),
                "missing_fields": missing,
            }
        )
    return entries


def claim_boundary_for(kind: str) -> Dict[str, Any]:
    if kind == "arm_a_mock_remove_obstacle":
        return {
            "allowed_claims": [
                "Only ARM_REMOVE_OBSTACLE mock action chain validation is claimed.",
                "10/10 mock actions succeeded with 0 failed_safe.",
                "Mock actions required base_zero_ok_before=true.",
                "Mock actions did not publish cmd_vel.",
                "mock=True and obstacle_removed=True are mock result fields only.",
            ],
            "disallowed_claims": [
                "Do not claim real mechanical-arm control.",
                "Do not claim bus-servo hardware validation.",
                "Do not claim obstacle removal in the physical world.",
                "Do not claim grasping, pushing, contact, or obstacle interaction success.",
            ],
        }
    if kind == "p4x_hold_capture":
        return {
            "allowed_claims": [
                "Only safe stationary D435 HOLD_CAPTURE visual/depth evidence-chain capture is claimed.",
                "RGB/depth/camera_info/odom/meta/risk_point evidence was saved.",
                "10/10 HOLD_CAPTURE actions succeeded with 0 failed_safe.",
                "HOLD_CAPTURE ran after base_zero_ok_before=true.",
                "HOLD_CAPTURE did not publish cmd_vel.",
            ],
            "disallowed_claims": [
                "Do not claim visual detection accuracy.",
                "Do not claim autonomous semantic reasoning.",
                "Do not claim real mechanical-arm operation.",
                "Do not claim P4-X3 new fields are hardware-validated.",
            ],
        }
    if kind == "arm_b_no_load":
        return {
            "allowed_claims": [
                "Only K1 bus-servo hardware control and no-load mechanical-arm execution are claimed.",
                "No-load actions required base_zero_ok_before=true.",
                "No-load actions did not publish cmd_vel.",
                "Contact and obstacle removal were not allowed.",
                "The final sequence returned to safe_idle_home_like_6b when the source episode reports that result.",
            ],
            "disallowed_claims": [
                "Do not claim grasping or payload handling.",
                "Do not claim contact with obstacles.",
                "Do not claim real obstacle removal.",
                "Do not claim autonomous execution.",
                "Do not claim a ROS arm executor has been validated.",
            ],
        }
    if kind == "arm_c0_map_to_arm_dryrun":
        return {
            "allowed_claims": [
                "Only map risk point to arm no-load action candidate dry-run mapping is claimed.",
                "MAP_GATED_ARM_CANDIDATE actions are candidates, not real arm execution.",
                "Candidates are restricted to the validated no-load ARM_SAMPLE_NO_LOAD sequence.",
                "No ROS process, cmd_vel publish, serial access, or arm hardware execution is claimed.",
                "`base_zero_checked=false` is the expected dry-run state; Arm-C1 hardware execution must re-check live `base_zero_ok_before_arm=true`.",
            ],
            "disallowed_claims": [
                "Do not claim real mechanical-arm motion.",
                "Do not claim obstacle removal.",
                "Do not claim grasping, contact, or payload handling.",
                "Do not claim SLAM, autonomous navigation, or path planning.",
                "Do not claim LLM control of the robot.",
            ],
        }
    if kind == "arm_c1_map_gated_no_load_once":
        return {
            "allowed_claims": [
                "Only Arm-C1 map-gated no-load integrated validation is claimed.",
                "A fresh live base-zero evidence file was consumed before the no-load arm action.",
                "The selected action is restricted to the validated ARM_SAMPLE_NO_LOAD sequence.",
                "The run may claim one real no-load hardware execution only if the source episode reports hardware_executed=true and status=succeeded.",
                "The run did not publish cmd_vel when the source episode reports published_cmd_vel=false.",
                "Contact and obstacle removal were not allowed.",
            ],
            "disallowed_claims": [
                "Do not claim grasping.",
                "Do not claim contact or payload handling.",
                "Do not claim physical obstacle removal.",
                "Do not claim autonomous navigation or path planning.",
                "Do not claim SLAM or high-precision mapping.",
                "Do not claim LLM control of the robot.",
                "Do not claim a ROS arm executor has been validated.",
                "Do not claim the live base-zero evidence can be reused after its freshness window.",
            ],
        }
    if kind == "step7_integrated_offline_flow":
        return {
            "allowed_claims": [
                "Only Step7-A offline integrated rule-flow validation is claimed.",
                "Existing P4-Y2 guarded policy stress-stop evidence is consumed as planning/mapping safety evidence.",
                "Existing P4-X stationary D435 evidence is consumed as the visual trigger source.",
                "Existing Map-A0 approximate risk map points are consumed as mapping evidence.",
                "Arm-C0 no-load candidates are consumed as simulated mechanical-arm trigger outputs.",
                "The output can be summarized by deterministic LLM-A without online API calls.",
            ],
            "disallowed_claims": [
                "Do not claim new live D435 capture in the Step7 offline run.",
                "Do not claim ROS was started.",
                "Do not claim cmd_vel was published.",
                "Do not claim real mechanical-arm motion from the Step7 offline run.",
                "Do not claim grasping, contact, payload handling, or obstacle removal.",
                "Do not claim full autonomous navigation, path planning success, or high-precision SLAM.",
                "Do not claim LLM control of the robot.",
            ],
        }
    if kind == "step7b_live_stationary_flow":
        return {
            "allowed_claims": [
                "Only Step7-B0 live stationary integration is claimed.",
                "A live base-zero evidence gate was checked before live D435 HOLD_CAPTURE.",
                "A new live D435 capture was consumed by Map-A0 projection.",
                "Arm-C0 generated map-gated no-load candidates as dry-run outputs.",
                "No cmd_vel publication is claimed for this runner.",
            ],
            "disallowed_claims": [
                "Do not claim chassis motion during Step7-B0.",
                "Do not claim autonomous navigation or path planning success.",
                "Do not claim SLAM or high-precision mapping.",
                "Do not claim grasping, contact, payload handling, or obstacle removal.",
                "Do not claim LLM control of the robot.",
                "Do not claim Arm-C1-H hardware execution unless the source report explicitly shows arm_c1_hardware_executed=true.",
            ],
        }
    if kind == "step7c_guarded_d435_mockrisk_arm_noload":
        return {
            "allowed_claims": [
                "Only Step7-C guarded stationary live integration is claimed.",
                "A live base-zero gate was checked before D435 HOLD_CAPTURE.",
                "A deterministic mock anomaly trigger was generated from the live D435 evidence chain.",
                "Map-A0 projected the live risk_point into an approximate risk-map point.",
                "Arm-C0 generated a map-gated no-load action candidate.",
                "Arm-C1 dry-run may be claimed when hardware_executed=false and serial_bytes_written=0.",
                "One no-load hardware response may be claimed only if the source report explicitly shows hardware_executed=true, serial_bytes_written>0, contact_allowed=false, obstacle_removed=false, and status=succeeded.",
            ],
            "disallowed_claims": [
                "Do not claim real visual anomaly detection accuracy.",
                "Do not claim grasping, contact, payload handling, or obstacle removal.",
                "Do not claim autonomous navigation or path planning success.",
                "Do not claim SLAM or high-precision mapping.",
                "Do not claim LLM control of the robot.",
                "Do not claim autonomous mechanical-arm disposal based on the map.",
            ],
        }
    if kind == "step7d_guarded_motion_d435_arm_flow":
        return {
            "allowed_claims": [
                "Only Step7-D guarded micro-motion integration is claimed.",
                "Chassis motion is routed through the existing P4 guarded policy and N10P safety chain.",
                "A base-zero gate is checked after guarded motion before D435 HOLD_CAPTURE.",
                "A deterministic mock anomaly trigger is generated from live D435 evidence.",
                "Map-A0 projects the live risk_point into an approximate risk-map point.",
                "Arm response may be claimed as dry-run when hardware_executed=false.",
                "One no-load hardware arm response may be claimed only if the source report explicitly shows hardware_executed=true, contact_allowed=false, obstacle_removed=false, and status=succeeded.",
            ],
            "disallowed_claims": [
                "Do not claim direct cmd_vel bypass.",
                "Do not claim autonomous navigation or path planning success.",
                "Do not claim SLAM or high-precision mapping.",
                "Do not claim real visual anomaly detection accuracy.",
                "Do not claim grasping, contact, payload handling, or obstacle removal.",
                "Do not claim LLM control of the robot.",
            ],
        }
    if kind == "step7e_event_triggered_capture_arm_flow":
        return {
            "allowed_claims": [
                "Only Step7-E0 N10P/front_p10 event-triggered capture integration is claimed.",
                "Chassis motion is routed through the existing P4 guarded policy and N10P safety chain.",
                "D435 HOLD_CAPTURE is executed only after the source episode reports event_triggered=true and base_zero_ok_before_capture=true.",
                "The risk trigger source is N10P front range evidence; the risk point remains deterministic/mock.",
                "Map-A0 projects the live risk_point into an approximate risk-map point.",
                "Arm response may be claimed as dry-run when hardware_executed=false and serial_bytes_written=0.",
                "One no-load hardware arm response may be claimed only if the source report explicitly shows hardware_executed=true, contact_allowed=false, obstacle_removed=false, and status=succeeded.",
            ],
            "disallowed_claims": [
                "Do not claim direct cmd_vel bypass.",
                "Do not claim autonomous navigation or path planning success.",
                "Do not claim SLAM or high-precision mapping.",
                "Do not claim real visual anomaly detection accuracy.",
                "Do not claim grasping, contact, payload handling, or obstacle removal.",
                "Do not claim LLM control of the robot.",
            ],
        }
    if kind == "step7e1_red_rule_stationary_flow":
        return {
            "allowed_claims": [
                "Only Step7-E1 stationary D435 red-rule trigger integration is claimed.",
                "The trigger source is a deterministic HSV red color rule applied to live D435 RGB evidence.",
                "Depth median and approximate camera-frame point may be claimed when present in the source report.",
                "Map-A0 projects the red-rule risk_point into an approximate risk-map point.",
                "Arm response may be claimed as dry-run when hardware_executed=false and serial_bytes_written=0.",
                "One no-load hardware response may be claimed only when the source report shows hardware_executed=true, serial_bytes_written>0, contact_allowed=false, obstacle_removed=false, and status=succeeded.",
                "No online API or real LLM is used for the report.",
            ],
            "disallowed_claims": [
                "Do not claim trained model inference.",
                "Do not claim visual detection accuracy or robust recognition generalization.",
                "Do not claim chassis motion during the Step7-E1 stationary flow.",
                "Do not claim autonomous navigation or path planning success.",
                "Do not claim SLAM or high-precision mapping.",
                "Do not claim repeated or autonomous mechanical-arm motion.",
                "Do not claim grasping, contact, payload handling, or obstacle removal.",
                "Do not claim LLM control of the robot.",
            ],
        }
    if kind == "step7e2_guarded_motion_red_rule_flow":
        return {
            "allowed_claims": [
                "Only Step7-E2 guarded micro-motion followed by D435 red-rule trigger integration is claimed.",
                "Chassis motion is routed through the existing P4 guarded policy and N10P safety chain.",
                "D435 red-rule capture is executed only after the source episode reports base_zero_ok_after_motion=true.",
                "The trigger source is a deterministic HSV red color rule applied to live D435 RGB evidence after guarded motion.",
                "Depth median and approximate camera-frame point may be claimed when present in the source report.",
                "Map-A0 projects the red-rule risk_point into an approximate risk-map point.",
                "Arm response may be claimed as dry-run when hardware_executed=false and serial_bytes_written=0.",
                "One no-load hardware response may be claimed only when the source report shows hardware_executed=true, serial_bytes_written>0, contact_allowed=false, obstacle_removed=false, and status=succeeded.",
                "No online API or real LLM is used for the report.",
            ],
            "disallowed_claims": [
                "Do not claim direct cmd_vel bypass.",
                "Do not claim trained model inference.",
                "Do not claim visual detection accuracy or robust recognition generalization.",
                "Do not claim autonomous navigation or path planning success.",
                "Do not claim SLAM or high-precision mapping.",
                "Do not claim repeated or autonomous mechanical-arm motion.",
                "Do not claim grasping, contact, payload handling, or obstacle removal.",
                "Do not claim LLM control of the robot.",
            ],
        }
    return {
        "allowed_claims": ["Summarize only fields present in episode_report.json."],
        "disallowed_claims": ["Do not infer missing capabilities or hardware validation."],
    }


def next_step_for(kind: str) -> str:
    if kind == "arm_a_mock_remove_obstacle":
        return "Arm-B: before real arm integration, run bus-servo dry-run / no-load tests while keeping base-zero and no-cmd_vel constraints."
    if kind == "p4x_hold_capture":
        return "P4-X3: run real ROS/D435 validation for the new header/frame/depth-ratio evidence fields."
    if kind == "arm_b_no_load":
        return "Freeze Arm-B no-load evidence before considering Arm-C/D contact or any ROS arm executor integration."
    if kind == "arm_c0_map_to_arm_dryrun":
        return "Arm-C1 must be a separate explicitly confirmed hardware step: re-check live base_zero_ok_before_arm=true before any no-load execution, keep no-contact/no-obstacle-removal boundaries, and do not let LLM or map output directly control hardware."
    if kind == "arm_c1_map_gated_no_load_once":
        return "Freeze Arm-C1-H evidence and generate a human-readable validation report before considering any contact, grasping, payload, or obstacle-removal work."
    if kind == "step7_integrated_offline_flow":
        return "Step7-B should be a separate simulation or K1 live guarded validation with explicit ROS process gates, no-contact arm gates, and fresh base-zero evidence before any hardware motion."
    if kind == "step7b_live_stationary_flow":
        return "Freeze Step7-B0 live stationary evidence before adding chassis motion or optional Arm-C1-H hardware execution."
    if kind == "step7c_guarded_d435_mockrisk_arm_noload":
        return "Freeze Step7-C0 dry-run evidence first; Step7-C1 hardware should be a separate single no-load run with fresh base-zero evidence, no contact, and explicit operator confirmation."
    if kind == "step7d_guarded_motion_d435_arm_flow":
        return "Freeze Step7-D guarded-motion evidence before increasing path length, adding repeated runs, or enabling any contact/grasping/payload task."
    if kind == "step7e_event_triggered_capture_arm_flow":
        return "Freeze Step7-E0 dry-run evidence first; any Arm-C1 hardware extension must be a separate single no-load run with fresh base-zero evidence, no contact, and explicit operator confirmation."
    if kind == "step7e1_red_rule_stationary_flow":
        return "Freeze Step7-E1 red-rule evidence; any hardware run must remain a single no-load response with fresh base-zero evidence and operator confirmation before considering guarded-motion red-rule work."
    if kind == "step7e2_guarded_motion_red_rule_flow":
        return "Freeze Step7-E2 guarded-motion red-rule dry-run evidence before considering a separate single no-load hardware run or a longer guarded-motion path."
    return "Define the next validation step from the episode claim boundary before adding hardware actions."


def build_report(source_path: Path) -> Dict[str, Any]:
    episode = load_json(source_path)
    action_types = [action.get("action_type") for action in episode.get("actions") or []]
    kind = detect_episode_kind(action_types, str(episode.get("protocol_version") or ""))
    safety_summary = build_safety_summary(episode)
    status = status_from_report(
        episode,
        safety_summary["failed_safe_count"],
        safety_summary["published_cmd_vel_any"],
    )
    return {
        "report_version": REPORT_VERSION,
        "generated_at": now_iso(),
        "generator": "tools/generate_llm_a_risk_report.py",
        "deterministic_baseline": True,
        "llm_used": False,
        "online_api_used": False,
        "local_model_used": False,
        "source_episode_report": str(source_path),
        "episode_kind": kind,
        "episode_summary": {
            "episode_id": episode.get("episode_id"),
            "protocol_version": episode.get("protocol_version"),
            "status": status,
            "started_at": episode.get("started_at"),
            "ended_at": episode.get("ended_at"),
            "action_count": len(episode.get("actions") or []),
            "action_result_count": len(episode.get("action_results") or []),
            "capture_count": len(episode.get("captures") or []),
            "risk_point_count": len(episode.get("risk_points") or []),
            "source_summary": episode.get("summary") or {},
        },
        "safety_summary": safety_summary,
        "action_trace": build_action_trace(episode),
        "evidence_summary": build_evidence_summary(episode),
        "risk_point_summary": build_risk_point_summary(episode),
        "claim_boundary": claim_boundary_for(kind),
        "next_recommended_step": next_step_for(kind),
        "source_errors": episode.get("errors") or [],
    }


def val(value: Any) -> str:
    if value is None:
        return "missing"
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False)
    return str(value)


def markdown_table(headers: List[str], rows: List[List[Any]]) -> List[str]:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(val(item).replace("\n", " ") for item in row) + " |")
    return lines


def render_markdown(report: Dict[str, Any]) -> str:
    episode = report["episode_summary"]
    safety = report["safety_summary"]
    lines = [
        "# LLM-A Deterministic Risk Report",
        "",
        "- deterministic_baseline: `True`",
        "- llm_used: `False`",
        "- online_api_used: `False`",
        "- local_model_used: `False`",
        "",
        "## Episode Summary",
        "",
        f"- episode_id: `{episode.get('episode_id')}`",
        f"- status: `{episode.get('status')}`",
        f"- action count: `{episode.get('action_count')}`",
        f"- action result count: `{episode.get('action_result_count')}`",
        f"- source: `{report.get('source_episode_report')}`",
        "",
        "## Safety Summary",
        "",
        f"- base_zero satisfied: `{safety.get('base_zero_ok_before_all')}`",
        f"- published cmd_vel: `{safety.get('published_cmd_vel_any')}`",
        f"- requires_base_zero all true: `{safety.get('requires_base_zero_all')}`",
        f"- failed_safe count: `{safety.get('failed_safe_count')}`",
        f"- source errors count: `{safety.get('errors_count')}`",
    ]
    if report.get("episode_kind") in (
        "arm_c0_map_to_arm_dryrun",
        "arm_c1_map_gated_no_load_once",
        "step7_integrated_offline_flow",
        "step7b_live_stationary_flow",
        "step7c_guarded_d435_mockrisk_arm_noload",
        "step7d_guarded_motion_d435_arm_flow",
        "step7e_event_triggered_capture_arm_flow",
        "step7e1_red_rule_stationary_flow",
        "step7e2_guarded_motion_red_rule_flow",
    ):
        lines.extend(
            [
                f"- base_zero_checked: `{safety.get('base_zero_checked')}`",
                f"- hardware_executed: `{safety.get('hardware_executed')}`",
                f"- serial_port_opened: `{safety.get('serial_port_opened')}`",
                f"- serial_bytes_written: `{safety.get('serial_bytes_written')}`",
                f"- contact_allowed: `{safety.get('contact_allowed')}`",
                f"- obstacle_removed: `{safety.get('obstacle_removed')}`",
                f"- succeeded_dry_run: `{safety.get('succeeded_dry_run')}`",
                f"- blocked: `{safety.get('blocked')}`",
            ]
        )
    lines.extend(["", "## Action Trace", ""])
    lines.extend(
        markdown_table(
            [
                "action_id",
                "action_type",
                "status",
                "base_zero_ok_before",
                "published_cmd_vel",
                "mock",
                "obstacle_removed",
            ],
            [
                [
                    item.get("action_id"),
                    item.get("action_type"),
                    item.get("status"),
                    item.get("base_zero_ok_before"),
                    item.get("published_cmd_vel"),
                    item.get("mock"),
                    item.get("obstacle_removed"),
                ]
                for item in report["action_trace"]
            ],
        )
    )
    if report.get("episode_kind") == "arm_c0_map_to_arm_dryrun":
        lines.extend(["", "## Arm-C0 Candidate Details", ""])
        lines.extend(
            markdown_table(
                [
                    "action_id",
                    "selected_action",
                    "selected_sequence",
                    "base_zero_checked",
                    "hardware_executed",
                    "serial_bytes_written",
                    "contact_allowed",
                    "projection_precision",
                ],
                [
                    [
                        item.get("action_id"),
                        item.get("selected_action"),
                        item.get("selected_sequence"),
                        item.get("base_zero_checked"),
                        item.get("hardware_executed"),
                        item.get("serial_bytes_written"),
                        item.get("contact_allowed"),
                        item.get("projection_precision"),
                    ]
                    for item in report["action_trace"]
                ],
            )
        )
    if report.get("episode_kind") == "arm_c1_map_gated_no_load_once":
        lines.extend(["", "## Arm-C1 No-Load Execution Details", ""])
        lines.extend(
            markdown_table(
                [
                    "action_id",
                    "selected_action",
                    "selected_sequence",
                    "hardware_executed",
                    "serial_port_opened",
                    "serial_bytes_written",
                    "contact_allowed",
                    "obstacle_removed",
                    "projection_precision",
                ],
                [
                    [
                        item.get("action_id"),
                        item.get("selected_action"),
                        item.get("selected_sequence"),
                        item.get("hardware_executed"),
                        item.get("serial_port_opened"),
                        item.get("serial_bytes_written"),
                        item.get("contact_allowed"),
                        item.get("obstacle_removed"),
                        item.get("projection_precision"),
                    ]
                    for item in report["action_trace"]
                ],
            )
        )
    if report.get("episode_kind") == "step7_integrated_offline_flow":
        source_summary = episode.get("source_summary") or {}
        lines.extend(["", "## Step7 Upstream Policy Evidence", ""])
        lines.extend(
            [
                f"- p4y2_policy_loaded: `{source_summary.get('p4y2_policy_loaded')}`",
                f"- p4y2_step_count: `{source_summary.get('p4y2_step_count')}`",
                f"- p4y2_executed_count: `{source_summary.get('p4y2_executed_count')}`",
                f"- p4y2_sequence_stop_reason: `{source_summary.get('p4y2_sequence_stop_reason')}`",
                f"- p4y2_stopped_by_guard: `{source_summary.get('p4y2_stopped_by_guard')}`",
                f"- p4y2_base_zero_ok: `{source_summary.get('p4y2_base_zero_ok')}`",
                f"- p4y2_final_map_saved: `{source_summary.get('p4y2_final_map_saved')}`",
                f"- p4y2_critical_map_saved: `{source_summary.get('p4y2_critical_map_saved')}`",
                f"- p4y2_cumulative_positive_forward_m: `{source_summary.get('p4y2_cumulative_positive_forward_m')}`",
                "",
            ]
        )
        lines.extend(["", "## Step7 Trigger Details", ""])
        lines.extend(
            markdown_table(
                [
                    "action_id",
                    "action_type",
                    "status",
                    "selected_action",
                    "selected_sequence",
                    "hardware_executed",
                    "serial_bytes_written",
                    "contact_allowed",
                    "obstacle_removed",
                ],
                [
                    [
                        item.get("action_id"),
                        item.get("action_type"),
                        item.get("status"),
                        item.get("selected_action"),
                        item.get("selected_sequence"),
                        item.get("hardware_executed"),
                        item.get("serial_bytes_written"),
                        item.get("contact_allowed"),
                        item.get("obstacle_removed"),
                    ]
                    for item in report["action_trace"]
                ],
            )
        )
    lines.extend(["", "## Evidence Summary", ""])
    captures = report["evidence_summary"]["captures"]
    if captures:
        for capture in captures:
            lines.append(f"### Capture `{capture.get('capture_id')}`")
            for key in VISUAL_PATH_KEYS:
                lines.append(f"- {key}: `{val((capture.get('paths') or {}).get(key))}`")
            missing = capture.get("missing_paths") or []
            lines.append(f"- missing evidence paths: `{', '.join(missing) if missing else 'none'}`")
            lines.append("")
    else:
        lines.append("- no capture records present")
        for result in report["evidence_summary"]["action_results"]:
            lines.append(f"- action `{result.get('action_id')}` evidence_paths: `{val(result.get('evidence_paths'))}`")
        if report.get("episode_kind") == "arm_b_no_load":
            lines.append("- visual evidence paths: `not applicable for Arm-B no-load action episode`")
        elif report.get("episode_kind") == "arm_c0_map_to_arm_dryrun":
            lines.append("- visual evidence paths: `referenced through source_evidence_paths only; no new capture is generated in Arm-C0`")
        elif report.get("episode_kind") == "arm_c1_map_gated_no_load_once":
            lines.append("- visual evidence paths: `referenced through Arm-C0 source evidence; no new capture is generated in Arm-C1`")
        elif report.get("episode_kind") == "step7_integrated_offline_flow":
            lines.append("- visual evidence paths: `referenced through upstream P4-X source evidence; no new capture is generated in Step7-A offline flow`")
        elif report.get("episode_kind") == "step7b_live_stationary_flow":
            lines.append("- visual evidence paths: `referenced through the Step7-B live D435 HOLD_CAPTURE sub-report`")
        elif report.get("episode_kind") == "step7c_guarded_d435_mockrisk_arm_noload":
            lines.append("- visual evidence paths: `referenced through the Step7-C live D435 HOLD_CAPTURE and mock_risk sub-reports`")
        else:
            lines.append("- visual evidence paths missing: `rgb, depth_raw, depth_vis, camera_info, odom, capture_meta, risk_point`")
        lines.append("")

    lines.extend(["## Risk Point Summary", ""])
    risk_points = report["risk_point_summary"]
    if risk_points:
        lines.extend(
            markdown_table(
                [
                    "risk_point_id",
                    "capture_id",
                    "label/category",
                    "depth_median_m",
                    "camera_point_xyz_m",
                    "depth_scale_m",
                    "bbox_valid_depth_ratio",
                    "missing_fields",
                ],
                [
                    [
                        item.get("risk_point_id"),
                        item.get("capture_id"),
                        item.get("label") or item.get("category"),
                        item.get("depth_median_m"),
                        item.get("camera_point_xyz_m"),
                        item.get("depth_scale_m"),
                        item.get("bbox_valid_depth_ratio"),
                        ", ".join(item.get("missing_fields") or []),
                    ]
                    for item in risk_points
                ],
            )
        )
    else:
        lines.append("- no risk point records present")

    lines.extend(["", "## Claim Boundary", ""])
    for claim in report["claim_boundary"]["allowed_claims"]:
        lines.append(f"- allowed: {claim}")
    for claim in report["claim_boundary"]["disallowed_claims"]:
        lines.append(f"- disallowed: {claim}")
    lines.extend(["", "## Next Recommended Step", "", report["next_recommended_step"], ""])
    return "\n".join(lines)


def render_claim_boundary(report: Dict[str, Any]) -> str:
    episode = report["episode_summary"]
    lines = [
        "# Claim Boundary",
        "",
        f"- episode_id: `{episode.get('episode_id')}`",
        f"- episode_kind: `{report.get('episode_kind')}`",
        f"- deterministic_baseline: `{report.get('deterministic_baseline')}`",
        f"- llm_used: `{report.get('llm_used')}`",
        f"- online_api_used: `{report.get('online_api_used')}`",
        f"- local_model_used: `{report.get('local_model_used')}`",
        "",
        "## Allowed Claims",
        "",
    ]
    for claim in report["claim_boundary"]["allowed_claims"]:
        lines.append(f"- {claim}")
    lines.extend(["", "## Disallowed Claims", ""])
    for claim in report["claim_boundary"]["disallowed_claims"]:
        lines.append(f"- {claim}")
    lines.extend(
        [
            "",
            "## LLM-A Boundary",
            "",
            "- Current LLM-A is a deterministic baseline.",
            "- `llm_used=false`.",
            "- `online_api_used=false`.",
            "- `local_model_used=false`.",
            "- Do not claim real LLM reasoning.",
            "- Do not claim a local large model is deployed.",
            "- Only claim rule-based generation from `episode_report.json` to risk report.",
            "",
        ]
    )
    return "\n".join(lines)


def render_readme(report: Dict[str, Any]) -> str:
    return f"""# LLM-A Risk Report Output

This directory contains deterministic baseline reports generated from:

```text
{report['source_episode_report']}
```

Files:

- `risk_report.md`
- `risk_report.json`
- `claim_boundary.md`
- `README.md`

This is not a real online LLM/API output. It is a rules-based report baseline for later LLM or UI integration.
"""


def render_root_readme() -> str:
    return """# LLM-A Deterministic Risk Report Baseline

This directory contains local deterministic LLM-A risk report outputs.

## Inputs

- `outputs/p4x_d435_hold_capture_v1/episode_report.json`
- `outputs/arm_a_mock_remove_obstacle_v1/episode_report.json`
- `outputs/arm_b3_no_load_sample_sequence_v1/hw_sequence_002/episode_report.json`
- `outputs/arm_c0_map_to_arm_dryrun_v1/offline_p4x/episode_report.json`
- `outputs/arm_c1_map_gated_no_load_once_v1/hw_candidate_001_with_live_precheck_003/episode_report.json`
- `outputs/step7_integrated_offline_flow_v1/offline_p4x_arm_c0/episode_report.json`
- `outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/dryrun_001/episode_report.json`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_negative_002/episode_report.json`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/episode_report.json`

## Outputs

- `p4x/risk_report.md`
- `p4x/risk_report.json`
- `p4x/claim_boundary.md`
- `arm_a/risk_report.md`
- `arm_a/risk_report.json`
- `arm_a/claim_boundary.md`
- `arm_b3_hw_sequence_002/risk_report.md`
- `arm_b3_hw_sequence_002/risk_report.json`
- `arm_b3_hw_sequence_002/claim_boundary.md`
- `arm_c0_map_to_arm_dryrun/risk_report.md`
- `arm_c0_map_to_arm_dryrun/risk_report.json`
- `arm_c0_map_to_arm_dryrun/claim_boundary.md`
- `arm_c1_map_gated_no_load_once/risk_report.md`
- `arm_c1_map_gated_no_load_once/risk_report.json`
- `arm_c1_map_gated_no_load_once/claim_boundary.md`
- `step7_integrated_offline_flow/risk_report.md`
- `step7_integrated_offline_flow/risk_report.json`
- `step7_integrated_offline_flow/claim_boundary.md`
- `step7c_guarded_d435_mockrisk_arm_noload/risk_report.md`
- `step7c_guarded_d435_mockrisk_arm_noload/risk_report.json`
- `step7c_guarded_d435_mockrisk_arm_noload/claim_boundary.md`
- `step7e2_guarded_red_rule_negative/risk_report.md`
- `step7e2_guarded_red_rule_negative/risk_report.json`
- `step7e2_guarded_red_rule_negative/claim_boundary.md`
- `step7e2_guarded_red_rule_arm_hw_fastdemo/risk_report.md`
- `step7e2_guarded_red_rule_arm_hw_fastdemo/risk_report.json`
- `step7e2_guarded_red_rule_arm_hw_fastdemo/claim_boundary.md`

## Boundary

- No online API is used.
- ROS is not started.
- Real mechanical-arm hardware is not accessed.
- Old evidence is not modified.
- `outputs/` is ignored by `.gitignore`; package evidence directories manually when needed.
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate deterministic Markdown/JSON risk report from episode_report.json."
    )
    parser.add_argument("--episode-report", required=True)
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source_path = Path(args.episode_report)
    output_dir = Path(args.output_dir)
    report = build_report(source_path)
    write_json(output_dir / "risk_report.json", report)
    write_text(output_dir / "risk_report.md", render_markdown(report))
    write_text(output_dir / "claim_boundary.md", render_claim_boundary(report))
    write_text(output_dir / "README.md", render_readme(report))
    write_text(output_dir.parent / "README.md", render_root_readme())
    print(
        json.dumps(
            {
                "ok": True,
                "episode_id": report["episode_summary"]["episode_id"],
                "status": report["episode_summary"]["status"],
                "episode_kind": report["episode_kind"],
                "output_dir": str(output_dir),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
