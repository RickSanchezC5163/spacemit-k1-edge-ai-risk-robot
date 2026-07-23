"""Pure validation and perception helpers for discrete arm clearance."""

from __future__ import annotations

import hashlib
import json
import math
import statistics
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

try:
    from .arm_safety import ArmSafety, MultiServoCommand
except ImportError:  # pragma: no cover - supports direct imports from tools/
    from arm_safety import ArmSafety, MultiServoCommand


GRID_IDS = ("LEFT", "CENTER", "RIGHT")
MOUNT_MODES = ("BODY_FIXED", "ARM_MOVING_FIXED_OBSERVATION")
EXECUTION_PHASES = {
    "no_load": "arm_b3_full_no_load_sequence",
    "foam_contact": "arm_d_light_contact_foam",
}
REQUIRED_PHASE_ORDER = (
    "observation",
    "pre_grasp",
    "grasp",
    "short_lift",
    "lift",
    "transfer",
    "place",
    "retreat",
    "observation_verify",
    "home",
)


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _box(value: Any) -> tuple[float, float, float, float] | None:
    if not isinstance(value, list) or len(value) != 4:
        return None
    if not all(isinstance(item, (int, float)) for item in value):
        return None
    x1, y1, x2, y2 = (float(item) for item in value)
    if x1 >= x2 or y1 >= y2:
        return None
    return x1, y1, x2, y2


def point_in_box(center_uv: tuple[float, float], box: Iterable[float]) -> bool:
    u, v = center_uv
    x1, y1, x2, y2 = box
    return x1 <= u <= x2 and y1 <= v <= y2


def boxes_overlap(a: Iterable[float], b: Iterable[float]) -> bool:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    return max(ax1, bx1) < min(ax2, bx2) and max(ay1, by1) < min(ay2, by2)


def validate_clearance_config(config: dict[str, Any]) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []
    if config.get("schema_version") != "arm_discrete_clearance_v1":
        errors.append("schema_version must be arm_discrete_clearance_v1")
    mount_mode = config.get("camera_mount_mode")
    if mount_mode not in MOUNT_MODES:
        errors.append(f"camera_mount_mode must be one of {MOUNT_MODES}")

    camera = config.get("near_camera") or {}
    if camera.get("device") in (None, ""):
        errors.append("near_camera.device is required")
    for key in ("stabilize_frames", "capture_frames"):
        if not isinstance(camera.get(key), int) or int(camera[key]) <= 0:
            errors.append(f"near_camera.{key} must be a positive integer")
    if not isinstance(camera.get("frame_timeout_s"), (int, float)) or float(camera["frame_timeout_s"]) <= 0:
        errors.append("near_camera.frame_timeout_s must be positive")
    if mount_mode == "ARM_MOVING_FIXED_OBSERVATION":
        if camera.get("mount_location") != "ARM_TIP":
            errors.append("near_camera.mount_location must be ARM_TIP for the configured mount mode")
        if camera.get("localization_valid_only_at_observation_pose") is not True:
            errors.append(
                "near_camera.localization_valid_only_at_observation_pose must be true for an arm-tip camera"
            )

    marker = config.get("marker_detection") or {}
    if marker.get("method") != "HSV_CONTOUR":
        errors.append("marker_detection.method must be HSV_CONTOUR in v1")
    if marker.get("calibrated") is not True:
        errors.append("marker_detection.calibrated must be true before classification")
    for key, limit in (("hsv_lower", (179, 255, 255)), ("hsv_upper", (179, 255, 255))):
        value = marker.get(key)
        if not isinstance(value, list) or len(value) != 3 or not all(isinstance(x, int) for x in value):
            errors.append(f"marker_detection.{key} must contain three integers")
            continue
        if any(x < 0 or x > maximum for x, maximum in zip(value, limit)):
            errors.append(f"marker_detection.{key} is outside OpenCV HSV limits")
    if not isinstance(marker.get("min_area_px"), (int, float)) or marker.get("min_area_px", 0) <= 0:
        errors.append("marker_detection.min_area_px must be positive")
    if not isinstance(marker.get("stable_frames_required"), int) or marker.get("stable_frames_required", 0) <= 0:
        errors.append("marker_detection.stable_frames_required must be positive")

    regions = config.get("grid_regions") or {}
    parsed_regions: dict[str, tuple[float, float, float, float]] = {}
    for grid_id in GRID_IDS:
        region = regions.get(grid_id) or {}
        parsed = _box(region.get("xyxy_px"))
        if parsed is None:
            errors.append(f"grid_regions.{grid_id}.xyxy_px must be a calibrated [x1,y1,x2,y2] box")
        else:
            parsed_regions[grid_id] = parsed
    for index, first in enumerate(GRID_IDS):
        for second in GRID_IDS[index + 1 :]:
            if first in parsed_regions and second in parsed_regions:
                if boxes_overlap(parsed_regions[first], parsed_regions[second]):
                    errors.append(f"grid regions overlap: {first} and {second}")

    reject_regions = config.get("reject_regions")
    if not isinstance(reject_regions, list) or not reject_regions:
        errors.append("reject_regions must contain explicit boundary reject boxes")
    else:
        for index, region in enumerate(reject_regions):
            if _box((region or {}).get("xyxy_px")) is None:
                errors.append(f"reject_regions[{index}].xyxy_px is invalid")

    sequence_paths = config.get("sequence_paths") or {}
    for grid_id in GRID_IDS:
        if not sequence_paths.get(grid_id):
            errors.append(f"sequence_paths.{grid_id} is required")

    base_zero = config.get("base_zero") or {}
    if base_zero.get("required") is not True:
        errors.append("base_zero.required must remain true")
    if float(base_zero.get("stable_duration_s", 0.0)) <= 0:
        errors.append("base_zero.stable_duration_s must be positive")

    hardware = config.get("hardware") or {}
    if hardware.get("one_shot_confirmation_required") is not True:
        errors.append("hardware.one_shot_confirmation_required must remain true")
    if hardware.get("automatic_retry") is not False:
        errors.append("hardware.automatic_retry must remain false")
    if hardware.get("automatic_home_on_fault") is not False:
        errors.append("hardware.automatic_home_on_fault must remain false")
    if hardware.get("enabled") is True:
        warnings.append("hardware.enabled=true; explicit CLI confirmations are still required")
        if hardware.get("validated_stop_interface") is not True:
            errors.append("hardware.validated_stop_interface must be true before hardware enablement")
        if not hardware.get("validated_stop_interface_evidence"):
            errors.append("hardware.validated_stop_interface_evidence is required before hardware enablement")
        if hardware.get("live_base_monitor_verified") is not True:
            errors.append("hardware.live_base_monitor_verified must be true before hardware enablement")
        if not hardware.get("live_base_monitor_evidence"):
            errors.append("hardware.live_base_monitor_evidence is required before hardware enablement")
    return errors, warnings


def classify_grid(center_uv: tuple[float, float], config: dict[str, Any]) -> dict[str, Any]:
    for region in config.get("reject_regions") or []:
        box = _box((region or {}).get("xyxy_px"))
        if box and point_in_box(center_uv, box):
            return {
                "decision": "REJECT",
                "grid_id": None,
                "reason": f"center_in_reject_region:{region.get('name', 'unnamed')}",
            }

    matches: list[str] = []
    for grid_id in GRID_IDS:
        box = _box(((config.get("grid_regions") or {}).get(grid_id) or {}).get("xyxy_px"))
        if box and point_in_box(center_uv, box):
            matches.append(grid_id)
    if len(matches) == 1:
        return {"decision": "ACCEPT", "grid_id": matches[0], "reason": "center_in_single_grid"}
    if len(matches) > 1:
        return {"decision": "REJECT", "grid_id": None, "reason": "center_in_overlapping_grids"}
    return {"decision": "REJECT", "grid_id": None, "reason": "center_outside_calibrated_grids"}


def summarize_marker_observations(
    observations: list[dict[str, Any]], config: dict[str, Any]
) -> dict[str, Any]:
    marker = config["marker_detection"]
    valid = [item for item in observations if item.get("detected") and item.get("center_uv")]
    required = int(marker["stable_frames_required"])
    if len(valid) < required:
        return {
            "decision": "REJECT",
            "grid_id": None,
            "reason": f"insufficient_stable_frames:{len(valid)}<{required}",
            "valid_frames": len(valid),
        }
    selected = valid[-required:]
    us = [float(item["center_uv"][0]) for item in selected]
    vs = [float(item["center_uv"][1]) for item in selected]
    center = (statistics.median(us), statistics.median(vs))
    jitter = max(math.hypot(u - center[0], v - center[1]) for u, v in zip(us, vs))
    max_jitter = float(marker.get("max_center_jitter_px", 8.0))
    if jitter > max_jitter:
        return {
            "decision": "REJECT",
            "grid_id": None,
            "reason": f"marker_jitter_too_large:{jitter:.2f}>{max_jitter:.2f}",
            "marker_center_uv": [round(center[0], 2), round(center[1], 2)],
            "max_jitter_px": round(jitter, 3),
            "valid_frames": len(valid),
        }
    classification = classify_grid(center, config)
    classification.update(
        {
            "marker_center_uv": [round(center[0], 2), round(center[1], 2)],
            "marker_area_px": round(statistics.median(float(item["area_px"]) for item in selected), 2),
            "max_jitter_px": round(jitter, 3),
            "valid_frames": len(valid),
            "stable_frames": required,
        }
    )
    return classification


def verify_yolo_result(data: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    verification = config.get("object_verification") or {}
    expected = str(verification.get("yolo_class") or "")
    threshold = float(verification.get("confidence_min", 1.0))
    detections = data.get("detections") or []
    matching = []
    for detection in detections:
        if not isinstance(detection, dict):
            continue
        class_name = str(detection.get("class_name") or detection.get("label") or "")
        confidence = float(detection.get("confidence") or 0.0)
        if class_name == expected and confidence >= threshold:
            matching.append({"class_name": class_name, "confidence": confidence, "bbox_xywh": detection.get("bbox_xywh")})
    require_single = verification.get("require_single_detection", True) is True
    accepted = len(matching) == 1 if require_single else bool(matching)
    return {
        "accepted": accepted,
        "expected_class": expected,
        "confidence_min": threshold,
        "matching_detections": matching,
        "total_detection_count": len(detections),
        "reason": "verified" if accepted else "expected_single_verified_object_not_found",
    }


def validate_sequence_document(sequence: dict[str, Any], grid_id: str) -> list[str]:
    errors: list[str] = []
    if sequence.get("schema_version") != "arm_discrete_sequence_v1":
        errors.append("sequence schema_version must be arm_discrete_sequence_v1")
    if sequence.get("grid_id") != grid_id:
        errors.append(f"sequence grid_id must be {grid_id}")
    if sequence.get("verified") is not True:
        errors.append("sequence.verified must be true")
    if sequence.get("calibration_required") is not False:
        errors.append("sequence.calibration_required must be false")
    for key in ("calibration_id", "verified_by", "verified_at"):
        if not sequence.get(key):
            errors.append(f"sequence.{key} is required")
    if sequence.get("interpolation_allowed") is not False:
        errors.append("sequence.interpolation_allowed must remain false")

    steps = sequence.get("sequence")
    if not isinstance(steps, list) or not steps:
        errors.append("sequence.sequence must be non-empty")
        return errors
    observed_phases: list[str] = []
    for index, step in enumerate(steps):
        if not isinstance(step, dict):
            errors.append(f"sequence step {index} must be an object")
            continue
        phase = step.get("phase")
        if phase not in REQUIRED_PHASE_ORDER and phase not in ("prepare", "release", "grasp_confirm"):
            errors.append(f"sequence step {index} has unsupported phase {phase!r}")
        if phase in REQUIRED_PHASE_ORDER and phase not in observed_phases:
            observed_phases.append(phase)
        if not step.get("name"):
            errors.append(f"sequence step {index} has no name")
        duration = step.get("duration_ms")
        if not isinstance(duration, int) or duration <= 0:
            errors.append(f"sequence step {index} duration_ms must be positive")
        servos = step.get("servos")
        if not isinstance(servos, dict) or not servos:
            errors.append(f"sequence step {index} servos must be non-empty")
            continue
        for servo_id, pulse in servos.items():
            if str(servo_id) not in {"1", "2", "3", "4", "5"}:
                errors.append(f"sequence step {index} has invalid servo ID {servo_id}")
            if not isinstance(pulse, int):
                errors.append(f"sequence step {index} servo {servo_id} pulse must be an integer")
            elif pulse == 0:
                errors.append(f"sequence step {index} servo {servo_id} uses forbidden placeholder pulse 0")
    if observed_phases != list(REQUIRED_PHASE_ORDER):
        errors.append(
            "sequence phases must contain the required order: " + ",".join(REQUIRED_PHASE_ORDER)
        )
    return errors


def audit_sequence_with_arm_safety(
    sequence: dict[str, Any],
    arm_config_path: Path,
    execution_stage: str,
    hardware_mode: bool,
    workspace_policy: dict[str, Any],
) -> dict[str, Any]:
    errors = validate_sequence_document(sequence, str(sequence.get("grid_id") or ""))
    warnings: list[str] = []
    records: list[dict[str, Any]] = []
    if errors:
        return {"allowed": False, "errors": errors, "warnings": warnings, "steps": records}
    if execution_stage not in EXECUTION_PHASES:
        return {"allowed": False, "errors": [f"unsupported execution_stage={execution_stage}"], "warnings": [], "steps": []}

    arm_config = load_json(arm_config_path)
    gripper = arm_config.get("joints", {}).get("5", {})
    configured_open = gripper.get("gripper_open_pulse")
    soft_lo = gripper.get("soft_limit_lower_pulse")
    soft_hi = gripper.get("soft_limit_upper_pulse")
    if not isinstance(configured_open, int) or not isinstance(soft_lo, int) or not isinstance(soft_hi, int):
        errors.append("gripper open pulse or soft limits are missing")
    elif not soft_lo <= configured_open <= soft_hi:
        errors.append(
            f"configured gripper_open_pulse={configured_open} is outside soft limits [{soft_lo},{soft_hi}]"
        )

    safety = ArmSafety(str(arm_config_path))
    phase_result = safety.set_phase(EXECUTION_PHASES[execution_stage])
    if not phase_result.allowed:
        errors.append(phase_result.reason)
        return {"allowed": False, "errors": errors, "warnings": warnings, "steps": records}
    safety.update_base_zero(True)
    safety.update_robot_driving(False)
    safety.heartbeat()

    require_workspace = workspace_policy.get("hardware_policy", "REQUIRE_ESTIMATE_PASS") == "REQUIRE_ESTIMATE_PASS"
    override_enabled = workspace_policy.get("allow_preverified_override") is True
    sequence_override = (
        sequence.get("workspace_verified") is True
        and bool(sequence.get("workspace_evidence"))
        and override_enabled
    )

    for index, step in enumerate(sequence["sequence"]):
        command = MultiServoCommand(
            servos={int(key): int(value) for key, value in step["servos"].items()},
            time_ms=int(step["duration_ms"]),
            label=str(step["name"]),
        )
        results = safety.validate_multi(command)
        step_errors: list[str] = []
        step_warnings: list[str] = []
        for result in results:
            if not result.allowed:
                step_errors.append(f"servo {result.servo_id}: {result.reason}")
            if result.rule_checks.get("L3_soft_limit") is False:
                step_errors.append(f"servo {result.servo_id}: soft limit failed")
            if result.rule_checks.get("L5_workspace") is False:
                message = f"servo {result.servo_id}: workspace estimate failed"
                if hardware_mode and require_workspace and not sequence_override:
                    step_errors.append(message)
                else:
                    step_warnings.append(message)
            step_warnings.extend(result.warnings)
        records.append(
            {
                "index": index,
                "name": step["name"],
                "phase": step["phase"],
                "allowed": not step_errors,
                "errors": step_errors,
                "warnings": sorted(set(step_warnings)),
                "frame_review": safety.build_move_frame(command)["frame_hex"],
            }
        )
        warnings.extend(f"{step['name']}: {item}" for item in step_warnings)
        if step_errors:
            errors.extend(f"{step['name']}: {item}" for item in step_errors)
            break
        safety.record_multi(command)

    home = arm_config.get("poses", {}).get("safe_idle_home_like_6b", {}).get("servos", {})
    last_servos = (sequence["sequence"][-1].get("servos") or {}) if sequence.get("sequence") else {}
    if {str(k): int(v) for k, v in last_servos.items()} != {str(k): int(v) for k, v in home.items()}:
        errors.append("last sequence step must command the complete safe_idle_home_like_6b pose")
    return {
        "allowed": not errors,
        "errors": errors,
        "warnings": sorted(set(warnings)),
        "steps": records,
        "phase": EXECUTION_PHASES[execution_stage],
        "workspace_preverified_override_used": sequence_override,
    }


def evaluate_base_zero_evidence(path: Path, max_age_s: float) -> dict[str, Any]:
    result = {"allowed": False, "errors": [], "path": str(path), "raw": None}
    try:
        raw = load_json(path)
    except Exception as exc:
        result["errors"].append(f"failed to load base-zero evidence: {exc}")
        return result
    result["raw"] = raw
    if raw.get("evidence_type") != "live_base_zero_observation":
        result["errors"].append("evidence_type must be live_base_zero_observation")
    if raw.get("valid_for_arm_c1_hardware") is not True:
        result["errors"].append("valid_for_arm_c1_hardware must be true")
    nested_base = raw.get("base_zero") if isinstance(raw.get("base_zero"), dict) else {}
    base_zero = raw.get(
        "base_zero_ok_before_arm",
        raw.get("base_zero_ok", nested_base.get("base_zero_ok")),
    )
    if base_zero is not True:
        result["errors"].append("base_zero_ok_before_arm must be true")
    published_cmd_vel = raw.get("published_cmd_vel", nested_base.get("published_cmd_vel"))
    if published_cmd_vel is not False:
        result["errors"].append("published_cmd_vel must be false")
    generated_at = raw.get("generated_at")
    try:
        generated = datetime.fromisoformat(str(generated_at).replace("Z", "+00:00"))
        if generated.tzinfo is None:
            generated = generated.replace(tzinfo=timezone.utc)
        age_s = max(0.0, (datetime.now(timezone.utc) - generated).total_seconds())
        result["age_s"] = age_s
        if age_s > max_age_s:
            result["errors"].append(f"base-zero evidence is stale: {age_s:.1f}s > {max_age_s:.1f}s")
    except (TypeError, ValueError):
        result["errors"].append("base-zero evidence generated_at is missing or invalid")
    result["allowed"] = not result["errors"]
    return result
