#!/usr/bin/env python3
"""Fail-closed discrete-grid arm clearance audit, perception, and execution."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.arm_safety import ArmSafety, MultiServoCommand  # noqa: E402
from src.discrete_arm_clearance import (  # noqa: E402
    EXECUTION_PHASES,
    GRID_IDS,
    audit_sequence_with_arm_safety,
    evaluate_base_zero_evidence,
    load_json,
    now_iso,
    sha256_file,
    summarize_marker_observations,
    validate_clearance_config,
    validate_sequence_document,
    verify_yolo_result,
)

DEFAULT_CONFIG = ROOT / "configs" / "arm_discrete_clearance_config.json"
DEFAULT_ARM_CONFIG = ROOT / "configs" / "arm_safety_config.json"
DEFAULT_OUTPUT_ROOT = ROOT / "outputs" / "arm_clearance"
CONFIRMATION_TEXT = "I_UNDERSTAND_THIS_MOVES_THE_ARM_ONCE"
PERCEPTION_ERROR_PREFIXES = (
    "near_camera.",
    "marker_detection.",
    "grid_regions.",
    "grid regions overlap",
    "reject_regions",
)


def run_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def append_jsonl(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(data, ensure_ascii=False) + "\n")


def resolve_repo_path(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def load_main_config(path: Path) -> dict[str, Any]:
    data = load_json(path)
    if not isinstance(data, dict):
        raise ValueError("clearance config must be a JSON object")
    return data


def detect_hsv_marker(frame: Any, marker: dict[str, Any]) -> dict[str, Any]:
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError as exc:
        return {"detected": False, "reason": f"opencv_or_numpy_unavailable:{exc}"}
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array(marker["hsv_lower"], dtype=np.uint8)
    upper = np.array(marker["hsv_upper"], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    kernel_size = max(1, int(marker.get("morphology_kernel_px", 3)))
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = float(marker["min_area_px"])
    max_area = float(marker.get("max_area_px", float("inf")))
    candidates = [(cv2.contourArea(contour), contour) for contour in contours]
    candidates = [(area, contour) for area, contour in candidates if min_area <= area <= max_area]
    if len(candidates) != 1:
        return {
            "detected": False,
            "reason": f"expected_one_marker_contour_got_{len(candidates)}",
            "candidate_count": len(candidates),
        }
    area, contour = candidates[0]
    moments = cv2.moments(contour)
    if not moments.get("m00"):
        return {"detected": False, "reason": "marker_zero_moment"}
    center = [moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]]
    return {
        "detected": True,
        "center_uv": [round(center[0], 3), round(center[1], 3)],
        "area_px": round(float(area), 3),
        "candidate_count": 1,
    }


def camera_device(value: Any) -> Any:
    if isinstance(value, int):
        return value
    text = str(value)
    return int(text) if text.isdigit() else text


def capture_marker_sequence(
    config: dict[str, Any],
    output_dir: Path,
    input_images: list[Path] | None,
) -> tuple[list[dict[str, Any]], Path | None, dict[str, Any]]:
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        return [], None, {"opened": False, "error": f"opencv_unavailable:{exc}"}
    camera = config["near_camera"]
    marker = config["marker_detection"]
    observations: list[dict[str, Any]] = []
    representative = None
    started = time.monotonic()
    frames: list[Any] = []
    source: dict[str, Any]
    if input_images:
        source = {"kind": "image_files", "paths": [str(path) for path in input_images], "opened": True}
        for path in input_images:
            frame = cv2.imread(str(path))
            if frame is None:
                observations.append({"detected": False, "reason": f"failed_to_read_image:{path}"})
            else:
                frames.append(frame)
    else:
        device = camera_device(camera["device"])
        source = {"kind": "v4l2_camera", "device": str(device), "opened": False}
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                source["error"] = "failed_to_open_near_camera"
                return [], None, source
            source["opened"] = True
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(camera.get("width", 640)))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(camera.get("height", 480)))
            if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
                cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, int(float(camera["frame_timeout_s"]) * 1000))
            deadline = time.monotonic() + float(camera["frame_timeout_s"])
            for _ in range(int(camera["stabilize_frames"])):
                if time.monotonic() > deadline:
                    source["error"] = "near_camera_stabilize_timeout"
                    return [], None, source
                cap.read()
            for _ in range(int(camera["capture_frames"])):
                if time.monotonic() > deadline:
                    observations.append({"detected": False, "reason": "near_camera_capture_timeout"})
                    break
                ok, frame = cap.read()
                if ok and frame is not None:
                    frames.append(frame)
                else:
                    observations.append({"detected": False, "reason": "camera_frame_read_failed"})
        finally:
            cap.release()

    for index, frame in enumerate(frames):
        observation = detect_hsv_marker(frame, marker)
        observation["frame_index"] = index
        observations.append(observation)
        if observation.get("detected"):
            representative = frame
    image_path = None
    if representative is not None:
        image_path = output_dir / "near_before.jpg"
        output_dir.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(image_path), representative):
            image_path = None
            source["write_error"] = "failed_to_write_near_before"
    source["frame_count"] = len(frames)
    source["elapsed_s"] = round(time.monotonic() - started, 3)
    return observations, image_path, source


def run_yolo_once(image_path: Path, config: dict[str, Any], output_dir: Path) -> dict[str, Any]:
    verification = config["object_verification"]
    model = resolve_repo_path(verification["model"])
    yolo_dir = output_dir / "yolo"
    command = [
        sys.executable,
        str(ROOT / "tools" / "run_yolo_inference_once.py"),
        "--image",
        str(image_path),
        "--model",
        str(model),
        "--output-dir",
        str(yolo_dir),
        "--imgsz",
        str(verification.get("imgsz", 640)),
        "--conf",
        str(verification.get("inference_confidence_floor", 0.15)),
        "--iou",
        str(verification.get("iou", 0.45)),
        "--max-det",
        str(verification.get("max_det", 10)),
        "--providers",
        str(verification.get("providers", "SpaceMITExecutionProvider,CPUExecutionProvider")),
    ]
    started = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=str(ROOT),
        check=False,
        capture_output=True,
        text=True,
        timeout=float(verification.get("timeout_s", 60.0)),
    )
    detection_path = yolo_dir / "risk_detection.json"
    result: dict[str, Any] = {
        "command": command,
        "returncode": completed.returncode,
        "elapsed_s": round(time.monotonic() - started, 3),
        "stdout": completed.stdout[-4000:],
        "stderr": completed.stderr[-4000:],
        "detection_path": str(detection_path),
        "accepted": False,
    }
    if completed.returncode == 0 and detection_path.exists():
        result.update(verify_yolo_result(load_json(detection_path), config))
    else:
        result["reason"] = "yolo_inference_failed"
    return result


def classify_command(args: argparse.Namespace) -> int:
    config_path = Path(args.config)
    config = load_main_config(config_path)
    errors, warnings = validate_clearance_config(config)
    episode_id = args.episode_id or f"discrete_camera_{run_stamp()}"
    output_dir = Path(args.output_dir) if args.output_dir else DEFAULT_OUTPUT_ROOT / episode_id
    output_dir.mkdir(parents=True, exist_ok=True)
    if errors:
        result = {"status": "blocked_config", "errors": errors, "warnings": warnings, "hardware_executed": False}
        write_json(output_dir / "near_detection.json", result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 2
    input_images = [Path(path) for path in args.input_image] if args.input_image else None
    observations, image_path, capture = capture_marker_sequence(config, output_dir, input_images)
    marker_summary = summarize_marker_observations(observations, config)
    if args.skip_yolo_development:
        yolo = {"accepted": False, "reason": "skipped_for_development_only"}
    elif image_path is None:
        yolo = {"accepted": False, "reason": "no_representative_image"}
    else:
        try:
            yolo = run_yolo_once(image_path, config, output_dir)
        except Exception as exc:
            yolo = {"accepted": False, "reason": f"yolo_exception:{exc}"}
    base_evidence = None
    if args.base_zero_evidence:
        base_evidence = evaluate_base_zero_evidence(Path(args.base_zero_evidence), args.base_zero_max_age_s)
    evidence_gates_ok = bool(
        args.confirm_arm_at_safe_observation
        and args.confirm_d435_inference_paused
        and base_evidence
        and base_evidence.get("allowed") is True
    )
    accepted = (
        marker_summary.get("decision") == "ACCEPT"
        and yolo.get("accepted") is True
        and evidence_gates_ok
    )
    development_ready = marker_summary.get("decision") == "ACCEPT" and (
        args.skip_yolo_development or not evidence_gates_ok
    )
    decision = "ACCEPT" if accepted else ("DEVELOPMENT_ONLY" if development_ready else "REJECT")
    result = {
        "schema_version": "arm_discrete_detection_v1",
        "generated_at": now_iso(),
        "episode_id": episode_id,
        "status": "classified" if accepted else "rejected",
        "decision": decision,
        "grid_id": marker_summary.get("grid_id"),
        "object_verified": yolo.get("accepted") is True,
        "marker": marker_summary,
        "yolo": yolo,
        "capture": capture,
        "observations": observations,
        "camera_mount_mode": config["camera_mount_mode"],
        "arm_observation_confirmed": bool(args.confirm_arm_at_safe_observation),
        "d435_inference_paused_confirmed": bool(args.confirm_d435_inference_paused),
        "base_zero_confirmed": bool(base_evidence and base_evidence.get("allowed") is True),
        "base_zero_evidence": {k: v for k, v in (base_evidence or {}).items() if k != "raw"},
        "hardware_executed": False,
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "config_path": str(config_path),
        "config_sha256": sha256_file(config_path),
        "image_path": str(image_path) if image_path else None,
        "errors": errors,
        "warnings": warnings,
    }
    write_json(output_dir / "near_detection.json", result)
    print(json.dumps({key: result[key] for key in ("status", "decision", "grid_id", "object_verified", "capture")}, ensure_ascii=False, indent=2))
    return 0 if accepted or decision == "DEVELOPMENT_ONLY" else 1


def load_detection_grid(path: Path, max_age_s: float) -> tuple[str | None, list[str], dict[str, Any]]:
    errors: list[str] = []
    data = load_json(path)
    if data.get("decision") != "ACCEPT" or data.get("object_verified") is not True:
        errors.append("detection evidence must have decision=ACCEPT and object_verified=true")
    if data.get("arm_observation_confirmed") is not True:
        errors.append("detection evidence must confirm the safe observation pose")
    if data.get("base_zero_confirmed") is not True:
        errors.append("detection evidence must confirm base_zero")
    if data.get("d435_inference_paused_confirmed") is not True:
        errors.append("detection evidence must confirm D435 inference was paused")
    grid_id = data.get("grid_id")
    if grid_id not in GRID_IDS:
        errors.append("detection evidence grid_id is invalid")
        grid_id = None
    try:
        generated = datetime.fromisoformat(str(data.get("generated_at")).replace("Z", "+00:00"))
        if generated.tzinfo is None:
            generated = generated.replace(tzinfo=timezone.utc)
        age_s = max(0.0, (datetime.now(timezone.utc) - generated).total_seconds())
        if age_s > max_age_s:
            errors.append(f"detection evidence is stale: {age_s:.1f}s > {max_age_s:.1f}s")
    except (TypeError, ValueError):
        errors.append("detection evidence generated_at is invalid")
    return grid_id, errors, data


def sequence_path_for_grid(config: dict[str, Any], grid_id: str) -> Path:
    return resolve_repo_path(config["sequence_paths"][grid_id])


def audit_command(args: argparse.Namespace) -> int:
    config_path = Path(args.config)
    arm_config_path = Path(args.arm_config)
    config = load_main_config(config_path)
    errors, warnings = validate_clearance_config(config)
    sequences: dict[str, Any] = {}
    for grid_id in GRID_IDS:
        path = sequence_path_for_grid(config, grid_id)
        if not path.exists():
            sequences[grid_id] = {"allowed": False, "errors": [f"missing sequence file: {path}"]}
            errors.append(f"missing sequence file for {grid_id}: {path}")
            continue
        sequence = load_json(path)
        static_errors = validate_sequence_document(sequence, grid_id)
        audit = audit_sequence_with_arm_safety(
            sequence,
            arm_config_path,
            "no_load",
            hardware_mode=False,
            workspace_policy=config.get("workspace") or {},
        )
        sequences[grid_id] = {
            "path": str(path),
            "sha256": sha256_file(path),
            "static_errors": static_errors,
            "arm_safety": audit,
            "allowed": not static_errors and audit.get("allowed") is True,
        }
    ready = not errors and all(item.get("allowed") for item in sequences.values())
    result = {
        "generated_at": now_iso(),
        "status": "ready" if ready else "blocked_unconfigured",
        "ready": ready,
        "hardware_executed": False,
        "config_errors": errors,
        "warnings": warnings,
        "sequences": sequences,
    }
    output = Path(args.output) if args.output else DEFAULT_OUTPUT_ROOT / f"audit_{run_stamp()}" / "audit.json"
    write_json(output, result)
    print(json.dumps({"status": result["status"], "output": str(output), "config_errors": errors}, ensure_ascii=False, indent=2))
    return 0 if ready else 2


def runtime_confirmation_errors(args: argparse.Namespace, config: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    hardware = config.get("hardware") or {}
    if hardware.get("enabled") is not True:
        errors.append("hardware.enabled is false in arm_discrete_clearance_config.json")
    if hardware.get("validated_stop_interface") is not True:
        errors.append("hardware.validated_stop_interface is not true")
    if hardware.get("live_base_monitor_verified") is not True:
        errors.append("hardware.live_base_monitor_verified is not true")
    for key in ("validated_stop_interface_evidence", "live_base_monitor_evidence"):
        value = hardware.get(key)
        if not value:
            errors.append(f"hardware.{key} is missing")
        elif not resolve_repo_path(value).exists():
            errors.append(f"hardware.{key} does not exist: {resolve_repo_path(value)}")
    required = {
        "--confirm-discrete-clearance-once": args.confirm_discrete_clearance_once,
        "--confirm-operator-supervision": args.confirm_operator_supervision,
        "--confirm-d435-inference-paused": args.confirm_d435_inference_paused,
        "--confirm-stop-interface-ready": args.confirm_stop_interface_ready,
        "--confirm-no-auto-home": args.confirm_no_auto_home,
    }
    missing = [name for name, value in required.items() if not value]
    if missing:
        errors.append("missing hardware confirmations: " + ", ".join(missing))
    if args.confirmation_text != CONFIRMATION_TEXT:
        errors.append("confirmation text does not match the required one-shot phrase")
    if args.execution_stage == "foam_contact" and not args.confirm_foam_only:
        errors.append("--confirm-foam-only is required for foam_contact")
    if args.execution_stage == "foam_contact" and hardware.get("foam_contact_enabled") is not True:
        errors.append("hardware.foam_contact_enabled is false")
    return errors


def enable_runtime_gates(safety: ArmSafety, execution_stage: str) -> None:
    gates = safety.config.setdefault("safety_gates", {})
    gates["arm_enabled"] = True
    gates["hardware_access_allowed"] = True
    gates["serial_write_allowed"] = True
    gates["dry_run"] = False
    gates["contact_allowed"] = execution_stage == "foam_contact"
    gates["obstacle_removal_allowed"] = False


def build_executable_frames(
    sequence: dict[str, Any], arm_config_path: Path, execution_stage: str
) -> tuple[ArmSafety, list[dict[str, Any]], list[str]]:
    safety = ArmSafety(str(arm_config_path))
    phase_result = safety.set_phase(EXECUTION_PHASES[execution_stage])
    errors: list[str] = []
    if not phase_result.allowed:
        errors.append(phase_result.reason)
    enable_runtime_gates(safety, execution_stage)
    safety.update_base_zero(True)
    safety.update_robot_driving(False)
    safety.heartbeat()
    frames: list[dict[str, Any]] = []
    for step in sequence["sequence"]:
        command = MultiServoCommand(
            servos={int(key): int(value) for key, value in step["servos"].items()},
            time_ms=int(step["duration_ms"]),
            label=str(step["name"]),
        )
        validation = safety.validate_all(command)
        if not validation.allowed:
            errors.append(f"{step['name']}: {validation.reason}")
            break
        frame = safety.build_move_frame(command)
        if not frame.get("serial_write_allowed_effective") or frame.get("frame_bytes") is None:
            errors.append(f"{step['name']}: executable serial frame was not built")
            break
        frames.append({"step": step, "frame": frame})
        safety.record_multi(command)
    return safety, frames, errors


def execute_hardware(
    args: argparse.Namespace,
    sequence: dict[str, Any],
    arm_config_path: Path,
    base_evidence_path: Path,
    output_dir: Path,
) -> dict[str, Any]:
    log_path = output_dir / "execution_log.jsonl"
    safety, frames, errors = build_executable_frames(sequence, arm_config_path, args.execution_stage)
    result: dict[str, Any] = {
        "hardware_executed": False,
        "serial_port_opened": False,
        "serial_bytes_written": 0,
        "completed_steps": 0,
        "stop_frame_attempted": False,
        "stop_frame_written": False,
        "automatic_home_attempted": False,
        "errors": errors,
    }
    if errors:
        return result
    stop_frame = safety.build_stop_frame().get("frame_bytes")
    handle = None
    current_step = None
    try:
        import serial  # type: ignore

        handle = serial.Serial(args.serial_port, args.baudrate, timeout=0.5)
        result["serial_port_opened"] = True
        for index, item in enumerate(frames):
            current_step = item["step"]
            base_check = evaluate_base_zero_evidence(base_evidence_path, args.base_zero_max_age_s)
            if not base_check["allowed"]:
                raise RuntimeError("base_zero_recheck_failed:" + ";".join(base_check["errors"]))
            safety.heartbeat()
            frame_bytes = item["frame"]["frame_bytes"]
            written = handle.write(frame_bytes)
            handle.flush()
            result["serial_bytes_written"] += int(written)
            if written != len(frame_bytes):
                raise RuntimeError(f"serial_write_length_mismatch:{written}!={len(frame_bytes)}")
            append_jsonl(
                log_path,
                {
                    "timestamp": now_iso(),
                    "event": "state_enter",
                    "state_enter_time": now_iso(),
                    "step_index": index,
                    "step_name": current_step["name"],
                    "phase": current_step["phase"],
                    "commanded_pulses": current_step["servos"],
                    "duration_ms": current_step["duration_ms"],
                    "frame_hex": item["frame"]["frame_hex"],
                },
            )
            deadline = time.monotonic() + float(current_step["duration_ms"]) / 1000.0
            while time.monotonic() < deadline:
                time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
                base_check = evaluate_base_zero_evidence(base_evidence_path, args.base_zero_max_age_s)
                if not base_check["allowed"]:
                    raise RuntimeError("base_zero_during_step_failed:" + ";".join(base_check["errors"]))
            result["completed_steps"] += 1
            append_jsonl(
                log_path,
                {
                    "timestamp": now_iso(),
                    "event": "state_exit",
                    "state_exit_time": now_iso(),
                    "step_index": index,
                    "step_name": current_step["name"],
                    "phase": current_step["phase"],
                    "status": "completed",
                },
            )
            if args.execution_stage == "foam_contact" and current_step["phase"] == "short_lift":
                response = input("Type GRASP_CONFIRMED to continue after supervised short lift: ").strip()
                if response != "GRASP_CONFIRMED":
                    raise RuntimeError("operator_grasp_not_confirmed")
        result["hardware_executed"] = result["completed_steps"] == len(frames)
    except BaseException as exc:
        result["errors"].append(f"hardware_execution_failed:{type(exc).__name__}:{exc}")
        append_jsonl(
            log_path,
            {
                "timestamp": now_iso(),
                "event": "failed_safe",
                "step_name": current_step.get("name") if current_step else None,
                "error": str(exc),
                "automatic_home_attempted": False,
            },
        )
        if handle is not None and stop_frame is not None:
            result["stop_frame_attempted"] = True
            try:
                written = handle.write(stop_frame)
                handle.flush()
                result["serial_bytes_written"] += int(written)
                result["stop_frame_written"] = written == len(stop_frame)
            except Exception as stop_exc:
                result["errors"].append(f"stop_frame_failed:{stop_exc}")
    finally:
        if handle is not None:
            handle.close()
    return result


def run_command(args: argparse.Namespace) -> int:
    config_path = Path(args.config)
    arm_config_path = Path(args.arm_config)
    config = load_main_config(config_path)
    config_errors, warnings = validate_clearance_config(config)
    hardware_requested = bool(args.enable_hardware_write)
    detection: dict[str, Any] | None = None
    grid_id: str | None = None
    errors: list[str] = []

    if args.detection_evidence:
        grid_id, detection_errors, detection = load_detection_grid(
            Path(args.detection_evidence),
            float((config.get("object_verification") or {}).get("evidence_max_age_s", 60.0)),
        )
        errors.extend(detection_errors)
        if detection.get("camera_mount_mode") != config.get("camera_mount_mode"):
            errors.append("detection evidence camera mount mode does not match the active configuration")
        if detection.get("config_sha256") != sha256_file(config_path):
            errors.append("detection evidence was not produced from the active clearance configuration")
    elif args.grid:
        grid_id = args.grid
        if not args.manual_grid_for_no_load or args.execution_stage != "no_load" or hardware_requested:
            errors.append("manual --grid is allowed only for non-hardware no_load with --manual-grid-for-no-load")
    else:
        errors.append("--detection-evidence is required unless using manual no-load dry-run")

    allow_uncalibrated_perception = bool(
        args.grid and args.manual_grid_for_no_load and args.execution_stage == "no_load" and not hardware_requested
    )
    for error in config_errors:
        if allow_uncalibrated_perception and error.startswith(PERCEPTION_ERROR_PREFIXES):
            warnings.append("manual no-load ignored perception config error: " + error)
        else:
            errors.append(error)

    episode_id = args.episode_id or f"discrete_arm_{run_stamp()}"
    output_dir = Path(args.output_dir) if args.output_dir else DEFAULT_OUTPUT_ROOT / episode_id
    output_dir.mkdir(parents=True, exist_ok=True)
    sequence = None
    sequence_path = None
    audit: dict[str, Any] = {"allowed": False, "errors": ["sequence not loaded"]}
    if grid_id in GRID_IDS:
        sequence_path = sequence_path_for_grid(config, grid_id)
        if not sequence_path.exists():
            errors.append(f"sequence file does not exist: {sequence_path}")
        else:
            sequence = load_json(sequence_path)
            static_errors = validate_sequence_document(sequence, grid_id)
            errors.extend(static_errors)
            audit = audit_sequence_with_arm_safety(
                sequence,
                arm_config_path,
                args.execution_stage,
                hardware_mode=hardware_requested,
                workspace_policy=config.get("workspace") or {},
            )
            errors.extend(audit.get("errors") or [])
            warnings.extend(audit.get("warnings") or [])

    base_evidence = None
    if hardware_requested:
        errors.extend(runtime_confirmation_errors(args, config))
        if detection is None:
            errors.append("hardware execution requires accepted detection evidence")
        if not args.base_zero_evidence:
            errors.append("hardware execution requires --base-zero-evidence")
        else:
            base_evidence = evaluate_base_zero_evidence(Path(args.base_zero_evidence), args.base_zero_max_age_s)
            errors.extend(base_evidence["errors"])
        if not Path(args.serial_port).exists():
            errors.append(f"serial port does not exist: {args.serial_port}")

    if sequence is not None:
        write_json(output_dir / "selected_sequence.json", sequence)
    if detection is not None:
        write_json(output_dir / "near_detection.json", detection)
    plan = {
        "generated_at": now_iso(),
        "episode_id": episode_id,
        "grid_id": grid_id,
        "execution_stage": args.execution_stage,
        "hardware_requested": hardware_requested,
        "hardware_executed": False,
        "sequence_path": str(sequence_path) if sequence_path else None,
        "sequence_sha256": sha256_file(sequence_path) if sequence_path and sequence_path.exists() else None,
        "arm_safety_audit": audit,
        "base_zero_evidence": {k: v for k, v in (base_evidence or {}).items() if k != "raw"},
        "errors": sorted(set(errors)),
        "warnings": sorted(set(warnings)),
    }
    write_json(output_dir / "plan.json", plan)
    if errors or sequence is None:
        result = {
            "status": "blocked_safe",
            "hardware_executed": False,
            "serial_port_opened": False,
            "serial_bytes_written": 0,
            "automatic_home_attempted": False,
            "errors": sorted(set(errors)),
            "output_dir": str(output_dir),
        }
        write_json(output_dir / "result.json", result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 2

    if not hardware_requested:
        result = {
            "status": "succeeded_dry_run",
            "grid_id": grid_id,
            "execution_stage": args.execution_stage,
            "hardware_executed": False,
            "serial_port_opened": False,
            "serial_bytes_written": 0,
            "automatic_home_attempted": False,
            "output_dir": str(output_dir),
            "warnings": sorted(set(warnings)),
        }
        write_json(output_dir / "result.json", result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0

    hardware_result = execute_hardware(
        args,
        sequence,
        arm_config_path,
        Path(args.base_zero_evidence),
        output_dir,
    )
    status = "succeeded" if hardware_result["hardware_executed"] else "failed_safe"
    result = {
        "status": status,
        "grid_id": grid_id,
        "execution_stage": args.execution_stage,
        "automatic_retry": False,
        "automatic_home_attempted": False,
        "output_dir": str(output_dir),
        **hardware_result,
    }
    write_json(output_dir / "result.json", result)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if status == "succeeded" else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--arm-config", default=str(DEFAULT_ARM_CONFIG))
    subparsers = parser.add_subparsers(dest="command", required=True)

    audit = subparsers.add_parser("audit", help="Audit configuration and all three sequences")
    audit.add_argument("--output")
    audit.set_defaults(func=audit_command)

    classify = subparsers.add_parser("classify", help="Run near-camera marker and YOLO classification only")
    classify.add_argument("--episode-id")
    classify.add_argument("--output-dir")
    classify.add_argument("--input-image", action="append", default=[])
    classify.add_argument("--skip-yolo-development", action="store_true")
    classify.add_argument("--base-zero-evidence")
    classify.add_argument("--base-zero-max-age-s", type=float, default=10.0)
    classify.add_argument("--confirm-arm-at-safe-observation", action="store_true")
    classify.add_argument("--confirm-d435-inference-paused", action="store_true")
    classify.set_defaults(func=classify_command)

    run = subparsers.add_parser("run", help="Validate or execute one selected discrete sequence")
    run.add_argument("--episode-id")
    run.add_argument("--output-dir")
    run.add_argument("--grid", choices=GRID_IDS)
    run.add_argument("--detection-evidence")
    run.add_argument("--manual-grid-for-no-load", action="store_true")
    run.add_argument("--execution-stage", choices=tuple(EXECUTION_PHASES), default="no_load")
    run.add_argument("--base-zero-evidence")
    run.add_argument("--base-zero-max-age-s", type=float, default=10.0)
    run.add_argument("--serial-port", default="/dev/arm_bus")
    run.add_argument("--baudrate", type=int, default=9600)
    run.add_argument("--enable-hardware-write", action="store_true")
    run.add_argument("--confirm-discrete-clearance-once", action="store_true")
    run.add_argument("--confirm-operator-supervision", action="store_true")
    run.add_argument("--confirm-d435-inference-paused", action="store_true")
    run.add_argument("--confirm-stop-interface-ready", action="store_true")
    run.add_argument("--confirm-no-auto-home", action="store_true")
    run.add_argument("--confirm-foam-only", action="store_true")
    run.add_argument("--confirmation-text", default="")
    run.set_defaults(func=run_command)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except Exception as exc:
        payload = {
            "status": "failed_safe",
            "hardware_executed": False,
            "automatic_home_attempted": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
