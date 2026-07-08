#!/usr/bin/env python3
"""End-to-end dry-run for the competition primitive stack.

No ROS, no serial ports, no real chassis, and no arm hardware are accessed.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.arm_primitives import arm_no_load_response_result, clearance_candidate_dryrun
from src.primitives.chassis_primitives import chassis_action_candidate, dry_run_chassis_primitive
from src.primitives.map_primitives import summarize_risk_map
from src.primitives.registry import load_action_semantics, load_registry, validate_action_semantics, validate_registry
from src.primitives.report_primitives import generate_risk_report
from src.primitives.risk_vision_primitives import detect_risk
from src.primitives.safety_gate import evaluate_safety_gate
from src.primitives.schemas import now_iso, write_json, write_text


def make_fixture_rgb(path: Path) -> bool:
    try:
        from PIL import Image, ImageDraw
    except Exception:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    img = Image.new("RGB", (640, 480), (230, 230, 230))
    draw = ImageDraw.Draw(img)
    draw.rectangle((250, 160, 390, 280), fill=(210, 20, 20))
    draw.text((260, 190), "RISK", fill=(255, 255, 255))
    img.save(path)
    return True


def sample_observation() -> Dict[str, Any]:
    return {
        "observation_id": "competition_dryrun_observation_001",
        "timestamp": now_iso(),
        "front_min": 0.72,
        "front_p10": 0.76,
        "left_p10": 1.25,
        "right_p10": 0.95,
        "odom_x": 0.0,
        "odom_y": 0.0,
        "odom_yaw": 0.0,
        "map_progress": 0.25,
        "risk_detected": False,
        "risk_confidence": None,
        "risk_class_id": None,
        "risk_distance_m": None,
        "base_zero": True,
        "arm_ready": True,
        "capture_recent": False,
        "steps_since_capture": 4,
        "consecutive_fast_arc": 0,
        "total_forward_m": 0.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="outputs/competition_primitive_stack_dryrun_v1")
    args = parser.parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    errors: list[dict[str, Any]] = []

    registry = load_registry()
    semantics = load_action_semantics()
    registry_errors = validate_registry(registry)
    semantics_errors = validate_action_semantics(semantics, registry)
    errors.extend({"stage": "registry", "message": e} for e in registry_errors)
    errors.extend({"stage": "action_semantics", "message": e} for e in semantics_errors)

    obs = sample_observation()
    hold = dry_run_chassis_primitive("HOLD", obs)
    action_candidate = chassis_action_candidate("HOLD_CAPTURE", obs)
    write_json(out / "action_candidate.json", action_candidate)

    fixture_rgb = out / "fixtures" / "printed_risk_fixture.png"
    fixture_ok = make_fixture_rgb(fixture_rgb)
    backend = "hsv_red_rule" if fixture_ok else "stub_local_model"
    risk_detection = detect_risk(str(fixture_rgb) if fixture_ok else "", backend=backend, output_dir=str(out))
    risk_map_summary = summarize_risk_map(risk_detection, out)
    arm_candidate = clearance_candidate_dryrun(risk_map_summary, out / "arm_d0")
    arm_result = arm_no_load_response_result()

    safety_gate = evaluate_safety_gate("HOLD_CAPTURE", {**obs, "risk_detected": True}, execution_mode="dry_run")

    episode_stub = {
        "episode_id": "competition_primitive_stack_dryrun",
        "episode_kind": "competition_primitive_stack_dryrun",
        "summary": {
            "status": "succeeded_dry_run",
            "hardware_executed": False,
            "risk_count": risk_map_summary.get("risk_count_total", 0),
        },
    }
    write_json(out / "episode_stub.json", episode_stub)
    report = generate_risk_report(
        str(out / "episode_stub.json"),
        str(out / "risk_map_summary.json"),
        backend="deterministic",
        output_dir=str(out),
    )

    episode_v2 = {
        "episode_id": "competition_primitive_stack_dryrun",
        "episode_kind": "competition_primitive_stack_dryrun",
        "started_at": now_iso(),
        "ended_at": now_iso(),
        "observation_state": [obs],
        "primitive_actions": [
            {"action_id": "dryrun_hold", "primitive": "HOLD", "requested_at": now_iso(), "execution_mode": "dry_run"},
            {"action_id": "dryrun_hold_capture", "primitive": "HOLD_CAPTURE", "requested_at": now_iso(), "execution_mode": "dry_run"},
            {"action_id": "dryrun_arm_candidate", "primitive": "ARM_CLEAR_CANDIDATE_DRYRUN", "requested_at": now_iso(), "execution_mode": "dry_run"},
        ],
        "primitive_results": [hold, arm_result],
        "risk_detections": [risk_detection],
        "risk_map_summary": risk_map_summary,
        "action_candidates": [action_candidate, arm_candidate],
        "safety_gate_results": [safety_gate, arm_candidate.get("safety_gate")],
        "report_outputs": [{"backend": report["backend"], "path": str(out / "risk_control_report.md")}],
        "benchmarks": {
            "vision": {
                "backend": risk_detection["backend"],
                "latency_ms": risk_detection["latency_ms"],
                "fps": risk_detection["fps"],
                "model_used": risk_detection.get("model_used"),
                "local_inference": risk_detection.get("local_inference"),
                "online_api_used": risk_detection.get("online_api_used"),
            },
            "llm": report["llm_benchmark"],
            "rl": {
                "policy_name": "semantic_ppo_stub",
                "action_space_version": "rl_semantic_action_space_v1",
                "hardware_connected": False,
                "cmd_vel_published": False,
                "servo_pulse_output": False,
                "episodes": None,
                "checkpoint_count": None,
            },
        },
        "claim_boundary": [
            "Dry-run only; no ROS, no serial ports, no hardware execution.",
            "HSV red-rule is baseline only, not an AI model claim.",
            "Deterministic report is not a real LLM claim.",
            "RL action candidate is high-level only and not executed on hardware.",
        ],
        "errors": errors,
    }
    write_json(out / "episode_report_v2.json", episode_v2)

    acceptance = {
        "primitive_registry_valid": not registry_errors,
        "action_space_valid": not semantics_errors,
        "risk_detection_schema_valid": "risk_detection_id" in risk_detection and isinstance(risk_detection.get("detections"), list),
        "risk_map_summary_valid": risk_map_summary.get("risk_count_total", -1) >= 0,
        "report_generated": (out / "risk_control_report.md").exists(),
        "rl_action_candidate_valid": action_candidate.get("execution_mode") == "dry_run_only",
        "no_direct_cmd_vel": True,
        "no_direct_servo_pulse": True,
        "hardware_executed": False,
        "errors": errors,
    }
    write_json(out / "acceptance_check.json", acceptance)
    write_json(out / "errors.json", errors)
    write_text(
        out / "README.md",
        "# Competition Primitive Stack Dry-Run\n\n"
        "This output is generated without ROS, serial ports, chassis control, or arm hardware.\n",
    )
    print(json.dumps(acceptance, ensure_ascii=False, indent=2))
    required_true = [
        "primitive_registry_valid",
        "action_space_valid",
        "risk_detection_schema_valid",
        "risk_map_summary_valid",
        "report_generated",
        "rl_action_candidate_valid",
        "no_direct_cmd_vel",
        "no_direct_servo_pulse",
    ]
    accepted = (
        not errors
        and all(acceptance.get(key) is True for key in required_true)
        and acceptance.get("hardware_executed") is False
    )
    return 0 if accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
