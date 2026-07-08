#!/usr/bin/env python3
"""Minimal validator for episode_report_v2."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.schemas import read_json


REQUIRED = [
    "episode_id",
    "episode_kind",
    "started_at",
    "ended_at",
    "observation_state",
    "primitive_actions",
    "primitive_results",
    "risk_detections",
    "risk_map_summary",
    "action_candidates",
    "safety_gate_results",
    "report_outputs",
    "benchmarks",
    "claim_boundary",
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-report", required=True)
    args = parser.parse_args()
    data = read_json(args.episode_report)
    errors = [f"missing {key}" for key in REQUIRED if key not in data]
    observations = data.get("observation_state")
    if not isinstance(observations, list):
        errors.append("observation_state must be a list")
    else:
        for idx, observation in enumerate(observations):
            if not isinstance(observation, dict):
                errors.append(f"observation_state[{idx}] must be an object")
                continue
            for key in ("observation_id", "timestamp", "base_zero"):
                if key not in observation:
                    errors.append(f"observation_state[{idx}] missing {key}")
            if "scan_left" in observation or "scan_right" in observation:
                errors.append(f"observation_state[{idx}] uses deprecated scan_left/scan_right")
            for key in ("left_p10", "right_p10", "consecutive_fast_arc", "total_forward_m"):
                if key not in observation:
                    errors.append(f"observation_state[{idx}] missing {key}")
    if not isinstance(data.get("primitive_actions", []), list):
        errors.append("primitive_actions must be a list")
    if not isinstance(data.get("primitive_results", []), list):
        errors.append("primitive_results must be a list")
    benchmarks = data.get("benchmarks")
    if not isinstance(benchmarks, dict):
        errors.append("benchmarks must be an object")
    else:
        required_benchmark_fields = {
            "vision": ("backend", "latency_ms", "fps", "model_used", "local_inference", "online_api_used"),
            "llm": ("backend", "local_llm_used", "online_api_used", "model_name", "model_size_mb", "ttft_ms", "tokens_per_second", "total_tokens", "peak_memory_mb"),
            "rl": ("policy_name", "action_space_version", "hardware_connected", "cmd_vel_published", "servo_pulse_output"),
        }
        for section, keys in required_benchmark_fields.items():
            value = benchmarks.get(section)
            if not isinstance(value, dict):
                errors.append(f"benchmarks.{section} must be an object")
                continue
            for key in keys:
                if key not in value:
                    errors.append(f"benchmarks.{section} missing {key}")
    result = {"episode_report_v2_valid": not errors, "errors": errors, "episode_id": data.get("episode_id")}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
