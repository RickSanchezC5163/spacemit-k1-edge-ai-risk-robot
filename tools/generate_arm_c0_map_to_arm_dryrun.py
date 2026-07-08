#!/usr/bin/env python3
"""Generate Arm-C0 map-gated no-load arm action candidates without hardware access."""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


PROTOCOL_VERSION = "arm_c0_map_to_arm_dryrun_v1"
ACTION_TYPE = "MAP_GATED_ARM_CANDIDATE"
SELECTED_ACTION = "ARM_SAMPLE_NO_LOAD"
SELECTED_SEQUENCE = "arm_b3_8_step_safety_adjusted_no_load_sample"
STATUS_SUCCEEDED_DRY_RUN = "succeeded_dry_run"
STATUS_BLOCKED = "blocked"


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def slug_time() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


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


def point_xy_from_base_or_odom(point: Dict[str, Any]) -> Tuple[Optional[float], Optional[float], str]:
    base_point = point.get("base_point_xyz_m") or {}
    base_x = as_number(base_point.get("x"))
    base_y = as_number(base_point.get("y"))
    if base_x is not None and base_y is not None:
        return base_x, base_y, "base_point_xyz_m"

    odom_point = point.get("odom_point_xy_m") or {}
    robot_pose = point.get("robot_odom_pose") or {}
    odom_x = as_number(odom_point.get("x"))
    odom_y = as_number(odom_point.get("y"))
    robot_x = as_number(robot_pose.get("x"))
    robot_y = as_number(robot_pose.get("y"))
    if odom_x is not None and odom_y is not None and robot_x is not None and robot_y is not None:
        return odom_x - robot_x, odom_y - robot_y, "odom_point_xy_m_minus_robot_pose"
    return None, None, "missing_xy"


def classify_zone(point: Dict[str, Any]) -> Dict[str, Any]:
    x, y, basis = point_xy_from_base_or_odom(point)
    if x is None or y is None:
        return {
            "direction_zone": "unknown",
            "distance_zone": "unknown",
            "zones": ["unknown"],
            "distance_m": None,
            "basis": basis,
        }
    distance = math.hypot(x, y)
    if y > 0.25:
        direction = "left"
    elif y < -0.25:
        direction = "right"
    elif x > 0.0 and abs(y) <= 0.25:
        direction = "front"
    else:
        direction = "unknown"
    distance_zone = "near" if distance <= 0.5 else "far"
    return {
        "direction_zone": direction,
        "distance_zone": distance_zone,
        "zones": [direction, distance_zone],
        "distance_m": distance,
        "basis": basis,
    }


def build_candidates(package: Dict[str, Any], risk_map_path: Path) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    candidates: List[Dict[str, Any]] = []
    errors: List[Dict[str, Any]] = []
    risk_points = package.get("risk_map_points") or []
    if not risk_points:
        errors.append(
            {
                "timestamp": now_iso(),
                "level": "error",
                "code": "missing_risk_map_points",
                "message": "risk_map_points.json contains no risk_map_points",
            }
        )

    for index, point in enumerate(risk_points, start=1):
        projection_status = point.get("projection_status")
        tf_validated = bool(point.get("tf_validated") is True)
        map_projection_valid = projection_status == "projected"
        zone = classify_zone(point)
        block_reasons: List[str] = []
        if projection_status != "projected":
            block_reasons.append(f"projection_status={projection_status}")
        if "unknown" in zone.get("zones", []):
            block_reasons.append("missing usable x/y point for zone classification")
        status = STATUS_BLOCKED if block_reasons else STATUS_SUCCEEDED_DRY_RUN
        if block_reasons:
            errors.append(
                {
                    "timestamp": now_iso(),
                    "level": "warning",
                    "code": "candidate_blocked",
                    "map_point_id": point.get("map_point_id"),
                    "risk_point_id": point.get("risk_point_id"),
                    "block_reasons": block_reasons,
                }
            )

        candidate_id = f"arm_c0_candidate_{index:03d}"
        candidate = {
            "candidate_id": candidate_id,
            "status": status,
            "block_reasons": block_reasons,
            "source_risk_map_points": str(risk_map_path),
            "source_episode_id": point.get("episode_id") or package.get("episode_id"),
            "source_map_point_id": point.get("map_point_id"),
            "source_risk_point_id": point.get("risk_point_id"),
            "source_capture_id": point.get("capture_id"),
            "source_hold_capture_action_id": point.get("action_id"),
            "risk_label": point.get("risk_label"),
            "risk_category": point.get("risk_category"),
            "depth_median_m": point.get("depth_median_m"),
            "depth_scale_m": point.get("depth_scale_m"),
            "camera_point_xyz_m": point.get("camera_point_xyz_m"),
            "base_point_xyz_m": point.get("base_point_xyz_m"),
            "odom_point_xy_m": point.get("odom_point_xy_m"),
            "robot_odom_pose": point.get("robot_odom_pose"),
            "zone": zone,
            "map_projection_valid": map_projection_valid,
            "projection_status": projection_status,
            "projection_precision": "tf_validated" if tf_validated else "approximate",
            "projection_mode": point.get("projection_mode") or package.get("projection_mode"),
            "tf_validated": tf_validated,
            "slam_used": bool(point.get("slam_used") is True),
            "navigation_used": bool(point.get("navigation_used") is True),
            "selected_action": SELECTED_ACTION if status == STATUS_SUCCEEDED_DRY_RUN else None,
            "selected_sequence": SELECTED_SEQUENCE if status == STATUS_SUCCEEDED_DRY_RUN else None,
            "validated_no_load_action": status == STATUS_SUCCEEDED_DRY_RUN,
            "hardware_executed": False,
            "serial_port_opened": False,
            "serial_bytes_written": 0,
            "contact_allowed": False,
            "obstacle_removed": False,
            "base_zero_required": True,
            "base_zero_checked": False,
            "base_zero_ok_before": None,
            "base_zero_check_mode": "not_checked_offline_dry_run",
            "published_cmd_vel": False,
            "cmd_vel_publish_allowed": False,
            "source_evidence_paths": point.get("source_evidence_paths") or {},
            "notes": [
                "dry-run candidate only",
                "real arm execution remains blocked until a live base_zero gate is checked",
                "candidate uses only an already validated no-load Arm-B3 sequence",
            ],
        }
        candidates.append(candidate)
    return candidates, errors


def flatten(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)


def write_candidates_csv(path: Path, candidates: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "candidate_id",
        "status",
        "source_map_point_id",
        "source_risk_point_id",
        "risk_label",
        "risk_category",
        "zone",
        "map_projection_valid",
        "projection_precision",
        "tf_validated",
        "selected_action",
        "selected_sequence",
        "validated_no_load_action",
        "hardware_executed",
        "serial_port_opened",
        "serial_bytes_written",
        "contact_allowed",
        "obstacle_removed",
        "base_zero_required",
        "base_zero_checked",
        "published_cmd_vel",
        "block_reasons",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for candidate in candidates:
            writer.writerow({field: flatten(candidate.get(field)) for field in fields})


def md_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> List[str]:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(flatten(item).replace("\n", " ") for item in row) + " |")
    return lines


def render_report(
    risk_map_path: Path,
    output_dir: Path,
    package: Dict[str, Any],
    candidates: Sequence[Dict[str, Any]],
    errors: Sequence[Dict[str, Any]],
) -> str:
    succeeded = sum(1 for item in candidates if item.get("status") == STATUS_SUCCEEDED_DRY_RUN)
    blocked = sum(1 for item in candidates if item.get("status") == STATUS_BLOCKED)
    lines = [
        "# Arm-C0 Map-To-Arm Candidate Dry-Run",
        "",
        "## Summary",
        "",
        f"- source risk_map_points: `{risk_map_path}`",
        f"- source episode_id: `{package.get('episode_id')}`",
        f"- input risk_map_points: `{len(package.get('risk_map_points') or [])}`",
        f"- generated candidates: `{len(candidates)}`",
        f"- succeeded_dry_run: `{succeeded}`",
        f"- blocked: `{blocked}`",
        f"- errors.json entries: `{len(errors)}`",
        "",
        "## Input Risk Map",
        "",
        f"- projection_mode: `{package.get('projection_mode')}`",
        f"- tf_validated: `{package.get('tf_validated')}`",
        f"- slam_used: `{package.get('slam_used')}`",
        f"- navigation_used: `{package.get('navigation_used')}`",
        "- Map-A0 output is read only; this tool does not modify Map-A0 or P4-X evidence.",
        "",
        "## Candidate Selection Rules",
        "",
        "- Only `projection_status=projected` points can generate `succeeded_dry_run` candidates.",
        "- `tf_validated=false` is allowed for dry-run, but candidates are marked `projection_precision=approximate`.",
        "- Direction zone uses robot-relative x/y: `front` when `x > 0` and `|y| <= 0.25`, `left` when `y > 0.25`, `right` when `y < -0.25`.",
        "- Distance zone is `near` when distance is `<= 0.5 m`, otherwise `far`.",
        f"- Selected action is restricted to `{SELECTED_ACTION}`.",
        f"- Selected sequence is restricted to `{SELECTED_SEQUENCE}`.",
        "",
        "## Generated Candidates",
        "",
    ]
    if candidates:
        lines.extend(
            md_table(
                [
                    "candidate_id",
                    "source_map_point_id",
                    "status",
                    "zones",
                    "projection_precision",
                    "selected_action",
                    "selected_sequence",
                ],
                [
                    [
                        item.get("candidate_id"),
                        item.get("source_map_point_id"),
                        item.get("status"),
                        (item.get("zone") or {}).get("zones"),
                        item.get("projection_precision"),
                        item.get("selected_action"),
                        item.get("selected_sequence"),
                    ]
                    for item in candidates
                ],
            )
        )
    else:
        lines.append("- no candidates generated")

    lines.extend(["", "## Blocked Candidates", ""])
    blocked_items = [item for item in candidates if item.get("status") == STATUS_BLOCKED]
    if blocked_items:
        lines.extend(
            md_table(
                ["candidate_id", "source_map_point_id", "block_reasons"],
                [
                    [
                        item.get("candidate_id"),
                        item.get("source_map_point_id"),
                        item.get("block_reasons"),
                    ]
                    for item in blocked_items
                ],
            )
        )
    else:
        lines.append("- none")

    lines.extend(
        [
            "",
            "## Safety Gates",
            "",
            "- hardware_executed: `false`",
            "- serial_port_opened: `false`",
            "- serial_bytes_written: `0`",
            "- contact_allowed: `false`",
            "- obstacle_removed: `false`",
            "- base_zero_required: `true`",
            "- base_zero_checked: `false` for this offline dry-run",
            "- published_cmd_vel: `false`",
            "",
            "## Claim Boundary",
            "",
            "- This stage only claims dry-run mapping from map risk points to arm no-load action candidates.",
            "- Do not claim real mechanical-arm motion.",
            "- Do not claim obstacle removal.",
            "- Do not claim grasping, contact, or payload handling.",
            "- Do not claim SLAM, autonomous navigation, or path planning.",
            "- Do not claim LLM control of the robot.",
            "",
            "## Next Recommended Step",
            "",
            "Arm-C1 should remain a separate, explicitly confirmed hardware step: before any real no-load execution, check live `base_zero_ok_before_arm=true`, keep `published_cmd_vel=false`, and execute only a validated Arm-B no-load sequence with no contact.",
            "",
            "## Outputs",
            "",
            f"- map_gated_arm_candidates.json: `{output_dir / 'map_gated_arm_candidates.json'}`",
            f"- map_gated_arm_candidates.csv: `{output_dir / 'map_gated_arm_candidates.csv'}`",
            f"- episode_report.json: `{output_dir / 'episode_report.json'}`",
            f"- errors.json: `{output_dir / 'errors.json'}`",
        ]
    )
    return "\n".join(lines)


def render_readme(risk_map_path: Path, candidates: Sequence[Dict[str, Any]], errors: Sequence[Dict[str, Any]]) -> str:
    succeeded = sum(1 for item in candidates if item.get("status") == STATUS_SUCCEEDED_DRY_RUN)
    blocked = sum(1 for item in candidates if item.get("status") == STATUS_BLOCKED)
    return f"""# Arm-C0 Map-To-Arm Candidate Dry-Run

Input:

```text
{risk_map_path}
```

Files:

- `map_gated_arm_candidates.json`
- `map_gated_arm_candidates.csv`
- `arm_c0_dryrun_report.md`
- `episode_report.json`
- `README.md`
- `errors.json`

Summary:

- candidates: `{len(candidates)}`
- succeeded_dry_run: `{succeeded}`
- blocked: `{blocked}`
- errors: `{len(errors)}`

Boundary:

- no ROS
- no cmd_vel publish
- no serial port access
- no mechanical-arm control
- no Arm-B3 hardware run
- no contact, grasping, payload handling, or obstacle removal claim
"""


def build_episode_report(
    risk_map_path: Path,
    output_dir: Path,
    package: Dict[str, Any],
    candidates: Sequence[Dict[str, Any]],
    errors: Sequence[Dict[str, Any]],
) -> Dict[str, Any]:
    started_at = now_iso()
    episode_id = f"arm_c0_map_to_arm_dryrun_{slug_time()}"
    candidate_json_path = output_dir / "map_gated_arm_candidates.json"
    actions: List[Dict[str, Any]] = []
    results: List[Dict[str, Any]] = []
    for index, candidate in enumerate(candidates, start=1):
        action_id = f"{episode_id}_action_{index:02d}"
        requested_at = now_iso()
        action = {
            "action_id": action_id,
            "action_type": ACTION_TYPE,
            "requested_at": requested_at,
            "requires_base_zero": True,
            "publishes_cmd_vel": False,
            "reason": "Arm-C0 offline map-gated no-load action candidate dry-run",
            "params": {
                "candidate_id": candidate.get("candidate_id"),
                "source_map_point_id": candidate.get("source_map_point_id"),
                "source_risk_point_id": candidate.get("source_risk_point_id"),
                "selected_action": candidate.get("selected_action"),
                "selected_sequence": candidate.get("selected_sequence"),
                "zone": candidate.get("zone"),
                "hardware_execution_allowed": False,
            },
        }
        result = {
            "action_id": action_id,
            "action_type": ACTION_TYPE,
            "status": candidate.get("status"),
            "started_at": requested_at,
            "ended_at": now_iso(),
            "base_zero_ok_before": None,
            "published_cmd_vel": False,
            "evidence_paths": {
                "risk_map_points": str(risk_map_path),
                "map_gated_arm_candidates": str(candidate_json_path),
                "source_episode_report": package.get("source_episode_report"),
                "source_evidence_paths": candidate.get("source_evidence_paths"),
            },
            "error": "; ".join(candidate.get("block_reasons") or []) or None,
            "details": candidate,
        }
        actions.append(action)
        results.append(result)

    succeeded = sum(1 for item in candidates if item.get("status") == STATUS_SUCCEEDED_DRY_RUN)
    blocked = sum(1 for item in candidates if item.get("status") == STATUS_BLOCKED)
    return {
        "episode_id": episode_id,
        "started_at": started_at,
        "ended_at": now_iso(),
        "protocol_version": PROTOCOL_VERSION,
        "policy_state": {
            "state_id": f"{episode_id}_state_offline",
            "timestamp": now_iso(),
            "base_zero_ok": None,
            "base_zero": {
                "required_for_real_arm_execution": True,
                "checked": False,
                "check_mode": "not_checked_offline_dry_run",
                "reason": "Arm-C0 generates action candidates only and does not execute hardware.",
            },
            "odom": None,
            "front_min_range_m": None,
            "front_p10_range_m": None,
            "source": "generate_arm_c0_map_to_arm_dryrun",
            "notes": [
                "offline map-to-arm candidate generation",
                "no ROS process started",
                "no serial port opened",
                "no mechanical-arm command issued",
            ],
        },
        "actions": actions,
        "action_results": results,
        "map_gated_arm_candidates": list(candidates),
        "summary": {
            "input_risk_map_points": len(package.get("risk_map_points") or []),
            "candidates": len(candidates),
            "succeeded_dry_run": succeeded,
            "blocked": blocked,
            "failed_safe": 0,
            "published_cmd_vel": False,
            "hardware_executed": False,
            "serial_port_opened": False,
            "serial_bytes_written": 0,
            "contact_allowed": False,
            "obstacle_removed": False,
            "base_zero_required": True,
            "base_zero_checked": False,
        },
        "errors": list(errors),
        "output_root": str(output_dir),
    }


def build_candidate_package(
    risk_map_path: Path,
    output_dir: Path,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    package = load_json(risk_map_path)
    candidates, errors = build_candidates(package, risk_map_path)
    candidate_package = {
        "schema_version": PROTOCOL_VERSION,
        "generated_at": now_iso(),
        "generator": "tools/generate_arm_c0_map_to_arm_dryrun.py",
        "source_risk_map_points": str(risk_map_path),
        "source_episode_id": package.get("episode_id"),
        "selected_action_policy": {
            "selected_action": SELECTED_ACTION,
            "selected_sequence": SELECTED_SEQUENCE,
            "validated_no_load_action_required": True,
            "hardware_execution_allowed": False,
        },
        "safety_boundary": {
            "ros_started": False,
            "cmd_vel_published": False,
            "serial_port_opened": False,
            "serial_bytes_written": 0,
            "hardware_executed": False,
            "contact_allowed": False,
            "obstacle_removed": False,
            "base_zero_required": True,
            "base_zero_checked": False,
        },
        "candidates": candidates,
        "summary": {
            "input_risk_map_points": len(package.get("risk_map_points") or []),
            "candidates": len(candidates),
            "succeeded_dry_run": sum(
                1 for item in candidates if item.get("status") == STATUS_SUCCEEDED_DRY_RUN
            ),
            "blocked": sum(1 for item in candidates if item.get("status") == STATUS_BLOCKED),
            "errors": len(errors),
            "output_dir": str(output_dir),
        },
        "errors": errors,
    }
    episode_report = build_episode_report(
        risk_map_path=risk_map_path,
        output_dir=output_dir,
        package=package,
        candidates=candidates,
        errors=errors,
    )
    return candidate_package, episode_report


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate Arm-C0 dry-run arm candidates from Map-A0 risk map points."
    )
    parser.add_argument("--risk-map-points", required=True)
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    risk_map_path = Path(args.risk_map_points)
    output_dir = Path(args.output_dir)
    map_package = load_json(risk_map_path)
    candidate_package, episode_report = build_candidate_package(risk_map_path, output_dir)
    candidates = candidate_package["candidates"]
    errors = candidate_package["errors"]

    write_json(output_dir / "map_gated_arm_candidates.json", candidate_package)
    write_candidates_csv(output_dir / "map_gated_arm_candidates.csv", candidates)
    write_json(output_dir / "episode_report.json", episode_report)
    write_json(output_dir / "errors.json", errors)
    write_text(
        output_dir / "arm_c0_dryrun_report.md",
        render_report(
            risk_map_path=risk_map_path,
            output_dir=output_dir,
            package=map_package,
            candidates=candidates,
            errors=errors,
        ),
    )
    write_text(output_dir / "README.md", render_readme(risk_map_path, candidates, errors))

    print(
        json.dumps(
            {
                "ok": True,
                "source_episode_id": candidate_package.get("source_episode_id"),
                "candidates": len(candidates),
                "succeeded_dry_run": candidate_package["summary"]["succeeded_dry_run"],
                "blocked": candidate_package["summary"]["blocked"],
                "errors": len(errors),
                "hardware_executed": False,
                "serial_port_opened": False,
                "serial_bytes_written": 0,
                "published_cmd_vel": False,
                "output_dir": str(output_dir),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
