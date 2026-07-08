#!/usr/bin/env python3
"""Validate that an episode_report.json follows the lightweight P4-Z schema."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from edge_robot_protocol import ACTION_HOLD_CAPTURE, STATUS_FAILED_SAFE, STATUS_SUCCEEDED  # noqa: E402


DEFAULT_REPORT = ROOT / "outputs" / "p4x_d435_hold_capture_v1" / "episode_report.json"


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def require_keys(errors: List[str], obj: Dict[str, Any], keys: Iterable[str], where: str) -> None:
    for key in keys:
        if key not in obj:
            errors.append(f"{where} missing key: {key}")


def require_type(errors: List[str], value: Any, expected: type, where: str) -> None:
    if not isinstance(value, expected):
        errors.append(f"{where} expected {expected.__name__}, got {type(value).__name__}")


def validate_episode_report(report: Dict[str, Any], expect_p4x: bool = False) -> Dict[str, Any]:
    errors: List[str] = []
    warnings: List[str] = []

    require_keys(
        errors,
        report,
        (
            "episode_id",
            "started_at",
            "ended_at",
            "protocol_version",
            "policy_state",
            "actions",
            "action_results",
            "summary",
            "errors",
            "output_root",
        ),
        "episode_report",
    )
    require_type(errors, report.get("actions"), list, "episode_report.actions")
    require_type(errors, report.get("action_results"), list, "episode_report.action_results")

    policy_state = report.get("policy_state") or {}
    if policy_state:
        require_keys(
            errors,
            policy_state,
            ("state_id", "timestamp", "base_zero_ok", "base_zero"),
            "policy_state",
        )

    actions = report.get("actions") or []
    action_results = report.get("action_results") or []
    captures = report.get("captures") or []
    risk_points = report.get("risk_points") or []

    action_ids = set()
    for index, action in enumerate(actions):
        where = f"actions[{index}]"
        require_keys(
            errors,
            action,
            (
                "action_id",
                "action_type",
                "requested_at",
                "requires_base_zero",
                "publishes_cmd_vel",
            ),
            where,
        )
        action_id = action.get("action_id")
        if action_id in action_ids:
            errors.append(f"{where} duplicate action_id: {action_id}")
        action_ids.add(action_id)

    result_action_ids = set()
    capture_ids_from_results = set()
    for index, result in enumerate(action_results):
        where = f"action_results[{index}]"
        require_keys(
            errors,
            result,
            (
                "action_id",
                "action_type",
                "status",
                "started_at",
                "ended_at",
                "base_zero_ok_before",
                "published_cmd_vel",
                "evidence_paths",
            ),
            where,
        )
        if result.get("action_id") not in action_ids:
            errors.append(f"{where} action_id has no matching action: {result.get('action_id')}")
        result_action_ids.add(result.get("action_id"))
        if result.get("status") not in (STATUS_SUCCEEDED, STATUS_FAILED_SAFE):
            warnings.append(f"{where} non-standard status: {result.get('status')}")
        if result.get("capture_id"):
            capture_ids_from_results.add(result.get("capture_id"))
        if result.get("base_zero_ok_before") is not True and result.get("status") == STATUS_SUCCEEDED:
            errors.append(f"{where} succeeded without base_zero_ok_before=true")

    missing_results = action_ids - result_action_ids
    if missing_results:
        errors.append(f"actions without action_results: {sorted(missing_results)}")

    capture_ids = set()
    for index, capture in enumerate(captures):
        where = f"captures[{index}]"
        require_keys(
            errors,
            capture,
            ("capture_id", "action_id", "timestamp", "topics", "paths"),
            where,
        )
        capture_ids.add(capture.get("capture_id"))
        if capture.get("action_id") not in action_ids:
            errors.append(f"{where} action_id has no matching action: {capture.get('action_id')}")
        depth = capture.get("depth") or {}
        if "depth_scale_m" not in depth:
            errors.append(f"{where}.depth missing depth_scale_m")

    risk_capture_ids = set()
    for index, risk_point in enumerate(risk_points):
        where = f"risk_points[{index}]"
        require_keys(
            errors,
            risk_point,
            (
                "risk_point_id",
                "capture_id",
                "label",
                "bbox_xywh",
                "depth_median_m",
                "camera_point_xyz_m",
                "evidence_paths",
            ),
            where,
        )
        risk_capture_ids.add(risk_point.get("capture_id"))
        if risk_point.get("capture_id") not in capture_ids:
            errors.append(
                f"{where} capture_id has no matching capture: {risk_point.get('capture_id')}"
            )

    if expect_p4x:
        for index, action in enumerate(actions):
            if action.get("action_type") != ACTION_HOLD_CAPTURE:
                errors.append(f"actions[{index}] expected HOLD_CAPTURE")
            if action.get("requires_base_zero") is not True:
                errors.append(f"actions[{index}] requires_base_zero must be true")
            if action.get("publishes_cmd_vel") is not False:
                errors.append(f"actions[{index}] publishes_cmd_vel must be false")
        for index, result in enumerate(action_results):
            if result.get("published_cmd_vel") is not False:
                errors.append(f"action_results[{index}] published_cmd_vel must be false")
        unresolved = capture_ids_from_results - capture_ids
        if unresolved:
            errors.append(f"result capture_ids without capture records: {sorted(unresolved)}")
        no_risk = capture_ids - risk_capture_ids
        if no_risk:
            errors.append(f"captures without risk_points: {sorted(no_risk)}")

    return {
        "ok": not errors,
        "errors": errors,
        "warnings": warnings,
        "counts": {
            "actions": len(actions),
            "action_results": len(action_results),
            "captures": len(captures),
            "risk_points": len(risk_points),
        },
        "summary": report.get("summary") or {},
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate an episode_report.json against the lightweight P4-Z schema."
    )
    parser.add_argument("--episode-report", default=str(DEFAULT_REPORT))
    parser.add_argument("--expect-p4x", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report_path = Path(args.episode_report)
    report = load_json(report_path)
    result = validate_episode_report(report, expect_p4x=args.expect_p4x)
    result["episode_report"] = str(report_path)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
