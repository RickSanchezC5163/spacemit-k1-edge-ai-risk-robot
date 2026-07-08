#!/usr/bin/env python3
"""Run Arm-A mock ARM_REMOVE_OBSTACLE validation without controlling hardware."""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from edge_robot_protocol import (  # noqa: E402
    ACTION_ARM_REMOVE_OBSTACLE,
    STATUS_FAILED_SAFE,
    STATUS_SUCCEEDED,
    ActionResult,
    EpisodeReport,
    PolicyAction,
    PolicyState,
    now_iso,
    to_jsonable,
)


DEFAULT_SOURCE_REPORT = ROOT / "outputs" / "p4x_d435_hold_capture_v1" / "episode_report.json"
DEFAULT_OUTPUT_ROOT = ROOT / "outputs" / "arm_a_mock_remove_obstacle_v1"


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(to_jsonable(data), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def source_base_zero_state(source_report: Path) -> Dict[str, Any]:
    report = load_json(source_report)
    action_results = report.get("action_results") or []
    policy_state = report.get("policy_state") or {}
    successful_results = [
        result for result in action_results if result.get("status") == STATUS_SUCCEEDED
    ]
    all_success_base_zero = bool(successful_results) and all(
        result.get("base_zero_ok_before") is True for result in successful_results
    )
    summary = report.get("summary") or {}
    source_ok = bool(
        policy_state.get("base_zero_ok") is True
        and all_success_base_zero
        and summary.get("published_cmd_vel") is False
    )
    return {
        "base_zero_ok": source_ok,
        "source_episode_report": str(source_report),
        "source_episode_id": report.get("episode_id"),
        "live_ros_checked": False,
        "mock_precondition": True,
        "source_policy_state_base_zero_ok": policy_state.get("base_zero_ok"),
        "source_successful_results": len(successful_results),
        "source_all_success_base_zero_before": all_success_base_zero,
        "source_published_cmd_vel": summary.get("published_cmd_vel"),
        "source_summary": summary,
        "source_base_zero": policy_state.get("base_zero") or {},
    }


def write_status_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    fieldnames = [
        "sequence",
        "action_id",
        "status",
        "base_zero_ok_before",
        "obstacle_removed",
        "mock",
        "published_cmd_vel",
        "action_result_path",
        "error",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def summary_for(results: List[ActionResult], requested_count: int) -> Dict[str, Any]:
    succeeded = sum(1 for result in results if result.status == STATUS_SUCCEEDED)
    failed_safe = sum(1 for result in results if result.status == STATUS_FAILED_SAFE)
    return {
        "requested_actions": int(requested_count),
        "completed_actions": len(results),
        "succeeded": int(succeeded),
        "failed_safe": int(failed_safe),
        "success_rate": None if not results else round(succeeded / len(results), 3),
        "acceptance_10_runs_10_success": bool(requested_count >= 10 and succeeded == requested_count),
        "all_base_zero_ok_before": all(
            result.base_zero_ok_before is True for result in results
        ),
        "all_unpublished_cmd_vel": all(
            result.published_cmd_vel is False for result in results
        ),
        "all_mock": all(result.mock is True for result in results),
        "obstacle_removed_count": sum(
            1 for result in results if result.obstacle_removed is True
        ),
        "published_cmd_vel": False,
    }


def write_readme(path: Path, episode_id: str, summary: Dict[str, Any], args: argparse.Namespace) -> None:
    text = f"""# Arm-A Mock ARM_REMOVE_OBSTACLE Validation

episode_id: `{episode_id}`

This is a mock-only validation. It does not control the real mechanical arm,
does not access the bus servo controller, and does not publish `cmd_vel`.

## Source Precondition

- source_episode_report: `{args.base_zero_source}`
- base-zero mode: mock precondition derived from frozen P4-X evidence
- live_ros_checked: `false`

## Summary

- requested_actions: `{summary['requested_actions']}`
- succeeded: `{summary['succeeded']}`
- failed_safe: `{summary['failed_safe']}`
- all_base_zero_ok_before: `{summary['all_base_zero_ok_before']}`
- all_unpublished_cmd_vel: `{summary['all_unpublished_cmd_vel']}`
- all_mock: `{summary['all_mock']}`
- obstacle_removed_count: `{summary['obstacle_removed_count']}`

## Files

- `episode_report.json`
- `arm_a_mock_status.csv`
- `errors.json`
- `actions/<action_id>/action_result.json`
"""
    path.write_text(text, encoding="utf-8")


def run_validation(args: argparse.Namespace) -> Dict[str, Any]:
    output_root = Path(args.output_dir)
    output_root.mkdir(parents=True, exist_ok=True)
    source_report = Path(args.base_zero_source)
    base_zero = source_base_zero_state(source_report)
    episode_id = args.episode_id or f"arm_a_mock_remove_obstacle_{time.strftime('%Y%m%d_%H%M%S')}"
    started_at = now_iso()

    policy_state = PolicyState(
        state_id=f"{episode_id}_state_00",
        timestamp=now_iso(),
        base_zero_ok=bool(base_zero.get("base_zero_ok")),
        base_zero=base_zero,
        source="arm_a_mock_remove_obstacle_validation",
        notes=[
            "mock-only arm validation",
            "base-zero precondition derived from frozen P4-X episode report",
            "no real bus servo controller access",
        ],
    )

    actions: List[PolicyAction] = []
    results: List[ActionResult] = []
    rows: List[Dict[str, Any]] = []
    errors: List[Dict[str, Any]] = []

    for sequence in range(1, args.count + 1):
        action_id = f"{episode_id}_action_{sequence:02d}"
        action_dir = output_root / "actions" / action_id
        action_result_path = action_dir / "action_result.json"
        action_started_at = now_iso()
        action = PolicyAction(
            action_id=action_id,
            action_type=ACTION_ARM_REMOVE_OBSTACLE,
            requested_at=action_started_at,
            requires_base_zero=True,
            publishes_cmd_vel=False,
            reason="Arm-A mock obstacle removal validation",
            params={
                "sequence": sequence,
                "mock": True,
                "sleep_s": args.sleep_s,
            },
        )
        actions.append(action)

        base_zero_ok_before = bool(base_zero.get("base_zero_ok"))
        if not base_zero_ok_before:
            error = "base_zero_ok_before=false; mock arm action skipped"
            errors.append(
                {
                    "timestamp": now_iso(),
                    "action_id": action_id,
                    "stage": "base_zero_precheck",
                    "error": error,
                }
            )
            result = ActionResult(
                action_id=action_id,
                action_type=ACTION_ARM_REMOVE_OBSTACLE,
                status=STATUS_FAILED_SAFE,
                started_at=action_started_at,
                ended_at=now_iso(),
                base_zero_ok_before=False,
                published_cmd_vel=False,
                evidence_paths={
                    "action_result": str(action_result_path),
                    "source_episode_report": str(source_report),
                },
                base_zero=base_zero,
                error=error,
                details={"mock": True, "sleep_s": 0.0},
                obstacle_removed=False,
                mock=True,
            )
        else:
            time.sleep(max(0.0, float(args.sleep_s)))
            result = ActionResult(
                action_id=action_id,
                action_type=ACTION_ARM_REMOVE_OBSTACLE,
                status=STATUS_SUCCEEDED,
                started_at=action_started_at,
                ended_at=now_iso(),
                base_zero_ok_before=True,
                published_cmd_vel=False,
                evidence_paths={
                    "action_result": str(action_result_path),
                    "source_episode_report": str(source_report),
                },
                base_zero=base_zero,
                error=None,
                details={
                    "mock": True,
                    "sleep_s": float(args.sleep_s),
                    "real_arm_controlled": False,
                    "bus_servo_controller_accessed": False,
                },
                obstacle_removed=True,
                mock=True,
            )

        results.append(result)
        write_json(action_result_path, result)
        rows.append(
            {
                "sequence": sequence,
                "action_id": action_id,
                "status": result.status,
                "base_zero_ok_before": result.base_zero_ok_before,
                "obstacle_removed": result.obstacle_removed,
                "mock": result.mock,
                "published_cmd_vel": result.published_cmd_vel,
                "action_result_path": str(action_result_path),
                "error": result.error,
            }
        )
        write_status_csv(output_root / "arm_a_mock_status.csv", rows)
        write_json(output_root / "errors.json", errors)

    summary = summary_for(results, args.count)
    report = EpisodeReport(
        episode_id=episode_id,
        started_at=started_at,
        ended_at=now_iso(),
        protocol_version="arm_a_mock_remove_obstacle_v1",
        policy_state=policy_state,
        actions=actions,
        action_results=results,
        captures=[],
        risk_points=[],
        summary=summary,
        errors=errors,
        output_root=str(output_root),
    )
    write_json(output_root / "episode_report.json", report)
    write_readme(output_root / "README.md", episode_id, summary, args)
    return {
        "episode_id": episode_id,
        "output_root": str(output_root),
        "summary": summary,
        "errors": errors,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run mock-only ARM_REMOVE_OBSTACLE validation."
    )
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--base-zero-source", default=str(DEFAULT_SOURCE_REPORT))
    parser.add_argument("--episode-id", default=None)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--sleep-s", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = run_validation(args)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    summary = result["summary"]
    return 0 if summary["succeeded"] == args.count and not result["errors"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
