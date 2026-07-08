#!/usr/bin/env python3
"""Convert a Step7 episode report into a lightweight RL trace."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.schemas import read_json, write_json


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-report", required=True)
    parser.add_argument("--output-dir", default="outputs/rl_trace_v1/step7_episode")
    args = parser.parse_args()
    episode = read_json(args.episode_report)
    summary = episode.get("summary") or {}
    actions = []
    for result in episode.get("action_results") or []:
        details = result.get("details") or {}
        actions.append(
            {
                "action_id": result.get("action_id"),
                "primitive": result.get("action_type"),
                "status": result.get("status"),
                "base_zero_ok_before": result.get("base_zero_ok_before"),
                "published_cmd_vel": result.get("published_cmd_vel"),
                "hardware_executed": details.get("hardware_executed", summary.get("hardware_executed", False)),
            }
        )
    trace = {
        "schema_version": "step7_to_rl_trace_v1",
        "source_episode_report": args.episode_report,
        "episode_id": episode.get("episode_id"),
        "summary": {
            "status": summary.get("status"),
            "risk_detected": bool(summary.get("red_object_detected") or summary.get("risk_point_generated")),
            "risk_count": summary.get("risk_map_points", 0),
            "hardware_executed": summary.get("hardware_executed", False),
        },
        "actions": actions,
        "claim_boundary": ["Trace is for offline RL alignment only; it does not control hardware."],
    }
    out = Path(args.output_dir)
    write_json(out / "rl_trace.json", trace)
    with (out / "rl_trace.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["action_id", "primitive", "status", "base_zero_ok_before", "published_cmd_vel", "hardware_executed"])
        writer.writeheader()
        writer.writerows(actions)
    print(json.dumps(trace, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
