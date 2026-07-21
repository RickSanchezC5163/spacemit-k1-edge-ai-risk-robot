#!/usr/bin/env python3
"""Replay a recorded K1 semantic-control run through the HTTP controller."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip().lstrip("\ufeff")
        if not line or not line.startswith("{"):
            continue
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
    return events


def build_replay(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    replay: list[dict[str, Any]] = []
    pending: dict[str, Any] | None = None
    previous_result_time: float | None = None

    for event in events:
        event_type = event.get("event")
        if event_type == "semantic_start":
            if pending is not None:
                raise ValueError(f"missing result before {event.get('code')}")
            start_time = float(event["wall_time"])
            pending = {
                "index": len(replay) + 1,
                "semantic": str(event["code"]),
                "label": str(event.get("label", event["code"])),
                "pause_before_s": (
                    0.0 if previous_result_time is None else max(0.0, start_time - previous_result_time)
                ),
                "recorded_start_time": start_time,
            }
        elif event_type == "semantic_result":
            if pending is None:
                raise ValueError(f"result without start for {event.get('semantic')}")
            if event.get("semantic") != pending["semantic"]:
                raise ValueError(
                    f"result mismatch: expected {pending['semantic']}, got {event.get('semantic')}"
                )
            pending["recorded_result"] = str(event.get("result"))
            pending["recorded_progress"] = float(event.get("progress", 0.0))
            pending["recorded_duration_s"] = max(
                0.0, float(event["wall_time"]) - pending["recorded_start_time"]
            )
            previous_result_time = float(event["wall_time"])
            replay.append(pending)
            pending = None

    if pending is not None:
        raise ValueError(f"unfinished final semantic {pending['semantic']}")
    if not replay:
        raise ValueError("no complete semantic motions found")
    return replay


def apply_timing_profile(
    replay: list[dict[str, Any]],
    ordinary_pause_s: float | None,
    arm_hold_before_steps: set[int],
    arm_hold_s: float,
) -> None:
    for step in replay:
        index = int(step["index"])
        if index == 1:
            step["pause_before_s"] = 0.0
            step["pause_reason"] = "initial"
        elif index in arm_hold_before_steps:
            step["pause_before_s"] = arm_hold_s
            step["pause_reason"] = "arm_operation"
        elif ordinary_pause_s is not None:
            step["pause_before_s"] = ordinary_pause_s
            step["pause_reason"] = "ordinary"
        else:
            step["pause_reason"] = "recorded"


def http_json(base_url: str, path: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}{path}",
        data=data,
        headers={"Content-Type": "application/json"},
        method="GET" if data is None else "POST",
    )
    with urllib.request.urlopen(request, timeout=3.0) as response:
        return json.loads(response.read().decode("utf-8"))


def stop(base_url: str) -> None:
    try:
        http_json(base_url, "/api/stop", {})
    except (OSError, urllib.error.URLError):
        pass


def wait_for_result(
    base_url: str,
    semantic: str,
    accepted_after: float,
    timeout_s: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            state = http_json(base_url, "/api/state")
        except (OSError, urllib.error.URLError):
            time.sleep(0.2)
            continue
        motion = state.get("motion", {})
        if (
            motion.get("semantic") == semantic
            and motion.get("finished_at") is not None
            and float(motion["finished_at"]) >= accepted_after
            and motion.get("state") == "idle"
        ):
            return motion
        time.sleep(0.1)
    raise TimeoutError(f"{semantic} did not finish within {timeout_s:.1f}s")


def validate_preflight(state: dict[str, Any]) -> None:
    if state.get("motion", {}).get("state") != "idle":
        raise RuntimeError("controller is not idle")
    odom = state.get("odom")
    map_pose = state.get("map_pose")
    scan = state.get("scan")
    if not odom or float(odom.get("age_s", math.inf)) > 0.5:
        raise RuntimeError("odom is missing or stale")
    if not map_pose or float(map_pose.get("age_s", math.inf)) > 0.8:
        raise RuntimeError("map pose is missing or stale")
    if not scan or float(scan.get("age_s", math.inf)) > 0.5:
        raise RuntimeError("scan is missing or stale")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("events_jsonl", type=Path)
    parser.add_argument(
        "--base-url",
        default=os.environ.get("K1_SEMANTIC_CONTROLLER_URL", "http://127.0.0.1:8769"),
    )
    parser.add_argument("--export-json", type=Path)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--skip-pauses", action="store_true")
    parser.add_argument("--timeout-scale", type=float, default=2.5)
    parser.add_argument("--ordinary-pause-s", type=float)
    parser.add_argument("--arm-hold-before-step", type=int, action="append", default=[])
    parser.add_argument("--arm-hold-s", type=float, default=8.0)
    parser.add_argument("--start-step", type=int, default=1)
    parser.add_argument("--end-step", type=int)
    args = parser.parse_args()

    replay = build_replay(load_jsonl(args.events_jsonl))
    arm_hold_before_steps = set(args.arm_hold_before_step)
    invalid_arm_steps = sorted(index for index in arm_hold_before_steps if index < 2 or index > len(replay))
    if invalid_arm_steps:
        parser.error(f"invalid --arm-hold-before-step values: {invalid_arm_steps}")
    if args.ordinary_pause_s is not None and args.ordinary_pause_s < 0:
        parser.error("--ordinary-pause-s must be non-negative")
    if args.arm_hold_s < 0:
        parser.error("--arm-hold-s must be non-negative")
    end_step = len(replay) if args.end_step is None else args.end_step
    if args.start_step < 1 or end_step > len(replay) or args.start_step > end_step:
        parser.error(f"step range must be within 1..{len(replay)}")
    apply_timing_profile(
        replay,
        args.ordinary_pause_s,
        arm_hold_before_steps,
        args.arm_hold_s,
    )
    artifact = {
        "schema_version": "k1_semantic_replay_v1",
        "source": str(args.events_jsonl),
        "step_count": len(replay),
        "timing_profile": {
            "ordinary_pause_s": args.ordinary_pause_s,
            "arm_hold_before_steps": sorted(arm_hold_before_steps),
            "arm_hold_s": args.arm_hold_s,
        },
        "steps": replay,
    }
    if args.export_json:
        args.export_json.parent.mkdir(parents=True, exist_ok=True)
        args.export_json.write_text(
            json.dumps(artifact, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    recorded_total = sum(
        step["pause_before_s"] + step["recorded_duration_s"] for step in replay
    )
    print(f"steps={len(replay)} recorded_total_s={recorded_total:.1f}")
    for step in replay:
        print(
            f"{step['index']:02d} pause={step['pause_before_s']:.2f}s "
            f"{step['semantic']} ({step['label']})"
        )

    if not args.execute:
        print("dry-run only; pass --execute after placing the robot at the recorded start pose")
        return 0

    execution_steps = [
        step for step in replay if args.start_step <= int(step["index"]) <= end_step
    ]
    validate_preflight(http_json(args.base_url, "/api/state"))
    try:
        for step in execution_steps:
            if not args.skip_pauses and step["pause_before_s"] > 0:
                if step.get("pause_reason") == "arm_operation":
                    print(
                        f"ARM HOLD before step {step['index']:02d}: "
                        f"{step['pause_before_s']:.1f}s"
                    )
                time.sleep(step["pause_before_s"])
            accepted_after = time.time()
            response = http_json(
                args.base_url,
                "/api/motion",
                {"semantic": step["semantic"]},
            )
            if not (response.get("accepted") or response.get("ok")):
                raise RuntimeError(f"step {step['index']} rejected: {response}")
            timeout_s = max(8.0, step["recorded_duration_s"] * args.timeout_scale + 3.0)
            motion = wait_for_result(
                args.base_url,
                step["semantic"],
                accepted_after,
                timeout_s,
            )
            expected = step["recorded_result"]
            if motion.get("result") != expected:
                raise RuntimeError(
                    f"step {step['index']} result {motion.get('result')}, expected {expected}"
                )
            print(
                f"DONE {step['index']:02d}/{len(replay)} {step['semantic']} "
                f"progress={motion.get('progress')}"
            )
    except (Exception, KeyboardInterrupt):
        stop(args.base_url)
        raise

    stop(args.base_url)
    print("replay complete; controller stopped at zero velocity")
    return 0


if __name__ == "__main__":
    sys.exit(main())
