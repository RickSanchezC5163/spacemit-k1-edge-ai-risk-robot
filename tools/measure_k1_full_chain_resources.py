#!/usr/bin/env python3
"""Measure K1 full-chain CPU, RSS, load, pressure, and thermal state via /proc."""

from __future__ import annotations

import argparse
import json
import os
import statistics
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


GROUP_PATTERNS = (
    ("yolo", ("run_prelim_remote_mapping_yolo_arm_demo.py",)),
    ("risk_approach", ("run_real_k1_risk_approach_from_event.py",)),
    ("rrt", ("sim_rrt_frontier_explorer.py",)),
    ("guard", ("scan_safety_guard_node",)),
    ("base", ("wheeltec_tank_base_safe.py",)),
    ("slam", ("slam_toolbox",)),
    ("lidar", ("lslidar_driver",)),
    ("nav2_controller", ("nav2_controller", "controller_server")),
    ("nav2_planner", ("nav2_planner", "planner_server")),
    ("nav2_bt", ("nav2_bt_navigator", "bt_navigator")),
    ("nav2_behavior", ("nav2_behaviors", "behavior_server")),
    ("nav2_smoother", ("nav2_smoother", "smoother_server")),
    ("nav2_velocity_smoother", ("nav2_velocity_smoother", "velocity_smoother")),
    ("nav2_waypoint", ("nav2_waypoint_follower", "waypoint_follower")),
    ("nav2_lifecycle", ("nav2_lifecycle_manager", "lifecycle_manager")),
)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def percentile(values: Iterable[float], pct: float) -> Optional[float]:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    index = int(round((pct / 100.0) * (len(ordered) - 1)))
    return ordered[max(0, min(len(ordered) - 1, index))]


def process_group(command: str) -> Optional[str]:
    for group, patterns in GROUP_PATTERNS:
        if any(pattern in command for pattern in patterns):
            return group
    return None


def read_processes() -> Dict[int, Dict[str, Any]]:
    result: Dict[int, Dict[str, Any]] = {}
    page_size = os.sysconf("SC_PAGE_SIZE")
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            ).strip()
            group = process_group(command)
            if group is None:
                continue
            stat = (entry / "stat").read_text(encoding="utf-8").split()
            statm = (entry / "statm").read_text(encoding="utf-8").split()
            result[pid] = {
                "pid": pid,
                "group": group,
                "command": command,
                "ticks": int(stat[13]) + int(stat[14]),
                "rss_bytes": int(statm[1]) * page_size,
            }
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError, IndexError):
            continue
    return result


def read_system_cpu() -> Tuple[int, int]:
    fields = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()[1:]
    values = [int(value) for value in fields]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def read_meminfo() -> Dict[str, int]:
    values: Dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        key, raw = line.split(":", 1)
        values[key] = int(raw.strip().split()[0]) * 1024
    return values


def read_pressure(resource: str) -> Dict[str, float]:
    path = Path("/proc/pressure") / resource
    if not path.exists():
        return {}
    result: Dict[str, float] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        prefix = parts[0]
        for item in parts[1:]:
            key, value = item.split("=", 1)
            if key != "total":
                result[f"{prefix}_{key}"] = float(value)
    return result


def read_thermal() -> Dict[str, float]:
    result: Dict[str, float] = {}
    for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*/temp")):
        try:
            value = float(path.read_text(encoding="utf-8").strip())
            if value > 1000.0:
                value /= 1000.0
            result[path.parent.name] = value
        except (OSError, ValueError):
            continue
    return result


def summarize(samples: List[Dict[str, Any]], output: Path) -> Dict[str, Any]:
    groups: Dict[str, Dict[str, List[float]]] = defaultdict(lambda: defaultdict(list))
    system: Dict[str, List[float]] = defaultdict(list)
    commands: Dict[str, set[str]] = defaultdict(set)
    for sample in samples:
        for group, values in sample["groups"].items():
            groups[group]["cpu_percent"].append(float(values["cpu_percent"]))
            groups[group]["rss_mib"].append(float(values["rss_mib"]))
            for command in values.get("commands", []):
                commands[group].add(command)
        for key in (
            "tracked_cpu_percent",
            "system_cpu_percent",
            "load1",
            "load5",
            "mem_available_mib",
            "cpu_pressure_some_avg10",
            "cpu_pressure_full_avg10",
        ):
            value = sample.get(key)
            if value is not None:
                system[key].append(float(value))

    def stats(values: List[float]) -> Dict[str, Optional[float]]:
        return {
            "mean": None if not values else round(statistics.mean(values), 3),
            "p50": None if not values else round(float(percentile(values, 50) or 0.0), 3),
            "p95": None if not values else round(float(percentile(values, 95) or 0.0), 3),
            "max": None if not values else round(max(values), 3),
        }

    summary = {
        "schema_version": "k1_full_chain_resource_measurement_v1",
        "started_at": samples[0]["timestamp"] if samples else None,
        "finished_at": samples[-1]["timestamp"] if samples else None,
        "sample_count": len(samples),
        "interval_s": None if len(samples) < 2 else round(samples[1]["elapsed_s"] - samples[0]["elapsed_s"], 3),
        "groups": {
            group: {
                "cpu_percent": stats(values["cpu_percent"]),
                "rss_mib": stats(values["rss_mib"]),
                "commands": sorted(commands[group]),
            }
            for group, values in sorted(groups.items())
        },
        "system": {key: stats(values) for key, values in sorted(system.items())},
    }
    output.with_suffix(".summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=180.0)
    parser.add_argument("--interval-s", type=float, default=2.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    clock_ticks = float(os.sysconf("SC_CLK_TCK"))
    previous_processes = read_processes()
    previous_total, previous_idle = read_system_cpu()
    previous_time = time.monotonic()
    started = previous_time
    samples: List[Dict[str, Any]] = []

    with output.open("w", encoding="utf-8") as handle:
        while True:
            deadline = min(started + float(args.duration_s), previous_time + float(args.interval_s))
            time.sleep(max(0.0, deadline - time.monotonic()))
            current_time = time.monotonic()
            elapsed = current_time - previous_time
            current_processes = read_processes()
            current_total, current_idle = read_system_cpu()
            meminfo = read_meminfo()
            groups: Dict[str, Dict[str, Any]] = defaultdict(
                lambda: {"cpu_percent": 0.0, "rss_mib": 0.0, "pids": [], "commands": []}
            )
            for pid, process in current_processes.items():
                group = groups[process["group"]]
                previous = previous_processes.get(pid)
                if previous is not None and elapsed > 0.0:
                    tick_delta = max(0, process["ticks"] - previous["ticks"])
                    group["cpu_percent"] += tick_delta / clock_ticks / elapsed * 100.0
                group["rss_mib"] += process["rss_bytes"] / (1024.0 * 1024.0)
                group["pids"].append(pid)
                group["commands"].append(process["command"])
            for values in groups.values():
                values["cpu_percent"] = round(values["cpu_percent"], 3)
                values["rss_mib"] = round(values["rss_mib"], 3)
                values["commands"] = sorted(set(values["commands"]))

            total_delta = max(1, current_total - previous_total)
            idle_delta = max(0, current_idle - previous_idle)
            loads = [float(value) for value in Path("/proc/loadavg").read_text().split()[:3]]
            cpu_pressure = read_pressure("cpu")
            sample = {
                "timestamp": now_iso(),
                "elapsed_s": round(current_time - started, 3),
                "groups": dict(groups),
                "tracked_cpu_percent": round(
                    sum(float(value["cpu_percent"]) for value in groups.values()), 3
                ),
                "system_cpu_percent": round((1.0 - idle_delta / total_delta) * 100.0, 3),
                "load1": loads[0],
                "load5": loads[1],
                "load15": loads[2],
                "mem_available_mib": round(meminfo.get("MemAvailable", 0) / (1024.0 * 1024.0), 3),
                "cpu_pressure_some_avg10": cpu_pressure.get("some_avg10"),
                "cpu_pressure_full_avg10": cpu_pressure.get("full_avg10"),
                "memory_pressure": read_pressure("memory"),
                "thermal_c": read_thermal(),
            }
            handle.write(json.dumps(sample, ensure_ascii=False, sort_keys=True) + "\n")
            handle.flush()
            samples.append(sample)
            previous_processes = current_processes
            previous_total, previous_idle = current_total, current_idle
            previous_time = current_time
            if current_time - started >= float(args.duration_s):
                break

    print(json.dumps(summarize(samples, output), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
