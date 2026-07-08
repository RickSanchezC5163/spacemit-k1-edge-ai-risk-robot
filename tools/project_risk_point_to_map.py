#!/usr/bin/env python3
"""Project P4-X risk points into an approximate local odom/map view offline."""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


PROJECTION_VERSION = "map_a_offline_risk_point_projection_v1"
PROJECTION_MODE = "approximate_static_camera_offset"
CAMERA_OFFSET_BASE_M = (0.15, 0.0, 0.20)
CAMERA_YAW_OFFSET_RAD = 0.0
TF_VALIDATED = False
SLAM_USED = False
NAVIGATION_USED = False
AXIS_MAPPING = (
    "p4x_d435_optical_approx: base_x=camera_z+offset_x, "
    "base_y=-camera_x+offset_y, base_z=-camera_y+offset_z"
)


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


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
        if math.isfinite(float(value)):
            return float(value)
        return None
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return None
    if math.isfinite(converted):
        return converted
    return None


def dict_xyz(raw: Any) -> Optional[Dict[str, float]]:
    if not isinstance(raw, dict):
        return None
    x = as_number(raw.get("x"))
    y = as_number(raw.get("y"))
    z = as_number(raw.get("z"))
    if x is None or y is None or z is None:
        return None
    return {"x": x, "y": y, "z": z}


def robot_pose_from_odom(odom: Any) -> Optional[Dict[str, float]]:
    if not isinstance(odom, dict):
        return None
    pose = odom.get("pose") or {}
    position = pose.get("position") or {}
    x = as_number(position.get("x"))
    y = as_number(position.get("y"))
    z = as_number(position.get("z"))
    yaw = as_number(pose.get("yaw_rad"))
    if x is None or y is None or yaw is None:
        return None
    return {
        "x": x,
        "y": y,
        "z": 0.0 if z is None else z,
        "yaw_rad": yaw,
        "yaw_deg": math.degrees(yaw),
        "frame_id": (odom.get("header") or {}).get("frame_id"),
        "child_frame_id": odom.get("child_frame_id"),
        "stamp": (odom.get("header") or {}).get("stamp"),
    }


def camera_point_to_base_point(camera_point: Dict[str, float]) -> Dict[str, float]:
    # P4-X stores a D435 optical-frame point: x right, y down, z forward.
    # Without TF, this maps optical z to base forward x and adds static offset.
    return {
        "x": camera_point["z"] + CAMERA_OFFSET_BASE_M[0],
        "y": -camera_point["x"] + CAMERA_OFFSET_BASE_M[1],
        "z": -camera_point["y"] + CAMERA_OFFSET_BASE_M[2],
    }


def base_point_to_odom_xy(base_point: Dict[str, float], robot_pose: Dict[str, float]) -> Dict[str, float]:
    yaw = robot_pose["yaw_rad"] + CAMERA_YAW_OFFSET_RAD
    c = math.cos(yaw)
    s = math.sin(yaw)
    base_x = base_point["x"]
    base_y = base_point["y"]
    return {
        "x": robot_pose["x"] + c * base_x - s * base_y,
        "y": robot_pose["y"] + s * base_x + c * base_y,
    }


def by_capture_id(items: Iterable[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    indexed: Dict[str, Dict[str, Any]] = {}
    for item in items:
        capture_id = item.get("capture_id")
        if capture_id:
            indexed[str(capture_id)] = item
    return indexed


def actions_by_capture(actions: Iterable[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    indexed: Dict[str, Dict[str, Any]] = {}
    for action in actions:
        params = action.get("params") or {}
        capture_id = params.get("capture_id")
        if capture_id:
            indexed[str(capture_id)] = action
    return indexed


def first_non_none(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def depth_scale_for(
    risk: Dict[str, Any],
    capture: Dict[str, Any],
    action_result: Dict[str, Any],
) -> Optional[float]:
    risk_scale = as_number(risk.get("depth_scale_m"))
    capture_scale = as_number(((capture.get("depth") or {}).get("depth_scale_m")))
    details_scale = as_number(((action_result.get("details") or {}).get("depth_scale_m")))
    return first_non_none(risk_scale, capture_scale, details_scale)


def merge_evidence_paths(
    episode_report_path: Path,
    risk: Dict[str, Any],
    capture: Dict[str, Any],
    action_result: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "source_episode_report": str(episode_report_path),
        "risk_point": risk.get("evidence_paths") or {},
        "capture": capture.get("paths") or {},
        "action_result": action_result.get("evidence_paths") or {},
        "capture_meta_path": action_result.get("capture_meta_path"),
        "risk_point_path": action_result.get("risk_point_path"),
    }


def build_risk_map_points(
    report: Dict[str, Any],
    episode_report_path: Path,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    actions = report.get("actions") or []
    action_results = report.get("action_results") or []
    captures = report.get("captures") or []
    risk_points = report.get("risk_points") or []
    actions_for_capture = actions_by_capture(actions)
    results_for_capture = by_capture_id(action_results)
    captures_for_id = by_capture_id(captures)
    fallback_odom = (report.get("policy_state") or {}).get("odom")

    output_points: List[Dict[str, Any]] = []
    errors: List[Dict[str, Any]] = []
    if not risk_points:
        errors.append(
            {
                "timestamp": now_iso(),
                "level": "error",
                "code": "missing_risk_points",
                "message": "episode_report.json does not contain risk_points",
            }
        )

    for index, risk in enumerate(risk_points, start=1):
        capture_id = risk.get("capture_id")
        capture = captures_for_id.get(str(capture_id), {}) if capture_id else {}
        action = actions_for_capture.get(str(capture_id), {}) if capture_id else {}
        result = results_for_capture.get(str(capture_id), {}) if capture_id else {}
        action_id = first_non_none(result.get("action_id"), action.get("action_id"))
        action_type = first_non_none(result.get("action_type"), action.get("action_type"))
        camera_point = dict_xyz(risk.get("camera_point_xyz_m"))
        odom = capture.get("odom") or fallback_odom
        robot_pose = robot_pose_from_odom(odom)

        missing_required: List[str] = []
        if camera_point is None:
            missing_required.append("camera_point_xyz_m")
        if not isinstance(odom, dict):
            missing_required.append("odom")
        elif robot_pose is None:
            missing_required.append("odom.pose.position.x/y or odom.pose.yaw_rad")

        depth_median_m = as_number(risk.get("depth_median_m"))
        depth_scale_m = depth_scale_for(risk, capture, result)
        base_point = None
        odom_point = None
        status = "projected"
        if missing_required:
            status = "missing_required_field"
            errors.append(
                {
                    "timestamp": now_iso(),
                    "level": "error",
                    "code": "missing_required_field",
                    "risk_point_id": risk.get("risk_point_id"),
                    "capture_id": capture_id,
                    "missing_fields": missing_required,
                }
            )
        else:
            assert camera_point is not None
            assert robot_pose is not None
            base_point = camera_point_to_base_point(camera_point)
            odom_point = base_point_to_odom_xy(base_point, robot_pose)

        output_points.append(
            {
                "map_point_id": f"map_point_{index:03d}",
                "episode_id": report.get("episode_id"),
                "action_id": action_id,
                "action_type": action_type,
                "risk_point_id": risk.get("risk_point_id"),
                "capture_id": capture_id,
                "risk_label": risk.get("label"),
                "risk_category": risk.get("risk_category") or risk.get("category"),
                "camera_point_xyz_m": camera_point,
                "base_point_xyz_m": base_point,
                "odom_point_xy_m": odom_point,
                "robot_odom_pose": robot_pose,
                "depth_median_m": depth_median_m,
                "depth_scale_m": depth_scale_m,
                "projection_status": status,
                "missing_required_fields": missing_required,
                "projection_mode": PROJECTION_MODE,
                "tf_validated": TF_VALIDATED,
                "slam_used": SLAM_USED,
                "navigation_used": NAVIGATION_USED,
                "camera_offset_base_m": list(CAMERA_OFFSET_BASE_M),
                "camera_yaw_offset_rad": CAMERA_YAW_OFFSET_RAD,
                "axis_mapping": AXIS_MAPPING,
                "source_evidence_paths": merge_evidence_paths(
                    episode_report_path=episode_report_path,
                    risk=risk,
                    capture=capture,
                    action_result=result,
                ),
            }
        )
    return output_points, errors


def flatten(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False, sort_keys=True)
    return str(value)


def write_points_csv(path: Path, points: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "map_point_id",
        "episode_id",
        "action_id",
        "action_type",
        "risk_point_id",
        "capture_id",
        "risk_label",
        "risk_category",
        "camera_point_xyz_m",
        "base_point_xyz_m",
        "odom_point_xy_m",
        "robot_odom_pose",
        "depth_median_m",
        "depth_scale_m",
        "projection_status",
        "projection_mode",
        "tf_validated",
        "slam_used",
        "navigation_used",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for point in points:
            writer.writerow({field: flatten(point.get(field)) for field in fields})


def plot_snapshot(path: Path, points: Sequence[Dict[str, Any]]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 6), dpi=150)
    ax.set_title("Map-A0 Offline Risk Point Projection")
    ax.set_xlabel("odom x (m)")
    ax.set_ylabel("odom y (m)")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    plotted_robot = False
    x_values: List[float] = []
    y_values: List[float] = []

    for point in points:
        robot_pose = point.get("robot_odom_pose") or {}
        rx = as_number(robot_pose.get("x"))
        ry = as_number(robot_pose.get("y"))
        yaw = as_number(robot_pose.get("yaw_rad"))
        if rx is not None and ry is not None:
            x_values.append(rx)
            y_values.append(ry)
            if not plotted_robot:
                ax.scatter([rx], [ry], marker="s", s=80, color="#2f5597", label="robot")
                plotted_robot = True
            if yaw is not None:
                ax.arrow(
                    rx,
                    ry,
                    0.35 * math.cos(yaw),
                    0.35 * math.sin(yaw),
                    width=0.006,
                    head_width=0.06,
                    length_includes_head=True,
                    color="#2f5597",
                )

        odom_point = point.get("odom_point_xy_m") or {}
        px = as_number(odom_point.get("x"))
        py = as_number(odom_point.get("y"))
        if px is None or py is None:
            continue
        x_values.append(px)
        y_values.append(py)
        ax.scatter([px], [py], marker="x", s=60, color="#c00000", label="_risk")
        label = point.get("action_id") or point.get("map_point_id")
        ax.annotate(
            str(label).replace("p4x_hold_capture_20260629_223453_", ""),
            (px, py),
            textcoords="offset points",
            xytext=(5, 5),
            fontsize=7,
            color="#7f0000",
        )

    if any((point.get("odom_point_xy_m") or {}) for point in points):
        ax.scatter([], [], marker="x", s=60, color="#c00000", label="risk point")
    if not x_values:
        x_values = [0.0]
        y_values = [0.0]
    margin = 0.5
    ax.set_xlim(min(x_values) - margin, max(x_values) + margin)
    ax.set_ylim(min(y_values) - margin, max(y_values) + margin)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def md_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> List[str]:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(flatten(item).replace("\n", " ") for item in row) + " |")
    return lines


def render_projection_report(
    source_path: Path,
    output_dir: Path,
    report: Dict[str, Any],
    points: Sequence[Dict[str, Any]],
    errors: Sequence[Dict[str, Any]],
) -> str:
    summary = report.get("summary") or {}
    lines = [
        "# Map-A0 Offline Risk Point Projection",
        "",
        "## Map-A0 Summary",
        "",
        f"- episode_id: `{report.get('episode_id')}`",
        f"- protocol_version: `{report.get('protocol_version')}`",
        f"- source risk_points: `{len(report.get('risk_points') or [])}`",
        f"- projected risk_map_points: `{len(points)}`",
        f"- projected status count: `{sum(1 for point in points if point.get('projection_status') == 'projected')}`",
        f"- missing/error count: `{len(errors)}`",
        f"- P4-X source succeeded: `{summary.get('succeeded')}`",
        "",
        "## Input Evidence",
        "",
        f"- source episode_report: `{source_path}`",
        "- old P4-X evidence is read only; no frozen evidence is rewritten or backfilled.",
        "- ROS is not started.",
        "- cmd_vel is not published.",
        "- arm hardware and serial ports are not accessed.",
        "",
        "## Projection Method",
        "",
        f"- projection_mode: `{PROJECTION_MODE}`",
        f"- axis_mapping: `{AXIS_MAPPING}`",
        f"- camera_offset_base_m: `{list(CAMERA_OFFSET_BASE_M)}`",
        f"- camera_yaw_offset_rad: `{CAMERA_YAW_OFFSET_RAD}`",
        f"- tf_validated: `{TF_VALIDATED}`",
        f"- slam_used: `{SLAM_USED}`",
        f"- navigation_used: `{NAVIGATION_USED}`",
        "- D435 optical-frame `camera_point_xyz_m` is mapped approximately into base frame, then base x/y is rotated by odom yaw into the odom plane.",
        "",
        "## Risk Map Points",
        "",
    ]
    if points:
        lines.extend(
            md_table(
                [
                    "map_point_id",
                    "action_id",
                    "risk_label/category",
                    "depth_median_m",
                    "base_point_xyz_m",
                    "odom_point_xy_m",
                    "status",
                ],
                [
                    [
                        point.get("map_point_id"),
                        point.get("action_id"),
                        point.get("risk_label") or point.get("risk_category"),
                        point.get("depth_median_m"),
                        point.get("base_point_xyz_m"),
                        point.get("odom_point_xy_m"),
                        point.get("projection_status"),
                    ]
                    for point in points
                ],
            )
        )
    else:
        lines.append("- no risk map points generated")

    lines.extend(["", "## Missing Fields / Errors", ""])
    if errors:
        lines.extend(
            md_table(
                ["code", "risk_point_id", "capture_id", "missing_fields", "message"],
                [
                    [
                        error.get("code"),
                        error.get("risk_point_id"),
                        error.get("capture_id"),
                        error.get("missing_fields"),
                        error.get("message"),
                    ]
                    for error in errors
                ],
            )
        )
    else:
        lines.append("- errors.json is empty.")

    lines.extend(
        [
            "",
            "## Claim Boundary",
            "",
            "- This stage only claims offline risk point projection.",
            "- Do not claim SLAM.",
            "- Do not claim autonomous navigation.",
            "- Do not claim path planning.",
            "- Do not claim absolute high-precision risk point coordinates.",
            "- Do not claim the mechanical arm autonomously handled a mapped risk point.",
            "",
            "## Next Recommended Step",
            "",
            "Arm-C0 / P5-D follow-up: consume `risk_map_points.json` in dry-run mode and generate a map-gated no-load arm action candidate without serial writes or hardware execution.",
            "",
            "## Output Files",
            "",
            f"- risk_map_points.json: `{output_dir / 'risk_map_points.json'}`",
            f"- risk_map_points.csv: `{output_dir / 'risk_map_points.csv'}`",
            f"- risk_map_snapshot.png: `{output_dir / 'risk_map_snapshot.png'}`",
            f"- errors.json: `{output_dir / 'errors.json'}`",
        ]
    )
    return "\n".join(lines)


def render_readme(source_path: Path, points: Sequence[Dict[str, Any]], errors: Sequence[Dict[str, Any]]) -> str:
    return f"""# Map-A0 Offline Risk Point Projection

This directory contains offline risk point projection evidence generated from:

```text
{source_path}
```

Files:

- `risk_map_points.json`
- `risk_map_points.csv`
- `risk_map_snapshot.png`
- `projection_report.md`
- `README.md`
- `errors.json`

Summary:

- risk_map_points: `{len(points)}`
- errors: `{len(errors)}`
- projection_mode: `{PROJECTION_MODE}`
- tf_validated: `{TF_VALIDATED}`
- slam_used: `{SLAM_USED}`
- navigation_used: `{NAVIGATION_USED}`

Boundary:

- offline projection only
- no ROS process
- no cmd_vel publish
- no arm control
- no serial access
- no SLAM/autonomous-navigation/path-planning claim
"""


def build_projection_package(source_path: Path, output_dir: Path) -> Dict[str, Any]:
    report = load_json(source_path)
    points, errors = build_risk_map_points(report, source_path)
    return {
        "schema_version": PROJECTION_VERSION,
        "generated_at": now_iso(),
        "generator": "tools/project_risk_point_to_map.py",
        "source_episode_report": str(source_path),
        "episode_id": report.get("episode_id"),
        "projection_mode": PROJECTION_MODE,
        "tf_validated": TF_VALIDATED,
        "slam_used": SLAM_USED,
        "navigation_used": NAVIGATION_USED,
        "camera_offset_base_m": list(CAMERA_OFFSET_BASE_M),
        "camera_yaw_offset_rad": CAMERA_YAW_OFFSET_RAD,
        "axis_mapping": AXIS_MAPPING,
        "risk_map_points": points,
        "summary": {
            "risk_map_points": len(points),
            "projected": sum(1 for point in points if point.get("projection_status") == "projected"),
            "missing_required_field": sum(
                1 for point in points if point.get("projection_status") == "missing_required_field"
            ),
            "errors": len(errors),
            "output_dir": str(output_dir),
        },
        "errors": errors,
    }


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Map-A0 offline projection from P4-X episode_report risk points."
    )
    parser.add_argument("--episode-report", required=True)
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    source_path = Path(args.episode_report)
    output_dir = Path(args.output_dir)
    package = build_projection_package(source_path=source_path, output_dir=output_dir)
    points = package["risk_map_points"]
    errors = package["errors"]
    original_report = load_json(source_path)

    write_json(output_dir / "risk_map_points.json", package)
    write_points_csv(output_dir / "risk_map_points.csv", points)
    snapshot_written = False
    try:
        plot_snapshot(output_dir / "risk_map_snapshot.png", points)
        snapshot_written = True
    except Exception as exc:  # noqa: BLE001 - K1 may not have matplotlib installed.
        errors.append(
            {
                "timestamp": now_iso(),
                "level": "warning",
                "code": "snapshot_plot_skipped",
                "message": f"risk_map_snapshot.png was not generated: {exc}",
            }
        )
        write_text(
            output_dir / "risk_map_snapshot_skipped.txt",
            "risk_map_snapshot.png was not generated because the plotting backend is unavailable.\n"
            f"error: {exc}\n",
        )
    package["errors"] = errors
    package["summary"]["errors"] = len(errors)
    package["summary"]["snapshot_written"] = snapshot_written
    write_json(output_dir / "risk_map_points.json", package)
    write_json(output_dir / "errors.json", errors)
    write_text(
        output_dir / "projection_report.md",
        render_projection_report(
            source_path=source_path,
            output_dir=output_dir,
            report=original_report,
            points=points,
            errors=errors,
        ),
    )
    write_text(output_dir / "README.md", render_readme(source_path, points, errors))

    print(
        json.dumps(
            {
                "ok": True,
                "episode_id": package.get("episode_id"),
                "risk_map_points": len(points),
                "errors": len(errors),
                "projection_mode": PROJECTION_MODE,
                "tf_validated": TF_VALIDATED,
                "slam_used": SLAM_USED,
                "navigation_used": NAVIGATION_USED,
                "snapshot_written": snapshot_written,
                "output_dir": str(output_dir),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
