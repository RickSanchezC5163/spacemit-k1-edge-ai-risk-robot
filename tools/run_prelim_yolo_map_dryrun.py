#!/usr/bin/env python3
"""Offline dry-run for YOLO risk detection and approximate map annotation.

Input is one or more saved D435 capture directories containing:

- rgb.png
- depth_raw.npy
- camera_info.json
- odom.json, or --synthetic-odom-if-missing for labeled offline dry-run only

The script runs local YOLO, applies bbox depth localization, projects each
detection into an approximate odom/map point, writes event/map/report/dashboard
evidence, and generates manual arm no-load response candidates. It does not
start ROS, publish cmd_vel, open serial ports, or control hardware.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.run_yolo_inference_once import (  # noqa: E402
    apply_depth,
    draw_overlay,
    postprocess,
    preprocess,
    resolve_model_input_size,
    run_inference,
)


DEFAULT_MODEL = "models/risk_vision/yolov8n_320_q_truncated_balanced.onnx"
DEFAULT_OUTPUT_DIR = "outputs/prelim_yolo_map_dryrun_v1"
PROTOCOL_VERSION = "prelim_yolo_map_dryrun_v1"

CAMERA_OFFSET_BASE_M = (0.105, 0.0, 0.11)
AXIS_MAPPING = "d435_optical_to_base_approx:z_forward_to_x_forward,x_right_to_y_right"

CLASS_TO_EVENT = {
    "crack": ("hard_obstacle", "high", "stop_and_report"),
    "corrosion": ("hard_obstacle", "medium", "stop_and_recheck"),
    "leakage": ("hard_obstacle", "high", "stop_and_report"),
    "blockage": ("blocked_path", "high", "stop_and_report"),
}


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def now_id() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")[:-4]


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def append_jsonl(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(data, ensure_ascii=False, sort_keys=True) + "\n")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def optional_json(path: Path) -> Any:
    if not path.exists():
        return None
    return load_json(path)


def class_name(det: Dict[str, Any]) -> str:
    return str(det.get("class_name") or det.get("label") or "risk")


def risk_info_for_class(name: str) -> Tuple[str, str, str]:
    return CLASS_TO_EVENT.get(name, ("hard_obstacle", "medium", "stop_and_recheck"))


def depth_scale_for_capture(capture_dir: Path, fallback: float) -> float:
    meta = optional_json(capture_dir / "capture_meta.json")
    if isinstance(meta, dict):
        for path in (
            ("depth_scale_m",),
            ("depth", "depth_scale_m"),
        ):
            value: Any = meta
            try:
                for key in path:
                    value = value[key]
                return float(value)
            except (KeyError, TypeError, ValueError):
                pass
    return float(fallback)


def camera_point_to_base_point(camera_point: Dict[str, float]) -> Dict[str, float]:
    return {
        "x": round(float(camera_point["z"]) + CAMERA_OFFSET_BASE_M[0], 4),
        "y": round(-float(camera_point["x"]) + CAMERA_OFFSET_BASE_M[1], 4),
        "z": round(-float(camera_point["y"]) + CAMERA_OFFSET_BASE_M[2], 4),
    }


def base_point_to_odom_xy(base_point: Dict[str, float], odom: Dict[str, Any]) -> Optional[Dict[str, float]]:
    pose = odom.get("pose") or {}
    position = pose.get("position") or {}
    try:
        yaw = float(pose.get("yaw_rad"))
        robot_x = float(position["x"])
        robot_y = float(position["y"])
        base_x = float(base_point["x"])
        base_y = float(base_point["y"])
    except (KeyError, TypeError, ValueError):
        return None
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return {
        "x": round(robot_x + base_x * cos_y - base_y * sin_y, 4),
        "y": round(robot_y + base_x * sin_y + base_y * cos_y, 4),
    }


def synthetic_odom(args: argparse.Namespace) -> Dict[str, Any]:
    yaw = float(args.synthetic_odom_yaw_rad)
    return {
        "header": {"frame_id": "odom", "stamp": {"sec": 0, "nanosec": 0}},
        "child_frame_id": "base_footprint",
        "pose": {
            "position": {
                "x": float(args.synthetic_odom_x),
                "y": float(args.synthetic_odom_y),
                "z": 0.0,
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": round(math.sin(yaw / 2.0), 8),
                "w": round(math.cos(yaw / 2.0), 8),
            },
            "yaw_rad": yaw,
            "yaw_deg": round(math.degrees(yaw), 4),
        },
        "twist": {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        },
        "odom_source": "synthetic_dry_run",
        "claim_boundary": [
            "Synthetic odom is for offline dry-run visualization only.",
            "Do not use synthetic odom as evidence of real robot map localization.",
        ],
    }


def dedup_key(det: Dict[str, Any], odom_point: Optional[Dict[str, float]], map_grid_m: float) -> str:
    name = class_name(det)
    if odom_point:
        ix = int(math.floor(float(odom_point["x"]) / map_grid_m))
        iy = int(math.floor(float(odom_point["y"]) / map_grid_m))
        return f"{name}:map:{ix}:{iy}"
    bbox = det.get("bbox_xywh") or [0, 0, 0, 0]
    x, y, w, h = [float(v) for v in bbox[:4]]
    return f"{name}:image:{int((x + w / 2) / 80)}:{int((y + h / 2) / 80)}"


def build_manual_arm_candidate(event: Dict[str, Any], output_dir: Path, args: argparse.Namespace) -> Dict[str, Any]:
    candidate_dir = output_dir / "manual_arm_response_candidates" / str(event["event_id"])
    dry_run_cmd = [
        sys.executable,
        str(ROOT / "tools" / "run_arm_b3_no_load_sample_sequence.py"),
        "--output-dir",
        str(candidate_dir / "dry_run"),
        "--serial-port",
        str(args.arm_serial_port),
        "--baudrate",
        str(int(args.arm_baudrate)),
    ]
    hardware_cmd = [
        sys.executable,
        str(ROOT / "tools" / "run_arm_b3_no_load_sample_sequence.py"),
        "--output-dir",
        str(candidate_dir / "hardware"),
        "--serial-port",
        str(args.arm_serial_port),
        "--baudrate",
        str(int(args.arm_baudrate)),
        "--enable-hardware-write",
        "--confirm-no-load-sample-sequence",
    ]
    candidate = {
        "candidate_generated": True,
        "candidate_id": f"{event['event_id']}_manual_arm_candidate",
        "generated_at": now_iso(),
        "source_event_id": event["event_id"],
        "operator_decides_when_where": True,
        "auto_execution_by_this_script": False,
        "target_final_pose": "safe_idle_home_like_6b",
        "risk_odom_point_xy_m": event.get("odom_point_xy_m"),
        "dry_run_command": dry_run_cmd,
        "hardware_command_after_operator_confirmation": hardware_cmd,
        "required_before_manual_execution": [
            "Remote-control the robot to a safe response pose selected by the operator.",
            "Confirm base_zero and no cmd_vel motion before arm movement.",
            "Confirm no person/object is in the arm workspace.",
            "Keep this as no-load: no contact, grasping, payload handling, or obstacle removal claim.",
        ],
    }
    write_json(candidate_dir / "manual_arm_response_candidate.json", candidate)
    return candidate


def plot_map_snapshot(output_dir: Path, points: List[Dict[str, Any]]) -> bool:
    if not points:
        return False
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        write_text(output_dir / "risk_map_snapshot_skipped.txt", str(exc))
        return False

    fig, ax = plt.subplots(figsize=(7.0, 5.0))
    ax.set_title("YOLO Risk Map Dry-Run")
    ax.set_xlabel("odom/map x (m)")
    ax.set_ylabel("odom/map y (m)")
    ax.grid(True, linestyle="--", alpha=0.35)
    for point in points:
        odom_xy = point.get("odom_point_xy_m") or {}
        robot_pose = point.get("robot_odom_pose") or {}
        robot_position = robot_pose.get("position") or {}
        try:
            px = float(odom_xy["x"])
            py = float(odom_xy["y"])
            rx = float(robot_position.get("x", 0.0))
            ry = float(robot_position.get("y", 0.0))
        except (KeyError, TypeError, ValueError):
            continue
        ax.scatter([rx], [ry], marker="o", color="#2b6cb0", s=28)
        ax.scatter([px], [py], marker="x", color="#c00000", s=72)
        ax.plot([rx, px], [ry, py], color="#888888", linewidth=0.8, alpha=0.7)
        ax.annotate(str(point.get("class_name")), (px, py), textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax.scatter([], [], marker="o", color="#2b6cb0", label="robot pose")
    ax.scatter([], [], marker="x", color="#c00000", label="risk point")
    ax.axis("equal")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_dir / "risk_map_snapshot.png", dpi=160)
    plt.close(fig)
    return True


def write_report(output_dir: Path, events: List[Dict[str, Any]], points: List[Dict[str, Any]]) -> None:
    lines = [
        "# YOLO Risk Map Dry-Run Report",
        "",
        f"- generated_at: `{now_iso()}`",
        f"- event_count: `{len(events)}`",
        f"- risk_map_points: `{len(points)}`",
        f"- projected: `{sum(1 for p in points if p.get('projection_status') == 'projected')}`",
        "- cmd_vel_published: `false`",
        "- arm_auto_executed: `false`",
        "",
        "## Events",
        "",
    ]
    if not events:
        lines.append("- no YOLO risk event detected")
    for event in events:
        lines.append(
            f"- `{event['event_id']}` {event['class_name']} level={event['risk_level']} "
            f"conf={event['confidence']} distance={event.get('distance_m')}m "
            f"projection={event['projection_status']} odom_source={event.get('odom_source')}"
        )
    lines.extend(
        [
            "",
            "## Boundary",
            "",
            "- Offline dry-run only.",
            "- Risk-point projection is approximate bbox + depth + odom projection.",
            "- Manual arm no-load candidates are generated but not executed.",
        ]
    )
    write_text(output_dir / "risk_control_report.md", "\n".join(lines) + "\n")


def write_dashboard(output_dir: Path, events: List[Dict[str, Any]]) -> None:
    latest = events[-1] if events else {}
    latest_overlay = latest.get("overlay_path")
    overlay_rel = None
    if latest_overlay:
        try:
            overlay_rel = Path(latest_overlay).resolve().relative_to(output_dir.resolve()).as_posix()
        except Exception:
            overlay_rel = latest_overlay
    map_img = "risk_map_snapshot.png" if (output_dir / "risk_map_snapshot.png").exists() else None
    rows = "\n".join(
        "<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>".format(
            event.get("event_id"),
            event.get("class_name"),
            event.get("risk_level"),
            event.get("distance_m"),
            f"{event.get('projection_status')} / {event.get('odom_source')}",
        )
        for event in events
    )
    html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>YOLO Risk Map Dry-Run</title>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 20px; background: #f6f7f8; color: #1f2933; }}
    .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }}
    .panel {{ background: #fff; border: 1px solid #d8dee4; border-radius: 8px; padding: 12px; }}
    img {{ max-width: 100%; border: 1px solid #d8dee4; }}
    table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
    th, td {{ border-bottom: 1px solid #e5e7eb; text-align: left; padding: 6px; }}
    .alarm {{ color: #b91c1c; font-weight: 700; }}
  </style>
</head>
<body>
  <h1>YOLO Risk Map Dry-Run</h1>
  <p>Events: {len(events)} | Updated: {now_iso()}</p>
  <div class="grid">
    <div class="panel"><h2>Latest Overlay</h2>{f'<img src="{overlay_rel}">' if overlay_rel else '<p>No detections.</p>'}</div>
    <div class="panel"><h2>Risk Map</h2>{f'<img src="{map_img}">' if map_img else '<p>No projected map point.</p>'}</div>
    <div class="panel"><h2>Latest Alarm</h2><p class="alarm">{latest.get('risk_level', 'none')} {latest.get('class_name', '')}</p><p>{latest.get('recommended_action', 'none')}</p></div>
    <div class="panel"><h2>Events</h2><table><tr><th>event</th><th>class</th><th>level</th><th>distance</th><th>projection/source</th></tr>{rows}</table></div>
  </div>
</body>
</html>
"""
    write_text(output_dir / "dashboard.html", html)


def process_capture(
    capture_dir: Path,
    model_path: Path,
    output_dir: Path,
    args: argparse.Namespace,
    events_by_key: Dict[str, Dict[str, Any]],
    events: List[Dict[str, Any]],
    points: List[Dict[str, Any]],
) -> Dict[str, Any]:
    rgb_path = capture_dir / "rgb.png"
    depth_path = capture_dir / "depth_raw.npy"
    camera_info_path = capture_dir / "camera_info.json"
    odom_path = capture_dir / "odom.json"
    required_paths = (rgb_path, depth_path, camera_info_path)
    missing = [str(path) for path in required_paths if not path.exists()]
    if not odom_path.exists() and not args.synthetic_odom_if_missing:
        missing.append(str(odom_path))
    if missing:
        return {"capture_dir": str(capture_dir), "status": "skipped_missing_inputs", "missing": missing}

    cap_out = output_dir / "captures" / capture_dir.name
    cap_out.mkdir(parents=True, exist_ok=True)
    input_size_hw = resolve_model_input_size(model_path, args.imgsz)
    image_bgr, tensor, ratio, pad = preprocess(rgb_path, input_size_hw)
    raw_output, runtime_backend, backend_name, load_latency_ms, inference_latency_ms = run_inference(model_path, tensor, args)
    detections = postprocess(
        raw_output,
        image_shape=image_bgr.shape[:2],
        ratio=ratio,
        pad=pad,
        conf_thres=args.conf,
        iou_thres=args.iou,
        max_det=args.max_det,
    )
    depth_scale_m = depth_scale_for_capture(capture_dir, args.depth_scale_m)
    depth_status = apply_depth(detections, str(depth_path), str(camera_info_path), depth_scale_m)
    draw_overlay(image_bgr, detections, cap_out / "overlay.png")
    odom_source = "capture_odom_json"
    if odom_path.exists():
        odom = load_json(odom_path)
    else:
        odom = synthetic_odom(args)
        odom_source = "synthetic_dry_run"
        write_json(cap_out / "synthetic_odom.json", odom)
    capture_events = []

    for det in detections:
        name = class_name(det)
        event_type, risk_level, recommended_action = risk_info_for_class(name)
        camera_point = det.get("camera_point_xyz_m")
        base_point = camera_point_to_base_point(camera_point) if isinstance(camera_point, dict) else None
        odom_point = base_point_to_odom_xy(base_point, odom) if base_point else None
        projection_status = "projected" if odom_point else "missing_pose_or_depth"
        key = dedup_key(det, odom_point, args.dedup_map_grid_m)
        existing = events_by_key.get(key)
        if existing:
            existing["seen_count"] = int(existing.get("seen_count", 1)) + 1
            existing["last_seen_capture"] = str(capture_dir)
            existing["confidence_max"] = max(float(existing.get("confidence_max") or 0.0), float(det.get("confidence") or 0.0))
            continue
        event_id = f"risk_event_{len(events) + 1:03d}_{now_id()}"
        point = {
            "map_point_id": f"map_point_{len(points) + 1:03d}",
            "event_id": event_id,
            "capture_dir": str(capture_dir),
            "class_name": name,
            "event_type": event_type,
            "risk_level": risk_level,
            "confidence": det.get("confidence"),
            "distance_m": det.get("depth_median_m"),
            "bbox_xywh": det.get("bbox_xywh"),
            "camera_point_xyz_m": camera_point,
            "base_point_xyz_m": base_point,
            "odom_point_xy_m": odom_point,
            "projection_status": projection_status,
            "projection_mode": "d435_bbox_depth_odom_approx",
            "axis_mapping": AXIS_MAPPING,
            "odom_source": odom_source,
            "robot_odom_pose": (odom or {}).get("pose"),
        }
        event = {
            "event_id": event_id,
            "dedup_key": key,
            "first_seen": now_iso(),
            "last_seen_capture": str(capture_dir),
            "seen_count": 1,
            "class_name": name,
            "event_type": event_type,
            "risk_level": risk_level,
            "recommended_action": recommended_action,
            "confidence": det.get("confidence"),
            "confidence_max": det.get("confidence"),
            "distance_m": det.get("depth_median_m"),
            "projection_status": projection_status,
            "odom_point_xy_m": odom_point,
            "odom_source": odom_source,
            "overlay_path": str(cap_out / "overlay.png"),
            "manual_arm_response_candidate": None,
        }
        event["manual_arm_response_candidate"] = build_manual_arm_candidate(event, output_dir, args)
        events_by_key[key] = event
        events.append(event)
        points.append(point)
        capture_events.append(event)
        append_jsonl(output_dir / "risk_events.jsonl", event)

    risk_detection = {
        "risk_detection_id": f"risk_detection_{capture_dir.name}",
        "timestamp": now_iso(),
        "capture_dir": str(capture_dir),
        "backend": backend_name,
        "runtime_backend": runtime_backend,
        "model_path": str(model_path),
        "model_used": True,
        "local_inference": True,
        "online_api_used": False,
        "load_latency_ms": round(load_latency_ms, 3),
        "inference_latency_ms": round(inference_latency_ms, 3),
        "inference_fps": round(1000.0 / inference_latency_ms, 3) if inference_latency_ms > 0 else None,
        "depth_scale_m": depth_scale_m,
        "odom_source": odom_source,
        **depth_status,
        "detections": detections,
        "events_created": len(capture_events),
    }
    write_json(cap_out / "risk_detection.json", risk_detection)
    return {
        "capture_dir": str(capture_dir),
        "status": "processed",
        "detections": len(detections),
        "events_created": len(capture_events),
        "odom_source": odom_source,
        "inference_latency_ms": risk_detection["inference_latency_ms"],
        "inference_fps": risk_detection["inference_fps"],
    }


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", action="append", required=True)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--iou", type=float, default=0.70)
    parser.add_argument("--max-det", type=int, default=20)
    parser.add_argument("--runtime", choices=["auto", "onnxruntime", "opencv_dnn"], default="auto")
    parser.add_argument("--providers", default="CPUExecutionProvider")
    parser.add_argument("--depth-scale-m", type=float, default=0.001)
    parser.add_argument("--dedup-map-grid-m", type=float, default=0.40)
    parser.add_argument("--synthetic-odom-if-missing", action="store_true")
    parser.add_argument("--synthetic-odom-x", type=float, default=0.0)
    parser.add_argument("--synthetic-odom-y", type=float, default=0.0)
    parser.add_argument("--synthetic-odom-yaw-rad", type=float, default=0.0)
    parser.add_argument("--arm-serial-port", default="/dev/arm_bus")
    parser.add_argument("--arm-baudrate", type=int, default=9600)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"model not found: {model_path}")
    events_by_key: Dict[str, Dict[str, Any]] = {}
    events: List[Dict[str, Any]] = []
    points: List[Dict[str, Any]] = []
    capture_results = []
    errors: List[Dict[str, Any]] = []

    for raw_capture_dir in args.capture_dir:
        capture_dir = Path(raw_capture_dir)
        try:
            capture_results.append(process_capture(capture_dir, model_path, output_dir, args, events_by_key, events, points))
        except Exception as exc:  # noqa: BLE001 - batch dry-run should preserve later captures.
            error = {"timestamp": now_iso(), "capture_dir": str(capture_dir), "error": str(exc)}
            errors.append(error)
            capture_results.append({"capture_dir": str(capture_dir), "status": "failed", "error": str(exc)})

    snapshot_written = plot_map_snapshot(output_dir, points)
    write_json(
        output_dir / "risk_event_index.json",
        {
            "schema_version": "risk_event_index_v1",
            "updated_at": now_iso(),
            "event_count": len(events),
            "events": events,
            "dedup": {"map_grid_m": args.dedup_map_grid_m, "key_count": len(events_by_key)},
        },
    )
    write_json(
        output_dir / "risk_map_points.json",
        {
            "schema_version": "risk_map_points_v1",
            "updated_at": now_iso(),
            "projection_mode": "d435_bbox_depth_odom_approx",
            "axis_mapping": AXIS_MAPPING,
            "camera_offset_base_m": list(CAMERA_OFFSET_BASE_M),
            "risk_map_points": points,
            "summary": {
                "risk_map_points": len(points),
                "projected": sum(1 for p in points if p.get("projection_status") == "projected"),
                "snapshot_written": snapshot_written,
                "synthetic_odom_points": sum(1 for p in points if p.get("odom_source") == "synthetic_dry_run"),
            },
        },
    )
    write_json(
        output_dir / "episode_report.json",
        {
            "episode_id": "prelim_yolo_map_dryrun",
            "protocol_version": PROTOCOL_VERSION,
            "created_at": now_iso(),
            "status": "succeeded" if not errors else "completed_with_errors",
            "captures": capture_results,
            "risk_points": points,
            "summary": {
                "capture_count": len(capture_results),
                "event_count": len(events),
                "risk_map_points": len(points),
                "synthetic_odom_if_missing": bool(args.synthetic_odom_if_missing),
                "synthetic_odom_points": sum(1 for p in points if p.get("odom_source") == "synthetic_dry_run"),
                "cmd_vel_published": False,
                "arm_auto_executed": False,
            },
            "errors": errors,
            "claim_boundary": [
                "Offline dry-run only.",
                "No ROS, cmd_vel, serial, chassis, or arm control is used.",
                "Manual arm no-load candidates are generated, not executed.",
                "Synthetic odom, when enabled, is only for visualization and not real map-localization evidence.",
            ],
        },
    )
    write_json(output_dir / "errors.json", errors)
    write_report(output_dir, events, points)
    write_dashboard(output_dir, events)
    write_text(
        output_dir / "README.md",
        "# Preliminary YOLO Map Dry-Run\n\n"
        "This directory contains offline dry-run evidence for YOLO risk detection and approximate map annotation.\n\n"
        "- `dashboard.html`\n"
        "- `risk_control_report.md`\n"
        "- `risk_event_index.json`\n"
        "- `risk_map_points.json`\n"
        "- `episode_report.json`\n",
    )
    print(
        json.dumps(
            {
                "output_dir": str(output_dir),
                "captures": len(capture_results),
                "events": len(events),
                "risk_map_points": len(points),
                "projected": sum(1 for p in points if p.get("projection_status") == "projected"),
                "synthetic_odom_points": sum(1 for p in points if p.get("odom_source") == "synthetic_dry_run"),
                "snapshot_written": snapshot_written,
                "errors": len(errors),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
