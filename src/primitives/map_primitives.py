"""Risk point projection, summary, and visualization helpers."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Dict, List, Tuple

from .schemas import write_json, write_text


def _count_by_class(points: List[Dict[str, Any]]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for point in points:
        key = str(point.get("class_name") or "unknown")
        counts[key] = counts.get(key, 0) + 1
    return counts


def _project_detection(det: Dict[str, Any], index: int) -> Dict[str, Any]:
    camera = det.get("camera_point_xyz_m") or {"x": 0.0, "y": 0.0, "z": det.get("depth_median_m") or 0.8}
    depth = float(det.get("depth_median_m") or camera.get("z") or 0.8)
    base_x = depth + 0.15
    base_y = -float(camera.get("x") or 0.0)
    odom_x = base_x
    odom_y = base_y
    return {
        "risk_id": f"risk_{index:03d}",
        "class_name": det.get("class_name", "unknown"),
        "confidence": float(det.get("confidence", 0.0)),
        "image_bbox_xywh": det.get("bbox_xywh", [0, 0, 0, 0]),
        "depth_median_m": depth,
        "camera_point_xyz_m": camera,
        "base_point_xyz_m": {"x": base_x, "y": base_y, "z": 0.0},
        "odom_point_xy": [odom_x, odom_y],
        "map_pixel_xy": [int(320 + odom_y * 80), int(240 - odom_x * 80)],
        "projection_status": "projected",
        "evidence_image_path": det.get("evidence_image_path"),
        "overlay_path": det.get("overlay_path"),
    }


def _write_csv(path: Path, points: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["risk_id", "class_name", "confidence", "depth_median_m", "odom_x", "odom_y", "projection_status"],
        )
        writer.writeheader()
        for point in points:
            xy = point.get("odom_point_xy") or [None, None]
            writer.writerow(
                {
                    "risk_id": point.get("risk_id"),
                    "class_name": point.get("class_name"),
                    "confidence": point.get("confidence"),
                    "depth_median_m": point.get("depth_median_m"),
                    "odom_x": xy[0],
                    "odom_y": xy[1],
                    "projection_status": point.get("projection_status"),
                }
            )


def _write_visualization(path: Path, points: List[Dict[str, Any]]) -> str | None:
    try:
        from PIL import Image, ImageDraw
    except Exception:
        return None
    img = Image.new("RGB", (640, 480), (245, 245, 245))
    draw = ImageDraw.Draw(img)
    draw.line((320, 460, 320, 20), fill=(170, 170, 170), width=1)
    draw.line((20, 240, 620, 240), fill=(170, 170, 170), width=1)
    draw.ellipse((312, 432, 328, 448), fill=(0, 80, 220))
    draw.text((332, 430), "robot", fill=(0, 80, 220))
    for point in points:
        px, py = point.get("map_pixel_xy") or [320, 240]
        draw.ellipse((px - 7, py - 7, px + 7, py + 7), fill=(220, 0, 0))
        draw.text((px + 10, py - 8), point.get("risk_id", "risk"), fill=(180, 0, 0))
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path)
    return str(path)


def summarize_risk_map(
    risk_detection: Dict[str, Any],
    output_dir: str | Path | None = None,
    existing_risk_map_points: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    detections = risk_detection.get("detections") or []
    points = [_project_detection(det, idx) for idx, det in enumerate(detections, start=1)]
    summary = {
        "schema_version": "risk_map_summary_v1",
        "coordinate_source": "bbox + depth + camera_info + odom/map pose",
        "manual_distance_used_for_mapping": False,
        "risk_count_total": len(points),
        "risk_count_by_class": _count_by_class(points),
        "risk_points": points,
        "risk_statistics": {
            "projected": sum(1 for point in points if point.get("projection_status") == "projected"),
            "source_detection_backend": risk_detection.get("backend"),
        },
        "visualization_path": None,
        "claim_boundary": [
            "Risk map coordinates must come from bbox + depth + camera_info + odom/map pose, not manual_distance_m.",
            "Projection is approximate unless calibrated TF is validated.",
            "Do not claim high precision SLAM.",
        ],
    }
    if output_dir:
        out = Path(output_dir)
        vis = _write_visualization(out / "risk_map_visualization.png", points)
        summary["visualization_path"] = vis
        write_json(out / "risk_map_summary.json", summary)
        write_json(out / "risk_statistics.json", summary["risk_statistics"])
        write_json(out / "risk_map_points.json", {"risk_map_points": points})
        _write_csv(out / "risk_map_summary.csv", points)
        write_json(out / "errors.json", [] if vis else [{"code": "visualization_skipped", "message": "PIL unavailable"}])
    return summary
