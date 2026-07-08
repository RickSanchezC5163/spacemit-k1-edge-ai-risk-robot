"""D435 bbox/depth localization helpers.

This module is offline and file-based. It does not start ROS, open camera
devices, or claim calibrated high-precision 3D localization.
"""

from __future__ import annotations

import json
from pathlib import Path
from statistics import median
from typing import Any, Dict, Iterable, List, Sequence


def _empty_result(bbox_xywh: Sequence[float] | None, status: str, reason: str) -> Dict[str, Any]:
    return {
        "bbox_xywh": list(bbox_xywh or [0, 0, 0, 0]),
        "depth_median_m": None,
        "bbox_valid_depth_ratio": 0.0,
        "camera_point_xyz_m": None,
        "depth_status": status,
        "reason": reason,
        "claim_boundary": [
            "D435 bbox depth localization is approximate unless TF and camera calibration are validated.",
            "Missing or invalid depth/camera_info must not be backfilled with inferred evidence.",
        ],
    }


def _load_depth_values(path: Path) -> tuple[int, int, Any] | None:
    if not path.exists():
        return None
    suffix = path.suffix.lower()
    if suffix == ".npy":
        import numpy as np

        arr = np.load(path)
        if arr.ndim != 2:
            return None
        return int(arr.shape[1]), int(arr.shape[0]), arr
    try:
        from PIL import Image

        img = Image.open(path)
        arr = list(img.getdata())
        return img.width, img.height, arr
    except Exception:
        return None


def _depth_at(depth_data: Any, width: int, x: int, y: int) -> float:
    try:
        return float(depth_data[y, x])
    except Exception:
        return float(depth_data[y * width + x])


def _camera_intrinsics(camera_info: Dict[str, Any]) -> tuple[float, float, float, float] | None:
    for key in ("k", "K"):
        values = camera_info.get(key)
        if isinstance(values, list) and len(values) >= 9:
            return float(values[0]), float(values[4]), float(values[2]), float(values[5])
    matrix = camera_info.get("camera_matrix")
    if isinstance(matrix, dict):
        values = matrix.get("data")
        if isinstance(values, list) and len(values) >= 9:
            return float(values[0]), float(values[4]), float(values[2]), float(values[5])
    intrinsics = camera_info.get("intrinsics")
    if isinstance(intrinsics, dict):
        keys = ("fx", "fy", "cx", "cy")
        if all(key in intrinsics for key in keys):
            return tuple(float(intrinsics[key]) for key in keys)  # type: ignore[return-value]
    if all(key in camera_info for key in ("fx", "fy", "cx", "cy")):
        return tuple(float(camera_info[key]) for key in ("fx", "fy", "cx", "cy"))  # type: ignore[return-value]
    return None


def _load_camera_info(path: Path) -> Dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _bbox_bounds(bbox_xywh: Sequence[float], width: int, height: int) -> tuple[int, int, int, int] | None:
    if len(bbox_xywh) < 4:
        return None
    x, y, w, h = [int(round(float(value))) for value in bbox_xywh[:4]]
    if w <= 0 or h <= 0:
        return None
    x0 = max(0, min(width - 1, x))
    y0 = max(0, min(height - 1, y))
    x1 = max(0, min(width, x + w))
    y1 = max(0, min(height, y + h))
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def localize_bbox_with_depth(
    bbox_xywh: Sequence[float],
    depth_path: str | None,
    camera_info_path: str | None,
    depth_scale_m: float = 0.001,
    min_valid_depth_m: float = 0.15,
    max_valid_depth_m: float = 5.0,
) -> Dict[str, Any]:
    """Compute median depth and rough camera-frame point for a bbox.

    The back-projection uses the bbox center and camera intrinsics. It is a
    file-level evidence helper, not a calibrated TF projection.
    """

    if not depth_path:
        return _empty_result(bbox_xywh, "missing", "depth_path missing")
    if not camera_info_path:
        return _empty_result(bbox_xywh, "missing", "camera_info_path missing")

    depth = _load_depth_values(Path(depth_path))
    if depth is None:
        return _empty_result(bbox_xywh, "missing", "depth file missing or unreadable")
    width, height, depth_data = depth
    bounds = _bbox_bounds(bbox_xywh, width, height)
    if bounds is None:
        return _empty_result(bbox_xywh, "invalid", "bbox invalid or outside depth image")

    camera_info = _load_camera_info(Path(camera_info_path))
    if camera_info is None:
        return _empty_result(bbox_xywh, "missing", "camera_info missing or unreadable")
    intrinsics = _camera_intrinsics(camera_info)
    if intrinsics is None:
        return _empty_result(bbox_xywh, "missing", "camera intrinsics missing")
    fx, fy, cx, cy = intrinsics
    if fx == 0.0 or fy == 0.0:
        return _empty_result(bbox_xywh, "invalid", "camera intrinsics invalid")

    x0, y0, x1, y1 = bounds
    valid_depths: List[float] = []
    sample_count = 0
    for yy in range(y0, y1):
        for xx in range(x0, x1):
            sample_count += 1
            raw = _depth_at(depth_data, width, xx, yy)
            depth_m = raw * depth_scale_m
            if min_valid_depth_m <= depth_m <= max_valid_depth_m:
                valid_depths.append(depth_m)

    if not valid_depths or sample_count == 0:
        return _empty_result(bbox_xywh, "invalid", "no valid depth in bbox")

    depth_median_m = float(median(valid_depths))
    center_u = (x0 + x1 - 1) / 2.0
    center_v = (y0 + y1 - 1) / 2.0
    x_m = (center_u - cx) * depth_median_m / fx
    y_m = (center_v - cy) * depth_median_m / fy
    valid_ratio = len(valid_depths) / float(sample_count)
    return {
        "bbox_xywh": list(bbox_xywh[:4]),
        "depth_median_m": round(depth_median_m, 4),
        "bbox_valid_depth_ratio": round(valid_ratio, 4),
        "camera_point_xyz_m": {
            "x": round(x_m, 4),
            "y": round(y_m, 4),
            "z": round(depth_median_m, 4),
        },
        "depth_status": "valid",
        "depth_scale_m": depth_scale_m,
        "min_valid_depth_m": min_valid_depth_m,
        "max_valid_depth_m": max_valid_depth_m,
        "claim_boundary": [
            "D435 bbox depth localization is approximate unless TF and camera calibration are validated.",
            "Do not claim high-precision 3D localization from this helper alone.",
        ],
    }
