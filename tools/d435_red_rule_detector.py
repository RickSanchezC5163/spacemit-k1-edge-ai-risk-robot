#!/usr/bin/env python3
"""Detect a red region in a D435 capture using a deterministic HSV rule."""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - K1 may not have cv2.
    cv2 = None

try:
    from PIL import Image, ImageDraw
except ImportError as exc:  # pragma: no cover - required for PNG IO.
    raise SystemExit(f"PIL is required for d435_red_rule_detector.py: {exc}") from exc


DETECTOR_VERSION = "d435_red_rule_detector_v1"
RISK_TRIGGER_SOURCE = "D435_red_color_rule"
DETECTION_MODE = "hsv_rule_based_red_color"


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


def rgb_to_hsv_numpy(rgb: np.ndarray) -> np.ndarray:
    rgb_f = rgb.astype(np.float32) / 255.0
    r = rgb_f[:, :, 0]
    g = rgb_f[:, :, 1]
    b = rgb_f[:, :, 2]
    cmax = np.max(rgb_f, axis=2)
    cmin = np.min(rgb_f, axis=2)
    delta = cmax - cmin
    hue = np.zeros_like(cmax)

    mask = delta > 1e-6
    rmax = mask & (cmax == r)
    gmax = mask & (cmax == g)
    bmax = mask & (cmax == b)
    hue[rmax] = ((g[rmax] - b[rmax]) / delta[rmax]) % 6.0
    hue[gmax] = ((b[gmax] - r[gmax]) / delta[gmax]) + 2.0
    hue[bmax] = ((r[bmax] - g[bmax]) / delta[bmax]) + 4.0
    hue = hue * 30.0  # OpenCV HSV hue scale: 0..179.

    sat = np.zeros_like(cmax)
    nonzero = cmax > 1e-6
    sat[nonzero] = delta[nonzero] / cmax[nonzero]
    val = cmax
    hsv = np.stack(
        [
            np.clip(hue, 0, 179).astype(np.uint8),
            np.clip(sat * 255.0, 0, 255).astype(np.uint8),
            np.clip(val * 255.0, 0, 255).astype(np.uint8),
        ],
        axis=2,
    )
    return hsv


def red_mask(rgb: np.ndarray) -> np.ndarray:
    if cv2 is not None:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        lower1 = np.array([0, 70, 40], dtype=np.uint8)
        upper1 = np.array([12, 255, 255], dtype=np.uint8)
        lower2 = np.array([168, 70, 40], dtype=np.uint8)
        upper2 = np.array([179, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        mask = cv2.medianBlur(mask, 5)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask > 0

    hsv = rgb_to_hsv_numpy(rgb)
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    return ((hue <= 12) | (hue >= 168)) & (sat >= 70) & (val >= 40)


def components_cv(mask: np.ndarray, min_area_px: int) -> List[Dict[str, Any]]:
    assert cv2 is not None
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    components: List[Dict[str, Any]] = []
    for label in range(1, num_labels):
        x, y, w, h, area = stats[label]
        if int(area) < min_area_px:
            continue
        cx, cy = centroids[label]
        components.append(
            {
                "label_index": int(label),
                "bbox_xywh": [int(x), int(y), int(w), int(h)],
                "area_px": int(area),
                "centroid_xy": [float(cx), float(cy)],
            }
        )
    components.sort(key=lambda item: item["area_px"], reverse=True)
    return components


def components_numpy(mask: np.ndarray, min_area_px: int) -> List[Dict[str, Any]]:
    h, w = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    components: List[Dict[str, Any]] = []
    label = 0
    for y0 in range(h):
        xs = np.where(mask[y0] & ~visited[y0])[0]
        for x0_raw in xs:
            x0 = int(x0_raw)
            if visited[y0, x0] or not mask[y0, x0]:
                continue
            label += 1
            queue: deque[Tuple[int, int]] = deque([(x0, y0)])
            visited[y0, x0] = True
            points: List[Tuple[int, int]] = []
            while queue:
                x, y = queue.popleft()
                points.append((x, y))
                for ny in range(max(0, y - 1), min(h, y + 2)):
                    for nx in range(max(0, x - 1), min(w, x + 2)):
                        if visited[ny, nx] or not mask[ny, nx]:
                            continue
                        visited[ny, nx] = True
                        queue.append((nx, ny))
            area = len(points)
            if area < min_area_px:
                continue
            xs_arr = np.array([p[0] for p in points])
            ys_arr = np.array([p[1] for p in points])
            x_min = int(xs_arr.min())
            y_min = int(ys_arr.min())
            x_max = int(xs_arr.max())
            y_max = int(ys_arr.max())
            components.append(
                {
                    "label_index": int(label),
                    "bbox_xywh": [x_min, y_min, x_max - x_min + 1, y_max - y_min + 1],
                    "area_px": int(area),
                    "centroid_xy": [float(xs_arr.mean()), float(ys_arr.mean())],
                }
            )
    components.sort(key=lambda item: item["area_px"], reverse=True)
    return components


def component_pixel_mask(mask: np.ndarray, selected: Dict[str, Any]) -> np.ndarray:
    x, y, w, h = selected["bbox_xywh"]
    roi = mask[y : y + h, x : x + w]
    if cv2 is None:
        return roi
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(roi.astype(np.uint8), connectivity=8)
    best_label = 0
    best_area = 0
    for label in range(1, num_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area > best_area:
            best_label = label
            best_area = area
    return labels == best_label if best_label else roi


def depth_and_point(
    selected: Optional[Dict[str, Any]],
    mask: np.ndarray,
    depth: np.ndarray,
    camera_info: Dict[str, Any],
    depth_scale_m: float,
) -> Tuple[Optional[float], Optional[float], Optional[Dict[str, float]]]:
    if selected is None:
        return None, None, None
    x, y, w, h = selected["bbox_xywh"]
    component_mask = component_pixel_mask(mask, selected)
    depth_roi = depth[y : y + h, x : x + w]
    valid = (depth_roi > 0) & component_mask
    if not np.any(valid):
        return None, 0.0, None
    depth_values = depth_roi[valid].astype(np.float64) * float(depth_scale_m)
    depth_median_m = float(np.median(depth_values))
    valid_ratio = float(valid.sum() / max(1, component_mask.sum()))

    k = camera_info.get("k") or camera_info.get("K") or []
    fx = float(k[0])
    fy = float(k[4])
    cx0 = float(k[2])
    cy0 = float(k[5])
    u, v = selected["centroid_xy"]
    point = {
        "x": float((u - cx0) * depth_median_m / fx),
        "y": float((v - cy0) * depth_median_m / fy),
        "z": float(depth_median_m),
    }
    return depth_median_m, valid_ratio, point


def draw_overlay(rgb: np.ndarray, mask: np.ndarray, selected: Optional[Dict[str, Any]], depth_median_m: Optional[float]) -> np.ndarray:
    overlay = rgb.copy()
    red_layer = np.zeros_like(overlay)
    red_layer[:, :, 0] = 255
    overlay = np.where(mask[:, :, None], (0.55 * overlay + 0.45 * red_layer).astype(np.uint8), overlay)
    image = Image.fromarray(overlay)
    draw = ImageDraw.Draw(image)
    if selected is not None:
        x, y, w, h = selected["bbox_xywh"]
        draw.rectangle([x, y, x + w, y + h], outline=(255, 255, 0), width=3)
        label = "red rule bbox"
        if depth_median_m is not None:
            label += f" depth={depth_median_m:.2f}m"
        draw.text((x, max(0, y - 16)), label, fill=(255, 255, 0))
    return np.array(image)


def detect(args: argparse.Namespace) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    capture_dir = Path(args.capture_dir)
    output_dir = Path(args.output_dir) if args.output_dir else capture_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    rgb_path = Path(args.rgb) if args.rgb else capture_dir / "rgb.png"
    depth_path = Path(args.depth_raw) if args.depth_raw else capture_dir / "depth_raw.npy"
    camera_info_path = Path(args.camera_info) if args.camera_info else capture_dir / "camera_info.json"
    capture_meta_path = capture_dir / "capture_meta.json"

    rgb = np.array(Image.open(rgb_path).convert("RGB"))
    depth = np.load(depth_path)
    camera_info = load_json(camera_info_path)
    mask = red_mask(rgb)
    components = (
        components_cv(mask, args.min_area_px)
        if cv2 is not None
        else components_numpy(mask, args.min_area_px)
    )
    selected = components[0] if components else None
    depth_median_m, valid_ratio, camera_point = depth_and_point(
        selected, mask, depth, camera_info, args.depth_scale_m
    )
    detected = selected is not None and depth_median_m is not None

    detection_path = output_dir / "red_object_rule_detection.json"
    mask_path = output_dir / "red_object_mask.png"
    overlay_path = output_dir / "red_object_overlay.png"
    risk_path = output_dir / "risk_point.json"
    Image.fromarray((mask.astype(np.uint8) * 255)).save(mask_path)
    Image.fromarray(draw_overlay(rgb, mask, selected, depth_median_m)).save(overlay_path)

    capture_meta = load_json(capture_meta_path) if capture_meta_path.exists() else {}
    capture_id = args.capture_id or capture_meta.get("capture_id") or capture_dir.name
    risk_point_id = args.risk_point_id or f"{capture_id}_red_rule_risk"
    evidence_paths = {
        "rgb": str(rgb_path),
        "depth_raw": str(depth_path),
        "camera_info": str(camera_info_path),
        "capture_meta": str(capture_meta_path) if capture_meta_path.exists() else None,
        "red_mask": str(mask_path),
        "red_overlay": str(overlay_path),
        "red_detection": str(detection_path),
        "risk_point": str(risk_path),
    }
    detection = {
        "schema_version": DETECTOR_VERSION,
        "generated_at": now_iso(),
        "detector": "deterministic_hsv_red_threshold",
        "detection_mode": DETECTION_MODE,
        "risk_trigger_source": RISK_TRIGGER_SOURCE,
        "model_used": False,
        "real_visual_model_used": False,
        "accuracy_claimed": False,
        "online_api_used": False,
        "source_rgb": str(rgb_path),
        "source_depth_raw": str(depth_path),
        "source_camera_info": str(camera_info_path),
        "image_shape_hwc": list(rgb.shape),
        "red_mask_pixels": int(mask.sum()),
        "red_mask_ratio": float(mask.sum() / (rgb.shape[0] * rgb.shape[1])),
        "component_count": len(components),
        "red_object_detected": bool(detected),
        "selected_component": selected,
        "bbox_xywh": selected.get("bbox_xywh") if selected else None,
        "depth_scale_m": float(args.depth_scale_m),
        "depth_median_m": depth_median_m,
        "bbox_valid_depth_ratio": valid_ratio,
        "camera_point_xyz_m": camera_point,
        "evidence_paths": evidence_paths,
        "claim_boundary": [
            "Fixed HSV red color rule only.",
            "No trained model or visual detection accuracy claim.",
            "No arm/contact/clearing action is triggered by this detector alone.",
        ],
    }
    risk_point = {
        "risk_point_id": risk_point_id,
        "capture_id": capture_id,
        "label": "red_object_rule",
        "category": "visual_rule_red_object",
        "risk_category": "visual_rule_red_object",
        "risk_trigger_source": RISK_TRIGGER_SOURCE,
        "detection_mode": DETECTION_MODE,
        "model_used": False,
        "accuracy_claimed": False,
        "red_object_detected": bool(detected),
        "bbox_xywh": selected.get("bbox_xywh") if selected else None,
        "bbox": {
            "x": selected["bbox_xywh"][0],
            "y": selected["bbox_xywh"][1],
            "width": selected["bbox_xywh"][2],
            "height": selected["bbox_xywh"][3],
        }
        if selected
        else None,
        "depth_median_m": depth_median_m,
        "depth_scale_m": float(args.depth_scale_m),
        "bbox_valid_depth_ratio": valid_ratio,
        "camera_point_xyz_m": camera_point,
        "confidence": None,
        "generated_by": "tools/d435_red_rule_detector.py",
        "evidence_paths": evidence_paths,
    }
    write_json(detection_path, detection)
    write_json(risk_path, risk_point)
    write_text(
        output_dir / "README_red_rule_detector.md",
        "# D435 Red-Rule Detector\n\n"
        f"- red_object_detected: `{detected}`\n"
        f"- detection_mode: `{DETECTION_MODE}`\n"
        f"- risk_trigger_source: `{RISK_TRIGGER_SOURCE}`\n"
        f"- bbox_xywh: `{risk_point.get('bbox_xywh')}`\n"
        f"- depth_median_m: `{depth_median_m}`\n"
        f"- bbox_valid_depth_ratio: `{valid_ratio}`\n\n"
        "Boundary: deterministic color rule only; no trained model or accuracy claim.\n",
    )
    return detection, risk_point


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", required=True)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--rgb", default=None)
    parser.add_argument("--depth-raw", default=None)
    parser.add_argument("--camera-info", default=None)
    parser.add_argument("--capture-id", default=None)
    parser.add_argument("--risk-point-id", default=None)
    parser.add_argument("--depth-scale-m", type=float, default=0.001)
    parser.add_argument("--min-area-px", type=int, default=80)
    parser.add_argument("--allow-no-detection", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    detection, risk_point = detect(args)
    detected = detection.get("red_object_detected") is True
    result = {
        "ok": bool(detected or args.allow_no_detection),
        "red_object_detected": detection.get("red_object_detected"),
        "negative_control_pass": bool(args.allow_no_detection and not detected),
        "risk_trigger_source": detection.get("risk_trigger_source"),
        "bbox_xywh": detection.get("bbox_xywh"),
        "depth_median_m": detection.get("depth_median_m"),
        "bbox_valid_depth_ratio": detection.get("bbox_valid_depth_ratio"),
        "camera_point_xyz_m": detection.get("camera_point_xyz_m"),
        "risk_point_id": risk_point.get("risk_point_id"),
        "capture_id": risk_point.get("capture_id"),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
