#!/usr/bin/env python3
"""Generate one mock risk point from a captured RGB/depth evidence bundle."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from edge_robot_protocol import RiskPoint, now_iso, to_jsonable  # noqa: E402


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(to_jsonable(data), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def infer_depth_scale(depth: np.ndarray, capture_meta: Optional[Dict[str, Any]]) -> float:
    if capture_meta:
        depth_meta = capture_meta.get("depth") or {}
        scale = depth_meta.get("depth_scale_m")
        if scale is not None:
            return float(scale)
    if depth.dtype == np.dtype(np.uint16):
        return 0.001
    return 1.0


def parse_bbox(raw: Optional[str], width: int, height: int) -> Dict[str, int]:
    if raw:
        parts = [int(v.strip()) for v in raw.split(",")]
        if len(parts) != 4:
            raise ValueError("--bbox must be x,y,w,h")
        x, y, w, h = parts
    else:
        w = max(8, int(round(width * 0.25)))
        h = max(8, int(round(height * 0.25)))
        x = max(0, int(round((width - w) / 2.0)))
        y = max(0, int(round((height - h) / 2.0)))
    if w <= 0 or h <= 0:
        raise ValueError(f"invalid bbox size: {x},{y},{w},{h}")
    x = max(0, min(int(x), width - 1))
    y = max(0, min(int(y), height - 1))
    w = max(1, min(int(w), width - x))
    h = max(1, min(int(h), height - y))
    return {"x": x, "y": y, "w": w, "h": h}


def camera_intrinsics(camera_info: Dict[str, Any]) -> Tuple[float, float, float, float]:
    k = camera_info.get("k")
    if not isinstance(k, list) or len(k) < 6:
        raise ValueError("camera_info.json missing K intrinsics")
    fx = float(k[0])
    fy = float(k[4])
    cx = float(k[2])
    cy = float(k[5])
    if fx == 0.0 or fy == 0.0:
        raise ValueError(f"invalid camera intrinsics fx={fx} fy={fy}")
    return fx, fy, cx, cy


def compute_mock_risk_point(
    capture_dir: Path,
    bbox_raw: Optional[str] = None,
    output_path: Optional[Path] = None,
    label: str = "mock_risk_point",
    confidence: float = 0.5,
) -> RiskPoint:
    capture_meta_path = capture_dir / "capture_meta.json"
    depth_raw_path = capture_dir / "depth_raw.npy"
    camera_info_path = capture_dir / "camera_info.json"
    rgb_path = capture_dir / "rgb.png"
    depth_vis_path = capture_dir / "depth_vis.png"
    if not capture_meta_path.exists():
        raise FileNotFoundError(f"missing {capture_meta_path}")
    if not depth_raw_path.exists():
        raise FileNotFoundError(f"missing {depth_raw_path}")
    if not camera_info_path.exists():
        raise FileNotFoundError(f"missing {camera_info_path}")

    capture_meta = load_json(capture_meta_path)
    camera_info = load_json(camera_info_path)
    depth_raw = np.load(depth_raw_path)
    if depth_raw.ndim == 3:
        depth_raw = np.squeeze(depth_raw)
    if depth_raw.ndim != 2:
        raise ValueError(f"depth_raw.npy must be 2D after squeeze, got {depth_raw.shape}")

    height, width = depth_raw.shape
    bbox = parse_bbox(bbox_raw, width, height)
    x0 = bbox["x"]
    y0 = bbox["y"]
    x1 = x0 + bbox["w"]
    y1 = y0 + bbox["h"]
    roi_raw = depth_raw[y0:y1, x0:x1]
    depth_scale_m = infer_depth_scale(depth_raw, capture_meta)
    roi_m = roi_raw.astype(np.float32) * float(depth_scale_m)
    valid = roi_m[np.isfinite(roi_m) & (roi_m > 0.0)]
    if valid.size == 0:
        raise ValueError(f"bbox has no valid depth samples: {bbox}")

    bbox_total_pixels = int(bbox["w"] * bbox["h"])
    bbox_valid_depth_samples = int(valid.size)
    bbox_valid_depth_ratio = (
        0.0 if bbox_total_pixels == 0 else bbox_valid_depth_samples / bbox_total_pixels
    )
    depth_median_m = float(np.median(valid))
    fx, fy, cx, cy = camera_intrinsics(camera_info)
    u = float(x0 + (bbox["w"] - 1) / 2.0)
    v = float(y0 + (bbox["h"] - 1) / 2.0)
    camera_point = {
        "x": float((u - cx) * depth_median_m / fx),
        "y": float((v - cy) * depth_median_m / fy),
        "z": depth_median_m,
    }
    capture_id = str(capture_meta.get("capture_id") or capture_dir.name)
    evidence_paths = {
        "capture_dir": str(capture_dir),
        "capture_meta": str(capture_meta_path),
        "rgb": str(rgb_path),
        "depth_raw": str(depth_raw_path),
        "depth_vis": str(depth_vis_path),
        "camera_info": str(camera_info_path),
    }
    risk_point = RiskPoint(
        risk_point_id=f"risk_{capture_id}_{int(time.time() * 1000)}",
        capture_id=capture_id,
        label=label,
        bbox_xywh=bbox,
        depth_median_m=depth_median_m,
        camera_point_xyz_m=camera_point,
        confidence=max(0.0, min(1.0, float(confidence))),
        evidence_paths=evidence_paths,
        depth_scale_m=float(depth_scale_m),
        bbox_valid_depth_samples=bbox_valid_depth_samples,
        bbox_valid_depth_ratio=float(bbox_valid_depth_ratio),
        notes=[
            "mock detector: fixed/configured bbox",
            f"valid_depth_samples={bbox_valid_depth_samples}",
            f"depth_scale_m={float(depth_scale_m)}",
        ],
    )
    out = output_path or (capture_dir / "risk_point.json")
    write_json(out, risk_point)
    return risk_point


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a mock risk point from capture RGB/depth/camera_info."
    )
    parser.add_argument("--capture-dir", required=True)
    parser.add_argument("--bbox", default=None, help="x,y,w,h. Defaults to center quarter.")
    parser.add_argument("--output", default=None)
    parser.add_argument("--label", default="mock_risk_point")
    parser.add_argument("--confidence", type=float, default=0.5)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    capture_dir = Path(args.capture_dir)
    output_path = Path(args.output) if args.output else None
    try:
        risk_point = compute_mock_risk_point(
            capture_dir=capture_dir,
            bbox_raw=args.bbox,
            output_path=output_path,
            label=args.label,
            confidence=args.confidence,
        )
        print(json.dumps(to_jsonable(risk_point), ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:  # noqa: BLE001 - CLI must write error evidence.
        error = {
            "timestamp": now_iso(),
            "tool": "mock_risk_detector",
            "capture_dir": str(capture_dir),
            "error": str(exc),
        }
        write_json(capture_dir / "risk_point_error.json", error)
        print(json.dumps(error, ensure_ascii=False, indent=2), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
