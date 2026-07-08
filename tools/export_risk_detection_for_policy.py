#!/usr/bin/env python3
"""Export risk detections into lightweight PolicyState risk fields.

This tool is format-only glue. It does not start ROS, publish cmd_vel, open
serial devices, or execute robot actions.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional


def _class_id(det: Dict[str, Any]) -> Optional[int]:
    for key in ("class_id", "class_index"):
        value = det.get(key)
        if value is not None:
            try:
                return int(value)
            except (TypeError, ValueError):
                return None
    return None


def _confidence(det: Dict[str, Any]) -> float:
    try:
        return float(det.get("confidence", 0.0))
    except (TypeError, ValueError):
        return 0.0


def detections_to_policy_risk_fields(detections: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Return PolicyState-compatible risk_* fields from detections."""

    valid = [det for det in detections if _confidence(det) > 0.0]
    if not valid:
        return {
            "risk_detected": False,
            "risk_confidence": None,
            "risk_class_id": None,
            "risk_class_name": None,
            "risk_distance_m": None,
            "risk_source": "yolov8n_onnx_cpu",
        }
    best = max(valid, key=_confidence)
    return {
        "risk_detected": True,
        "risk_confidence": round(_confidence(best), 4),
        "risk_class_id": _class_id(best),
        "risk_class_name": best.get("class_name"),
        "risk_distance_m": best.get("depth_median_m"),
        "risk_source": "yolov8n_onnx_cpu",
    }


def _detections_from_payload(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    if isinstance(payload.get("detections"), list):
        return payload["detections"]
    if isinstance(payload.get("results"), list):
        return payload["results"]
    return []


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--risk-detection", required=True, help="Path to risk_detection.json or detections JSON.")
    parser.add_argument("--output", default=None, help="Optional output JSON path. Defaults to stdout only.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    payload = json.loads(Path(args.risk_detection).read_text(encoding="utf-8"))
    result = detections_to_policy_risk_fields(_detections_from_payload(payload))
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
