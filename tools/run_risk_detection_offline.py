#!/usr/bin/env python3
"""Run an offline risk-detection backend without ROS or hardware."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.risk_vision_primitives import detect_risk


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rgb", default="")
    parser.add_argument("--depth", default=None)
    parser.add_argument("--camera-info", default=None)
    parser.add_argument("--backend", default="hsv_red_rule")
    parser.add_argument("--output-dir", default="outputs/risk_detection_offline_v1/dryrun_001")
    args = parser.parse_args()
    result = detect_risk(args.rgb, args.depth, args.camera_info, args.backend, args.output_dir)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
