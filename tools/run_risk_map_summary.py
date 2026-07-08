#!/usr/bin/env python3
"""Build a risk map summary from risk_detection.json."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.map_primitives import summarize_risk_map
from src.primitives.schemas import read_json


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--risk-detection", required=True)
    parser.add_argument("--output-dir", default="outputs/risk_map_summary_v1/dryrun_001")
    args = parser.parse_args()
    risk_detection = read_json(args.risk_detection)
    result = summarize_risk_map(risk_detection, args.output_dir)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
