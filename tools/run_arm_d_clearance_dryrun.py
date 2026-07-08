#!/usr/bin/env python3
"""Generate an Arm-D clearance candidate dry-run without hardware access."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.arm_primitives import clearance_candidate_dryrun
from src.primitives.schemas import read_json


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--risk-map-summary", default=None)
    parser.add_argument("--output-dir", default="outputs/arm_d_clearance_v1/d0_dryrun")
    args = parser.parse_args()
    summary = read_json(args.risk_map_summary) if args.risk_map_summary else {"risk_points": []}
    result = clearance_candidate_dryrun(summary, args.output_dir)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
