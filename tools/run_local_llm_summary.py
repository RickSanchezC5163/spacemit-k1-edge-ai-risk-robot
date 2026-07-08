#!/usr/bin/env python3
"""Generate a deterministic or local-LLM risk-control report."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.report_primitives import generate_risk_report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-report", required=True)
    parser.add_argument("--risk-map-summary", default=None)
    parser.add_argument("--backend", default="deterministic")
    parser.add_argument("--output-dir", default="outputs/local_llm_summary_v1/dryrun_001")
    args = parser.parse_args()
    result = generate_risk_report(args.episode_report, args.risk_map_summary, args.backend, args.output_dir)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
