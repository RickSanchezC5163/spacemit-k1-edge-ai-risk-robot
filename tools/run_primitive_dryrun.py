#!/usr/bin/env python3
"""Run one high-level primitive in dry-run mode."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.chassis_primitives import dry_run_chassis_primitive


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--primitive", default="HOLD")
    parser.add_argument("--base-zero", action="store_true")
    args = parser.parse_args()
    result = dry_run_chassis_primitive(args.primitive, {"base_zero": args.base_zero})
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["status"] != "blocked" else 1


if __name__ == "__main__":
    raise SystemExit(main())
