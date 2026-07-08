#!/usr/bin/env python3
"""Validate action semantics against the primitive registry."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.registry import load_action_semantics, load_registry, validate_action_semantics


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action-semantics", default="configs/action_semantics.yaml")
    parser.add_argument("--registry", default="configs/primitive_registry.yaml")
    args = parser.parse_args()
    registry = load_registry(args.registry)
    semantics = load_action_semantics(args.action_semantics)
    errors = validate_action_semantics(semantics, registry)
    result = {
        "action_semantics_valid": not errors,
        "action_count": len(semantics.get("actions") or {}),
        "errors": errors,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
