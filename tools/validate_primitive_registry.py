#!/usr/bin/env python3
"""Validate the unified primitive registry."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.registry import load_registry, validate_registry


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", default="configs/primitive_registry.yaml")
    args = parser.parse_args()
    registry = load_registry(args.registry)
    errors = validate_registry(registry)
    result = {
        "primitive_registry_valid": not errors,
        "primitive_count": len(registry.get("primitives") or {}),
        "category_count": len(registry.get("categories") or {}),
        "errors": errors,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
