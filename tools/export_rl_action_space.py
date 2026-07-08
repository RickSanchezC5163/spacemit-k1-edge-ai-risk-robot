#!/usr/bin/env python3
"""Export the semantic RL action space for training or review."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.primitives.registry import ordered_enabled_actions
from src.primitives.schemas import read_yaml, write_json, write_text


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/rl_action_space.yaml")
    parser.add_argument("--output", default="outputs/rl_semantic_action_space_v1")
    parser.add_argument("--include-disabled", action="store_true")
    args = parser.parse_args()
    cfg = read_yaml(args.config)
    actions = ordered_enabled_actions(cfg, include_disabled=args.include_disabled)
    out = Path(args.output)
    result = {
        "schema_version": "rl_semantic_action_space_export_v1",
        "action_space_valid": True,
        "action_count": len(actions),
        "actions": actions,
        "observation_fields": cfg.get("observation_fields") or [],
        "execution_boundary": cfg.get("execution_boundary") or {},
    }
    write_json(out / "action_space.json", result)
    write_text(out / "README.md", "# RL Semantic Action Space\n\nHigh-level actions only; no cmd_vel or servo pulse output.\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
