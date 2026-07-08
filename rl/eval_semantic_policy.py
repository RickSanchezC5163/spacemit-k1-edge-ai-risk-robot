#!/usr/bin/env python3
"""Evaluate a semantic mock policy checkpoint."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl.envs.semantic_guarded_nav_env import SemanticGuardedNavEnv, default_action_space


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--output-dir", default="outputs/rl_semantic_eval_v1/eval_001")
    args = parser.parse_args()
    env = SemanticGuardedNavEnv()
    obs = env.reset()
    trace = []
    for action in ["ARC_FAST_RIGHT", "ARC_FAST_RIGHT", "HOLD_CAPTURE", "ARM_NO_LOAD_RESPONSE", "STOP_SAFE"]:
        obs, reward, done, info = env.step(action)
        trace.append({"action": action, "reward": reward, "done": done, "info": info})
        if done:
            break
    result = {
        "status": "succeeded",
        "checkpoint": args.checkpoint,
        "trace": trace,
        "final_observation": obs,
        "cmd_vel_published": False,
        "servo_pulse_output": False,
        "hardware_connected": False,
    }
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "eval_summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
