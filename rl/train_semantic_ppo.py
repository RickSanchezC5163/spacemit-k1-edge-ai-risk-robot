#!/usr/bin/env python3
"""Train a tiny semantic-policy mock loop over high-level primitives."""

from __future__ import annotations

import argparse
import csv
import json
import random
import sys
from pathlib import Path
from typing import Any, Dict, Sequence

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl.envs.semantic_guarded_nav_env import SemanticGuardedNavEnv, default_action_space


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="rl/configs/rl_semantic_ppo.yaml")
    parser.add_argument("--output-dir", default="outputs/rl_semantic_train_v1/smoke_001")
    args = parser.parse_args(argv)
    cfg = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    rng = random.Random(int(cfg.get("seed", 11)))
    actions = default_action_space()
    episodes = int(cfg.get("episodes", 40))
    checkpoint_every = int(cfg.get("checkpoint_every", 20))
    env = SemanticGuardedNavEnv(max_steps=int((cfg.get("env") or {}).get("max_steps", 40)))
    out = Path(args.output_dir)
    rows = []
    policy_counts = {name: 1 for name in actions}

    for episode in range(1, episodes + 1):
        env.reset()
        done = False
        total_reward = 0.0
        steps = 0
        while not done:
            weights = [policy_counts[name] for name in actions]
            action = rng.choices(actions, weights=weights, k=1)[0]
            _obs, reward, done, _info = env.step(action)
            total_reward += reward
            steps += 1
            if reward > 0:
                policy_counts[action] += 1
        rows.append({"episode": episode, "reward": round(total_reward, 6), "steps": steps})
        if episode % checkpoint_every == 0 or episode == episodes:
            write_json(out / "checkpoints" / f"checkpoint_ep{episode:04d}.json", {"episode": episode, "policy_counts": policy_counts})

    out.mkdir(parents=True, exist_ok=True)
    with (out / "reward_curve.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["episode", "reward", "steps"])
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "stage": "RL-A1-semantic-mock",
        "status": "succeeded",
        "episodes": episodes,
        "checkpoint_count": len(list((out / "checkpoints").glob("checkpoint_ep*.json"))),
        "action_space": actions,
        "cmd_vel_published": False,
        "servo_pulse_output": False,
        "hardware_connected": False,
        "ros_started": False,
        "claim_boundary": cfg.get("claim_boundary", []),
    }
    write_json(out / "train_summary.json", summary)
    write_json(out / "config.yaml.json", cfg)
    write_json(out / "action_space.yaml.json", {"actions": actions})
    (out / "README.md").write_text("Semantic RL mock training only; no hardware control.\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
