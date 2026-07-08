#!/usr/bin/env python3
"""RL-A0 mock PPO-style training bring-up.

This script intentionally does not import ROS, publish cmd_vel, or access K1
hardware. It trains a tiny softmax policy in a deterministic mock environment
and writes training artifacts for server bring-up validation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

try:
    import yaml
except ImportError as exc:  # pragma: no cover - dependency check path.
    raise SystemExit("Missing dependency: pyyaml. Install requirements-rl.txt") from exc


FEATURE_COUNT = 4


@dataclass
class MockState:
    x: float
    yaw: float
    front_range_m: float
    step_index: int


def load_config(path: Path) -> Dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("config root must be a mapping")
    return data


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def features(state: MockState, target_x: float) -> List[float]:
    return [
        1.0,
        float(target_x - state.x),
        float(state.front_range_m),
        float(abs(state.yaw)),
    ]


def softmax(logits: Sequence[float]) -> List[float]:
    max_logit = max(logits)
    exps = [math.exp(value - max_logit) for value in logits]
    total = sum(exps)
    return [value / total for value in exps]


def choose_action(weights: List[List[float]], state_features: Sequence[float], rng: random.Random) -> Tuple[int, List[float]]:
    logits = [sum(w * x for w, x in zip(row, state_features)) for row in weights]
    probs = softmax(logits)
    sample = rng.random()
    acc = 0.0
    for index, prob in enumerate(probs):
        acc += prob
        if sample <= acc:
            return index, probs
    return len(probs) - 1, probs


def step_env(state: MockState, action_name: str, cfg: Dict[str, Any], rng: random.Random) -> Tuple[MockState, float, bool]:
    target_x = float(cfg.get("target_x", 1.0))
    warning_m = float((cfg.get("mock_safety") or {}).get("front_warning_m", 0.80))
    stop_m = float((cfg.get("mock_safety") or {}).get("front_stop_m", 0.30))

    dx = 0.0
    dyaw = 0.0
    if action_name in {"forward_small", "forward_0p15"} and state.front_range_m > stop_m:
        dx = 0.035
    elif action_name in {"arc_left", "arc_fast_left"}:
        dyaw = 0.08
    elif action_name in {"arc_right", "arc_fast_right"}:
        dyaw = -0.08

    next_x = max(-0.2, min(target_x + 0.2, state.x + dx))
    next_yaw = max(-1.2, min(1.2, state.yaw + dyaw))
    front_range = max(0.15, warning_m - 0.35 * next_x + rng.uniform(-0.015, 0.015))
    next_state = MockState(x=next_x, yaw=next_yaw, front_range_m=front_range, step_index=state.step_index + 1)

    progress_reward = 2.0 * dx
    yaw_penalty = 0.05 * abs(next_yaw)
    stop_penalty = 0.25 if front_range <= stop_m else 0.0
    goal_bonus = 1.0 if next_x >= target_x else 0.0
    reward = progress_reward + goal_bonus - yaw_penalty - stop_penalty
    done = bool(next_x >= target_x or next_state.step_index >= int(cfg.get("steps_per_episode", 40)))
    return next_state, reward, done


def discounted_returns(rewards: Sequence[float], gamma: float) -> List[float]:
    running = 0.0
    returns: List[float] = []
    for reward in reversed(rewards):
        running = float(reward) + gamma * running
        returns.append(running)
    returns.reverse()
    return returns


def save_checkpoint(path: Path, episode: int, weights: List[List[float]], cfg: Dict[str, Any]) -> None:
    write_json(
        path,
        {
            "episode": episode,
            "weights": weights,
            "config": cfg,
            "hardware_connected": False,
            "ros_started": False,
            "cmd_vel_published": False,
            "arm_controlled": False,
        },
    )


def maybe_plot_reward_curve(csv_path: Path, png_path: Path) -> bool:
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return False

    episodes: List[int] = []
    rewards: List[float] = []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            episodes.append(int(row["episode"]))
            rewards.append(float(row["reward"]))
    plt.figure(figsize=(7, 4))
    plt.plot(episodes, rewards, linewidth=1.5)
    plt.xlabel("episode")
    plt.ylabel("reward")
    plt.title("RL-A0 Mock PPO Reward Curve")
    plt.grid(True, alpha=0.3)
    png_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(png_path, dpi=140)
    plt.close()
    return True


def train(cfg: Dict[str, Any], output_dir: Path) -> Dict[str, Any]:
    rng = random.Random(int(cfg.get("seed", 7)))
    actions = list(cfg.get("action_space") or ["hold", "forward_small", "arc_left", "arc_right"])
    weights = [[rng.uniform(-0.02, 0.02) for _ in range(FEATURE_COUNT)] for _ in actions]
    lr = float(cfg.get("learning_rate", 0.03))
    gamma = float(cfg.get("gamma", 0.97))
    episodes = int(cfg.get("episodes", 60))
    steps_per_episode = int(cfg.get("steps_per_episode", 40))
    checkpoint_every = max(1, int(cfg.get("checkpoint_every", 20)))

    rows: List[Dict[str, Any]] = []
    for episode in range(1, episodes + 1):
        state = MockState(
            x=float(cfg.get("start_x", 0.0)),
            yaw=0.0,
            front_range_m=float((cfg.get("mock_safety") or {}).get("front_warning_m", 0.80)) + 0.05,
            step_index=0,
        )
        trajectory: List[Tuple[List[float], int, List[float], float]] = []
        total_reward = 0.0
        done = False
        while not done and state.step_index < steps_per_episode:
            state_features = features(state, float(cfg.get("target_x", 1.0)))
            action_index, probs = choose_action(weights, state_features, rng)
            next_state, reward, done = step_env(state, actions[action_index], cfg, rng)
            trajectory.append((state_features, action_index, probs, reward))
            total_reward += reward
            state = next_state

        returns = discounted_returns([item[3] for item in trajectory], gamma)
        baseline = sum(returns) / len(returns) if returns else 0.0
        for (state_features, action_index, probs, _reward), ret in zip(trajectory, returns):
            advantage = ret - baseline
            for action_row, prob in enumerate(probs):
                coeff = (1.0 if action_row == action_index else 0.0) - prob
                for feature_index, value in enumerate(state_features):
                    weights[action_row][feature_index] += lr * advantage * coeff * value

        rows.append(
            {
                "episode": episode,
                "reward": round(total_reward, 6),
                "steps": len(trajectory),
                "final_x": round(state.x, 6),
                "final_front_range_m": round(state.front_range_m, 6),
            }
        )
        if episode % checkpoint_every == 0 or episode == episodes:
            save_checkpoint(output_dir / "checkpoints" / f"checkpoint_ep{episode:04d}.json", episode, weights, cfg)

    reward_csv = output_dir / "reward_curve.csv"
    output_dir.mkdir(parents=True, exist_ok=True)
    with reward_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["episode", "reward", "steps", "final_x", "final_front_range_m"])
        writer.writeheader()
        writer.writerows(rows)

    plotted = maybe_plot_reward_curve(reward_csv, output_dir / "reward_curve.png")
    summary = {
        "stage": "RL-A0",
        "status": "succeeded",
        "episodes": episodes,
        "steps_per_episode": steps_per_episode,
        "checkpoint_count": len(list((output_dir / "checkpoints").glob("checkpoint_ep*.json"))),
        "reward_curve_csv": str(reward_csv),
        "reward_curve_png": str(output_dir / "reward_curve.png") if plotted else None,
        "train_summary": str(output_dir / "train_summary.json"),
        "hardware_connected": False,
        "ros_started": False,
        "cmd_vel_published": False,
        "arm_controlled": False,
        "step7e2_reference": cfg.get("step7e2_reference", {}),
        "mock_safety": cfg.get("mock_safety", {}),
        "claim_boundary": [
            "Mock server-side PPO-style bring-up only.",
            "No real K1 connection, no ROS, no cmd_vel publication, and no arm control.",
            "Do not claim RL has controlled hardware.",
            "Step7-E2 is used only as a schema and safety-boundary reference for future RL-A1 alignment.",
        ],
    }
    write_json(output_dir / "train_summary.json", summary)
    (output_dir / "README.md").write_text(
        "# RL-A0 Mock PPO Training Output\n\n"
        "This directory contains mock server-side training artifacts only.\n"
        "It does not connect to K1, start ROS, publish cmd_vel, or control the arm.\n",
        encoding="utf-8",
    )
    return summary


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="rl/configs/rl_a0_mock_ppo.yaml")
    parser.add_argument("--output-dir", default="outputs/rl_a0_mock_ppo_v1/smoke_001")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    cfg = load_config(Path(args.config))
    summary = train(cfg, Path(args.output_dir))
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
