# RL-A0 Server Training Bring-Up - 2026-07-01

## Goal

RL-A0 is a server-side training bring-up stage for the future policy-learning
track. It is intentionally disconnected from the real K1 robot.

Target output from a training smoke run:

- checkpoint files
- reward curve CSV
- optional reward curve PNG
- train summary JSON

## Boundaries

RL-A0 does not:

- connect to the real K1
- start ROS
- publish real `cmd_vel`
- control the chassis
- control the mechanical arm
- access the bus-servo controller
- claim RL has controlled hardware
- claim autonomous navigation on the real robot

RL-A0 only claims:

- a mock environment can run on a server
- a PPO-style training loop can produce repeatable artifacts
- checkpoints, reward curves, and summaries can be generated for later review

## Step7-E2 Alignment

RL-A0 should now be treated as a training bring-up that is aligned to the
current Step7-E2 fastdemo baseline only at the schema and safety-boundary level.
It must not be interpreted as a controller for the real robot.

Reference reproduction document:

```text
docs/step7e2_fastdemo_reproduction_20260630.md
```

Preferred live evidence baseline:

```text
outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_010/
```

Relevant Step7-E2 facts for future RL-A1 design:

- chassis commands must remain behind the existing safety chain:
  `/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded`
- stable live-demo guard parameters include:
  - `hard_stop_m=0.30`
  - `emergency_stop_m=0.20`
  - `slow_down_m=0.80`
  - `approach_stop_m=0.80`
  - `min_effective_forward=0.08`
  - `clear_max_linear=0.30`
  - `soft_max_linear=0.30`
- the reproducible scene window is approximately
  `0.60m <= front_p10 < 0.80m`
- the stable reference behavior is two `ARC_FAST_RIGHT` actions followed by
  `HOLD_MAX_FAST_ARC` with stop reason
  `max_consecutive_fast_arc_reached`
- Step7-E2 can be used as an observation/action schema reference, not as proof
  that RL controls hardware

The RL-A0 config therefore uses Step7-like mock action names:

```text
hold
forward_0p15
arc_fast_left
arc_fast_right
```

These are mock policy outputs only. A future RL-A1 runner should consume them
as recommendations or offline labels, and the real K1 must still execute only
through the P4/P5 guarded stack.

## Files

```text
rl/configs/rl_a0_mock_ppo.yaml
rl/train_ppo_mock.py
requirements-rl.txt
```

## Suggested Server Environment

```bash
cd /path/to/edge-ai-robot-k1
python3 -m venv .venv-rl
source .venv-rl/bin/activate
pip install -r requirements-rl.txt
```

The mock script avoids ROS and hardware dependencies. GPU is optional for this
bring-up; the first pass is allowed to run on CPU.

## Smoke Run

```bash
python3 rl/train_ppo_mock.py \
  --config rl/configs/rl_a0_mock_ppo.yaml \
  --output-dir outputs/rl_a0_mock_ppo_v1/smoke_001
```

Expected outputs:

```text
outputs/rl_a0_mock_ppo_v1/smoke_001/
  checkpoints/
  reward_curve.csv
  reward_curve.png        # optional if matplotlib is installed
  train_summary.json
  README.md
```

## Acceptance Criteria

- process exits with code `0`
- at least one checkpoint is written
- `reward_curve.csv` exists and has episode rows
- `train_summary.json` exists
- `train_summary.json.hardware_connected=false`
- `train_summary.json.ros_started=false`
- `train_summary.json.cmd_vel_published=false`
- `train_summary.json.arm_controlled=false`
- `train_summary.json.step7e2_reference` is present
- `train_summary.json.mock_safety.command_path` equals
  `/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded`

## Next Step After RL-A0

Only after RL-A0 artifacts are stable, consider RL-A1:

- align mock observation/action schema with Step7-E2
  `episode_report.json`, P4 guarded-policy reports, and Map-A risk-point
  records
- keep output actions as recommendations only
- keep real chassis control under the existing P4/P5 guarded stack
- do not put RL directly in the hardware control loop
- do not claim RL-driven navigation until a separate guarded dry-run, simulator,
  and hardware-gated validation chain exists
