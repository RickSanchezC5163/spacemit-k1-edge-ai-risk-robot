# Competition Completion Plan - 2026-07-01

## Current Validated Demo-Level Chain

```text
P4/N10P guarded motion
-> base_zero
-> D435 red-rule evidence
-> risk_point
-> approximate map projection
-> Arm-C1 no-load response
-> deterministic report
```

## Training-Ready Primitive Freeze

The following layers are now defined for Gazebo/RL and future real-vehicle
adapters:

- primitive registry
- action semantics
- observation/action/result schemas
- risk detection backend interface
- risk map summary interface
- local LLM/report interface
- RL semantic action space
- Arm-D clearance staging
- episode_report_v2, with multi-step `observation_state[]` and concrete
  `benchmarks.vision`, `benchmarks.llm`, and `benchmarks.rl` fields

The RL observation vocabulary is aligned to the real vehicle scan snapshot:
`front_min`, `front_p10`, `left_p10`, `right_p10`,
`consecutive_fast_arc`, and `total_forward_m`.

## Remaining Competition-Level Work

1. AI vision model: train/export a local lightweight model for printed A4 risk
   images and benchmark it on K1 CPU/NPU.
2. Local LLM: add llama.cpp or equivalent local backend and record TTFT,
   tokens/s, total tokens, and memory.
3. Gazebo/RL: connect semantic action space to simulated pipeline. Keep RL
   output as action candidates only.
4. Arm-D: only after no-load planning, separately validate soft contact and
   controlled displacement with hard safety gates.
5. Performance: collect FPS, tokens/s, memory, runtime stability, and task
   success rate.

## Claim Boundary

Do not claim:

- HSV is an AI model
- deterministic report is a real LLM
- RL has controlled real hardware
- no-load arm motion is clearance
- high-precision SLAM or autonomous navigation

Allowed current claim:

The project has a validated safety-gated hardware demo chain and now has a
training-ready high-level primitive interface for AI/Gazebo/RL expansion.
