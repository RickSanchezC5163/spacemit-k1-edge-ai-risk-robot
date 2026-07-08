# P4 Guarded Auto Mapping Micro - 2026-06-28

## Scope

This P4 run stayed inside the agreed safety boundary:

- no official RRT
- no AMCL main localization
- no long-running automatic exploration
- no unattended run
- no motion command bypassing `scan_safety_guard_node`

The intended guarded command chain was:

```text
/input_cmd_vel
-> scan_safety_guard_node
-> /cmd_vel_guarded
-> wheeltec_tank_base_safe.py
-> C30D
```

## Guarded Stack

The previous direct mapping launch was stopped and replaced with:

```bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  serial_port:=/dev/base_controller \
  max_linear_x:=0.45 \
  max_angular_z:=0.80 \
  brake_duration_sec:=1.5
```

Confirmed active processes:

- `n10p_tank_mapping_safety_guard.launch.py`
- `lslidar_driver_node`
- `wheeltec_tank_base_safe.py`
- `scan_safety_guard_node`
- `async_slam_toolbox_node`

Confirmed topics:

- `/input_cmd_vel`
- `/cmd_vel_guarded`
- `/safety/front_obstacle`
- `/scan`
- `/odom`
- `/map_metadata`

Final static status after yaw calibration:

```text
/safety/front_obstacle: state=warning, front_min=1.580 m, front_p10=1.589 m
/map_metadata: width=87, height=154, resolution=0.05 m/pixel
/cmd_vel_guarded: linear.x=0.0, angular.z=0.0
/robot_vel: x=0.0, z=0.0
```

Note: restarting the guarded SLAM stack reset the in-memory map. The previous manual map
`manual_mapping_snapshot_20260628_170300` remains the latest saved manual expansion result.

## Tool Added

Added:

```text
tools/guarded_auto_mapping_micro.py
```

The tool has two modes:

```bash
python3 tools/guarded_auto_mapping_micro.py --mode yaw-calibration --confirm YES
python3 tools/guarded_auto_mapping_micro.py --mode yaw-amplified --confirm YES
python3 tools/guarded_auto_mapping_micro.py --mode odom-micro-run --confirm YES
python3 tools/guarded_auto_mapping_micro.py --mode micro-run --confirm YES
```

Important behavior:

- publishes only to `/input_cmd_vel`
- records `/odom`, `/scan`, `/map_metadata`, `/safety/front_obstacle`
- checks `/cmd_vel_guarded`, `/robot_vel`, and `/rosout` base diagnostics for final zero state
- saves maps through `/slam_toolbox/save_map` in `micro-run`
- blocks forward primitives when `front_p10 < --forward-front-p10-min-m`

`micro-run` is the original duration-based diagnostic mode. Do not use it for P4
automatic map expansion now that odom has been validated as the control ruler.

Default `micro-run` primitive sequence:

```text
1. forward:    linear=0.28, angular=0.00, duration=0.8s
2. turn_left:  linear=0.00, angular=+0.50, duration=0.6s
3. forward:    linear=0.28, angular=0.00, duration=0.8s
4. turn_right: linear=0.00, angular=-0.50, duration=0.6s
5. forward:    linear=0.28, angular=0.00, duration=0.8s
```

Default `odom-micro-run` primitive sequence:

```text
1. forward:    odom local forward distance >= 0.20m, max linear=0.28m/s, timeout=3.0s
2. turn_left:  odom yaw delta >= +15deg, max angular=0.50rad/s, timeout=4.0s
3. forward:    odom local forward distance >= 0.20m, max linear=0.28m/s, timeout=3.0s
4. turn_right: odom yaw delta <= -15deg, max angular=0.50rad/s, timeout=4.0s
5. forward:    odom local forward distance >= 0.20m, max linear=0.28m/s, timeout=3.0s
```

The script stops the remaining sequence after any segment that does not reach
its odom target. Each odom segment records target, actual odom delta, local
forward/lateral delta, yaw delta, stop reason, front_p10 start/end, map size,
map save prefix, telemetry, and final base zero status.

## Yaw Calibration

Command:

```bash
python3 tools/guarded_auto_mapping_micro.py \
  --mode yaw-calibration \
  --zero-hold-s 4.0 \
  --report /home/soc/edge-ai-robot-k1/logs/p4_yaw_calibration_20260628.json \
  --confirm YES
```

Result:

| Test | Command | Odom yaw start | Odom yaw end | yaw_delta | base_zero_ok |
| --- | --- | ---: | ---: | ---: | --- |
| yaw_pos_0p40 | `angular.z=+0.40 x 1.0s` | 0.00 deg | 0.86 deg | +0.86 deg | true |
| yaw_neg_0p40 | `angular.z=-0.40 x 1.0s` | 0.86 deg | 0.75 deg | -0.11 deg | true |
| yaw_pos_0p80 | `angular.z=+0.80 x 1.0s` | 0.75 deg | 5.58 deg | +4.83 deg | true |
| yaw_neg_0p80 | `angular.z=-0.80 x 1.0s` | 5.58 deg | 1.73 deg | -3.85 deg | true |

Zero-state evidence after every segment:

```text
guarded_cmd_zero_ok=true
robot_vel_zero_ok=true
diag_zero_ok=true
latest_diag cmd=(0.000,0.000), serial=(0.000,0.000), feedback=(0.000,0.000)
```

## Decision

Yaw calibration did not pass the precondition for forward automatic micro mapping.

Reasons:

- `angular.z=+0.40 x 1.0s` produced only `+0.86 deg`.
- `angular.z=-0.40 x 1.0s` produced only `-0.11 deg`.
- `angular.z=+0.80 x 1.0s` produced only `+4.83 deg`.
- `angular.z=-0.80 x 1.0s` produced only `-3.85 deg`.
- Response is weak and asymmetric relative to the expected mapping turn primitive.

Because the user requirement was "if yaw calibration is normal, start guarded mapping",
the `micro-run` forward sequence was not executed.

No new P4 automatic-forward map was saved or pulled to Windows in this run.

Follow-up source audit:

```text
docs/yaw_angle_source_audit_20260628.md
```

The audit found that ROS yaw itself is just `yaw += firmware_Z_speed * dt`.
The likely fault is lower in the firmware: `Tank_Car` appears to use TIM2/TIM3
encoder channels for `MOTOR_A/B`, while the same source tree documents TIM5/TIM4
as the physical motor A/B encoder channels.

## Later Update

After restoring the clean original firmware with no TIM remapping and no USART
debug field changes, amplified yaw tests showed that odom yaw is credible.

Clean original firmware:

```text
K:\risc-vCar\ros相关\Mini小车_D版STM32源码_2025.01.13(默认GMR编码器)\OBJ\WHEELTEC_clean_original_no_tim_usart_debug_20260628_192504.hex
SHA256 E208AD13BA8394EAC1361B6FD7D23BF2E9E38BFC746A52656BCB1BBFD1126E4B
```

Relevant reports:

```text
logs/yaw_amplified_clean_original_fw_20260628_1932.json
logs/yaw_mark_50_left_right_20260628_193731.json
```

Ground-marked yaw result:

| Segment | Command | Odom yaw delta | Ground check |
| --- | --- | ---: | --- |
| left | `angular.z=+0.80 x 2.0s` | `+50.18 deg` | about 50 deg |
| right | `angular.z=-0.80 x 2.0s` | `-53.69 deg` | about 50 deg |

Conclusion:

- `/odom` yaw can be trusted as the motion ruler.
- `cmd_vel` velocity times duration is not a reliable distance or angle ruler.
- TIM5/TIM4 remapping made feedback worse; the clean original TIM2/TIM3 firmware
  is the baseline for the next test.
- Future P4 motion should be odom-target closed loop, not duration based.

## Next Action

Run the first P4 automatic expansion only with:

```bash
python3 /home/soc/edge-ai-robot-k1/tools/guarded_auto_mapping_micro.py \
  --mode odom-micro-run \
  --odom-forward-m 0.20 \
  --odom-turn-deg 15.0 \
  --forward-linear 0.25 \
  --turn-angular 0.50 \
  --zero-hold-s 5.0 \
  --save-every-segments 1 \
  --confirm YES
```

Do not run the legacy duration-based `micro-run` as the P4 expansion mode.

## P4 Odom Micro Run Result

Run time: `2026-06-28 19:49:43`

Reports pulled to Windows:

```text
logs/odom_micro_20260628_194943.json
logs/odom_micro_safe_20260628_194943.json
logs/odom_micro_guard_min_20260628_194943.json
```

The first run used `front_p10_min=1.60m` and correctly refused to move:

```text
odom_forward_1 stop_reason="front_blocked: front_p10 1.585 < 1.60"
base_zero_ok=true
```

The second run lowered the script threshold but used `forward_linear=0.22m/s`.
It also did not move because the guard launch has `min_effective_forward=0.28`:

```text
guarded_linear_x_command max_abs=0.0
diag_cmd_vx max_abs=0.0
stop_reason=timeout
base_zero_ok=true
```

The third run used `forward_linear=0.28m/s`, `odom_forward_m=0.15m`,
`odom_turn_deg=10deg`, and `front_p10_min=1.45m`.

| Segment | Target | Stop reason | Final odom result | front_p10 start/end | Map size | zero |
| --- | ---: | --- | ---: | ---: | ---: | --- |
| `odom_forward_1` | `0.15m` | `target_reached` | `forward_delta=0.3964m`, `yaw=0.06deg` | `2.249m -> 1.846m` | `115x146` | true |
| `odom_turn_left` | `+10deg` | `target_reached` | `yaw_delta=+49.01deg`, `forward_delta=-0.0256m` | `1.848m -> 1.034m` | `116x146` | true |
| `odom_forward_2` | `0.15m` | `front_blocked: front_p10 1.038 < 1.45` | `forward_delta=0.0m`, `yaw=0.0deg` | `1.030m -> 1.034m` | `116x146` | true |

Saved maps pulled to Windows:

```text
maps/odom_micro_guard_min_20260628_194943_01_odom_forward_1.{pgm,yaml,png}
maps/odom_micro_guard_min_20260628_194943_01_odom_forward_1_marked.png
maps/odom_micro_guard_min_20260628_194943_02_odom_turn_left.{pgm,yaml,png}
maps/odom_micro_guard_min_20260628_194943_02_odom_turn_left_marked.png
maps/odom_micro_guard_min_20260628_194943_03_odom_forward_2.{pgm,yaml,png}
maps/odom_micro_guard_min_20260628_194943_03_odom_forward_2_marked.png
```

Decision:

- The guarded command chain is valid: `forward_linear=0.28` reaches
  `/cmd_vel_guarded`, serial command, odom, map save, and final zero checks.
- `forward_linear < min_effective_forward` is suppressed by the guard and should
  not be used unless the launch lowers `min_effective_forward`.
- Odom-target control alone is not enough at current speed/brake settings:
  final settled odom overshot the forward target by about `0.25m` and the turn
  target by about `39deg`.
- Do not continue automatic expansion until the odom primitive adds a slow
  approach phase, an early stop margin, or the guard launch is rerun with a
  lower `min_effective_forward` so smaller velocities can be used.

## Forward Friction Threshold

Run time: `2026-06-28 20:30:52`

The guarded launch was started with a lower forward pass-through threshold:

```bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  min_effective_forward:=0.05 \
  clear_max_linear:=0.30 \
  soft_max_linear:=0.30
```

Reports pulled to Windows:

```text
logs/forward_threshold_20260628_203052.json
logs/forward_threshold_0p10_2s_20260628_203052.json
```

One-second speed sweep:

| Command | Guarded max | Final odom forward | Max odom vx | Final zero |
| ---: | ---: | ---: | ---: | --- |
| `0.10m/s x 1s` | `0.10m/s` | `0.0384m` | `0.060m/s` | true |
| `0.20m/s x 1s` | `0.20m/s` | `0.1098m` | `0.134m/s` | true |
| `0.30m/s x 1s` | `0.30m/s` | `0.1640m` | `0.212m/s` | true |

Focused `0.10m/s` two-second pulse:

```text
movement_detected=true
time_to_motion_s=1.769
first_motion forward_delta=0.0031m, odom_vx=0.017m/s, robot_vx=0.030m/s
final_forward_delta=0.1121m
base_zero_ok=true
```

Decision:

- The chassis can overcome static friction at `0.10m/s`, but the start delay is
  long, about `1.77s` in this run.
- `0.10m/s` is usable only as a slow approach command when time-to-motion delay
  is acceptable.
- `0.20m/s` is a better low-speed control candidate for odom PID because it
  creates a stronger, more prompt response while still much gentler than
  `0.28-0.30m/s`.
- Keep `min_effective_forward` below the PID minimum command when running odom
  PID tests; otherwise guard suppression will hide the chassis response.

## P4-D Forward Staged Control

Added a forward-only staged odom controller:

```bash
python3 /home/soc/edge-ai-robot-k1/tools/guarded_auto_mapping_micro.py \
  --mode forward-staged \
  --control-mode staged \
  --staged-test-set abc \
  --forward-front-p10-min-m 1.20 \
  --zero-hold-s 4.0 \
  --confirm YES
```

The controller does not use `cmd_vel * time` as the ruler. It uses odom local
forward delta and switches command speed by remaining distance:

```text
remaining <= brake_margin -> stop
remaining <= slow_zone    -> slow_speed
remaining <= mid_zone     -> mid_speed
otherwise                 -> fast_speed
```

Default single-test parameters:

```text
target_m=0.20
fast_speed=0.20
mid_speed=0.15
slow_speed=0.10
mid_zone_m=0.12
slow_zone_m=0.06
brake_margin_m=0.03
timeout_s=5.0
```

The built-in `abc` test set runs:

| Test | Target | Fast | Mid | Slow | Brake margin |
| --- | ---: | ---: | ---: | ---: | ---: |
| A | `0.15m` | `0.20m/s` | `0.15m/s` | `0.10m/s` | `0.03m` |
| B | `0.20m` | `0.20m/s` | `0.15m/s` | `0.10m/s` | `0.03m` |
| C | `0.20m` | `0.20m/s` | `0.12m/s` | `0.10m/s` | `0.05m` |

The guarded mapping launch default `min_effective_forward` was lowered from
`0.28` to `0.08` so staged low-speed commands can pass through guard while
hard-stop, slow-down, and front-distance checks remain active.

### P4-D Forward Staged ABC Result

Run time: `2026-06-28 21:29`

Report:

```text
logs/forward_staged_abc_20260628_212911.json
```

The guarded mapping launch was running with `min_effective_forward=0.08`.
The test only sent `/input_cmd_vel`; it did not bypass `scan_safety_guard_node`.

| Test | Target | Final odom forward | Overshoot | Stop reason | Time to motion | Front p10 start/end | Final zero |
| --- | ---: | ---: | ---: | --- | ---: | ---: | --- |
| A | `0.15m` | `0.4078m` | `+0.2578m` | `brake_margin` | `1.597s` | `2.081m -> 1.663m` | true |
| B | `0.20m` | `0.4205m` | `+0.2205m` | `brake_margin` | `1.816s` | `1.663m -> 1.248m` | true |
| C | `0.20m` | `0.1628m` | `-0.0372m` | `front_blocked: front_p10 1.160 < 1.20` | `1.668s` | `1.248m -> 1.059m` | true |

Sequence stop reason:

```text
staged_C_target_0p20_brake_0p05: front_blocked: front_p10 1.160 < 1.20
```

Decision:

- The staged controller is correctly routed through the guard and records the
  needed odom, safety, and base-zero fields.
- The current staged parameters are not acceptable for automatic map expansion:
  tests A and B still overshot by about `0.22-0.26m` after the command stopped.
- Test C should not be interpreted as a successful `0.20m` controller result;
  it stopped because the safety threshold was reached.
- Do not chain multiple forward staged tests in one run near obstacles, because
  each successful forward segment reduces `front_p10` for the next segment.
- Next controller change should compensate for the measured start delay:
  use a low-speed pre-roll until odom motion is detected, then switch to staged
  remaining-distance control, and/or move the brake margin much earlier.

### P4-E Forward Pre-Roll Dynamic Brake Result

Run time: `2026-06-28 21:50`

Report:

```text
logs/forward_staged_preroll_single_20260628_215009.json
```

Code change:

- `staged_forward_record()` now starts in `pre_roll`, sending only
  `slow_speed=0.10m/s`.
- When odom motion is detected, the current odom is captured as
  `control_odom_start`.
- Remaining-distance staged control then uses `control_odom_start`, not the
  original pre-roll odom.
- Brake margin is dynamic:

```text
effective_brake_margin =
  max(brake_margin_m, abs(odom_vx) * forward_brake_coef_s + forward_static_brake_margin_m)
```

Command:

```bash
python3 /home/soc/edge-ai-robot-k1/tools/guarded_auto_mapping_micro.py \
  --mode forward-staged \
  --control-mode staged \
  --staged-test-set single \
  --forward-target-m 0.20 \
  --forward-fast-speed 0.20 \
  --forward-mid-speed 0.15 \
  --forward-slow-speed 0.10 \
  --forward-brake-margin-m 0.03 \
  --forward-brake-coef-s 1.0 \
  --forward-static-brake-margin-m 0.02 \
  --forward-front-p10-min-m 1.20 \
  --zero-hold-s 4.0 \
  --confirm YES
```

Result:

| Target | Final odom forward | Overshoot | Control-frame final | Control-frame overshoot | Stop reason | Time to motion | Final zero |
| ---: | ---: | ---: | ---: | ---: | --- | ---: | --- |
| `0.20m` | `0.2481m` | `+0.0481m` | `0.2462m` | `+0.0462m` | `brake_margin_dynamic` | `2.062s` | true |

Other observations:

```text
front_p10: 2.356m -> 2.108m
max_odom_vx_during_command: 0.099m/s
duration_used_s: 3.455
staged_duration_s: 1.393
max_control_forward_delta_during_command: 0.0948m
max_total_forward_delta_during_command: 0.0967m
```

Decision:

- Pre-roll plus dynamic brake reduced the `0.20m` forward error from
  about `+0.22m` to `+0.048m`.
- This meets the earlier coarse acceptance band of `0.18-0.26m` for a
  `0.20m` target, but it is still slightly high for repeated automatic mapping.
- The controller stopped while commanded odom delta was only about `0.095m`;
  the remaining distance was consumed by brake/settle motion. This confirms
  that the brake-distance model is now the main calibration target.
- Next single-run test should keep pre-roll and increase the dynamic brake line,
  for example `forward_brake_coef_s=1.3-1.5` or
  `forward_static_brake_margin_m=0.04`, before trying any turn or map-expansion
  sequence.

### P4-F Odom-Only Pre-Roll Trigger

Run time: `2026-06-28 22:11-22:13`

Code change:

- `staged_forward_record()` no longer lets `/robot_vel` trigger the transition
  from `pre_roll` to `staged`.
- `robot_vx` is still recorded, but the control origin is now set only by odom:

```text
total_forward_delta >= threshold_detect_m
or abs(odom_vx) >= threshold_detect_vx
```

Reports:

```text
logs/forward_staged_odom_preroll105_single_20260628_221108.json
logs/forward_staged_odom_preroll110_single_20260628_221310.json
```

Results:

| Brake coef | Target | Final odom forward | Error | Control-frame final | Stop reason | Time to motion | First motion trigger | Final zero |
| ---: | ---: | ---: | ---: | ---: | --- | ---: | --- | --- |
| `1.05` | `0.20m` | `0.2508m` | `+0.0508m` | `0.2467m` | `brake_margin_dynamic` | `2.065s` | `odom_vx=0.034m/s` | true |
| `1.10` | `0.20m` | `0.1808m` | `-0.0192m` | `0.1773m` | `brake_margin_dynamic` | `2.184s` | `odom_vx=0.021m/s` | true |

Decision:

- Odom-only pre-roll removed the bad case where `robot_vx` started staged
  control before odom moved.
- The result is still not smooth enough to tune with only
  `forward_brake_coef_s`. At these velocities, changing coef by `0.05` changes
  the dynamic brake line by only a few millimeters, while the chassis settle
  variance is centimeters.
- The next useful control parameter is speed profile, not just brake coef.
  Current `fast_speed=0.20` still drives the robot into the brake line while
  moving around `0.09-0.10m/s`.
- Recommended next single test:

```text
forward_brake_coef_s=1.05
forward_static_brake_margin_m=0.02
fast_speed=0.15
mid_speed=0.12
slow_speed=0.10
mid_zone_m=0.16
slow_zone_m=0.10
```

This should reduce settle variance by lowering speed before the dynamic brake
line, instead of relying on a very sensitive brake coefficient.

### P4-G Forward Speed Profile Result

Run time: `2026-06-28 22:19`

Report:

```text
logs/forward_staged_speedprofile_single_20260628_221837.json
```

Command profile:

```text
target_m=0.20
fast_speed=0.15
mid_speed=0.12
slow_speed=0.10
mid_zone_m=0.16
slow_zone_m=0.10
brake_coef_s=1.05
static_brake_margin_m=0.02
front_p10_min_m=1.20
```

Result:

| Target | Final odom forward | Error | Control-frame final | Control-frame error | Stop reason | Time to motion | Final zero |
| ---: | ---: | ---: | ---: | ---: | --- | ---: | --- |
| `0.20m` | `0.2212m` | `+0.0212m` | `0.2186m` | `+0.0186m` | `brake_margin_dynamic` | `1.974s` | true |

Other observations:

```text
front_p10: 2.377m -> 2.157m
odom yaw drift: -2.01deg
duration_used_s: 3.397
staged_duration_s: 1.424
max_forward_delta_during_command_m: 0.0966
max_total_forward_delta_during_command_m: 0.0992
max_odom_vx_during_command: 0.095m/s
guarded_linear_x_command max_abs: 0.15m/s
diag_cmd_vx max_abs: 0.12m/s
base_zero_ok=true
latest_diag cmd=(0.000,0.000), serial=(0.000,0.000), feedback=(0.000,0.000)
```

Decision:

- Lowering the speed profile improved the `0.20m` staged forward result from
  the previous `0.2508m` run to `0.2212m`.
- The result is inside the coarse acceptance band of `0.18-0.26m` and is close
  enough to become the current forward primitive candidate.
- The controller still stops while commanded odom delta is only about `0.10m`;
  the remaining distance is brake and settle motion, so repeated tests are
  still needed before chaining primitives.
- Do not start turn or map-expansion sequences yet. Repeat this single-forward
  profile once, then run a separate angular threshold/staged-turn calibration.

### P4-H Forward Stability Repeat

Run time: `2026-06-28 22:26-22:27`

Reports:

```text
logs/forward_staged_repeat1_20260628_222635.json
logs/forward_staged_repeat2_20260628_222740.json
```

Same command profile as P4-G:

```text
target_m=0.20
fast_speed=0.15
mid_speed=0.12
slow_speed=0.10
mid_zone_m=0.16
slow_zone_m=0.10
brake_margin_m=0.03
brake_coef_s=1.05
static_brake_margin_m=0.02
front_p10_min_m=1.20
```

Results:

| Run | Target | Final odom forward | Error | Control-frame final | Yaw drift | front_p10 start/end | Time to motion | Final zero |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| P4-G baseline | `0.20m` | `0.2212m` | `+0.0212m` | `0.2186m` | `-2.01deg` | `2.377m -> 2.157m` | `1.974s` | true |
| repeat 1 | `0.20m` | `0.2105m` | `+0.0105m` | `0.2065m` | `+0.08deg` | `2.474m -> 2.270m` | `2.107s` | true |
| repeat 2 | `0.20m` | `0.2068m` | `+0.0068m` | `0.2034m` | `+0.94deg` | `2.366m -> 2.155m` | `2.175s` | true |

Decision:

- The same `0.20m` forward primitive passed three consecutive runs inside the
  requested `0.18-0.24m` band.
- Final yaw drift stayed within the requested `3-5deg` bound.
- `base_zero_ok=true` on every run, with final `cmd/serial/feedback=(0,0)`.
- Treat this forward primitive as provisionally stable, pending target-sweep
  confirmation.

### P4-I Forward Target Sweep

Run time: `2026-06-28 22:32-22:34`

Reports:

```text
logs/forward_target_0p15_20260628_223202.json
logs/forward_target_0p20_20260628_223253.json
logs/forward_target_0p25_20260628_223431.json
```

Speed profile stayed fixed:

```text
fast_speed=0.15
mid_speed=0.12
slow_speed=0.10
brake_margin_m=0.03
brake_coef_s=1.05
static_brake_margin_m=0.02
front_p10_min_m=1.20
```

Only the target zones changed:

| Target | mid_zone | slow_zone |
| ---: | ---: | ---: |
| `0.15m` | `0.12m` | `0.07m` |
| `0.20m` | `0.16m` | `0.10m` |
| `0.25m` | `0.18m` | `0.12m` |

Results:

| Target | Final odom forward | Error | Control-frame final | Yaw drift | front_p10 start/end | Time to motion | Final zero |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `0.15m` | `0.1754m` | `+0.0254m` | `0.1719m` | `+0.61deg` | `2.477m -> 2.308m` | `2.065s` | true |
| `0.20m` | `0.2276m` | `+0.0276m` | `0.2229m` | `-2.56deg` | `2.415m -> 2.181m` | `2.067s` | true |
| `0.25m` | `0.2594m` | `+0.0094m` | `0.2571m` | `+0.07deg` | `2.458m -> 2.204m` | `2.020s` | true |

Decision:

- The staged forward controller is not just accidentally tuned for `0.20m`;
  it also produced usable `0.15m` and `0.25m` results.
- Errors stayed in the centimeter range across all three targets.
- The forward primitive is suitable as the current P4 forward candidate.
- Do not combine with turning until the turn primitive is separately controlled.

### P4-J Turn Threshold Table

Run time: `2026-06-28 22:38`

Code change:

- Added `--mode turn-threshold` to `tools/guarded_auto_mapping_micro.py`.
- The mode sends no forward command and only tests fixed angular pulses through
  `/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded -> base`.
- Each segment records expected yaw, odom yaw delta, whether yaw motion was
  detected, overshoot, telemetry, and final base zero status.

Report:

```text
logs/turn_threshold_20260628_223828.json
```

Results:

| Test | Command | Expected yaw | Odom yaw delta | Turned | Overshoot | Final zero |
| --- | ---: | ---: | ---: | --- | --- | --- |
| `turn_pos_0p20_1p0s` | `+0.20rad/s x 1.0s` | `+11.46deg` | `+0.00deg` | false | false | true |
| `turn_neg_0p20_1p0s` | `-0.20rad/s x 1.0s` | `-11.46deg` | `+0.00deg` | false | false | true |
| `turn_pos_0p30_1p0s` | `+0.30rad/s x 1.0s` | `+17.19deg` | `+2.66deg` | true | false | true |
| `turn_neg_0p30_1p0s` | `-0.30rad/s x 1.0s` | `-17.19deg` | `-1.44deg` | true | false | true |
| `turn_pos_0p50_0p5s` | `+0.50rad/s x 0.5s` | `+14.32deg` | `+0.46deg` | false | false | true |
| `turn_neg_0p50_0p5s` | `-0.50rad/s x 0.5s` | `-14.32deg` | `-0.39deg` | false | false | true |

Command-path check:

| Test group | guarded wz max | diag cmd wz max | diag serial wz max | diag feedback wz max |
| --- | ---: | ---: | ---: | ---: |
| `0.20rad/s x 1.0s` | `0.20` | `0.20` | `0.20` | `0.00` |
| `0.30rad/s x 1.0s` | `0.30` | `0.30` | `0.30` | `0.11 / 0.036` |
| `0.50rad/s x 0.5s` | `0.50` | `0.50` | `0.50` | `0.00` |

Decision:

- The angular command was actually sent through guard and serial. This is not a
  guard pass-through problem.
- `0.20rad/s x 1s` did not overcome turn friction.
- `0.30rad/s x 1s` barely started rotation and is not enough for a reliable
  staged-turn primitive.
- `0.50rad/s x 0.5s` is too short to start meaningful rotation.
- Do not use the earlier proposed `0.30/0.20/0.12` staged-turn profile yet.
  The next turn-only test should use a stronger pre-roll or longer pulse, for
  example `0.50rad/s x 1.0s` and `0.80rad/s x 0.5-1.0s`, before attempting
  `target_yaw=10deg`.

### P4-K Strong Turn Threshold Table

Run time: `2026-06-28 22:49`

Report:

```text
logs/turn_threshold_strong_20260628_224957.json
```

Command:

```bash
python3 /home/soc/edge-ai-robot-k1/tools/guarded_auto_mapping_micro.py \
  --mode turn-threshold \
  --turn-threshold-set strong \
  --zero-hold-s 5.0 \
  --confirm YES
```

The run used only angular commands through:

```text
/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded -> wheeltec_tank_base_safe.py -> C30D
```

Results:

| Test | Command | Expected yaw | Final yaw delta | time_to_yaw_motion_s | max_odom_wz during command | diag cmd/serial wz max | diag feedback wz max | Forward drift | Lateral drift | Final zero |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `turn_pos_0p50_1p0s` | `+0.50 x 1.0s` | `+28.65deg` | `+0.60deg` | null | `0.000` | `0.50 / 0.50` | `0.000` | `-0.0012m` | `0.0000m` | true |
| `turn_neg_0p50_1p0s` | `-0.50 x 1.0s` | `-28.65deg` | `-0.44deg` | null | `0.000` | `0.50 / 0.50` | `0.000` | `-0.0008m` | `0.0000m` | true |
| `turn_pos_0p80_0p5s` | `+0.80 x 0.5s` | `+22.92deg` | `+0.11deg` | null | `0.000` | `0.80 / 0.80` | `0.000` | `+0.0002m` | `0.0000m` | true |
| `turn_neg_0p80_0p5s` | `-0.80 x 0.5s` | `-22.92deg` | `+0.00deg` | null | `0.000` | `0.80 / 0.80` | `0.000` | `+0.0000m` | `0.0000m` | true |
| `turn_pos_0p80_1p0s` | `+0.80 x 1.0s` | `+45.84deg` | `+8.61deg` | null | `0.000` | `0.80 / 0.80` | `0.073` | `-0.0110m` | `-0.0009m` | true |
| `turn_neg_0p80_1p0s` | `-0.80 x 1.0s` | `-45.84deg` | `-3.16deg` | null | `0.000` | `0.80 / 0.80` | `0.073` | `-0.0007m` | `0.0000m` | true |

Additional full-window telemetry:

| Test | Final yaw delta | max yaw observed during command | full-window odom_wz max | full-window robot_wz max |
| --- | ---: | ---: | ---: | ---: |
| `+0.50 x 1.0s` | `+0.60deg` | `0.00deg` | `0.036` | `0.036` |
| `-0.50 x 1.0s` | `-0.44deg` | `0.00deg` | `0.073` | `0.073` |
| `+0.80 x 0.5s` | `+0.11deg` | `0.00deg` | `0.000` | `0.000` |
| `-0.80 x 0.5s` | `+0.00deg` | `0.00deg` | `0.000` | `0.000` |
| `+0.80 x 1.0s` | `+8.61deg` | `0.00deg` | `0.294` | `0.294` |
| `-0.80 x 1.0s` | `-3.16deg` | `0.00deg` | `0.257` | `0.257` |

Decision:

- `cmd_wz` and `serial_wz` are correct for every case. The command path is not
  the limiting factor.
- `0.50rad/s x 1.0s` is still below the practical turn threshold on this floor;
  feedback stays effectively zero and yaw delta is under `1deg`.
- `0.80rad/s x 0.5s` is too short to start rotation.
- `0.80rad/s x 1.0s` finally produces measurable rotation, but it is still weak
  and asymmetric: `+8.61deg` versus `-3.16deg`.
- `time_to_yaw_motion_s` is null because no yaw motion crossed the detection
  threshold during the command window. The yaw/feedback appears mostly after
  command/stop latency, visible in the full-window `odom_wz` and `robot_wz`.
- Forward/lateral drift during turn-only pulses is small, so the main issue is
  not translation drift; it is angular dead zone/startup latency.

Next turn-only recommendation:

- Do not attempt `target_yaw=10deg` yet.
- Test an angular pre-roll that keeps `0.80rad/s` active until odom yaw changes
  by about `1deg` or until a timeout, then starts the turn controller's yaw
  measurement from that point.
- If pre-roll at `0.80rad/s` still takes close to `1s` to move, the staged-turn
  controller should use `0.80` only as an initial kick and then brake early.
- If `0.80rad/s` pre-roll remains weak or highly asymmetric, inspect the
  firmware angular command to left/right track differential scaling before
  combining forward and turn primitives.

### P4-L 0.80rad/s Turn Duration Sweep

Run time: `2026-06-28 23:02`

Code change:

- Added `--mode turn-duration-sweep`.
- `turn_threshold_record()` now records:

```text
yaw_delta_at_cmd_end_deg
yaw_delta_after_settle_deg
settle_extra_yaw_deg
forward_delta_at_cmd_end_m
lateral_delta_at_cmd_end_m
settle_extra_forward_m
settle_extra_lateral_m
```

Report:

```text
logs/turn_duration_sweep_20260628_230233.json
```

Results:

| Test | Command | yaw at cmd end | yaw after settle | settle extra yaw | time_to_yaw_motion | max odom wz during cmd | full-window odom wz | diag feedback wz max | Final zero |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `turn_pos_0p80_0p5s` | `+0.80 x 0.5s` | `0.00deg` | `+0.44deg` | `+0.44deg` | null | `0.000` | `0.073` | `0.000` | true |
| `turn_pos_0p80_1p0s` | `+0.80 x 1.0s` | `0.00deg` | `+7.75deg` | `+7.75deg` | null | `0.000` | `0.257` | `0.110` | true |
| `turn_pos_0p80_1p5s` | `+0.80 x 1.5s` | `0.00deg` | `+42.21deg` | `+42.21deg` | null | `0.000` | `0.700` | `0.515` | true |
| `turn_pos_0p80_2p0s` | `+0.80 x 2.0s` | `+4.67deg` | `+61.91deg` | `+57.24deg` | `1.713s` | `0.515` | `0.810` | `0.700` | true |
| `turn_neg_0p80_0p5s` | `-0.80 x 0.5s` | `0.00deg` | `+0.00deg` | `+0.00deg` | null | `0.000` | `0.000` | `0.000` | true |
| `turn_neg_0p80_1p0s` | `-0.80 x 1.0s` | `0.00deg` | `-9.58deg` | `-9.58deg` | null | `0.000` | `0.257` | `0.073` | true |
| `turn_neg_0p80_1p5s` | `-0.80 x 1.5s` | `0.00deg` | `-44.41deg` | `-44.41deg` | null | `0.000` | `0.810` | `0.626` | true |
| `turn_neg_0p80_2p0s` | `-0.80 x 2.0s` | `-1.35deg` | `-60.09deg` | `-58.74deg` | `1.763s` | `0.331` | `0.773` | `0.663` | true |

Decision:

- The pure turn response is highly nonlinear.
- `0.80 x 0.5s` is effectively below useful turn threshold.
- `0.80 x 1.0s` can produce about `8-10deg`, but yaw appears after the command
  window.
- `0.80 x 1.5s` jumps to about `42-44deg`; this is too aggressive for first
  staged-turn control.
- `0.80 x 2.0s` reaches about `60deg`, and only then does yaw motion cross the
  command-window detection threshold around `1.7s`.
- Most yaw is still measured as `settle_extra_yaw`, so a pure in-place turn
  controller cannot simply stop when current yaw reaches target. It needs a
  turn-specific braking/settle model or a different primitive.

### P4-M Arc Turn Threshold

Run time: `2026-06-28 23:06`

Code change:

- Added `--mode arc-turn-threshold`.
- The mode sends small linear velocity plus angular velocity and uses the same
  command-end versus after-settle yaw split.
- Forward arc segments check `front_p10` before moving and stop the sequence if
  the next segment would start too close.

Report:

```text
logs/arc_turn_threshold_20260628_230609.json
```

Command:

```bash
python3 /home/soc/edge-ai-robot-k1/tools/guarded_auto_mapping_micro.py \
  --mode arc-turn-threshold \
  --forward-front-p10-min-m 1.20 \
  --zero-hold-s 5.0 \
  --confirm YES
```

Results:

| Test | Command | yaw at cmd end | yaw after settle | settle extra yaw | Forward drift | Lateral drift | front_p10 start/end | diag feedback wz max | Final zero |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `arc_lin0p10_pos0p50_1p0s` | `linear=0.10, wz=+0.50, 1.0s` | `0.00deg` | `+7.59deg` | `+7.59deg` | `0.0350m` | `0.0024m` | `2.254m -> 2.228m` | `0.073` | true |
| `arc_lin0p10_neg0p50_1p0s` | `linear=0.10, wz=-0.50, 1.0s` | `0.00deg` | `-9.10deg` | `-9.10deg` | `0.0526m` | `-0.0044m` | `2.227m -> 2.160m` | `0.073` | true |
| `arc_lin0p12_pos0p80_1p0s` | `linear=0.12, wz=+0.80, 1.0s` | `0.00deg` | `+15.31deg` | `+15.31deg` | `0.0621m` | `0.0088m` | `2.161m -> 0.559m` | `0.147` | true |
| `arc_lin0p12_neg0p80_1p0s` | `linear=0.12, wz=-0.80, 1.0s` | blocked | blocked | blocked | `0.0000m` | `0.0000m` | `0.559m -> 0.562m` | null | true |

Decision:

- Arc turn is much more promising than pure in-place turning.
- `linear=0.10, wz=+/-0.50, 1.0s` produced usable `7-9deg` yaw changes with
  only `3.5-5.3cm` forward motion and small lateral drift.
- The yaw still appears after command/settle rather than during the command
  window, but the resulting primitive is smaller and more repeatable than pure
  `0.80` in-place turns.
- `linear=0.12, wz=+0.80, 1.0s` produced `+15.31deg`, but it changed the scan
  geometry enough that `front_p10` fell to `0.559m`; the following negative arc
  was correctly blocked.
- First combined mapping should prefer conservative arc turns:

```text
forward staged 0.15-0.20m
arc turn: linear=0.10, angular=+/-0.50, duration=1.0s
stop and settle 5s
save/check map
```

- Do not use `linear=0.12, angular=0.80` as the default first arc primitive
  near obstacles.

### P4-N Arc Step Repeat Attempt

Run time: `2026-06-28 23:18`

Code change:

- Added `--mode arc-step-repeat`.
- Default test plan:

```text
linear.x=0.10
angular.z=+0.50, duration=1.0s, repeat 3
angular.z=-0.50, duration=1.0s, repeat 3
zero_hold=5.0s
front_p10_min_m=1.20
```

Report:

```text
logs/arc_step_repeat_20260628_231848.json
```

Result:

| Step | Blocked | Reason | front_p10 start/end | Motion | Final zero |
| --- | --- | --- | ---: | --- | --- |
| `arc_step_left_1_lin0.10_wz+0.50_1.00s` | true | `front_p10 0.969 < 1.20` | `0.969m -> 0.968m` | no command motion | true |

Decision:

- The repeat test mode is ready, but the robot was already too close to the
  front obstacle for the agreed safety gate.
- No arc-step motion was executed in this attempt.
- Do not lower the safety threshold just to complete this test. Reposition the
  robot into a more open area or rotate it manually, then rerun the same
  `arc-step-repeat` command.

### P4-O Arc Step Repeat Rerun

Run time: `2026-06-28 23:24`

After repositioning the robot, the same arc-step repeat command was rerun:

```bash
python3 /home/soc/edge-ai-robot-k1/tools/guarded_auto_mapping_micro.py \
  --mode arc-step-repeat \
  --arc-step-linear 0.10 \
  --arc-step-angular 0.50 \
  --arc-step-duration-s 1.0 \
  --arc-step-repeats 3 \
  --forward-front-p10-min-m 1.20 \
  --zero-hold-s 5.0 \
  --confirm YES
```

Report:

```text
logs/arc_step_repeat_rerun_20260628_232407.json
```

Result:

| Step | Yaw after settle | Group cumulative yaw | Sequence cumulative yaw | Forward drift | Lateral drift | front_p10 start/end | Final zero |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| left 1 | `+11.67deg` | `+11.67deg` | `+11.67deg` | `0.0250m` | `0.0023m` | `2.080m -> 2.056m` | true |
| left 2 | `+15.89deg` | `+27.56deg` | `+27.56deg` | `0.0434m` | `0.0070m` | `2.053m -> 2.026m` | true |
| left 3 | `+14.44deg` | `+42.00deg` | `+42.00deg` | `0.0576m` | `0.0076m` | `2.026m -> 1.662m` | true |
| right 1 | `-5.81deg` | `-5.81deg` | `+36.19deg` | `0.0593m` | `-0.0035m` | `1.645m -> 1.698m` | true |
| right 2 | `-10.38deg` | `-16.19deg` | `+25.81deg` | `0.0494m` | `-0.0040m` | `1.688m -> 1.873m` | true |
| right 3 | `-15.21deg` | `-31.41deg` | `+10.59deg` | `0.0506m` | `-0.0077m` | `1.876m -> 1.465m` | true |

Telemetry check:

| Step group | diag cmd wz max | diag serial wz max | diag feedback wz max range | command-window max odom wz |
| --- | ---: | ---: | ---: | ---: |
| left arc steps | `0.50` | `0.50` | `0.110-0.221` | `0.000` |
| right arc steps | `0.50` | `0.50` | `0.110-0.294` | `0.000` |

Decision:

- `arc-step-repeat` passed the safety and zero checks.
- All six steps ended with `base_zero_ok=true`.
- `front_p10` stayed above the `1.20m` gate; the minimum final value was
  `1.465m`.
- Left arc steps are larger than the original single-step estimate, around
  `11.7-15.9deg` each, accumulating to `+42deg`.
- Right arc steps are more variable, around `-5.8` to `-15.2deg`, accumulating
  to `-31.4deg`.
- The command window still does not show odom yaw motion; yaw is mostly observed
  after stop/settle. Treat this as a fixed primitive with measured outcome, not
  as a real-time yaw servo.
- This is not an in-place turn. Each arc step also moves the robot forward by
  about `2.5-5.9cm` and has small lateral drift. Future planning must treat it
  as a spatial arc primitive, not as a pure yaw primitive.
- For first combined mapping, use at most one arc-step between forward staged
  moves and inspect/save after each step. Do not chain three arc steps in normal
  expansion near obstacles.

### P4-P Arc Yaw Closed Validation

Run time: `2026-06-28 23:40`

Code change:

- Added `--mode arc-yaw-closed`.
- This implements discrete yaw closure:

```text
execute one arc-step
zero_hold and wait for settle
read final odom yaw
repeat until cumulative yaw reaches the target band
```

It does not try to servo yaw during the command window.

Command:

```bash
python3 /home/soc/edge-ai-robot-k1/tools/guarded_auto_mapping_micro.py \
  --mode arc-yaw-closed \
  --arc-yaw-target-deg 30 \
  --arc-yaw-tolerance-deg 6 \
  --arc-step-linear 0.10 \
  --arc-step-angular 0.50 \
  --arc-step-duration-s 1.0 \
  --arc-max-steps 4 \
  --arc-yaw-direction both \
  --forward-front-p10-min-m 1.20 \
  --zero-hold-s 5.0 \
  --confirm YES
```

Report:

```text
logs/arc_yaw_closed_20260628_234024.json
```

Left result:

| Step | Step yaw | Cumulative yaw | Target band | Forward drift | Lateral drift | front_p10 start/end | Final zero |
| --- | ---: | ---: | --- | ---: | ---: | ---: | --- |
| 1 | `+6.87deg` | `+6.87deg` | no | `0.0511m` | `0.0038m` | `2.426m -> 2.376m` | true |
| 2 | `+13.23deg` | `+20.10deg` | no | `0.0552m` | `0.0070m` | `2.378m -> 2.324m` | true |
| 3 | `+15.78deg` | `+35.88deg` | yes | `0.0511m` | `0.0083m` | `2.326m -> 1.851m` | true |

Left summary:

```text
steps_used=3
final_cumulative_yaw=+35.88deg
target_band=24-36deg
stop_reason=target_band_reached
cumulative_forward=0.1502m
cumulative_lateral=0.0427m
```

Right result:

| Step | Step yaw | Cumulative yaw | Target band | Forward drift | Lateral drift | front_p10 start/end | Final zero |
| --- | ---: | ---: | --- | ---: | ---: | ---: | --- |
| 1 | `-5.86deg` | `-5.86deg` | no | `0.0562m` | `-0.0030m` | `1.871m -> 1.947m` | true |
| 2 | `-11.60deg` | `-17.45deg` | no | `0.0534m` | `-0.0054m` | `1.941m -> 2.162m` | true |
| 3 | `-12.29deg` | `-29.74deg` | yes | `0.0483m` | `-0.0052m` | `2.167m -> 2.110m` | true |

Right summary:

```text
steps_used=3
final_cumulative_yaw=-29.74deg
target_band=24-36deg
stop_reason=target_band_reached
cumulative_forward=0.1533m
cumulative_lateral=-0.0333m
```

Decision:

- The discrete yaw closed-loop approach passed for both directions.
- Both final yaw results landed inside the `24-36deg` acceptance band.
- All steps had `base_zero_ok=true`.
- `front_p10` stayed above the `1.20m` safety gate.
- Each direction used 3 arc-steps, and each 30deg-class turn also moved the
  robot forward about `0.15m`.
- This is now the preferred turn primitive for first combined P4 expansion:
  use `arc-yaw-closed` as a spatial arc maneuver, not as a pure rotation.

### P4-Q Spatial Micro Mapping Attempt

Run time: `2026-06-28 23:55`

This was the first combined spatial primitive test after the forward and arc
primitive validation. The intended minimal sequence was:

```text
save initial map
forward-staged 0.15m
save map
arc-yaw-closed left 30deg
save map
forward-staged 0.10m
save map
```

Command profile:

```text
mode=spatial-micro-run
front_p10_min_m=0.50
forward1_target=0.15m
forward2_target=0.10m
forward_fast/mid/slow=0.15/0.12/0.10m/s
forward_brake=0.03 + odom_vx*1.05 + 0.02
arc_yaw_target=30deg
arc_yaw_tolerance=6deg
arc_step linear=0.10m/s, angular=0.50rad/s, duration=1.0s
arc_max_steps=4
zero_hold=5.0s
```

Report:

```text
logs/spatial_micro_p4q_20260628_235500.json
```

Saved maps:

```text
maps/spatial_micro_p4q_20260628_235500_00_initial.{pgm,yaml,png}
maps/spatial_micro_p4q_20260628_235500_00_initial_marked.png
maps/spatial_micro_p4q_20260628_235500_01_forward_0p15.{pgm,yaml,png}
maps/spatial_micro_p4q_20260628_235500_01_forward_0p15_marked.png
maps/spatial_micro_p4q_20260628_235500_02_arc_left.{pgm,yaml,png}
maps/spatial_micro_p4q_20260628_235500_02_arc_left_marked.png
```

Map size progression:

| Snapshot | Size |
| --- | ---: |
| initial | `84x220` |
| after forward 0.15m | `86x221` |
| after left arc | `115x226` |

Segment results:

| Segment | Result | front_p10 start/end | Odom result | Final zero |
| --- | --- | ---: | --- | --- |
| `forward-staged 0.15m` | completed | `1.713m -> 1.528m` | `forward=0.1827m`, error `+0.0327m`, yaw `-2.79deg` | true |
| `arc-yaw-closed left 30deg` | stopped as overshot | step fronts stayed `>=1.50m` during starts | `3 steps`, cumulative yaw `+36.88deg`, forward `0.0992m`, lateral `0.0381m` | true |
| `forward-staged 0.10m` | not run | n/a | skipped after arc stop reason | n/a |

Arc step details:

| Step | Step yaw | Cumulative yaw | front_p10 start/end | Forward drift | Final zero |
| --- | ---: | ---: | ---: | ---: | --- |
| 1 | `+7.99deg` | `+7.99deg` | `1.529m -> 1.520m` | `0.0162m` | true |
| 2 | `+14.96deg` | `+22.95deg` | `1.520m -> 1.502m` | `0.0412m` | true |
| 3 | `+13.92deg` | `+36.88deg` | `1.504m -> 1.818m` | `0.0496m` | true |

Final sequence odom delta:

```text
forward=0.2836m
lateral=0.0293m
yaw=+34.09deg
```

Decision:

- The first combined primitive test partially passed and stayed guarded.
- `forward-staged` and `arc-yaw-closed` chained successfully.
- Map saves succeeded at initial, after forward, and after arc.
- Map size expanded from `84x220` to `115x226`.
- Every executed movement ended with `base_zero_ok=true`.
- The script conservatively stopped before the final `F0.10` because the arc
  cumulative yaw was `+36.88deg`, just above the `36deg` upper band.
- This is not a failure of the motion primitive; it is a conservative stop
  condition. Next run can either widen tolerance slightly, accept a small
  overshoot margin, or keep the same stop behavior and manually inspect after
  the arc.

### P4-R Spatial S-Run

Run time: `2026-06-29 00:10`

This was the first full S-shaped combined primitive run in a different
environment after the P4-Q partial run.

Sequence:

```text
F0.15 -> L30_arc -> F0.10 -> R30_arc -> F0.10
```

Command profile:

```text
mode=spatial-s-run
front_p10_min_m=0.50
forward1_target=0.15m
forward2_target=0.10m
forward_fast/mid/slow=0.15/0.12/0.10m/s
forward_brake=0.03 + odom_vx*1.05 + 0.02
arc_yaw_target=30deg
arc_yaw_tolerance=6deg
arc_yaw_overshoot_epsilon=1.5deg
arc_step linear=0.10m/s, angular=0.50rad/s, duration=1.0s
arc_max_steps=4
zero_hold=5.0s
```

Report:

```text
logs/spatial_s_p4r_20260629_001054.json
```

Saved maps:

```text
maps/spatial_s_p4r_20260629_001054_00_initial.{pgm,yaml,png}
maps/spatial_s_p4r_20260629_001054_00_initial_marked.png
maps/spatial_s_p4r_20260629_001054_00_initial_marked_zoom.png
maps/spatial_s_p4r_20260629_001054_01_forward_0p15.{pgm,yaml,png}
maps/spatial_s_p4r_20260629_001054_01_forward_0p15_marked.png
maps/spatial_s_p4r_20260629_001054_01_forward_0p15_marked_zoom.png
maps/spatial_s_p4r_20260629_001054_02_arc_left.{pgm,yaml,png}
maps/spatial_s_p4r_20260629_001054_02_arc_left_marked.png
maps/spatial_s_p4r_20260629_001054_02_arc_left_marked_zoom.png
maps/spatial_s_p4r_20260629_001054_03_forward_0p10_after_left.{pgm,yaml,png}
maps/spatial_s_p4r_20260629_001054_03_forward_0p10_after_left_marked.png
maps/spatial_s_p4r_20260629_001054_03_forward_0p10_after_left_marked_zoom.png
maps/spatial_s_p4r_20260629_001054_04_arc_right.{pgm,yaml,png}
maps/spatial_s_p4r_20260629_001054_04_arc_right_marked.png
maps/spatial_s_p4r_20260629_001054_04_arc_right_marked_zoom.png
maps/spatial_s_p4r_20260629_001054_05_forward_0p10_after_right.{pgm,yaml,png}
maps/spatial_s_p4r_20260629_001054_05_forward_0p10_after_right_marked.png
maps/spatial_s_p4r_20260629_001054_05_forward_0p10_after_right_marked_zoom.png
```

Map size progression:

| Snapshot | Size |
| --- | ---: |
| initial | `128x222` |
| after forward 0.15m | `128x222` |
| after left arc | `128x239` |
| after forward 0.10m after left | `128x239` |
| after right arc | `128x239` |
| after final forward 0.10m | `128x240` |

Segment results:

| Segment | Result | front_p10 start/end | Odom result | Final zero |
| --- | --- | ---: | --- | --- |
| `forward-staged 0.15m` | completed | `2.562m -> 2.180m` | `forward=0.1901m`, error `+0.0401m`, yaw `+2.09deg` | true |
| `arc-yaw-closed left 30deg` | completed | stayed clear | `2 steps`, cumulative yaw `+24.30deg`, forward `0.0478m`, lateral `0.0100m` | true |
| `forward-staged 0.10m` | completed | `2.323m -> 2.161m` | `forward=0.1278m`, error `+0.0278m`, yaw `+1.43deg` | true |
| `arc-yaw-closed right 30deg` | completed | stayed clear | `3 steps`, cumulative yaw `-28.35deg`, forward `0.1014m`, lateral `-0.0251m` | true |
| `forward-staged 0.10m` | completed | `1.921m -> 1.771m` | `forward=0.1449m`, error `+0.0449m`, yaw `-3.98deg` | true |

Arc step details:

| Arc | Step | Step yaw | Cumulative yaw | Forward drift | Final zero |
| --- | ---: | ---: | ---: | ---: | --- |
| left | 1 | `+9.43deg` | `+9.43deg` | `0.0187m` | true |
| left | 2 | `+14.87deg` | `+24.30deg` | `0.0301m` | true |
| right | 1 | `-4.09deg` | `-4.09deg` | `0.0171m` | true |
| right | 2 | `-9.05deg` | `-13.13deg` | `0.0413m` | true |
| right | 3 | `-15.22deg` | `-28.35deg` | `0.0462m` | true |

Final sequence odom delta:

```text
sequence_stop_reason=null
forward=0.5978m
lateral=0.0943m
yaw=-4.52deg
saved_maps=6
```

Decision:

- The complete guarded S-run passed.
- All five movement segments completed.
- Every segment ended with `base_zero_ok=true`.
- No safety block occurred; final observed `front_p10` was still `1.771m`,
  above the requested `0.50m` gate.
- `arc-yaw-closed` worked as a step-between closed loop in both directions.
- The map expanded from `128x222` to `128x240`, with the main visible expansion
  occurring after the left arc.
- The final zoomed marked map is:
  `maps/spatial_s_p4r_20260629_001054_05_forward_0p10_after_right_marked_zoom.png`.
- P4 now has a validated short guarded auto-mapping sequence using odom-closed
  forward primitives plus discrete arc-yaw primitives, without RRT, AMCL, or
  long autonomous exploration.
