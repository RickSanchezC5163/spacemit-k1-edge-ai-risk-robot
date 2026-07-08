# Step7-E2 Fastdemo Reproduction - 2026-06-30

## Demo Name

Step7-E2 fastdemo.

## Reproduction Goal

Reproduce the current showcase baseline:

```text
guarded micro-motion
-> base_zero after motion
-> D435 HSV red-rule trigger
-> depth risk_point
-> approximate risk map projection
-> Arm-C1 no-load once
-> deterministic LLM-A report
```

The reference evidence directory is:

```text
outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/
```

Reference result:

- `status=succeeded`
- `guarded_motion_executed=true`
- `motion_command_path=/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded`
- `direct_cmd_vel_bypass=false`
- `policy_executed_count=2`
- `policy_sequence_stop_reason=max_consecutive_fast_arc_reached`
- `cumulative_positive_forward_m=0.118`
- `base_zero_ok_after_motion=true`
- `red_object_detected=true`
- `bbox_xywh=[93,250,275,117]`
- `depth_median_m=0.561`
- `risk_map_points=1`
- `projected=1`
- `arm_execution_status=succeeded`
- `hardware_executed=true`
- `serial_bytes_written=180`
- `published_cmd_vel_during_capture=false`
- `published_cmd_vel_during_arm=false`
- `contact_allowed=false`
- `obstacle_removed=false`
- `errors=[]`
- operator confirmed final return to `6b` with no abnormal issue observed

## Hardware Checklist

- K1 chassis with Muse Pi Pro.
- N10P lidar connected and visible to the guarded stack.
- D435 connected and publishing color/depth/camera_info topics.
- Bus-servo arm controller connected to the K1 arm serial path.
- Arm servo power enabled.
- Chassis battery and servo battery sufficiently charged.
- Emergency power cut method within reach.

## Scene Placement

- Use a short, clear corridor-like area.
- Keep the floor around the robot clear.
- Keep the red target in the D435 field of view after guarded motion.
- Avoid placing people or fragile objects near the arm sweep area.
- Avoid reflective red patches outside the intended target.

## Red Target Placement

Place the red target in the lower-to-middle D435 image area after the robot
settles. The successful reference run detected:

```text
bbox_xywh=[93,250,275,117]
depth_median_m=0.561
```

Practical placement rule:

- Put the red target on the visible front face of the box/object.
- Keep it low enough that the D435 sees it after the chassis arc motion.
- If the overlay has no yellow bbox, move the red target toward the image
  center/lower-left region and rerun.

## Arm Initial Pose

The arm should start near the validated safe idle/home-like pose:

```text
6b: #1 P510 #2 P771 #3 P426 #4 P503 #5 P497
```

The arm must be unloaded:

- no object in the gripper
- no contact with the environment
- no payload
- no obstacle clearing attempt

## Battery and Power Check

Before running the demo:

- confirm chassis battery is not near cut-off
- confirm servo power switch is on
- confirm servo controller power is stable
- confirm K1 network remains reachable
- stop if the robot reboots, SSH drops repeatedly, or servo power browns out

## K1 Device Check

Read-only checks:

```bash
hostname
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
pgrep -af 'wheeltec|lslidar|slam|scan_safety_guard|realsense|camera|servo|step7' || true
```

Expected serial presence includes the arm bus path or underlying USB device.

## ROS Guarded Stack Requirement

The guarded stack and D435 must already be running:

```text
n10p_tank_mapping_safety_guard.launch.py
realsense2_camera rs_launch.py
```

Use the validated P4 guarded-stack launch parameters below. Do not use the
launch file defaults for this demo: the default `hard_stop_m=1.00` can hold
`/cmd_vel_guarded` at zero in the same scene where the validated P4 parameters
allow a warning-band guarded motion.

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source /home/soc/lslidar_ws/install/setup.bash
source /home/soc/edge-ai-robot-k1/ros2_ws/install/setup.bash

ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  hard_stop_m:=0.30 \
  emergency_stop_m:=0.20 \
  slow_down_m:=0.80 \
  approach_stop_m:=0.80 \
  min_effective_forward:=0.08 \
  clear_max_linear:=0.30 \
  soft_max_linear:=0.30
```

Start D435 with the validated 640x480 profiles. If RGB/depth frame rates are
zero, unplug/replug the D435 and restart this launch before running Step7-E2.

```bash
source /opt/ros/humble/setup.bash
source /home/soc/realsense_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640,480,30 \
  depth_module.infra_profile:=640,480,30 \
  rgb_camera.color_profile:=640,480,30
```

Required topics include:

```text
/input_cmd_vel
/cmd_vel_guarded
/scan
/odom
/camera/camera/color/image_raw
/camera/camera/depth/image_rect_raw
/camera/camera/color/camera_info
```

The chassis motion path must remain:

```text
/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded
```

Never publish directly to `/cmd_vel_guarded`.

## Reproduction Command

Run on K1 from the repository root:

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source /home/soc/lslidar_ws/install/setup.bash
source /home/soc/edge-ai-robot-k1/ros2_ws/install/setup.bash

python3 tools/run_step7e2_guarded_motion_red_rule_flow.py \
  --output-dir outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_<new_id> \
  --policy-steps 5 \
  --capture-timeout-s 3.0 \
  --demo-fast-reuse-policy-base-zero \
  --enable-guarded-motion \
  --confirm-guarded-micro-motion \
  --confirm-n10p-safety \
  --confirm-no-direct-cmd-vel \
  --enable-arm-hardware \
  --confirm-map-gated-no-load \
  --confirm-no-contact \
  --confirm-base-zero-live \
  --confirm-no-cmd-vel
```

Use a new output directory for each run. Do not overwrite
`e2_guarded_red_rule_arm_hw_fastdemo_002`.

## Key Parameters

- Guarded stack launch:
  - `hard_stop_m:=0.30`
  - `emergency_stop_m:=0.20`
  - `slow_down_m:=0.80`
  - `approach_stop_m:=0.80`
  - `min_effective_forward:=0.08`
  - `clear_max_linear:=0.30`
  - `soft_max_linear:=0.30`
- D435 launch:
  - `depth_module.depth_profile:=640,480,30`
  - `depth_module.infra_profile:=640,480,30`
  - `rgb_camera.color_profile:=640,480,30`
- `--policy-steps 5`: bounded guarded micro-motion.
- `--capture-timeout-s 3.0`: faster D435 capture timeout for video demo.
- `--demo-fast-reuse-policy-base-zero`: reuses the immediately preceding
  guarded policy final base-zero evidence instead of launching a second
  base-zero observer.
- `--enable-guarded-motion` plus confirmation flags: required to move only
  through the guarded chassis path.
- `--enable-arm-hardware` plus confirmation flags: required for one Arm-C1
  no-load hardware response.

## Scene Preconditions For The Reference Motion

The `fastdemo_002` reference was not produced from an arbitrary starting pose.
Its first two policy actions were both `ARC_FAST_RIGHT`, and the red target was
seen after that camera viewpoint change:

```text
step 1: front_p10=0.612, pre_state=warning, ARC_FAST_RIGHT, yaw=-28.23deg, fwd=0.055m
step 2: front_p10=0.798, pre_state=warning, ARC_FAST_RIGHT, yaw=-32.73deg, fwd=0.063m
total:  policy_executed_count=2, cumulative_positive_forward_m=0.118m
```

If the run starts with `front_p10` in the clear band, the policy may choose a
small forward step instead of the two right arcs, and the D435 may not see the
red target. For a video reproduction, place the robot/box so the initial N10P
front sector is in the warning band around `0.60m <= front_p10 < 0.80m`, with
the right side clearer than the left, matching the reference run behavior.

Do not force this by bypassing the guard. If the policy does not produce visible
motion or does not move the camera view toward the red target, stop and adjust
the scene rather than repeating blind runs.

## Recent Rerun Root Causes

The late 2026-06-30 reruns showed several repeatable failure or non-ideal
causes:

- Wrong guarded-stack thresholds: launching the safety stack with defaults can
  use `hard_stop_m=1.00`, which holds `/cmd_vel_guarded` at zero in this demo
  scene. The stable reproduction must use the P4 parameters listed above,
  especially `hard_stop_m:=0.30`, `slow_down_m:=0.80`, and
  `clear_max_linear:=0.30`.
- D435 stream not publishing after restart or USB disturbance: topic names may
  exist while `image_raw` and `depth` do not deliver frames. Confirm at least
  one RGB frame, one depth frame, one camera_info frame, one odom frame, and one
  scan frame before launching Step7-E2.
- Scene started in the clear band: when `front_p10` is above the warning band,
  the policy can choose `FORWARD_0P15` rather than the reference two-arc
  behavior. This can make the red target miss the D435 field of view.
- Scene asymmetry changed arc direction: if the left side is more open than the
  right side, valid runs may choose `ARC_FAST_LEFT`. This is safe and expected,
  but it differs from the `fastdemo_002` video framing.
- Red target placement out of view: positive red-target runs fail with
  `red_object_detected=false` when the red patch is not in the post-motion D435
  frame or is too small/dim for the HSV rule.
- Battery or power interruption: a K1 or servo-controller reboot invalidates
  the current evidence run. Start a new output directory after any reboot.
- K1 lacks matplotlib: `risk_map_points.json` and the SLAM `.pgm/.yaml` maps
  are still valid, but `risk_map_snapshot.png` can be skipped with
  `snapshot_plot_skipped`. Generate preview PNGs later on the workstation if
  needed.
- Operator confirmation is separate evidence: the runner can prove serial
  bytes and action status, but final `6b` return/no-abnormal observations must
  be recorded from the operator after the run.

Do not treat the normal `HOLD_MAX_FAST_ARC` stop as a demo failure. In the
validated runs the policy executes two fast arcs and then stops because
`max_consecutive_fast_arc_reached` is the intended bounded-motion insurance.

## Stable Reference Runs

`fastdemo_002` is the original showcase baseline:

```text
policy_executed_count=2
policy_sequence_stop_reason=max_consecutive_fast_arc_reached
cumulative_positive_forward_m=0.118m
arc direction=ARC_FAST_RIGHT
red_object_detected=true
depth_median_m=0.561m
risk_map_points=1
arm_execution_status=succeeded
serial_bytes_written=180
operator confirmed final return to 6b with no abnormal issue
```

`fastdemo_010` is the latest good reproduction after the scene reset:

```text
output_dir=outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_010/
policy_executed_count=2
policy_sequence_stop_reason=max_consecutive_fast_arc_reached
step1=ARC_FAST_RIGHT, fwd=0.0537m, yaw=-10.3deg
step2=ARC_FAST_RIGHT, fwd=0.0733m, yaw=-23.16deg
cumulative_positive_forward_m=0.127m
final_map_saved=true
final_map=maps/policy_p4w_branch_mixed_20260630_215220_final.pgm/.yaml
red_object_detected=true
bbox_xywh=[513,234,127,107]
depth_median_m=0.615m
bbox_valid_depth_ratio=0.797
risk_map_points=1
projected=1
arm_execution_status=succeeded
hardware_executed=true
serial_bytes_written=180
errors=[]
operator confirmed final return to 6b
```

Use `fastdemo_010` as the preferred current reproduction example when the
teacher asks for the complete chain. Use `fastdemo_002` as the historical
baseline for the original camera framing.

## Success Criteria

- `status=succeeded`
- `guarded_motion_executed=true`
- `motion_command_path=/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded`
- `direct_cmd_vel_bypass=false`
- `base_zero_ok_after_motion=true`
- `base_zero_ok_before_capture=true`
- `base_zero_ok_before_arm=true`
- `red_object_detected=true`
- `d435_live_capture_executed=true`
- `risk_point_generated=true`
- `risk_map_points>=1`
- `projected>=1`
- `arm_candidate_selected=true`
- `selected_sequence=arm_b3_8_step_safety_adjusted_no_load_sample`
- `arm_execution_status=succeeded`
- `hardware_executed=true`
- `serial_bytes_written>0`
- `published_cmd_vel_during_capture=false`
- `published_cmd_vel_during_arm=false`
- `contact_allowed=false`
- `obstacle_removed=false`
- `errors=[]`
- operator confirms final pose is `6b` and no abnormal issue is observed

## Failure Criteria

Stop and do not rerun blindly if any of these occur:

- `base_zero_ok_after_motion=false`
- `base_zero_ok_before_capture=false`
- `red_object_detected=false` in a positive red-target run
- `risk_map_points=0`
- `arm_execution_status` is not `succeeded`
- `published_cmd_vel_during_capture=true`
- `published_cmd_vel_during_arm=true`
- `contact_allowed=true`
- `obstacle_removed=true`
- abnormal arm sound, stall, hard stop, jitter, or failure to return to `6b`
- repeated SSH drops or suspected battery brown-out

## Safety Boundary

This demo only claims:

- guarded motion through the existing P4/N10P safety chain
- D435 deterministic HSV red-rule trigger
- approximate risk map projection
- one Arm-C1 no-load response
- deterministic LLM-A report generation

This demo does not claim:

- trained-model visual recognition accuracy
- autonomous navigation or path planning
- high-precision SLAM
- grasping, contact, payload handling, or obstacle clearing
- LLM control of the robot

## Video Recording Suggestions

- Start recording before the command is launched.
- Keep the red target visible in the first camera view.
- Capture the chassis moving briefly and stopping under the guarded stack.
- Capture the D435 overlay if available on the operator screen.
- Capture the arm no-load response and final return to `6b`.
- Avoid adding a real obstacle for contact or clearing.
- In narration, say "red-rule trigger" and "no-load response", not "object
  recognition accuracy" or "clearing".
- Show the final `episode_report.json`, acceptance check, and LLM-A report
  after the physical shot.
