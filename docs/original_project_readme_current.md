# K1 Edge AI Robot

This private repository contains the working integration snapshot for the
SpacemiT K1 Muse Pi Pro based edge AI inspection robot.

## Current Preliminary Showcase Baseline

The current first-round showcase path, as of 2026-07-03, is supervised remote
mapping with local risk perception and operator-gated arm response:

```text
remote low-speed guarded SLAM mapping
-> D435 local YOLO risk detection on K1 SpaceMIT EP
-> confidence/depth-gated risk alarm
-> deduplicated bbox + depth + odom risk point
-> approximate risk point rendered as an odom/map-frame snapshot
-> operator-selected manual arm no-load response
-> deterministic risk report / dashboard
```

This is the path to record for the preliminary video. RL is not used to control
the real vehicle; Gazebo/RL material is only supporting simulation evidence.
The mechanical arm response is not automatically triggered by YOLO. The
operator decides when and where to run the fixed no-load response after the
robot has been remotely positioned and stopped.

Current saved map evidence:

- `maps/prelim_remote_mapping/map_20260703_095806.yaml`
- `maps/prelim_remote_mapping/map_20260703_095806_preview.png`
- `maps/prelim_remote_mapping/map_risk_live_20260703_103217.yaml`
- `maps/prelim_remote_mapping/map_risk_live_20260703_103217_preview.png`

Primary K1 operation order:

1. Start guarded mapping with N10P lidar, Tank odom, and `slam_toolbox`.
2. Start D435 ROS streams at 640x480.
3. Remote-control the Tank through `/input_cmd_vel`, not direct `/cmd_vel`.
4. Start local YOLO risk mapping and alarm generation.
5. Save the map after the run.
6. Review saved overlay frames and tune confidence/depth gates.
7. Run the arm no-load response manually only after operator confirmation.

Guarded mapping launch:

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash

ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  hard_stop_m:=0.10 \
  emergency_stop_m:=0.10 \
  slow_down_m:=0.30 \
  approach_stop_m:=0.20 \
  min_effective_forward:=0.05 \
  clear_max_linear:=0.30 \
  soft_max_linear:=0.10
```

Tank teleop must be remapped into the safety guard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/input_cmd_vel
```

Tank driving notes:

- use lowercase `i` / `,` for forward/back
- use lowercase `j` / `l` for in-place left/right turns
- use lowercase `u` / `o` for arc turns
- do not use uppercase `J` / `L`; those publish holonomic strafe commands that
  the Tank chassis ignores
- keep linear speed near the guarded mapping range instead of repeatedly
  increasing it with `q`

D435 launch:

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640,480,30 \
  depth_module.infra_profile:=640,480,30 \
  rgb_camera.color_profile:=640,480,30
```

Current K1 YOLO model used for the 5 percent light EP run:

```text
models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx
provider: SpaceMITExecutionProvider
input: [1, 3, 480, 640]
camera: D435 640x480 at 15 FPS
```

Formal auto alarm/map gates for the current small-map demo:

```text
crack:    confidence >= 0.29, 0.60 m <= depth <= 0.80 m
blockage: confidence >= 0.23, 0.35 m <= depth <= 0.75 m
```

Other classes can still be visible in raw YOLO output, but they are not promoted
to formal alarms or risk map points for this demonstration.

Standalone D435 YOLO EP check:

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/run_k1_yolo_ep_cli_light5.sh cli_ep_480x640_truncated6_light5
```

From the Windows host, watch the K1 YOLO log in a separate window:

```powershell
powershell -ExecutionPolicy Bypass -File tools\watch_k1_yolo_cli_log.ps1 `
  -HostName 192.168.43.40 `
  -User soc `
  -RunId cli_ep_480x640_truncated6_light5
```

Integrated risk mapping runner:

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

sudo env PYTHONUNBUFFERED=1 python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
  --provider spacemit \
  --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
  --imgsz 640 \
  --conf 0.15 \
  --iou 0.45 \
  --max-det 10 \
  --min-depth-m 0.20 \
  --max-depth-m 1.20 \
  --auto-risk-gates crack:0.29:0.60:0.80,blockage:0.23:0.35:0.75 \
  --output-dir outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_001
```

K1 local display UI:

```bash
cd /home/soc/edge-ai-robot-k1/outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_001
python3 -m http.server 8765 --bind 0.0.0.0
```

Open the dashboard on the K1 display at `http://127.0.0.1:8765/dashboard.html`,
or from the Windows host at `http://192.168.43.40:8765/dashboard.html`.

Expected outputs:

```text
dashboard.html
alarm_state.json
risk_events.jsonl
risk_event_index.json
risk_map_points.json
risk_map_snapshot.png
risk_control_report.md
episode_report.json
captures/<risk_event_id>/overlay.png
manual_arm_response_candidates/<risk_event_id>/manual_arm_response_candidate.json
```

Save the map at the end of a mapping run:

```bash
mkdir -p /home/soc/edge-ai-robot-k1/maps/prelim_remote_mapping
ros2 run nav2_map_server map_saver_cli \
  -f /home/soc/edge-ai-robot-k1/maps/prelim_remote_mapping/map_<run_id>
```

Manual review UI for saved risk frames:

```powershell
python tools\review_risk_detection_labels.py `
  --run-dir outputs\k1_pull\prelim_remote_mapping_yolo_arm_demo_v1\live_cpu_480_20260703_101632_final `
  --sort confidence `
  --open-browser
```

Current review rule of thumb: confidence alone is not a stable boundary. Use
confidence together with bbox depth before promoting a detection into a formal
alarm or map point. For the preliminary demonstration, prefer conservative
class/depth gates and avoid claiming general defect detection accuracy.

Claim boundary for the current preliminary showcase:

- remote mapping is supervised; it is not autonomous exploration
- YOLO is local K1 inference on the D435 stream under supplemental lighting
- risk map points are approximate bbox + depth + odom projections
- repeated detections are deduplicated by class and map/image grid
- arm response is manual, operator-gated, and no-load only
- no real obstacle clearing, grasping, payload handling, or contact is claimed
- RL does not control the real vehicle

Full procedure: `docs/prelim_remote_mapping_yolo_arm_demo_20260703.md`.

## Legacy Step7-E2 Integrated Demo Baseline

Step7-E2 fastdemo remains an earlier validated fallback path:

```text
P4/N10P guarded micro-motion
-> base_zero after motion
-> D435 HSV red-rule trigger
-> depth risk_point
-> approximate risk map projection
-> Arm-C1 no-load once
-> deterministic LLM-A report
```

Validated reference runs:

- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_002/`
- `outputs/step7e2_guarded_motion_red_rule_flow_v1/e2_guarded_red_rule_arm_hw_fastdemo_010/`

Stable K1 launch parameters for the live demo:

```bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  hard_stop_m:=0.30 \
  emergency_stop_m:=0.20 \
  slow_down_m:=0.80 \
  approach_stop_m:=0.80 \
  min_effective_forward:=0.08 \
  clear_max_linear:=0.30 \
  soft_max_linear:=0.30

ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640,480,30 \
  depth_module.infra_profile:=640,480,30 \
  rgb_camera.color_profile:=640,480,30
```

Stable Step7-E2 command on K1:

```bash
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

Scene conditions matter. The reference behavior is produced when the initial
N10P front sector is in the warning band, roughly
`0.60m <= front_p10 < 0.80m`, and the red target remains in the D435 view after
the guarded arc. If `front_p10` starts in the clear band, the policy may choose
a forward step instead of the two right arcs. If the D435 frame stream is not
publishing, restart the D435 launch before running Step7-E2.

Claim boundary: this demo claims guarded motion through the existing safety
chain, D435 deterministic red-rule triggering, approximate risk map projection,
one no-load arm response, and deterministic report generation. It does not
claim trained visual recognition accuracy, autonomous navigation, path
planning, high-precision SLAM, grasping, contact, payload handling, obstacle
clearing, or LLM control of the robot.

## K1 Local YOLO Vision Baseline

The preferred standalone K1 risk-vision check now uses the D435 color stream,
5 percent supplemental light, and the SpaceMIT EP 480x640 model:

```text
D435 V4L2 color stream
-> yolov8n_480x640_q_truncated6_balanced_blockage03.onnx
-> ONNX Runtime SpaceMITExecutionProvider
-> realtime CLI log or OpenCV display window
```

Preferred model for the current light-on demo:

```text
models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx
provider: SpaceMITExecutionProvider
input: 480x640
condition: 5 percent supplemental light on
```

Stable CLI wrapper:

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/run_k1_yolo_ep_cli_light5.sh cli_ep_480x640_truncated6_light5
```

The log should show:

```text
Using ONNX Runtime providers: ['SpaceMITExecutionProvider', 'CPUExecutionProvider']
Active ONNX Runtime providers: ['SpaceMITExecutionProvider', 'CPUExecutionProvider']
Model input shape: [1, 3, 480, 640]
```

Earlier 320x320 balanced-calibration K1 run:

```text
outputs/k1_d435_yolo_realtime_v1/headless_320_q_truncated_balanced_spacemit_current_001/
detected_frame_count: 53/60
avg_latency_ms: 66.445
avg_infer_fps: 15.327
detected class in test scene: leakage
confidence range: 0.2080 - 0.3408
calibration list: models/risk_vision/xquant_yolov8n_320/calib_list_balanced.txt
balanced calibration first 128 images: crack 37, corrosion 37, leakage 40, blockage 17
```

FP32 fallback:

```text
models/risk_vision/yolov8n_320_fp32.onnx
provider: SpaceMITExecutionProvider
validated fallback fps: about 3.8 - 4.0
```

K1 display command when a monitor is attached:

```bash
cd /home/soc/edge-ai-robot-k1

sudo env DISPLAY=:0 XAUTHORITY=/home/soc/.Xauthority QT_QPA_PLATFORM=xcb \
  python3 tools/run_k1_d435_yolo_realtime_display.py \
  --provider spacemit \
  --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
  --width 640 --height 480 --fps 15 --imgsz 640 \
  --conf 0.15 --iou 0.45 --max-det 10 \
  --warmup-frames 90 \
  --output-dir outputs/k1_d435_yolo_realtime_v1/ui_ep_480x640_truncated6_light5
```

This visual recognition mode is local inference only:

- no online API
- no ROS startup
- no `cmd_vel` publish
- no serial port access
- no chassis or arm control

Claim boundary: this mode can be used to demonstrate local K1 YOLO inference
on D435 camera frames under supplemental lighting. It does not claim
real-world defect detection accuracy, low-light robustness without lighting,
mapping accuracy, autonomous navigation, or robot control.

## Preliminary Remote-Mapping Demo Bridge

This section documents the implementation bridge behind the current preliminary
showcase. The scope is operator-controlled guarded mapping plus local risk
perception:

```text
remote/manual guarded mapping
-> D435 local YOLO risk detection
-> deduplicated risk alarm
-> approximate bbox + depth + odom risk map point
-> manual arm no-load response candidate
-> deterministic risk report / dashboard
```

Primary runner:

```bash
python3 tools/run_prelim_remote_mapping_yolo_arm_demo.py \
  --provider spacemit \
  --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
  --imgsz 640 \
  --conf 0.15 \
  --iou 0.45 \
  --max-det 10 \
  --min-depth-m 0.20 \
  --max-depth-m 1.20 \
  --output-dir outputs/prelim_remote_mapping_yolo_arm_demo_v1/live_001
```

This runner does not publish `cmd_vel`, does not start chassis motion, and
does not automatically start the arm. It only generates a manual arm no-load
response candidate; the operator decides when and where to run the fixed
no-load sequence after remotely positioning the robot. Full procedure:
`docs/prelim_remote_mapping_yolo_arm_demo_20260703.md`.

Offline dry-run before the live run:

```bash
python tools/run_prelim_yolo_map_dryrun.py \
  --capture-dir outputs/p4x_d435_hold_capture_v1/captures/p4x_hold_capture_20260629_223453_capture_08 \
  --model models/risk_vision/yolov8n_480x640_q_truncated6_balanced_blockage03.onnx \
  --imgsz 640 \
  --conf 0.15 \
  --iou 0.45 \
  --max-det 10 \
  --output-dir outputs/prelim_yolo_map_dryrun_v1/offline_real_odom_check
```

If an image has D435 depth but no robot odom, use the synthetic-pose mode only
to verify the YOLO -> depth -> map-rendering data path:

```bash
python tools/run_prelim_yolo_map_dryrun.py \
  --capture-dir datasets/risk_print_yolo_v1/captures_raw/crack/20260702_152607_700_crack_01_01 \
  --model models/risk_vision/yolov8n.onnx \
  --imgsz 640 \
  --conf 0.25 \
  --synthetic-odom-if-missing \
  --output-dir outputs/prelim_yolo_map_dryrun_v1/offline_crack_synthetic_pose_001
```

Synthetic odom outputs are labeled `odom_source=synthetic_dry_run` and should
not be used as real robot localization evidence. For the preliminary video,
record one mapping-session capture where the risk card, depth, camera info,
and `/odom` are all available in the same run.

## Competition Interface Layer Baseline

The repository now includes a training-prep interface layer for Gazebo/RL,
local vision models, local report/LLM backends, and future real-vehicle
adapters. This layer freezes the high-level operation vocabulary so learning
systems target the same semantics that have already been validated on K1,
instead of issuing low-level hardware commands.

Primary files:

- `configs/primitive_registry.yaml`
- `configs/action_semantics.yaml`
- `configs/rl_action_space.yaml`
- `configs/risk_detection_backends.yaml`
- `configs/local_llm_config.yaml`
- `schemas/episode_report_v2.schema.json`
- `src/primitives/`
- `rl/envs/semantic_guarded_nav_env.py`
- `tools/run_competition_primitive_stack_dryrun.py`

Contract audit status:

- `episode_report_v2.observation_state` is an array so multi-step episodes can
  preserve per-step observations instead of collapsing them into one object.
- observation fields use the real vehicle sector naming:
  `front_min`, `front_p10`, `left_p10`, and `right_p10`.
- RL observations include policy guard counters: `consecutive_fast_arc` and
  `total_forward_m`.
- `benchmarks.vision`, `benchmarks.llm`, and `benchmarks.rl` use explicit
  fields for later competition review.
- `risk_map_summary.schema.json` uses inline risk-point definitions to avoid
  local `$ref` resolution problems.

Frozen high-level primitives include:

```text
HOLD
FORWARD_0P15
ARC_FAST_LEFT
ARC_FAST_RIGHT
SAVE_MAP
STOP_SAFE
HOLD_CAPTURE
D435_CAPTURE
RISK_DETECT_HSV_RED_RULE
RISK_DETECT_LOCAL_MODEL
RISK_CLASSIFY_PRINTED_RISK
RISK_PROJECT_TO_MAP
RISK_MAP_SUMMARY
ARM_HOME_6B
ARM_NO_LOAD_RESPONSE
ARM_CLEAR_CANDIDATE_DRYRUN
REPORT_DETERMINISTIC
REPORT_LOCAL_LLM
```

RL/Gazebo may output only high-level action candidates. The default RL action
space is:

```text
0 HOLD
1 FORWARD_0P15
2 ARC_FAST_LEFT
3 ARC_FAST_RIGHT
4 HOLD_CAPTURE
5 ARM_NO_LOAD_RESPONSE
6 STOP_SAFE
```

The RL observation vocabulary is aligned to the real scan-sector snapshot and
policy guard state:

```text
front_min
front_p10
left_p10
right_p10
odom_x
odom_y
odom_yaw
map_progress
risk_detected
risk_confidence
risk_class_id
risk_distance_m
base_zero
arm_ready
capture_recent
steps_since_capture
consecutive_fast_arc
total_forward_m
```

Real-vehicle safety boundary:

- chassis motion must route through
  `/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded`
- RL must not output raw `cmd_vel`
- no direct `/cmd_vel_guarded` publish from RL
- no direct chassis serial write from RL
- no direct arm servo pulse output from RL
- arm primitives require `base_zero=true`
- `ARM_NO_LOAD_RESPONSE` remains no-load only

Local AI/report boundary:

- `hsv_red_rule` is a deterministic baseline, not an AI accuracy claim
- local YOLO claims must cite measured K1 evidence, including provider, model
  path, input shape, FPS/latency, lighting condition, and saved overlays
- deterministic reports are not real local LLM inference
- local LLM backends must record TTFT, tokens/s, token count, model name, model
  size, and memory before competition claims

Dry-run validation:

```powershell
python tools\validate_primitive_registry.py --registry configs\primitive_registry.yaml
python tools\validate_action_semantics.py --action-semantics configs\action_semantics.yaml
python tools\export_rl_action_space.py --output outputs\rl_semantic_action_space_v1
python tools\run_competition_primitive_stack_dryrun.py --output-dir outputs\competition_primitive_stack_dryrun_v1
python tools\validate_episode_report_v2.py --episode-report outputs\competition_primitive_stack_dryrun_v1\episode_report_v2.json
```

Optional semantic RL smoke test:

```powershell
python rl\train_semantic_ppo.py --config rl\configs\rl_semantic_ppo.yaml --output-dir outputs\rl_semantic_train_v1\smoke_001
python rl\eval_semantic_policy.py --output-dir outputs\rl_semantic_eval_v1\eval_001
```

Expected dry-run acceptance:

```text
primitive_registry_valid=true
action_space_valid=true
risk_detection_schema_valid=true
risk_map_summary_valid=true
episode_report_v2_valid=true
no_direct_cmd_vel=true
no_direct_servo_pulse=true
hardware_executed=false
errors=[]
```

This interface layer does not start ROS, open serial ports, control the chassis,
control the arm, or modify prior Step7/P4 evidence. It is the contract layer for
the next Ubuntu/Gazebo/RL and local-model work.

## Scope

Included:

- ROS 2 Humble base driver package for the WHEELTEC L150Pro Tank chassis.
- C30D Tank drive profile and N10P mapping launch files.
- Non-arm K1 bring-up package for base, lidar, D435 placeholder, light, risk engine, and event logger.
- Intel RealSense D435 bring-up notes.
- Bus servo controller wiring and smoke-test tool.
- GPIO37 light control scripts.
- Real sensor event adapters for N10P `/scan` and D435/RGB low-light events.
- Current STM32 hex artifacts used during bring-up.
- Bring-up notes and system state.

Not included:

- Vendor document archives.
- Proxy subscription files.
- API keys or tokens.
- Large model artifacts and ROS bags.
- Automatic mechanical-arm startup or real arm motion.

## Non-arm Bring-up Package

This branch adds a non-arm bring-up layer for the next K1 upload. It does not
depend on an arm URDF and does not trigger real arm actions.

Build on K1:

```bash
cd ~/edge-ai-robot-k1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
cd ~/edge-ai-robot-k1
source ros2_ws/install/setup.bash
```

Safe static launch:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py
```

Publish a mock event:

```bash
cd ~/edge-ai-robot-k1
python3 tools/publish_mock_event.py --type soft_obstacle --distance 0.8 --confidence 0.9
```

Check risk outputs:

```bash
ros2 topic echo /risk/current_level
ros2 topic echo /risk/recommended_action
ros2 topic echo /risk/current_event
```

Safety defaults:

- no `/cmd_vel` is published by bring-up or smoke-test scripts
- arm actions are not launched
- light starts at brightness `0`
- base, lidar, and camera are disabled unless explicitly enabled
- event logging and risk rules can be tested with mock events
- real sensor adapters are disabled unless explicitly enabled

K1 validation on 2026-06-25: `colcon build --symlink-install` completed for
the non-arm workspace, and static risk/logger validation produced
`soft_obstacle -> medium / stop_and_recheck`. On the current K1 image, GPIO37
sysfs is owned by `root`; use `light_dry_run:=true` for normal-user static
testing, or install/run a root-side GPIO service for real lamp control.

Install boot-time light-off guard on K1:

```bash
cd ~/edge-ai-robot-k1
sudo scripts/install_gpio37_boot_low_service.sh
```

This forces GPIO37 low when Linux reaches systemd. For guaranteed off state
immediately after power is applied, add a physical pulldown resistor from the
light PWM/control input to GND.

## Real Sensor Event Adapter

The real sensor adapter layer converts N10P/D435 data into the same
`/perception/mock_event` JSON format used by the existing risk engine.

Safe combined adapter launch:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=true \
  use_camera:=true \
  camera_enable_depth:=false \
  use_light:=true \
  light_dry_run:=true \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_scan_event_adapter:=true \
  use_camera_low_light_adapter:=true
```

This launch still does not publish `/cmd_vel`.
By default, camera bring-up is RGB-only. Enable depth/point cloud explicitly
with `camera_enable_depth:=true enable_pointcloud:=true` after confirming D435
power and USB stability.

Automatic low-light lamp bridge, GPIO dry-run mode:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=true \
  camera_enable_depth:=false \
  use_light:=true \
  light_dry_run:=true \
  use_light_action_bridge:=true \
  light_auto_on_brightness:=5 \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_camera_low_light_adapter:=true
```

This maps `turn_on_light_and_recheck` to `/light/brightness_cmd` with a smooth
ramp and timed auto-off. Use `light_dry_run:=false` only after lamp current,
temperature, and wiring are checked.

Adaptive D435 RGB light control:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=true \
  camera_enable_depth:=false \
  use_light:=true \
  light_dry_run:=true \
  use_light_action_bridge:=false \
  use_adaptive_light_controller:=true \
  adaptive_light_max_brightness:=5 \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_camera_low_light_adapter:=true
```

This computes D435 RGB `mean_luma` and `dark_pixel_ratio`, publishes
`/light/adaptive_status`, and adjusts `/light/brightness_cmd` in safe 0-5%
steps by default. It is the preferred low-light demonstration mode; do not run it together
with `use_light_action_bridge:=true`.

## Mapping MVP

The first mapping target is a guarded, low-speed N10P + Tank odom +
`slam_toolbox` MVP. It is not autonomous exploration.

Current capability: supervised manual/remote low-speed mapping.

Not enabled yet: automatic obstacle avoidance, Nav2 autonomous navigation, RRT
automatic exploration, or unattended patrol.

The mapping launch uses the Python Tank base driver for the C30D ROS firmware.
It sends the official security keepalive, publishes `/odom`, and publishes
`odom -> base_footprint` TF for SLAM.

Minimum test order on K1:

```bash
cd ~/edge-ai-robot-k1
tools/mapping_preflight_check.sh

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash  # if lslidar_driver is installed separately
python3 tools/send_safe_zero_cmd.py

ros2 launch turn_on_wheeltec_robot n10p_tank_mapping.launch.py
```

In separate terminals:

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames
```

Only after `/scan`, `/odom`, and TF are healthy, run one guarded continuous
motion calibration segment:

```bash
python3 tools/continuous_drive_calibration.py --linear 0.30 --duration 3.0 --ramp-time 0.4
python3 tools/send_safe_zero_cmd.py
```

Save a map:

```bash
tools/save_slam_map.sh
```

Manual mapping with front scan safety guard:

```bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  serial_port:=/dev/base_controller \
  max_linear_x:=0.45 \
  max_angular_z:=0.80 \
  brake_duration_sec:=1.0
```

This guarded launch routes manual commands through:

```text
/input_cmd_vel -> scan_safety_guard_node -> /cmd_vel_guarded -> Tank base
```

Rules: `front_p10 <= 1.00 m`, `front_min <= 0.45 m`, rapid approach, or
time-to-collision below `1.20 s` forces zero speed and latches hard stop for
about `1.50 s`. `1.00-1.60 m` is warning only; it keeps the tested `0.30 m/s`
crawl cap instead of compressing to ineffective low speeds. Stale `/scan` fails
closed to zero. Forward commands below `0.28 m/s` are treated as zero because the
Tank chassis does not reliably overcome static friction below that range. It also publishes
`/safety/front_obstacle` and `/perception/mock_event` for risk logging. Full
checklist: `docs/manual_mapping_scan_safety_guard_2026-06-27.md`.

Safety limits: no high-speed motion, no arm, no automatic navigation, no RRT
exploration, and no unattended ground run. Full checklist:
`docs/mapping_mvp_test_2026-06-26.md`.

Ground test note: this Tank chassis does not reliably start moving at
`0.05-0.10 m/s`. For mapping validation, tune continuous guarded motion around
`0.30 m/s` before Nav2. See
`docs/c30d_pid_continuous_mapping_tuning_2026-06-26.md` and
`docs/reference_findings_continuous_mapping_2026-06-26.md`.

## Guarded Nav2 Small Goal Test

The current Nav2 target is a supervised small-goal validation on the saved map,
not SLAM exploration and not RRT. Nav2 must never publish directly to the base:

```text
Nav2 -> /cmd_vel_raw -> scan_safety_guard_node -> /cmd_vel_guarded -> Tank base
```

Start on K1:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash

ros2 launch turn_on_wheeltec_robot n10p_tank_nav2_guarded.launch.py
```

The default map is:

```text
~/edge-ai-robot-k1/maps/mapping_fixed_odom_20260628_085623.yaml
```

Before sending a goal, verify `/cmd_vel_guarded` is zero and `/scan`, `/odom`,
`/map`, and TF are healthy. Then send only one small supervised goal:

```bash
python3 tools/nav2_guarded_small_goal.py --distance 0.2 --confirm YES
```

Allowed goal distances are exactly `0.2`, `0.3`, and `0.5` meters. Cancel and
force zero:

```bash
python3 tools/nav2_cancel_and_zero.py --duration 6
```

Nav2 speed limits in this mode are capped at `max_vel_x <= 0.20`,
`max_vel_theta <= 0.35`, and `acc_lim_x <= 0.15`. Recovery spin/backup behavior
servers are not launched, and the behavior tree omits recovery actions. Full
checklist: `docs/nav2_guarded_small_goal_2026-06-28.md`.

## Main Commands

Base driver only:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 launch turn_on_wheeltec_robot tank_base_brake.launch.py
```

N10P lidar + Tank base + slam_toolbox:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash  # if lslidar_driver is installed separately
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping.launch.py
```

Light off:

```bash
sudo python3 ~/tools/gpio37_light_smooth.py off
```

Smooth light ramp:

```bash
sudo python3 ~/tools/gpio37_light_smooth.py set 5 --start 0 --ramp 2 --hold -1
```

## Current Status

See:

- `docs/vehicle_status_2026-06-25.md`
- `docs/bringup_commands.md`
- `docs/d435_realsense.md`
- `docs/bus_servo_controller.md`
- `docs/non_arm_bringup_2026-06-25.md`
- `docs/real_sensor_event_adapters_2026-06-25.md`
- `docs/tomorrow_test_plan_2026-06-25.md`
- `docs/today_work_plan_2026-06-25.md`
- `docs/today_work_plan_2026-06-26.md`
- `docs/mapping_mvp_test_2026-06-26.md`
- `docs/local_llm_research_2026-06-25.md`
- `docs/risk_dataset_design_2026-06-25.md`
- `docs/hazard_scene_simulation_plan_2026-06-25.md`
- `docs/primitive_action_semantics_20260701.md`
- `docs/risk_vision_model_interface_20260701.md`
- `docs/local_llm_report_interface_20260701.md`
- `docs/rl_semantic_action_space_20260701.md`
- `docs/arm_d_clearance_staging_20260701.md`
- `docs/competition_completion_plan_20260701.md`
- `docs/k1_d435_risk_vision_deployment_plan_20260701.md`
- `docs/d435_dataset_collection_workflow_20260701.md`
- `docs/k1_hold_capture_mapping_schema_20260702.md`
- `docs/risk_map_summary_interface_20260702.md`
- `docs/k1_yolov8n_onnx_deployment_20260702.md`
- `docs/k1_xquant_yolov8_truncated_quantization_20260702.md`
- `docs/prelim_remote_mapping_yolo_arm_demo_20260703.md`
- `configs/risk_classes.yaml`
- `configs/sop_knowledge_base.json`
- `tools/run_k1_yolo_ep_cli_light5.sh`
- `tools/run_prelim_remote_mapping_yolo_arm_demo.py`
- `tools/run_prelim_yolo_map_dryrun.py`
- `tools/review_risk_detection_labels.py`
- `maps/prelim_remote_mapping/`
- `firmware/README.md`
