# Real Sensor Event Adapters - 2026-06-25

This note describes the first real-sensor adapter layer for the non-arm K1
risk loop.

Goal:

```text
N10P / D435 real sensor input
-> /perception/mock_event compatible JSON
-> k1_risk_engine
-> k1_event_logger
```

The adapters only publish risk events. They do not publish `/cmd_vel`, do not
start the arm, and do not turn on the real 20 W lamp by default.

## Scan Event Adapter

Node:

```bash
scan_event_adapter_node
```

Input:

- topic: `/scan`
- type: `sensor_msgs/msg/LaserScan`

Output:

- topic: `/perception/mock_event`
- type: `std_msgs/msg/String`

Default parameters:

- `scan_topic:=/scan`
- `event_topic:=/perception/mock_event`
- `front_sector_deg:=30.0`
- `soft_threshold_m:=1.0`
- `blocked_threshold_m:=0.5`
- `publish_rate_hz:=2.0`
- `min_valid_range_m:=0.05`
- `max_valid_range_m:=8.0`
- `dry_run:=false`

Rules:

- Only the front sector is inspected, defaulting to `-15 deg` to `+15 deg`.
- NaN, inf, too-close, and too-far ranges are ignored.
- If `front_min_range_m < blocked_threshold_m`, publish `blocked_path`.
- Else if `front_min_range_m < soft_threshold_m`, publish `soft_obstacle`.
- Otherwise, do not publish an event.

Example event:

```json
{
  "event_type": "blocked_path",
  "distance_m": 0.42,
  "confidence": 0.9,
  "source": "n10p_scan",
  "front_min_range_m": 0.42,
  "front_sector_deg": 30.0
}
```

## Camera Low-Light Adapter

Node:

```bash
camera_low_light_adapter_node
```

Input:

- default topic: `/camera/camera/color/image_raw`
- type: `sensor_msgs/msg/Image`

Output:

- topic: `/perception/mock_event`
- type: `std_msgs/msg/String`

Default parameters:

- `image_topic:=/camera/camera/color/image_raw`
- `event_topic:=/perception/mock_event`
- `luma_threshold:=55.0`
- `dark_pixel_threshold:=50.0`
- `dark_ratio_threshold:=0.6`
- `publish_rate_hz:=1.0`
- `resize_width:=320`
- `warmup_seconds:=3.0`
- `required_consecutive_frames:=2`
- `dry_run:=false`

Rules:

- Supported encodings include `mono8`, `8UC1`, `rgb8`, `bgr8`, `rgba8`,
  `bgra8`, and basic `yuyv/yuv422`.
- The node computes `mean_luma` and `dark_pixel_ratio`.
- Startup frames are ignored for `warmup_seconds`, because RealSense auto
  exposure can produce a dark frame immediately after launch.
- A low-light event is published only after `required_consecutive_frames`
  consecutive low-light samples.
- If `mean_luma < luma_threshold` or
  `dark_pixel_ratio > dark_ratio_threshold`, publish `low_light`.
- Normal brightness does not publish events.

Example event:

```json
{
  "event_type": "low_light",
  "distance_m": -1,
  "confidence": 0.82,
  "source": "camera_luma",
  "mean_luma": 38.5,
  "dark_pixel_ratio": 0.72
}
```

## Safe Launch

Static non-arm stack without real sensor adapters:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=false \
  use_light:=true \
  light_dry_run:=true \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_scan_event_adapter:=false \
  use_camera_low_light_adapter:=false
```

N10P scan adapter:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=true \
  use_camera:=false \
  use_light:=true \
  light_dry_run:=true \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_scan_event_adapter:=true \
  use_camera_low_light_adapter:=false
```

D435/RGB low-light adapter:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=true \
  use_light:=true \
  light_dry_run:=true \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_scan_event_adapter:=false \
  use_camera_low_light_adapter:=true
```

Combined real sensor adapter launch:

```bash
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

The current K1 RealSense ROS package publishes the color image at
`/camera/camera/color/image_raw` when using the default namespace and camera
name. `non_arm_bringup.launch.py` defaults the adapter to that topic.

For depth or point cloud work, explicitly enable them:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=true \
  camera_enable_depth:=true \
  enable_pointcloud:=true \
  use_light:=true \
  light_dry_run:=true
```

Acetate tape can blur or scatter the picture without making it dark. In one K1
test, taped RGB frames measured `mean_luma ~= 112` and
`dark_pixel_ratio ~= 0.09`, so the default `luma_threshold:=55` correctly did
not classify it as low light. For a threshold-only low-light test, either cover
the lens with an opaque cap/tape or temporarily run
`camera_luma_threshold:=130.0`.

Black electrical tape did trigger the default low-light rule in the same setup:
RGB stayed at about 30 Hz, sampled frames measured `mean_luma ~= 57.5` and
`dark_pixel_ratio ~= 0.73`, and the adapter published
`low_light -> medium / turn_on_light_and_recheck`.

Normal indoor light with no tape measured `mean_luma ~= 110.75` and
`dark_pixel_ratio ~= 0.004` at about 29 Hz, so it should not trigger low-light.
One initial RealSense auto-exposure frame measured much darker, which is why
the adapter now defaults to a 3 second warmup and 2 consecutive low-light
samples before publishing.

## Test Commands

Check current topics:

```bash
ros2 topic list
```

Check adapter output through the existing risk loop:

```bash
ros2 topic echo --once /risk/current_event
ros2 topic echo --once /risk/current_level
ros2 topic echo --once /risk/recommended_action
```

Run the safe helper script:

```bash
tools/real_sensor_non_arm_test.sh
tools/topic_health_check.sh
```

## Safety Notes

- These adapters do not move the chassis.
- These adapters do not start or command the mechanical arm.
- Real lamp output remains disabled by using `light_dry_run:=true`.
- Enabling `use_lidar` or `use_camera` only starts sensor drivers and adapters;
  it must not be combined with autonomous motion until the chassis stop logic is
  separately validated.

## Known Limits

- The scan adapter only detects near front-sector range risk. It cannot classify
  whether an obstacle is soft or hard.
- The camera adapter only detects low light. It is not an object detector.
- YOLO or another visual model is still needed for cable, obstacle class, and
  scene-level risk recognition.
