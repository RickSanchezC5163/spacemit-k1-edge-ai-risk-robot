# Non-arm K1 Bring-up - 2026-06-25

This is the bring-up procedure for the non-arm part of the K1 inspection robot.

Scope:

- chassis driver
- N10P lidar
- D435 or camera placeholder
- GPIO37 light control
- real sensor event adapters
- mock-event risk engine
- JSONL event logger

Out of scope:

- no arm URDF dependency
- no real arm action
- no unsafe servo angle changes
- no automatic `/cmd_vel` movement in smoke tests

## 1. Upload Or Pull Code

From the PC, upload the repository to the K1:

```bash
scripts/deploy_to_k1.sh -u <K1_USER> -i 192.168.43.40 -d ~/edge-ai-robot-k1
```

Or on the K1, pull from GitHub if the repository is already cloned:

```bash
cd ~/edge-ai-robot-k1
git fetch
git switch codex/non-arm-system-integration-20260625-afternoon
git pull
```

## 2. Source ROS2

```bash
cd ~/edge-ai-robot-k1/ros2_ws
source /opt/ros/humble/setup.bash
```

## 3. Build

```bash
colcon build --symlink-install
cd ~/edge-ai-robot-k1
source ros2_ws/install/setup.bash
```

## 4. Start Non-arm Bring-up

Safe dry hardware mode, with only light/risk/logger enabled:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py
```

Default values are safe:

- `use_base:=false`
- `use_lidar:=false`
- `use_camera:=false`
- `use_light:=true`
- `use_risk_engine:=true`
- `use_event_logger:=true`
- `use_scan_event_adapter:=false`
- `use_camera_low_light_adapter:=false`

Hardware mode after ports are confirmed:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=true \
  use_lidar:=true \
  use_camera:=false \
  use_light:=true \
  use_risk_engine:=true \
  use_event_logger:=true
```

D435 mode after RealSense ROS wrapper is built:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=true \
  use_lidar:=true \
  use_camera:=true \
  use_light:=true \
  use_risk_engine:=true \
  use_event_logger:=true
```

The launch file does not publish `/cmd_vel`.

If running as the normal `soc` user and GPIO37 sysfs is owned by `root`, use
dry-run light mode for risk/logger validation:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=false \
  use_light:=true \
  use_risk_engine:=true \
  use_event_logger:=true \
  light_dry_run:=true \
  event_log_dir:=/home/soc/edge-ai-robot-k1/logs/events
```

Default GPIO mode should still keep brightness at `0`; if the user lacks sysfs
write permission, the light node logs a clear permission error and stays alive.

## 5. Real Sensor Event Adapters

The adapter layer converts real sensor data into the same
`/perception/mock_event` JSON accepted by `k1_risk_engine`.

N10P scan adapter only:

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

D435/RGB low-light adapter only:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=true \
  camera_enable_depth:=false \
  use_light:=true \
  light_dry_run:=true \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_scan_event_adapter:=false \
  use_camera_low_light_adapter:=true
```

Combined adapter launch:

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

The scan adapter publishes `soft_obstacle` or `blocked_path`. The camera
adapter publishes `low_light`. Neither adapter commands the chassis or arm.
The default D435 bring-up is RGB-only; enable depth and point cloud explicitly
after USB and power stability are confirmed.

The D435 low-light adapter defaults to `camera_warmup_seconds:=3.0` and
`camera_required_consecutive_frames:=2`. This avoids a false low-light event
from the first RealSense auto-exposure frames after launch.

## 6. Publish Mock Event

In another terminal:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

python3 tools/publish_mock_event.py \
  --type soft_obstacle \
  --distance 0.8 \
  --confidence 0.9
```

## 7. Check Risk Output

```bash
ros2 topic echo /risk/current_level
ros2 topic echo /risk/recommended_action
ros2 topic echo /risk/current_event
```

Expected result for `soft_obstacle`, `distance_m=0.8`:

```text
/risk/current_level: medium
/risk/recommended_action: stop_and_recheck
```

Additional supported event types:

- `soft_obstacle`
- `hard_obstacle`
- `blocked_path`
- `low_light`
- `cable_or_wire`
- `reflective_noise`
- `unknown`

## 8. Check Logs

```bash
find logs/events -type f -name 'events_*.jsonl' -print
tail -n 5 logs/events/events_*.jsonl
```

Each line should be one JSON object with:

- timestamp
- event_type
- risk_level
- recommended_action
- distance_m
- confidence
- source

## 9. Light Test

Default launch brightness is `0`, so the light should be off at startup.

Install the boot-time GPIO37 low guard before long tests:

```bash
cd ~/edge-ai-robot-k1
sudo scripts/install_gpio37_boot_low_service.sh
sudo systemctl status k1-gpio37-light-off.service --no-pager
```

The installed guard pulls GPIO37 low and grants write permission to
`/sys/class/gpio/gpio37/direction` and `/sys/class/gpio/gpio37/value`, so the
normal `soc` user can run the ROS light node after boot.

Immediate force-off command:

```bash
sudo sh -c 'GPIO=37; D=/sys/class/gpio/gpio$GPIO; [ -d "$D" ] || echo $GPIO > /sys/class/gpio/export; echo out > "$D/direction"; echo 0 > "$D/value"'
```

Set brightness to 5:

```bash
ros2 topic pub --once /light/brightness_cmd std_msgs/msg/Int32 "{data: 5}"
ros2 topic echo --once /light/status
```

Turn off:

```bash
ros2 topic pub --once /light/brightness_cmd std_msgs/msg/Int32 "{data: 0}"
```

Do not run the 20 W light at high brightness until current and temperature are
checked.

Automatic low-light to lamp command bridge, safe dry GPIO mode:

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
  light_auto_hold_seconds:=8.0 \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_scan_event_adapter:=false \
  use_camera_low_light_adapter:=true
```

In this mode a `low_light` event becomes:

```text
/risk/recommended_action: turn_on_light_and_recheck
/light/brightness_cmd: 0 -> 5
```

The bridge holds the light target for `light_auto_hold_seconds` after the last
matching risk action, then ramps back to `0`. With `light_dry_run:=true`, the
light node receives and reports brightness commands but does not write the GPIO.

Real lamp mode after current, temperature, and wiring are checked:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=true \
  camera_enable_depth:=false \
  use_light:=true \
  light_dry_run:=false \
  use_light_action_bridge:=true \
  light_auto_on_brightness:=5 \
  light_auto_hold_seconds:=8.0 \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_camera_low_light_adapter:=true
```

Keep `light_auto_on_brightness` at `0-5` for this 20 W lamp. The first real
test showed that `10-15` can already overexpose the D435 RGB image, so higher
values should be used only after adding diffusion, changing lamp angle, or
rechecking current and temperature.

Adaptive D435 RGB light controller, GPIO dry-run mode:

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
  adaptive_light_step_limit:=5 \
  adaptive_light_stable_frames:=3 \
  adaptive_light_update_rate_hz:=1.0 \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_camera_low_light_adapter:=true
```

The adaptive controller subscribes to D435 RGB frames, computes
`mean_luma` and `dark_pixel_ratio`, publishes `/light/adaptive_status`, and
publishes `/light/brightness_cmd`. Internally it uses this brightness request
table, then caps the command by `adaptive_light_max_brightness`:

```text
mean_luma >= 80      -> 0
65 <= mean_luma < 80 -> 10
50 <= mean_luma < 65 -> 15
35 <= mean_luma < 50 -> 20
mean_luma < 35       -> 25
```

Safety behavior:

- default `adaptive_light_max_brightness:=5`
- each update changes brightness by at most `adaptive_light_step_limit`
- `adaptive_light_stable_frames:=3` avoids flicker from exposure jitter
- no image for `adaptive_light_image_timeout_s:=3.0` forces brightness `0`
- node shutdown publishes brightness `0`
- it never publishes `/cmd_vel` and never controls the arm

Do not enable `use_light_action_bridge` and `use_adaptive_light_controller` at
the same time during normal tests; both publish `/light/brightness_cmd`.

Real adaptive lamp mode after current, temperature, and wiring are checked:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=true \
  camera_enable_depth:=false \
  use_light:=true \
  light_dry_run:=false \
  use_light_action_bridge:=false \
  use_adaptive_light_controller:=true \
  adaptive_light_max_brightness:=5 \
  use_risk_engine:=true \
  use_event_logger:=true \
  use_camera_low_light_adapter:=true
```

For this lamp, keep `adaptive_light_max_brightness` at `5` until a diffuser or
new lamp angle prevents D435 overexposure. Watch current draw and lamp
temperature for 5-10 minutes before considering any increase.

Systemd cannot control the pin before Linux userspace starts. If the light
briefly turns on right after power is applied, add a hardware pulldown resistor
from the PWM/control input to GND.

## 10. Optional Manual Chassis Test

No smoke test sends motion commands automatically.

If the chassis must be tested, do it manually with the robot suspended or
guarded:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Then stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 11. K1 Validation Notes

Validated on K1 / Bianbu LXQT v2.3.3 on 2026-06-25:

- `colcon build --symlink-install` completed for all 7 packages.
- Static launch with `use_base:=false`, `use_lidar:=false`,
  `use_camera:=false`, `light_dry_run:=true` started light/risk/logger.
- `soft_obstacle`, `distance_m=0.8` produced `medium` and
  `stop_and_recheck`.
- `low_light`, `distance_m=1.2` produced `medium` and
  `turn_on_light_and_recheck`.
- JSONL event logs were written under
  `/home/soc/edge-ai-robot-k1/logs/events/`.
- Default GPIO mode as normal user reports permission denied for
  `/sys/class/gpio/gpio37/direction`, but the light node does not crash.
