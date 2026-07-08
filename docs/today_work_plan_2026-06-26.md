# Today Work Plan - 2026-06-26

Goal: turn yesterday's working hardware bring-up into a repeatable non-arm
integration baseline, then decide whether the robot is ready for a first slow
N10P mapping run.

## Current Gate

The K1 was not reachable over SSH at `192.168.43.40` at the start of the day.
Do not start motion or mapping until the board is reachable again and the
following safety checks pass.

## 1. Reconnect And Deploy

When the K1 is online:

```bash
cd ~/edge-ai-robot-k1
git pull
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Expected repo version:

```text
10a1509 Cap automatic lamp brightness at five percent
```

## 2. Power And Light Safety

Run before connecting all sensors together:

```bash
cd ~/edge-ai-robot-k1
tools/collect_system_info.sh
sudo scripts/install_gpio37_boot_low_service.sh
systemctl status k1-gpio37-light-off.service --no-pager
cat /sys/class/gpio/gpio37/value
```

Manual checks:

- 3S battery cells are balanced and no cell is abnormally low.
- Light is off after Linux finishes booting.
- If the light turns on before Linux userspace starts, add a physical pulldown
  resistor from the lamp driver PWM/control input to GND.
- Keep automatic light control capped at `0-5%` until a diffuser or new lamp
  angle prevents D435 RGB overexposure.

## 3. Device Port Check

```bash
ls -l /dev/base_controller /dev/wheeltec_lidar 2>/dev/null || true
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH343USB* 2>/dev/null || true
lsusb
```

Pass criteria:

- C30D is visible as `/dev/base_controller` or a known tty.
- N10P lidar tty is visible.
- D435 appears in `lsusb` when connected.

## 4. D435 Depth Retest

Use a static scene first: boxes, chair legs, table edge, and a wall or board.
Keep people out of frame.

Run RGB/depth without chassis motion:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=true \
  camera_enable_depth:=true \
  enable_pointcloud:=false \
  use_light:=true \
  light_dry_run:=true
```

Pass criteria:

- RGB topic is stable.
- Depth topic is stable.
- RGB and depth are usable on static objects.
- No D435 disconnects when the light controller is present but dry-run.

Only test D435 point cloud separately after RGB/depth are stable. The K1 mapping
path should use N10P lidar rather than D435 point cloud.

Then test adaptive lighting at real low brightness only:

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

Pass criteria:

- Normal light keeps `/light/brightness_cmd` at `0`.
- Low light ramps only to `5`.
- Light returns to `0` after normal light returns or image timeout.
- D435 RGB is not saturated by the lamp.

## 5. N10P Lidar Static Check

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=true \
  use_camera:=false \
  use_light:=true
```

Pass criteria:

- Scan topic appears.
- Topic rate is stable enough for mapping.
- No lidar dropouts when D435 is not connected.

## 6. Chassis Safety Check

Only after the robot is suspended or physically guarded:

```bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=true \
  use_lidar:=false \
  use_camera:=false \
  use_light:=true
```

Pass criteria:

- No motion occurs without `/cmd_vel`.
- `/odom` publishes if the C30D link is healthy.
- Manual forward/back/left/right still match yesterday's safe profile.

## 7. First Mapping Candidate

Run only if sections 2-6 pass:

```bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping.launch.py
```

Conditions:

- Clear floor.
- One person physically guards the robot.
- Start with a small area and low speed.
- Stop immediately if odometry, lidar rate, or braking behavior looks wrong.
- Do not run automatic exploration yet.

## Not Today Unless Ahead Of Schedule

- Arm automation.
- Auto exploration / RRT.
- High-power lighting above `5%`.
- Local LLM deployment on K1.
