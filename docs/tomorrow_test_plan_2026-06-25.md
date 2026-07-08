# Tomorrow Test Plan - 2026-06-25

Goal: validate the non-arm K1 bring-up package without triggering arm movement
or automatic chassis motion.

## 1. Power Check

Steps:

```bash
cd ~/edge-ai-robot-k1
tools/collect_system_info.sh
```

Manual checks:

- battery pack voltage
- 3S cell balance
- K1 power stability
- light power path

Pass criteria:

- no cell is abnormally low
- K1 remains online after USB devices are connected
- light remains off by default

Install boot-time light-off guard before the rest of testing:

```bash
cd ~/edge-ai-robot-k1
sudo scripts/install_gpio37_boot_low_service.sh
```

Pass criteria:

- `systemctl status k1-gpio37-light-off.service` shows the service is enabled.
- GPIO37 is low after boot.
- `/sys/class/gpio/gpio37/direction` and `value` are writable by the normal
  `soc` user after the service runs.
- If the lamp still flashes before Linux starts, add a physical pulldown
  resistor from PWM/control input to GND.

## 2. Port Check

Commands:

```bash
cd ~/edge-ai-robot-k1
ls -l /dev/base_controller /dev/wheeltec_lidar 2>/dev/null || true
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH343USB* 2>/dev/null || true
lsusb
```

Pass criteria:

- C30D appears as `/dev/base_controller` or a known tty device
- N10P lidar serial device is visible
- D435 appears in `lsusb` when connected

## 3. Chassis No-motion Check

Start bring-up with base enabled only after the robot is suspended or guarded:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=true \
  use_lidar:=false \
  use_camera:=false \
  use_light:=true
```

Pass criteria:

- driver starts
- no wheel movement occurs without manual `/cmd_vel`
- `/odom` is published if the C30D link is healthy

## 4. N10P Lidar Check

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=true \
  use_camera:=false \
  use_light:=true
```

Pass criteria:

- lidar driver starts
- scan or point cloud topic appears
- topic rate is stable enough for mapping

## 5. D435 Check

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=true \
  use_light:=true
```

Pass criteria:

- RGB topic publishes
- depth topic publishes
- point cloud topic publishes if enabled
- USB cable reports stable data behavior

## 6. Light Brightness Check

For normal-user static testing, start with dry-run mode first:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py \
  use_base:=false \
  use_lidar:=false \
  use_camera:=false \
  use_light:=true \
  light_dry_run:=true
```

For real brightness control, confirm GPIO37 write permission or run through a
root-side service first. Then publish:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic pub --once /light/brightness_cmd std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /light/brightness_cmd std_msgs/msg/Int32 "{data: 5}"
ros2 topic pub --once /light/brightness_cmd std_msgs/msg/Int32 "{data: 0}"
```

Pass criteria:

- 0 turns light off
- 5 shows visible brightness without obvious D435 overexposure
- no overheating or power drop
- final 0 leaves light off

Adaptive RGB light controller dry-run:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
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
  use_camera_low_light_adapter:=true
```

Pass criteria:

- normal light publishes `/light/brightness_cmd: 0`
- low light ramps to `5` by default
- returning to light ramps back to `0`
- `/light/adaptive_status` includes `mean_luma`, `dark_pixel_ratio`, and
  current brightness
- no image for 3 seconds forces brightness `0`

## 7. Mock Risk Engine Check

```bash
python3 tools/publish_mock_event.py --type soft_obstacle --distance 0.8 --confidence 0.9
ros2 topic echo --once /risk/current_level
ros2 topic echo --once /risk/recommended_action
```

Pass criteria:

- `soft_obstacle`, `0.8 m` -> `medium`
- recommended action -> `stop_and_recheck`

Additional cases:

```bash
python3 tools/publish_mock_event.py --type hard_obstacle --distance 0.5 --confidence 0.95
python3 tools/publish_mock_event.py --type blocked_path --distance 1.5 --confidence 0.8
python3 tools/publish_mock_event.py --type low_light --distance 0.0 --confidence 0.7
python3 tools/publish_mock_event.py --type cable_or_wire --distance 0.9 --confidence 0.85
python3 tools/publish_mock_event.py --type reflective_noise --distance 1.4 --confidence 0.6
python3 tools/publish_mock_event.py --type unknown --distance 2.0 --confidence 0.5
```

Pass criteria:

- hard obstacle under 0.8 m -> high / stop_and_report
- blocked path -> high / stop_and_report
- low light -> medium / turn_on_light_and_recheck
- cable or wire under 1.2 m -> high / stop_and_report
- reflective noise -> medium / slow_down_and_recheck
- unknown -> low / continue_with_caution

## 8. Event Log Check

```bash
find logs/events -type f -name 'events_*.jsonl' -print
tail -n 20 logs/events/events_*.jsonl
```

Pass criteria:

- log directory is auto-created
- one JSON object per line
- fields include timestamp, event_type, risk_level, recommended_action,
  distance_m, confidence, and source

## 9. Full Non-arm Static Smoke Test

```bash
tools/non_arm_system_smoke_test.sh
```

Pass criteria:

- script completes without publishing `/cmd_vel`
- script does not start arm or servo actions
- script prints PASS/WARN/FAIL for environment and device checks
- script prints manual bring-up and mock-event commands

Optional static stack launch from smoke test:

```bash
RUN_BRINGUP=1 tools/non_arm_system_smoke_test.sh
```

This still uses `use_base:=false`, `use_lidar:=false`, and `use_camera:=false`.
