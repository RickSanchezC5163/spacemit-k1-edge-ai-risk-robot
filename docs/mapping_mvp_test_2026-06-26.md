# Mapping MVP Test - 2026-06-26

## Goal

Validate a small-area mapping MVP using:

- N10P lidar `/scan`
- Tank chassis odometry `/odom`
- `slam_toolbox`

This test is only for proving the minimum chain:

```text
N10P scan + C30D odom + TF -> slam_toolbox -> save map
```

It is not an automatic exploration run.

## Preflight Safety

Run the preflight script first:

```bash
cd ~/edge-ai-robot-k1
tools/mapping_preflight_check.sh
```

Manual checks before any motion:

- Battery cells are balanced and the pack is safe for load.
- Tank tracks are lifted off the ground for the first motion check.
- No residual ROS, RealSense, SLAM, lidar, or base driver processes are running.
- Light is off unless explicitly needed.
- `/cmd_vel` is zero before and after the test.
- One person is physically watching the robot during all ground motion.

Force zero speed before starting:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash  # if lslidar_driver is installed separately
python3 tools/send_safe_zero_cmd.py
```

## Start Mapping Stack

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash  # if lslidar_driver is installed separately
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping.launch.py
```

This launch should start:

- N10P lidar driver
- Official Wheeltec C++ Tank base driver
- `/odom` to `base_footprint` TF relay
- static `base_footprint -> laser` transform
- `slam_toolbox`

## Check `/scan`

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

Pass criteria:

- `/scan` publishes continuously.
- Rate is stable enough for mapping.
- Range values are not all zero, all inf, or obviously frozen.

## Check `/odom`

```bash
ros2 topic echo /odom --once
```

Pass criteria:

- `/odom` publishes.
- Frame IDs are consistent with the mapping launch.
- Position and velocity do not jump while the robot is stationary.

## Check TF

```bash
ros2 run tf2_tools view_frames
```

Pass criteria:

- `base_footprint`, `laser`, and odom/map-related frames are connected.
- No missing transform breaks the scan-to-base chain.

## Guarded Continuous Motion

Only after `/scan`, `/odom`, and TF are healthy:

```bash
python3 tools/continuous_drive_calibration.py --linear 0.30 --duration 3.0 --ramp-time 0.4
python3 tools/send_safe_zero_cmd.py
```

Recommended guarded mapping motion:

- `linear.x`: start at `0.30 m/s`; use up to `0.45 m/s` only if watched closely.
- `angular.z`: test separately, starting around `0.35-0.45 rad/s`.
- `duration`: start with `2-3 s` continuous motion, followed by `>=3 s` zero speed.

Use one continuous segment at a time while a person follows the robot. Do not
run a route loop until PID and stop behavior are stable.

2026-06-26 ground test note:

- The current Tank chassis does not reliably overcome floor friction at `0.05-0.10 m/s`.
- The official C++ `wheeltec_robot_node` moved the chassis; the Python safe base node did not actuate this firmware reliably.
- If floor motion is required, use guarded continuous tests around `0.30 m/s`,
  then tune C30D speed-loop PID until start, cruise, and stop are stable.
- Do not use these higher pulses for unattended navigation or automatic exploration.
- Detailed tuning workflow: `docs/c30d_pid_continuous_mapping_tuning_2026-06-26.md`.
- Local reference findings: `docs/reference_findings_continuous_mapping_2026-06-26.md`.
- Next supervised Nav2 step: `docs/nav2_slam_guarded_test_2026-06-26.md`.

## Save Map

```bash
tools/save_slam_map.sh
```

Equivalent direct command:

```bash
mkdir -p ~/edge-ai-robot-k1/maps
ros2 run nav2_map_server map_saver_cli -f ~/edge-ai-robot-k1/maps/test_map_20260626
```

The helper script uses a timestamped default:

```text
~/edge-ai-robot-k1/maps/test_map_YYYYMMDD_HHMMSS
```

## Acceptance Criteria

- `/scan` is stable.
- `/odom` is stable.
- TF chain is connected.
- `slam_toolbox` does not crash or explode the map.
- A map can be saved to `~/edge-ai-robot-k1/maps`.
- The robot can be stopped with `tools/send_safe_zero_cmd.py`.
- Tested map artifact: `~/edge-ai-robot-k1/maps/mapping_mvp_20260626_cpp_odom_tf.{pgm,yaml}`.

## Forbidden During This Test

- No high-speed motion.
- No mechanical arm startup or servo movement.
- No automatic inspection.
- No Nav2 autonomous navigation.
- No RRT or automatic exploration.
- No unattended ground run.
