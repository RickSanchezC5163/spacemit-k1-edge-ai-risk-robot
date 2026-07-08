# Guarded Manual Mapping Scan Safety Guard - 2026-06-27

## Current Capability

Current capability:

- guarded manual mapping only
- N10P `/scan`
- Python Tank base `/odom`
- `odom -> base_footprint`
- `base_footprint -> laser`
- `slam_toolbox /map`
- static obstacle hard-stop check has passed

Not enabled yet:

- automatic obstacle avoidance
- Nav2 autonomous navigation
- RRT automatic exploration
- unattended patrol
- dynamic approach is under diagnosis; do not use it as a collision-proof layer

## Goal

Add a low-risk front safety layer for supervised manual mapping:

```text
/input_cmd_vel
+ /scan front sector
-> scan_safety_guard_node
-> /cmd_vel_guarded
-> Python Tank base
```

The guard is intended for guarded manual mapping. It does not plan paths and does
not generate autonomous movement. Nav2, RRT, automatic exploration, and autonomous
goals remain paused until dynamic approach stopping is validated.

## Safety Behavior

Default rules:

- `front_p10 <= 1.00 m`: hard stop, publish zero speed.
- `front_min <= 0.45 m`: emergency hard stop, publish zero speed.
- `1.00 m < front_p10 < 1.60 m`: warning, but keep the tested crawl speed cap.
- `front_p10 >= 1.60 m`: clear, but forward speed is still capped.
- `front_p10 < 1.60 m` while approaching faster than `0.35 m/s`: hard stop.
- `time_to_collision < 1.20 s`: hard stop.
- hard-stop latch keeps the vehicle stopped for at least `1.50 s`.
- `0.00 m/s < requested forward speed < 0.28 m/s`: publish zero speed, because the
  tank chassis does not reliably overcome static friction below that range.
- stale `/scan`: fail closed and publish zero speed.

Default forward speed caps:

- clear zone: `0.30 m/s`
- slow zone: capped at `0.30 m/s`
- hard-stop zone: `0.00 m/s`

Topics:

- subscribes `/scan`
- subscribes `/input_cmd_vel`
- publishes `/cmd_vel_guarded`
- publishes `/safety/front_obstacle`
- publishes `/perception/mock_event`

Risk events:

- `soft_obstacle` when in warning zone
- `blocked_path` when in hard-stop zone

Diagnostics:

- `state`
- `front_min`
- `front_p10`
- `valid_count`
- `approach_rate`
- `time_to_collision`
- `hard_stop_latch_remaining`

## Launch

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash

ros2 launch turn_on_wheeltec_robot n10p_tank_mapping_safety_guard.launch.py \
  serial_port:=/dev/base_controller \
  max_linear_x:=0.45 \
  max_angular_z:=0.80 \
  brake_duration_sec:=1.0
```

This launch remaps the Python Tank base to consume `/cmd_vel_guarded`.
Operators should publish manual commands to `/input_cmd_vel`, not `/cmd_vel`.

## Checks

```bash
ros2 topic hz /scan
ros2 topic echo /safety/front_obstacle
ros2 topic echo /perception/mock_event
ros2 topic echo /cmd_vel_guarded
python3 tools/scan_safety_guard_static_check.py
```

## Manual Movement Test

Only run while the vehicle is suspended or guarded on the ground.

Clear front sector:

```bash
ros2 topic pub -r 20 /input_cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.30, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Stop:

```bash
ros2 topic pub --once /input_cmd_vel geometry_msgs/msg/Twist "{}"
python3 tools/send_safe_zero_cmd.py --topic /cmd_vel_guarded --duration 3
```

Obstacle checks:

- place an obstacle around `2.0 m` in front: state should be `clear`.
- place an obstacle around `1.5 m` in front: state should be `warning`.
- place an obstacle around `1.0 m` in front: state should be `hard_stop`.
- remove the obstacle after hard stop: state should remain latched for about `1.5 s`.
- place an obstacle around `0.5 m` in front: `/cmd_vel_guarded` should become zero.
- remove the obstacle: commands should pass through again.

## Acceptance Criteria

- `/scan` remains stable around 12 Hz.
- `/odom` remains stable around 20 Hz.
- `/map` continues updating.
- front obstacle status changes between `clear`, `warning`, and `hard_stop`.
- `blocked_path` and `soft_obstacle` events are logged by the risk/event stack.
- no automatic route, Nav2 goal, or RRT exploration is started.

## Forbidden During This Test

- do not publish directly to `/cmd_vel` while using this guard launch
- do not start Nav2 autonomous goals
- do not start RRT exploration
- do not run automatic exploration
- do not run the arm
- do not leave the robot unattended
