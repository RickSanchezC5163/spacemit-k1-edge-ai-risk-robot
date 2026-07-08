# Nav2 Guarded Small Goal Test - 2026-06-28

## Scope

This is a supervised Nav2 small-goal validation on an existing map.

It is not RRT, not automatic exploration, and not unattended navigation.

## Map

Default map:

```text
~/edge-ai-robot-k1/maps/mapping_fixed_odom_20260628_085623.yaml
```

This map was generated after fixing the Tank STM32 encoder line count and
restoring `odom_linear_scale=1.0`.

## Command Flow

```text
Nav2 controller / velocity_smoother
-> /cmd_vel_raw
-> scan_safety_guard_node
-> /cmd_vel_guarded
-> wheeltec_tank_base_safe
-> C30D serial
```

`wheeltec_tank_base_safe` must not subscribe to `/cmd_vel_raw`.

## Safety Defaults

- Nav2 output topic: `/cmd_vel_raw`
- Guard output topic: `/cmd_vel_guarded`
- Base input topic: `/cmd_vel_guarded`
- STOP_REQUEST topic: `/chassis/stop_request`
- `max_vel_x <= 0.20`
- `max_vel_theta <= 0.35`
- `acc_lim_x <= 0.15`
- No spin/backup recovery behavior server is launched.
- The NavigateToPose behavior tree omits recovery actions.
- No goal is sent by launch.

## Start

Stop residual Nav2/SLAM processes first if needed. Source both workspaces:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash
```

Launch:

```bash
ros2 launch turn_on_wheeltec_robot n10p_tank_nav2_guarded.launch.py
```

If N10P or the base is already running under supervision, disable launch-owned
copies explicitly:

```bash
ros2 launch turn_on_wheeltec_robot n10p_tank_nav2_guarded.launch.py \
  start_lidar:=false \
  start_base:=false
```

## Pre-goal Checks

Before sending any goal:

```bash
ros2 topic echo /cmd_vel_guarded --once
ros2 topic echo /cmd_vel_raw --once
ros2 topic echo /safety/front_obstacle --once
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /map --once --field info
ros2 action list | grep navigate_to_pose
```

Expected while idle:

```text
/cmd_vel_guarded is zero
/cmd_vel_raw is zero or absent until Nav2 computes
/scan has N10P data
/odom is stable
/map is loaded
```

## Small Goal

Only these forward distances are allowed:

```text
0.2 m
0.3 m
0.5 m
```

First test:

```bash
python3 tools/nav2_guarded_small_goal.py --distance 0.2 --confirm YES
```

Then inspect:

```bash
ros2 topic echo /cmd_vel_raw --once
ros2 topic echo /cmd_vel_guarded --once
tail -80 logs/dynamic_base_latest.log
```

## Cancel And Zero

Use this whenever the robot should stop or after a failed Nav2 result:

```bash
python3 tools/nav2_cancel_and_zero.py --duration 6
```

This cancels NavigateToPose goals, publishes zero to `/cmd_vel_raw` and
`/cmd_vel_guarded`, and sends STOP_REQUEST bursts on `/chassis/stop_request`.

## Acceptance

- Nav2 launch does not publish nonzero `/cmd_vel_guarded` before a goal.
- A `0.2 m` goal causes `/cmd_vel_raw` to show small nonzero velocity.
- `scan_safety_guard_node` forwards or blocks into `/cmd_vel_guarded`.
- The base moves only a short distance and stops.
- After completion or cancel:
  - `/cmd_vel_raw` is zero.
  - `/cmd_vel_guarded` is zero.
  - base log shows `serial=(0.000,0.000)`.
  - base log shows feedback near zero.

## Prohibited

- Do not start RRT.
- Do not start automatic exploration.
- Do not run recovery spin/backup.
- Do not send goals larger than `0.5 m`.
- Do not run without a human physically guarding the chassis.

