# Nav2 SLAM Guarded Test - 2026-06-26

## Goal

Move from manual SLAM mapping toward supervised autonomous navigation:

```text
N10P /scan + C30D /odom + slam_toolbox map->odom
  -> Nav2 planner/controller
  -> scan obstacle layers
  -> velocity smoother
  -> /cmd_vel_raw
  -> scan_safety_guard_node
  -> /cmd_vel_guarded
  -> C30D base driver
```

This is not RRT exploration and not unattended autonomy. It is one guarded
Nav2 goal at a time.

## Preconditions

Run these first:

```bash
cd ~/edge-ai-robot-k1
tools/mapping_preflight_check.sh

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash
tools/nav2_preflight_check.sh
python3 tools/send_safe_zero_cmd.py --duration 3
```

Before Nav2:

- Battery is healthy.
- A person is following the robot.
- `linear.x=0.30` continuous calibration starts by itself.
- Stop after zero command is acceptable.
- `/scan`, `/odom`, and TF are stable.
- No mechanical arm is powered or commanded.

## Bring Up SLAM + Nav2

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash

ros2 launch turn_on_wheeltec_robot n10p_tank_nav2_slam.launch.py
```

This starts:

- N10P lidar driver.
- Python C30D Tank base driver.
- `odom -> base_footprint` TF.
- `base_footprint -> laser` static TF.
- `slam_toolbox`.
- `scan_safety_guard_node`.
- Nav2 planner/controller/behavior/waypoint/velocity smoother stack.

It does not send a goal by itself.
In the ROS Humble `nav2_bringup` launch, the controller output is remapped to
`cmd_vel_nav`, then `velocity_smoother` publishes the smoothed output as
`/cmd_vel_raw`. The scan safety guard subscribes `/cmd_vel_raw`, filters it
with the N10P front sector, and publishes `/cmd_vel_guarded`. The C30D base
driver subscribes to `/cmd_vel_guarded` in this mode.

## Health Checks

```bash
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 topic echo /map --once
ros2 run tf2_tools view_frames
ros2 action list | grep navigate_to_pose
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /safety/front_obstacle --once
ros2 topic echo /cmd_vel_raw --once
ros2 topic echo /cmd_vel_guarded --once
```

Expected:

- `/scan` remains near the N10P rate.
- `/odom` updates while the chassis moves.
- TF connects `map -> odom -> base_footprint -> laser`.
- Local/global costmaps use `/scan` obstacles.
- No motion command is published until a goal is sent.
- Nav2 does not publish directly to the base driver.
- The base driver consumes `/cmd_vel_guarded`, not `/cmd_vel_raw`.

## Send One Guarded Goal

Use a small goal only. The tool cancels on timeout and publishes forced zero:

```bash
python3 tools/nav2_send_guarded_goal.py --x 0.25 --y 0.0 --yaw 0.0 --result-timeout 12
```

The tool requires typing `YES` and refuses goals beyond `0.5 m`.
The configured NavigateToPose BT omits automatic spin/backup recovery actions;
the behavior action servers remain loaded only so Nav2 lifecycle activation
does not fail while it initializes the unused NavigateThroughPoses navigator.

## Acceptance Criteria

- Nav2 action server accepts one small goal.
- Robot starts without pushing.
- Robot avoids obvious N10P-visible obstacles in the costmap.
- Robot stops after reaching, canceling, or failing the goal.
- `slam_toolbox` map remains coherent and does not tear badly.
- Saved map can be generated after the run:

```bash
tools/save_slam_map.sh
```

## Stop Commands

```bash
python3 tools/send_safe_zero_cmd.py --topic /cmd_vel_guarded --duration 3
ros2 topic pub --once /cmd_vel_guarded geometry_msgs/msg/Twist "{}"
```

Use power cut or physically lift the chassis if software stop is not enough.
If ROS has already been stopped and the serial port is free, a direct C30D
zero-frame fallback is available:

```bash
python3 tools/send_c30d_serial_zero.py --duration 5
```

## Do Not Run Yet

- Do not start RRT exploration.
- Do not run waypoint loops.
- Do not run without a person following the robot.
- Do not combine this with arm/servo actions.
- Do not raise speed above the guarded Nav2 config until PID stop behavior is
  verified on the floor.
