# Simulated Risk Approach Runbook

Date: 2026-07-11

This runbook adds a simulation-only risk loop on top of the existing Gazebo,
SLAM, RRT/Nav2 mapping setup.

## Scope

Implemented loop:

```text
Gazebo risk cards
-> simulated D435/YOLO marker detector
-> /risk/sim_detections
-> risk approach goal at 0.65 m stand-off
-> Nav2 NavigateToPose or /goal_pose
-> arrival evidence JSONL + camera PPM snapshot
```

Safety boundary:

- `risk_approach_goal_node.py` does not publish `Twist`.
- Chassis motion remains owned by Nav2 and the existing guarded command path.
- The Gazebo risk cards are visual-only and do not add collision geometry.

## Start The Base Simulation

Use the risk world:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracked_robot_description sim_mapping_safety_guard.launch.py \
  world:=$(ros2 pkg prefix tracked_robot_description)/share/tracked_robot_description/worlds/slam_rooms_risk_test.sdf \
  spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0
```

Start the existing RRT/Nav2 exploration stack in the same way as the last
validated mapping run. The risk layer only needs:

- `/map`
- `/odom`
- `map -> base_footprint` TF
- `/scan`
- `/camera/color/image_raw`
- Nav2 `navigate_to_pose` action or `/goal_pose` subscriber

## Start The Risk Layer

```bash
ros2 launch tracked_robot_description sim_risk_approach.launch.py \
  output_dir:=outputs/sim_risk_approach/demo_001
```

For a recording dry run where all risk points are immediately visible:

```bash
ros2 launch tracked_robot_description sim_risk_approach.launch.py \
  publish_all_as_detected:=true \
  output_dir:=outputs/sim_risk_approach/demo_all_visible_001
```

## Topics

- `/risk/sim_markers`: RViz marker array for risk cards and labels.
- `/risk/sim_detections`: JSON detection list.
- `/risk/current_event`: first active risk event, JSON.
- `/risk/approach_goal_marker`: RViz arrow for the selected observation pose.
- `/risk/approach_status`: JSON state for idle, approaching, and recorded states.
- `/goal_pose`: PoseStamped goal, published for Nav2/RViz compatibility.

## Evidence Output

The approach node writes:

- `risk_approach_records.jsonl`
- `latest_risk_approach.json`
- `<timestamp>_<risk_id>_camera.ppm`

Each record includes:

- risk class and map position
- observation goal pose
- final robot pose
- final distance to risk point
- yaw error toward the risk point
- camera snapshot path
- `published_cmd_vel=false`

## Demo Definition Of Success

For each simulated risk point:

1. The risk card is visible in Gazebo.
2. RViz shows the risk marker and approach goal arrow.
3. Nav2 drives the chassis to roughly `0.5-0.8 m` from the risk point.
4. The robot faces the risk point after arrival.
5. A JSONL record and camera snapshot are written.

If Nav2 is not active, the risk layer still publishes `/goal_pose` and markers,
but the robot will not move.
