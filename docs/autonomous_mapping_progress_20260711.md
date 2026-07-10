# Autonomous Mapping Progress - 2026-07-11

## Status

The Ubuntu simulation stack now has a working autonomous mapping baseline for
recording and further K1-aligned development.

Validated runtime:

- ROS 2 Humble on the local Ubuntu dual-boot machine.
- Gazebo simulation for the tracked base with N10P-style lidar and D435-style
  camera.
- Mechanical arm removed from the default vehicle simulation. The arm remains a
  separate MoveIt/RL workflow until the mechanical export is ready.
- `slam_toolbox` publishes the live occupancy grid.
- RViz displays robot model, laser scan, map, and `/trajectory`.
- Nav2 accepts RRT frontier goals and drives through the safety guard.
- RRT frontier selection is running on the ROS 2 port in
  `~/rrt_exploration_ws`.

## Current Working Launch Pattern

Build:

```bash
cd ~/Documents/GitHub/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
colcon build --base-paths sim ros2_ws/src --symlink-install
source install/setup.bash
```

Start the open-room mapping scene:

```bash
ros2 launch tracked_robot_description sim_mapping_safety_guard.launch.py \
  world:=/home/ubuntu/Documents/GitHub/edge-ai-robot-k1/sim/tracked_robot_description/worlds/slam_rooms_test.sdf \
  spawn_x:=2.2 \
  spawn_y:=0.25 \
  spawn_z:=0.08 \
  spawn_yaw:=3.1416
```

Keep Gazebo model pose aligned with odometry when the robot is spawned away
from the world origin:

```bash
python3 tools/gz_sync_model_pose_from_odom.py \
  --model tracked_robot \
  --offset-x 2.2 \
  --offset-y 0.25 \
  --offset-yaw 3.1416
```

Open RViz:

```bash
rviz2 -d sim/tracked_robot_description/config/sim_mapping.rviz
```

The key Nav2/RRT runtime topics are:

```text
/map
/odom
/scan
/trajectory
/input_cmd_vel
/cmd_vel_guarded
```

## Changes Made For The Recording Baseline

- Added spawn pose launch arguments to `sim_mapping_safety_guard.launch.py`.
- Added `/trajectory` publishing from `/odom` for RViz path visualization.
- Added Gazebo pose synchronization with spawn offset support.
- Added Humble RViz configs for mapping and odom debugging.
- Added three Gazebo test worlds:
  - `slam_rooms_test.sdf`: current recording map, open-room layout.
  - `slam_test_maze.sdf`: simpler structured test map.
  - `slam_hard_maze.sdf`: harder map; useful for stress testing but too
    enclosed for the current recording.
- Simplified the default vehicle simulation to chassis + lidar + D435. The arm
  is intentionally handled separately.
- Safety guard thresholds are now in three practical bands around 0.1 m,
  0.2 m, and 0.3 m.
- Nav2 turn behavior was tuned toward the visible 30-degree turn strategy used
  during manual trials.

## Current Map Choice

`slam_rooms_test.sdf` is the recommended recording map.

The latest layout avoids the previous over-enclosed three-cell structure. It
uses a 7 m x 7 m boundary, short half-walls, wide gaps, and isolated landmarks
so that:

- frontier exploration has continuous reachable free space;
- RViz map growth is visually obvious;
- Gazebo/RViz alignment is easy to inspect;
- the robot does not spend most of the run stuck in narrow cells.

## Completion Criterion For The Demo

For tonight's recording, define mapping complete when all of the following hold:

1. The map outer shape has not expanded for 30 seconds.
2. RRT goals repeatedly point to the same unreachable frontier, or the frontier
   queue is exhausted/stable.
3. The known valid cells inside the explored boundary reach at least 95%.
4. The robot is not actively discovering new reachable space.

This is a practical demo threshold, not a formal coverage proof. It is intended
to stop the run at the point where the map is visually complete and the
remaining frontiers are not useful for the recording.

## Pulled Recording Files

The following Ubuntu screen recordings were pulled back to the local repository:

```text
artifacts/recordings/录屏 2026年07月11日 00时59分50秒.webm
artifacts/recordings/录屏 2026年07月11日 01时19分55秒.webm
artifacts/recordings/录屏 2026年07月11日 01时42分10秒.webm
```

They capture the iterative debugging and the final better autonomous mapping
behavior with Gazebo/RViz visible.

## Current Caveats

- RViz can emit an OpenGL map shader warning on the Ubuntu display. The node
  still starts; if the map layer does not refresh, toggle the Map display or
  restart RViz.
- ROS graph warnings about duplicate node names may appear after hard restarts.
  Kill stale simulation, Nav2, and RRT processes before recording.
- `slam_hard_maze.sdf` is intentionally difficult and can create dead-end-heavy
  behavior. Use it after the open-room demo is recorded.
- The K1 hardware path remains separate from this Ubuntu simulation path.

## Next Work

- Turn the completion criterion into a reusable script that samples `/map`,
  computes the known-cell ratio, and checks RRT/Nav2 logs.
- Save the final map and a compact log bundle after each recording run.
- Reconnect D435 YOLO risk events to the simulated map stream.
- Add the mechanical arm once the mechanical team exports clean URDF/mesh
  assets.
