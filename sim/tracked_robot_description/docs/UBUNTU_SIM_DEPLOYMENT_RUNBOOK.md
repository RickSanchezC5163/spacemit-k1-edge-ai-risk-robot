# Ubuntu Simulation Deployment Runbook

This runbook is for pulling this repository on Ubuntu and validating the current
P0.5 simulation stack before adding Nav2, Frontier exploration, or RL.

Current target:

```text
Gazebo DiffDrive -> /odom bridge -> odom_tf_broadcaster
teleop/RL/Nav2 candidate -> /input_cmd_vel -> scan_safety_guard -> /cmd_vel_guarded
```

`/tf` is intentionally not bridged from Gazebo in this pass. The single ROS TF
source for `odom -> base_footprint` is `odom_tf_broadcaster`.

## Environment Assumptions

Recommended Ubuntu stack:

```text
Ubuntu 22.04 or the current lab Ubuntu image
ROS 2 Humble
Gazebo / ros_gz stack available for Humble
```

Open a new terminal and source ROS before every build or run:

```bash
source /opt/ros/humble/setup.bash
```

This project currently treats Humble as the simulation and K1-aligned ROS
distro. If you use a nonstandard Ubuntu image, first confirm that
`/opt/ros/humble/setup.bash` exists.

## Clone Or Update

```bash
cd ~
git clone https://github.com/RickSanchezC5163/edge-ai-robot-k1.git
cd edge-ai-robot-k1
```

If the repository already exists:

```bash
cd ~/edge-ai-robot-k1
git pull --ff-only
```

## Install Dependencies

Install the common tools first:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-ros-gz \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools
```

Initialize rosdep if this Ubuntu install has not done it before:

```bash
sudo rosdep init
rosdep update
```

Install package dependencies from both simulation and local ROS source packages:

```bash
cd ~/edge-ai-robot-k1
rosdep install --from-paths sim ros2_ws/src -y --ignore-src
```

Building with `ros2_ws/src` matters because `sim_mapping_safety_guard.launch.py`
uses `k1_sensor_event_adapter`.

## Build

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash

colcon build \
  --base-paths sim ros2_ws/src \
  --symlink-install

source install/setup.bash
```

Basic package checks:

```bash
ros2 pkg prefix tracked_robot_description
ros2 pkg executables tracked_robot_description
```

Expected executable:

```text
tracked_robot_description odom_tf_broadcaster.py
```

Xacro expansion check:

```bash
xacro sim/tracked_robot_description/urdf/tracked_robot.urdf.xacro \
  > /tmp/tracked_robot.urdf
```

## A. Base Gazebo Validation

Start the base simulation first, without SLAM or safety guard:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tracked_robot_description gazebo.launch.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
cd ~/edge-ai-robot-k1
source install/setup.bash

ros2 topic list | grep -E "clock|odom|tf|scan|cmd_vel|camera"
```

Expected topics include:

```text
/clock
/odom
/tf
/scan
/cmd_vel_guarded
/camera/color/image_raw
/camera/depth/image_raw
```

Check `/odom`:

```bash
ros2 topic info /odom -v
ros2 topic hz /odom
ros2 topic echo /odom --once
```

Expected frame IDs:

```text
header.frame_id: odom
child_frame_id: base_footprint
```

Check TF:

```bash
ros2 topic info /tf -v
ros2 run tf2_ros tf2_echo odom base_footprint
```

Acceptance:

```text
[ ] /odom publishes continuously
[ ] /odom frame_id is odom
[ ] /odom child_frame_id is base_footprint
[ ] /tf has odom_tf_broadcaster as the only odom->base_footprint source
[ ] tf2_echo odom base_footprint updates without extrapolation errors
```

## B. Base Drive Test

With `gazebo.launch.py` still running:

```bash
ros2 topic pub \
  /cmd_vel_guarded \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}" \
  -r 10
```

Acceptance:

```text
[ ] robot moves forward in Gazebo
[ ] /odom pose.position.x increases
[ ] yaw remains mostly stable
[ ] stopping the publisher stops the robot
```

Rotation check:

```bash
ros2 topic pub \
  /cmd_vel_guarded \
  geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.4}}" \
  -r 10
```

Acceptance:

```text
[ ] robot rotates in place
[ ] odom yaw changes
[ ] x/y drift is small
```

## C. Safety Mapping Launch

After the base Gazebo checks pass, stop the previous launch and run:

```bash
ros2 launch tracked_robot_description sim_mapping_safety_guard.launch.py
```

If `k1_sensor_event_adapter` is not built yet and you only want to validate
Gazebo, odom, and SLAM startup:

```bash
ros2 launch tracked_robot_description sim_mapping_safety_guard.launch.py \
  enable_safety_guard:=false
```

Teleop should publish to `/input_cmd_vel`, not directly to the Gazebo drive
topic:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/input_cmd_vel
```

Check the guarded chain:

```bash
ros2 topic hz /input_cmd_vel
ros2 topic hz /cmd_vel_guarded
ros2 topic hz /scan
ros2 topic hz /map
```

Check the TF chain:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

`map -> odom` appears only after `slam_toolbox` is running and receiving scans.

## D. Completion Gate Before Nav2

Do not start Nav2, Frontier, or RL until all of this is true:

```text
[ ] colcon build succeeds
[ ] /odom publishes continuously
[ ] odom frame_id and child_frame_id are correct
[ ] odom -> base_footprint has one ROS TF source
[ ] TF uses simulation time and has no extrapolation errors
[ ] /scan publishes continuously
[ ] /cmd_vel_guarded drives Gazebo in the base launch
[ ] teleop remapped to /input_cmd_vel reaches /cmd_vel_guarded
[ ] safety_guard can pass, slow, and hard-stop commands
[ ] slam_toolbox publishes /map
[ ] TF chain map -> odom -> base_footprint is complete
```

## Common Errors

### `colcon: command not found`

Install colcon:

```bash
sudo apt install -y python3-colcon-common-extensions
```

### `ros2: command not found`

ROS is not sourced, or ROS is not installed:

```bash
source /opt/ros/humble/setup.bash
```

If that file does not exist, install ROS 2 Humble first.

### `Cannot locate rosdep definition for [k1_sensor_event_adapter]`

You probably ran rosdep only on `sim`. Include the local ROS packages:

```bash
rosdep install --from-paths sim ros2_ws/src -y --ignore-src
```

For a temporary base Gazebo-only run, launch with:

```bash
ros2 launch tracked_robot_description sim_mapping_safety_guard.launch.py \
  enable_safety_guard:=false
```

### `Package 'k1_sensor_event_adapter' not found`

Build and source both source trees:

```bash
colcon build --base-paths sim ros2_ws/src --symlink-install
source install/setup.bash
```

### `Package 'tracked_robot_description' not found`

The workspace was not built or sourced:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
colcon build --base-paths sim ros2_ws/src --symlink-install
source install/setup.bash
ros2 pkg prefix tracked_robot_description
```

### `No executable found: odom_tf_broadcaster.py`

Rebuild and source the workspace:

```bash
colcon build --base-paths sim ros2_ws/src --symlink-install
source install/setup.bash
ros2 pkg executables tracked_robot_description
```

Expected:

```text
tracked_robot_description odom_tf_broadcaster.py
```

### `xacro: command not found`

Install xacro:

```bash
sudo apt install -y ros-humble-xacro
```

### Gazebo does not start, or `gz sim` is missing

Install the ROS/Gazebo integration:

```bash
sudo apt install -y ros-humble-ros-gz
```

Then re-source ROS:

```bash
source /opt/ros/humble/setup.bash
```

### `/odom` does not appear

Check these in order:

```bash
ros2 topic list | grep odom
ros2 topic list | grep cmd_vel_guarded
ros2 node list
```

Likely causes:

```text
bridge config not installed because workspace was not rebuilt
Gazebo robot did not spawn
Gazebo DiffDrive plugin did not load
wrong terminal was not sourced with install/setup.bash
```

Fix:

```bash
colcon build --base-paths sim ros2_ws/src --symlink-install
source install/setup.bash
ros2 launch tracked_robot_description gazebo.launch.py
```

### `tf2_echo odom base_footprint` reports extrapolation

Likely causes:

```text
/clock is missing
node did not use simulation time
odom_tf_broadcaster used wall time instead of /odom.header.stamp
/tf bridge was re-enabled and duplicates the transform
```

Checks:

```bash
ros2 topic list | grep clock
ros2 topic info /tf -v
```

The current expected setup is:

```text
/tf bridge disabled
odom_tf_broadcaster use_current_time=false
odom_tf_broadcaster use_sim_time=true
```

### Duplicate `odom -> base_footprint` TF

Run:

```bash
ros2 topic info /tf -v
```

If both `ros_gz_bridge` and `odom_tf_broadcaster` publish the same transform,
disable the `/tf` bridge in `config/ros_gz_bridge.yaml`. The first validation
pass should use only:

```text
/odom -> odom_tf_broadcaster -> /tf
```

### Robot does not move when publishing `/cmd_vel`

The Gazebo DiffDrive plugin now listens to `/cmd_vel_guarded`, not `/cmd_vel`.
Use:

```bash
ros2 topic pub /cmd_vel_guarded geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}" -r 10
```

For the safety launch, publish to `/input_cmd_vel` and let the guard forward to
`/cmd_vel_guarded`.

### `/odom` changes but the robot does not visibly move in Gazebo

First compare ROS odom with Gazebo model pose:

```bash
ros2 topic echo /odom --once
ign topic -e -t /world/tracked_robot_world/dynamic_pose/info -n 1
```

If `/odom.pose.pose.position.x` changes but the Gazebo model `tracked_robot`
pose stays near zero, the DiffDrive odometry is integrating wheel motion but the
physical model is not being driven. The current URDF keeps the visible track
boxes as visuals only and uses `left_virtual_drive_wheel_link` and
`right_virtual_drive_wheel_link` as the ground-contact collision bodies. Rebuild
after any URDF change:

```bash
colcon build --base-paths sim ros2_ws/src --symlink-install
source install/setup.bash
ros2 launch tracked_robot_description gazebo.launch.py
```

### Safety guard outputs zero speed

Check scan health and guard state:

```bash
ros2 topic hz /scan
ros2 topic echo /safety/front_obstacle --once
ros2 topic echo /cmd_vel_guarded --once
```

Common causes:

```text
/scan missing or stale
front obstacle inside hard_stop_m
teleop is publishing to /cmd_vel instead of /input_cmd_vel
guard latch is active after a hard stop
```

### `/map` does not appear

`/map` requires `slam_toolbox` plus `/scan`, `/odom`, and TF:

```bash
ros2 node list | grep slam
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

Move the robot slowly after startup. A static robot may not produce useful map
updates immediately.

### `map -> odom` is missing

This is expected before `slam_toolbox` starts or before it receives valid scan
and odom data. Validate in this order:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 node list | grep slam
ros2 run tf2_ros tf2_echo map odom
```

### Camera or scan topics are missing

Check that the robot spawned and the bridge is running:

```bash
ros2 node list
ros2 topic list | grep -E "scan|camera"
```

If the topics remain missing, rebuild and restart Gazebo. Do not continue to
SLAM or safety validation until `/scan` is stable.

## Suggested First Ubuntu Run Log

Record these outputs into a deployment note:

```bash
ros2 pkg prefix tracked_robot_description
ros2 pkg executables tracked_robot_description
ros2 topic list | sort
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 topic info /tf -v
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic hz /scan
ros2 topic hz /map
```

Save screenshots only after the terminal evidence confirms the chain is stable.
