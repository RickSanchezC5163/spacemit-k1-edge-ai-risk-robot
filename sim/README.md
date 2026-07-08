# Simulation

Gazebo / ROS 2 simulation assets live here.

Current package:

- `tracked_robot_description`: URDF/Xacro model for the tracked robot with N10P lidar, D435-style camera, and 5-ID bus-servo arm.

## Build

On Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic:

```bash
cd /path/to/edge-ai-robot-k1
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths sim -y --ignore-src
colcon build --base-paths sim --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch tracked_robot_description display.launch.py
ros2 launch tracked_robot_description gazebo.launch.py
```
