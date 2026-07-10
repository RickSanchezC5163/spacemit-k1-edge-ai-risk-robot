# Simulation

Gazebo / ROS 2 simulation assets live here.

Current package:

- `tracked_robot_description`: URDF/Xacro model for the tracked mobile base with N10P lidar and D435-style camera. The mechanical arm is simulated separately after SW2URDF export validation.

## Build

On Ubuntu with ROS 2 Humble and the matching Gazebo / ros_gz stack:

```bash
cd /path/to/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
rosdep install --from-paths sim -y --ignore-src
colcon build --base-paths sim --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch tracked_robot_description display.launch.py
ros2 launch tracked_robot_description gazebo.launch.py
```
