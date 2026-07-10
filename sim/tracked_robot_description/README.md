# tracked_robot_description

ROS 2 / Gazebo description package for the tracked robot.

Model contents:

- tracked chassis, simulated as skid steering through left/right drive joints
- lidar link and Gazebo GPU lidar sensor
- D435-style RGB and depth camera sensors
- no mechanical arm in the default vehicle model; arm simulation is handled
  separately after SW2URDF export validation
- primitive visuals/collisions for early simulation

The current dimensions are recorded in `MODEL_MEASUREMENTS.md`. Arm references
remain in `docs/arm_reference.png` for the separate arm workflow. The default
vehicle model only contains the mobile base plus sensors. The full vehicle
reference image is saved as `docs/vehicle_reference.png`. See
`docs/SIMULATION_SCOPE.md` for what should be simulated versus kept as visual
detail.

Mechanical-arm import and MoveIt notes are tracked in
`docs/ARM_SEPARATE_SIM_SW2URDF_WORKFLOW.md`.

Additional front, side, top, and raised-arm reference photos are stored in
`docs/reference_views/`.

Ubuntu deployment and troubleshooting runbook:
`docs/UBUNTU_SIM_DEPLOYMENT_RUNBOOK.md`.

## Build From This Repository On Ubuntu

```bash
cd /path/to/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
rosdep install --from-paths sim ros2_ws/src -y --ignore-src
colcon build --base-paths sim ros2_ws/src --symlink-install
source install/setup.bash
```

This builds the simulation package plus `k1_sensor_event_adapter`, which is
needed by `sim_mapping_safety_guard.launch.py`. For a base Gazebo-only check,
you can still launch `gazebo.launch.py` after building only `sim`, but the full
P0.5 validation path should use both source trees.

If you want a separate ROS workspace for only the base Gazebo model:

```bash
mkdir -p ~/ros2_ws/src
cp -r /path/to/edge-ai-robot-k1/sim/tracked_robot_description ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

## RViz check

```bash
ros2 launch tracked_robot_description display.launch.py
```

## Gazebo check

```bash
ros2 launch tracked_robot_description gazebo.launch.py
```

Drive test (use guarded topic):

```bash
ros2 topic pub /cmd_vel_guarded geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
```

Odom check:

```bash
ros2 topic info /odom -v
ros2 topic hz /odom
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

TF source note: `/tf` is not bridged from Gazebo in the first validation pass.
`odom_tf_broadcaster` reads ROS `/odom` and publishes the single
`odom -> base_footprint` transform using `/odom.header.stamp`.

Sensor topic check:

```bash
ros2 topic list | grep -E "scan|camera|clock|odom"
```

## CAD mesh replacement

Keep collision geometry simple. Put exported visual meshes under `meshes/`,
then replace the `<box>` or `<cylinder>` visual geometry in the Xacro files
with `<mesh filename="package://tracked_robot_description/meshes/name.dae"/>`.
