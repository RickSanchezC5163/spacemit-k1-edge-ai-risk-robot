# Bring-up Commands

## Build ROS Package On K1

```bash
cd ~/edge-ai-robot-k1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Base Driver

```bash
source /opt/ros/humble/setup.bash
source ~/edge-ai-robot-k1/ros2_ws/install/setup.bash
ros2 launch turn_on_wheeltec_robot tank_base_brake.launch.py
```

Expected driver profile:

- `/cmd_vel` input
- `/odom` output
- `/imu/data_raw` output
- max linear x: `0.45 m/s`
- max angular z: `2.40 rad/s`
- reverse brake cap: `0.50 s`

## N10P Mapping

```bash
source /opt/ros/humble/setup.bash
source ~/edge-ai-robot-k1/ros2_ws/install/setup.bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping.launch.py
```

This launches:

- `lslidar_driver`
- `wheeltec_robot_node`
- static `base_footprint -> laser` transform
- `slam_toolbox`

## Manual Motion Test

Only run these with the robot guarded.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Light Control

Force off:

```bash
sudo python3 ~/tools/gpio37_light_smooth.py off
```

Ramp to 5 percent and hold until interrupted:

```bash
sudo python3 ~/tools/gpio37_light_smooth.py set 5 --start 0 --ramp 2 --hold -1
```

Low-power breathing pattern:

```bash
sudo python3 ~/tools/gpio37_light_smooth.py breathe --low 0 --high 5 --ramp 3
```

## D435 RealSense

Install librealsense and ROS wrapper as described in:

```bash
docs/d435_realsense.md
```

Run camera:

```bash
source /opt/ros/humble/setup.bash
source ~/realsense_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640,480,30 \
  depth_module.infra_profile:=640,480,30 \
  rgb_camera.color_profile:=640,480,30 \
  pointcloud.enable:=false
```

Check streams:

```bash
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/color/points
```

## Bus Servo Smoke Test

Install dependency:

```bash
sudo apt install -y python3-serial
```

Dry run:

```bash
python3 ~/tools/arm_bus_servo_smoke.py --port /dev/ttyUSB0 --id 1
```

Small movement on safer servo IDs:

```bash
python3 ~/tools/arm_bus_servo_smoke.py --port /dev/ttyUSB0 --id 1 --center 500 --delta 30 --time-ms 800 --run
python3 ~/tools/arm_bus_servo_smoke.py --port /dev/ttyUSB0 --id 4 --center 500 --delta 30 --time-ms 800 --run
```

## Safety Checklist

- Suspend the chassis for first tests after flashing or rewiring.
- On ground tests, keep one hand ready to lift the robot or cut power.
- Keep light off by default.
- Verify battery cell balance before high-load runs.
- Do not start mapping in a cluttered area until manual `/cmd_vel` response is stable.
- Test servo IDs `2` and `3` only after confirming mechanical clearance.
