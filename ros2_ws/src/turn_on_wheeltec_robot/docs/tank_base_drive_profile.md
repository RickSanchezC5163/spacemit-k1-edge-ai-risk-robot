# L150Pro C30D Tank Base Drive Profile

## Tested Hardware State

- Chassis: WHEELTEC L150Pro Tank
- Controller: C30D, LED mode set to Tank
- Firmware: ROS protocol firmware with stop watchdog patch
- Serial device on K1: `/dev/base_controller`
- Baud rate: `115200`

## Ground-Tested Command Profile

The following values were tested on the ground and are suitable as the first mapping profile:

- Linear speed limit: `0.45 m/s`
- Angular speed limit: `2.40 rad/s`
- Command refresh rate: `50 Hz`
- Command timeout: `0.25 s`
- Reverse brake ratio: `1.0`
- Reverse brake duration: `min(motion_duration, 0.50 s)`
- Zero command is continuously sent after braking while no fresh `/cmd_vel` is received.

The direct serial validation sequence was:

```text
forward  +0.45 m/s for 0.50 s
brake    -0.45 m/s for 0.50 s
zero      0.00 m/s for 2.00 s
```

Backward, left turn, and right turn were validated with the same reverse-brake pattern.

## Launch Commands

Base only:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 launch turn_on_wheeltec_robot tank_base_brake.launch.py
```

N10P lidar + Tank base + slam_toolbox:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping.launch.py
```

## Safety Notes

- For first tests after flashing or wiring changes, keep the chassis suspended.
- For ground tests, keep one hand ready to lift the robot or cut power.
- Do not re-enable the old `brake_duration_gain=0.6` mapping profile; the tested profile uses `1.0` with a `0.50 s` cap.
