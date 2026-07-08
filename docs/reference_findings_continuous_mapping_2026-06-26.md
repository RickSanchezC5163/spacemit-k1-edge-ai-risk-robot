# Reference Findings for Continuous Mapping - 2026-06-26

## Purpose

This note records the useful references found in the local WHEELTEC, C30D STM32,
N10P lidar, and SpacemiT K1/Bianbu material for the next mapping step.

Current decision: mapping should use guarded continuous motion and C30D speed-loop
tuning. Short pulse tests are only for chain/safety checks, not for real SLAM
data collection.

## WHEELTEC ROS2 Base Driver

Reference tree:

- `ros-related/src/turn_on_wheeltec_robot`
- `turn_on_wheeltec_robot/src/wheeltec_robot.cpp`
- `turn_on_wheeltec_robot/launch/n10p_tank_mapping.launch.py`
- `turn_on_wheeltec_robot/config/slam_toolbox_n10p_tank.yaml`
- `turn_on_wheeltec_robot/docs/tank_base_drive_profile.md`

Findings:

- The official C++ `wheeltec_robot_node` subscribes `/cmd_vel`.
- It sends the existing C30D ROS frame `0x7B ... 0x7D` at a fixed rate.
- Frame fields are `linear.x`, `linear.y`, and `angular.z`, each scaled by
  `1000`.
- `n10p_tank_mapping.launch.py` already sets:
  - `max_linear_x=0.45`
  - `max_angular_z=2.40`
  - `send_rate_hz=50.0`
  - `cmd_timeout_sec=0.25`
  - reverse brake enabled with ratio `1.0`
- The existing mapping launch starts N10P, the C++ base node, static
  `base_footprint -> laser`, and `slam_toolbox`.

Conclusion:

- Use the C++ official node for mapping. It already drove the current firmware.
- Do not use the Python safe base node for mapping yet; it can parse feedback
  but did not actuate this firmware reliably.
- ROS can clamp, smooth, repeat, and reverse-brake commands, but it cannot fully
  fix lower-level speed-loop integral tail.

## STM32 C30D Firmware Logic

Reference tree:

- `Mini car D-version STM32 source`
- `BALANCE/system.c`
- `BALANCE/system.h`
- `BALANCE/balance.c`
- `BALANCE/control.c`
- `HARDWARE/usartx.c`
- `HARDWARE/stmflash.c`

Findings:

- `Tank_Car` is mode `5`.
- `robot_select_init.c` maps the potentiometer-selected Tank mode to Tank wheel
  geometry.
- APP parameters map to:
  - parameter 0 -> `RC_Velocity`
  - parameter 1 -> `Velocity_KP`
  - parameter 2 -> `Velocity_KI`
- The speed loop is incremental PI:

```c
Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
```

- `Pwm` and `Last_bias` are static inside each incremental PI function.
- Tank kinematics in `Drive_Motor()` support combined forward motion and yaw:

```c
MOTOR_A.Target = Vx - Vz * Wheel_spacing / 2.0f;
MOTOR_B.Target = Vx + Vz * Wheel_spacing / 2.0f;
```

- ROS serial frames refresh the command path through `USART3_IRQHandler()` and
  write `Move_X`, `Move_Y`, and `Move_Z`.
- The APP direction buttons are discrete. That can make the chassis feel like it
  only supports forward, backward, left, and right, but the Tank kinematics are
  not limited to that.

Conclusion:

- The user's concern is correct: if PI state is retained, ROS zero commands and
  ordinary parameter tuning alone may not produce immediate stop.
- For tonight, tune APP parameter 0/1/2 first and validate continuous motion.
- If long stop tail remains after reasonable KI reduction, the next firmware
  patch should reset/clamp PI state on explicit zero command or command timeout.

## N10P Lidar References

Reference tree:

- `N10 series material/2.ROS2_SDK`
- `lslidar_ros2/lslidar_driver.zip`
- `lslidar_ros2/lslidar_msgs.zip`
- `lslidar_ros2/wheeltec_udev.sh`
- `N10 series material/4.product manual`
- `N10 series material/5.user manual`

Findings:

- Local ROS2 material contains the LSLidar ROS2 driver and messages.
- The udev helper creates `/dev/wheeltec_lidar` for common CP2102/CH343/ACM
  serial adapters.
- Previous K1 validation showed N10P `/scan` was stable at about `12 Hz`.
- `slam_toolbox_n10p_tank.yaml` uses:
  - `scan_topic: /scan`
  - `base_frame: base_footprint`
  - `odom_frame: odom`
  - `map_frame: map`
  - `min_laser_range: 0.20`
  - `max_laser_range: 12.0`

Conclusion:

- N10P is not the current blocker. The active blocker is stable continuous base
  motion and stop behavior.

## Navigation and Exploration References

Reference tree:

- `ros-related/src/wheeltec_robot_nav2`
- `ros-related/src/wheeltec_robot_slam`
- `ros-related/src/wheeltec_robot_rrt2`

Findings:

- WHEELTEC Nav2 configs commonly use `/scan` as the costmap observation source.
- The `param_mini_diff_DWB.yaml` reference allows `max_vel_x` around `0.6 m/s`.
- RRT exploration launch exists and combines Nav2 + slam + RRT exploration, but
  it starts autonomous exploration behavior.

Conclusion:

- Do not run RRT exploration until base PID and stop behavior are stable.
- The official Nav2 examples confirm that `0.30-0.45 m/s` is not unreasonable
  for this class of chassis, but it must be matched to the real C30D speed loop.

## SpacemiT K1 / Bianbu References

Official references:

- K1 MUSE Pi Pro documentation:
  https://www.spacemit.com/community/document/info?lang=zh&nodepath=hardware/eco/k1_muse_pi_pro/root_overview.md
- Bianbu documentation:
  https://www.spacemit.com/community/document/info?lang=zh&nodepath=software/SDK/bianbu/root_overview.md
- Bianbu v2.3.3 image archive:
  https://archive.spacemit.com/image/k1/version/bianbu/v2.3.3/
- SpacemiT community resources:
  https://www.spacemit.com/community

Findings:

- These references support the K1/Bianbu platform, OS image, peripheral, and
  deployment side of the project.
- They do not replace WHEELTEC's C30D firmware and ROS base-drive material.

Conclusion:

- Use SpacemiT docs for K1 OS, image, pin/peripheral, and deployment issues.
- Use WHEELTEC C30D/ROS docs for chassis motion, PID, odom, and serial protocol.

## Practical Plan

1. Keep `n10p_tank_mapping.launch.py` as the mapping bringup.
2. Use `tools/continuous_drive_calibration.py` for guarded continuous tests:

```bash
python3 tools/continuous_drive_calibration.py --linear 0.30 --duration 3.0 --ramp-time 0.4
python3 tools/send_safe_zero_cmd.py --duration 3
```

3. Tune APP parameters one at a time:
   - too weak / needs push: raise speed scale, then KP;
   - surges / oscillates: lower KP;
   - long stop tail: lower KI first;
   - still tails after KI reduction: firmware PI reset is needed.
4. Do not enable RRT/Nav2 autonomous exploration until:
   - continuous `0.30 m/s` starts by itself;
   - stop after zero is around 1 second or better;
   - `/odom` and `/scan` remain stable during motion;
   - saved `slam_toolbox` map is coherent.
