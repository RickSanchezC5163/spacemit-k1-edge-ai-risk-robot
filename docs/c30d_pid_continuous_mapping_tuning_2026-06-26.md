# C30D PID Continuous Mapping Tuning - 2026-06-26

## Goal

Make the Tank chassis capable of continuous, controllable mapping motion before
starting Nav2 autonomy.

The target is not ultra-low command speed. On the current floor, the chassis
does not reliably move at `0.05-0.10 m/s`. The practical target is:

```text
continuous linear.x ~= 0.30 m/s
short guarded tests up to 0.45 m/s
stop response <= 1 s after zero command
no visible runaway after /cmd_vel returns to zero
```

## Why PID Matters

SLAM and Nav2 need smooth continuous motion. Short breakaway pulses can prove
the chain works, but they create stop-and-go maps and cannot support automatic
navigation.

If the chassis only moves at high command values or keeps moving after zero,
the lower C30D speed loop must be tuned. APP parameters are the first tuning
path:

- Parameter 0: chassis speed scale.
- Parameter 1: speed-loop KP.
- Parameter 2: speed-loop KI.

Record the original values before changing anything.

## Test Command

Start the mapping stack first:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source ~/lslidar_ws/install/setup.bash
ros2 launch turn_on_wheeltec_robot n10p_tank_mapping.launch.py
```

In another terminal, after `/scan`, `/odom`, and TF are healthy:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 tools/continuous_drive_calibration.py --linear 0.30 --duration 3.0 --ramp-time 0.4
python3 tools/send_safe_zero_cmd.py --duration 3
```

For turning, test angular motion separately:

```bash
python3 tools/continuous_drive_calibration.py --angular 0.45 --duration 2.0 --ramp-time 0.4
python3 tools/send_safe_zero_cmd.py --duration 3
```

Do not test linear and angular motion together until each axis is stable.

## Tuning Observations

Use the script output and direct observation:

- Does the chassis start without needing a push?
- Does it track a roughly steady speed?
- Does it stop within about 1 second after zero?
- Does it drift, oscillate, or keep crawling?
- Does `/odom` show nonzero displacement while moving and zero twist after stop?

## Adjustment Rules

Make one small change at a time in the MiniBalance APP, then run the same ROS
test again.

- If the chassis needs a push or stalls: increase speed scale first, then KP.
- If it starts but surges or oscillates: reduce KP.
- If it keeps moving after zero or has long tail: reduce KI first.
- If reducing KI makes it unable to hold speed: increase KP slightly, keep KI conservative.
- If zero command still leaves long tail after reasonable KI reduction: firmware
  needs an integrator reset/clamp or explicit brake behavior on zero command.

## Minimum Pass Before Nav2

Do not run autonomous navigation until all items pass:

- `linear.x=0.30` for `3 s` starts by itself.
- Robot stops after zero command without needing reverse joystick input.
- `/odom` accumulates displacement during motion.
- `/odom.twist` returns to zero after stop.
- `/scan` remains stable during motion.
- `slam_toolbox` keeps a coherent local map.
