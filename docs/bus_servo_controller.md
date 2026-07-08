# Bus Servo Controller Bring-up

This document records the current bus servo controller state and a conservative
test procedure for the robot arm.

## Current Status

- Controller: bus servo controller board from the local vendor package.
- External supply: 12 V connected on the controller power side.
- Logic/control path tested:
  - micro-B USB path can drive servos reliably.
  - USART header path worked only after TX/RX wiring and common GND were fixed.
- Servo IDs observed/configured: `1`, `2`, `3`, `4`.
- All servos were placed around center position `500` during bring-up.

Safety status:

- IDs `1` and `4` are safer for small demonstration moves.
- IDs `2` and `3` can collide with the mechanism and must be moved only after
  visual clearance is confirmed.

## Wiring Notes

For the USART header path:

- USB-TTL `TX` -> controller `RX`.
- USB-TTL `RX` -> controller `TX` if feedback is needed.
- USB-TTL `GND` -> controller `GND`.
- Common ground is required.
- Do not rely on the USB-TTL red 5 V wire to power the servo load.
- The controller/servo power should come from the dedicated external supply.

Observed controller feedback:

- Blue LED flash means the controller accepted a valid frame.
- Two beeps without LED response indicates a malformed frame or wiring/protocol
  issue.

## Serial Parameters

```text
Baud: 9600
Frame header: 0x55 0x55
Command: 0x03 for servo move
Position range: 0-1000
Center: 500
```

Single-servo move frame:

```text
55 55 08 03 01 TIME_L TIME_H ID POS_L POS_H
```

## Tool

The repo includes:

```bash
tools/arm_bus_servo_smoke.py
```

Copy it to the board if needed:

```bash
scp tools/arm_bus_servo_smoke.py soc@192.168.43.40:/home/soc/tools/
```

Install runtime dependency on the board:

```bash
sudo apt update
sudo apt install -y python3-serial
```

## Identify Serial Port

```bash
ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH343USB* 2>/dev/null || true
dmesg | tail -80
```

Common candidates:

- `/dev/ttyUSB0`
- `/dev/ttyCH343USB0`
- `/dev/ttyACM0`

If a stable symlink is needed, create a udev rule later after confirming the
exact adapter with:

```bash
udevadm info -a -n /dev/ttyUSB0 | head -120
```

## Safe Test Commands

Dry run, no motion:

```bash
python3 ~/tools/arm_bus_servo_smoke.py --port /dev/ttyUSB0 --id 1
```

Move one safe servo slightly:

```bash
python3 ~/tools/arm_bus_servo_smoke.py --port /dev/ttyUSB0 --id 1 --center 500 --delta 30 --time-ms 800 --run
```

Move servo 4 slightly:

```bash
python3 ~/tools/arm_bus_servo_smoke.py --port /dev/ttyUSB0 --id 4 --center 500 --delta 30 --time-ms 800 --run
```

Only test IDs `2` and `3` when the arm is physically clear:

```bash
python3 ~/tools/arm_bus_servo_smoke.py --port /dev/ttyUSB0 --id 2 --center 500 --delta 20 --time-ms 1000 --run
python3 ~/tools/arm_bus_servo_smoke.py --port /dev/ttyUSB0 --id 3 --center 500 --delta 20 --time-ms 1000 --run
```

Return one servo to center:

```bash
python3 ~/tools/arm_bus_servo_smoke.py --port /dev/ttyUSB0 --id 1 --center 500 --delta 0 --time-ms 800 --run
```

## Known Open Items

- Final mechanical limits for IDs `2` and `3` are not documented yet.
- A ROS2 action/service wrapper for arm motions is still needed.
- A named udev symlink such as `/dev/arm_bus` should be added after the final
  USB-TTL adapter is selected.
