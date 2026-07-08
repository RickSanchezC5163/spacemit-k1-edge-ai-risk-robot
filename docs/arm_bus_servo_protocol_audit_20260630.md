# Arm Bus Servo Protocol Audit

**Date**: 2026-06-30
**Type**: Read-only document review
**Hardware**: NOT accessed, NOT connected, NOT controlled

---

## 1. Scope

This document summarizes findings from a **read-only** audit of the Lobot bus servo
controller SDK at `K:/risc-vCar/总线舵机控制器`. No serial port was opened. No
control frames were sent. No servos were moved.

This audit informs Arm-B no-load validation planning only.

---

## 2. Source Directory

```
K:/risc-vCar/总线舵机控制器/
├── 1.教程资料/
│   ├── 1.总线舵机控制器使用说明/
│   │   ├── 01 总线舵机控制器使用说明.pdf
│   │   └── 03 上位机软件安装包/
│   │       ├── ch341ser.exe              # CH341 USB-UART driver
│   │       └── Bus Servo Control V3.2.exe # Windows GUI control tool
│   └── 2.总线舵机控制器二次开发教程/
│       ├── 01 控制器通信协议/
│       │   └── 01 控制器通信协议.pdf       # PRIMARY: protocol spec
│       ├── 02 Arduino版本开发/
│       ├── 03 C51版本开发/
│       ├── 04 STM32版本开发/
│       ├── 05 树莓派版本开发/               # Python SDK (Raspberry Pi)
│       │   └── 02 源码教程/
│       │       ├── Raspberry_BusServoControl_demo/
│       │       │   ├── BusServoControl.py        # single servo move
│       │       │   ├── BusServoSpeed.py           # servo speed control
│       │       │   ├── BusServoMoveByArray.py     # multi-servo sync move
│       │       │   ├── BusServoMedAndBias.py      # center/bias calibration
│       │       │   └── ServoControl.py            # core protocol impl
│       │       └── 案例1-4/                        # example scripts
│       └── 06 Jetson版本开发/               # Python SDK (Jetson — most K1-relevant)
│           └── 02 源码教程/
│               └── Jetson_BusServoControl_demo/
│                   ├── single_servo_control_turn/   # single servo turn
│                   ├── single_servo_control_speed/  # single servo speed
│                   ├── multi_servo_control/         # multi-servo sync
│                   │   ├── ServoControl.py          # core protocol impl
│                   │   └── BusServoMoveByArray.py   # multi-servo example
│                   └── servo_adjust/               # servo calibration
│                       └── ServoControl.py          # calibration protocol impl
├── 2.软件工具/
│   ├── 08 上位机软件/
│   └── 09 手机APP/
├── 3.拓展学习资料/
│   └── 二次开发串口通信协议/               # Additional protocol reference
└── 附录/库文件/
```

---

## 3. Key Findings

### 3.1 Serial Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Baudrate | 9600 | All Python/C examples |
| Data bits | 8 | Standard UART (implied by protocol) |
| Parity | None | Standard UART (implied) |
| Stop bits | 1 | Standard UART (implied) |
| Port (Jetson) | `/dev/ttyTHS1` | Jetson SDK examples |
| Port (Raspberry Pi) | `/dev/ttyS0` or `/dev/ttyAMA0` | RPi SDK examples |
| Port (K1 RISC-V) | `/dev/ttyS1` or USB UART `/dev/ttyUSB0` | TBD — needs K1 verification |
| Timeout | Not explicitly set in examples | Should be 0.5s minimum |

### 3.2 Control Frame Format

**Single servo move frame** (CMD_SERVO_MOVE = 3):
```
Offset  Size  Value          Description
------  ----  -----          -----------
0       2     0x55 0x55      Frame header
2       1     0x08           Data length (always 8 for single servo)
3       1     0x03           CMD_SERVO_MOVE
4       1     0x01           Servo count (1)
5       2     <time_lo hi>   Move time in ms, little-endian, range [0, 30000]
7       1     <servo_id>     Servo ID, range [1, 254]
8       2     <pulse_lo hi>  Target pulse, little-endian, range [0, 1000]
```

**Multi-servo sync move frame**:
```
Offset  Size  Value                    Description
------  ----  -----                    -----------
0       2     0x55 0x55                Frame header
2       1     <count*3+5>              Data length
3       1     0x03                     CMD_SERVO_MOVE
4       1     <count>                  Servo count [1, 254]
5       2     <time_lo hi>             Move time, shared by all servos
7       3*n   [<id> <pulse_lo hi>]*n   Per-servo ID + position pairs
```

**Action Group commands** (CMD 6/7/11):
```
CMD_ACTION_GROUP_RUN  (6): 0x55 0x55 0x05 <group_id> <count_lo> <count_hi>
CMD_ACTION_GROUP_STOP (7): 0x55 0x55 0x02
CMD_ACTION_GROUP_SPEED(11): 0x55 0x55 0x05 <group_id> <speed_lo> <speed_hi>
CMD_GET_BATTERY_VOLTAGE(15): (not fully documented in examples)
```

**Key protocol characteristics**:
- No checksum byte
- No response frame from servo controller (fire-and-forget)
- No position feedback / read-back command found in SDK
- No unload/torque-off command found in SDK
- Multi-servo sync: all servos share the same move time, start simultaneously

### 3.3 Position Units

| Property | Bus Servo | PWM Servo |
|----------|-----------|-----------|
| Pulse range | 0–1000 | 500–2500 |
| Center | 500 | 1500 |
| Time unit | ms | ms |
| Pulse-to-angle | Unknown (servo-model dependent) | Unknown |
| Direction inversion | Not supported in protocol — must handle in pulse mapping | Same |

**Pulse-to-angle**: The SDK does not provide a direct pulse-to-angle conversion.
The relationship is servo-model-specific. 0=min_angle, 500=center, 1000=max_angle,
but the actual angle range depends on the servo's mechanical design.

### 3.4 Available Commands (from SDK)

| Command | Code | Supported | Notes |
|---------|------|-----------|-------|
| SERVO_MOVE (single) | 3 | ✓ | 1 servo, pulse 0-1000 |
| SERVO_MOVE (multi) | 3 | ✓ | N servos, shared time, sync start |
| ACTION_GROUP_RUN | 6 | ✓ | Pre-programmed action group playback |
| ACTION_GROUP_STOP | 7 | ✓ | Stop current action group |
| ACTION_GROUP_SPEED | 11 | ✓ | Set action group playback speed |
| GET_BATTERY_VOLTAGE | 15 | ? | Defined in SDK but no example usage |
| Read position | — | ✗ | Not found in SDK |
| Unload / torque-off | — | ✗ | Not found in SDK |
| Read current | — | ✗ | Not found in SDK |
| Read temperature | — | ✗ | Not found in SDK |
| Read error status | — | ✗ | Not found in SDK |

**Critical gap**: The protocol is fire-and-forget. There is no feedback mechanism
to read current servo position, load, current draw, temperature, or stall status.
This means:
- All safety monitoring must be external (current sensor, visual inspection)
- The controller cannot confirm that a servo actually reached its target
- Stall detection must be done at the application level (if at all)

### 3.5 ROS/ROS2 Interface

**None found.** The SDK directory contains no ROS or ROS2 packages, no `.msg`
definitions, no launch files, and no ROS nodes. The bus servo controller is a
standalone UART device controlled via raw serial frames. Any ROS integration
must be built from scratch.

### 3.6 GUI Tools

- **Bus Servo Control V3.2.exe** (Windows): GUI for manual servo control,
  calibration, position setting, and action group programming. Located at
  `1.教程资料/1.总线舵机控制器使用说明/03 上位机软件安装包/`.
- Shortcut at `C:\Users\Public\Desktop\Bus Servo Control.lnk`

---

## 4. Safety Boundary

As of this audit, the following safety constraints are enforced:

### 4.1 Global Safety Gates (configs/arm_safety_config.json)

```json
"safety_gates": {
  "arm_enabled": false,
  "hardware_access_allowed": false,
  "serial_write_allowed": false,
  "contact_allowed": false,
  "obstacle_removal_allowed": false
}
```

### 4.2 Effective Gate Logic (src/arm_safety.py)

All gate properties now use: `effective = global_gate AND phase_gate`

A phase CANNOT bypass a global gate. Specifically:
- Even if B2/B3 phase sets `serial_write_allowed=true`, the global
  `serial_write_allowed=false` overrides, making effective `false`.
- `build_move_frame()` always builds the frame bytes for review (`frame_hex`),
  but only returns executable `frame_bytes` when `serial_write_allowed_effective=true`.
- The `frame_built_for_review_only` flag explicitly marks non-executable frames.

### 4.3 Current Claim Boundary

```
arm_enabled:                        false (global)
hardware_access_allowed:            false (global)
serial_write_allowed:               false (global)
contact_allowed:                    false (global)
obstacle_removal_allowed:           false (global)
hardware_executed:                  false
serial_bytes_sent:                  0
ROS nodes started:                  0
Real servo commands sent:           0
```

---

## 5. Arm-B Validation Mapping

| Phase | Description | Hardware | Serial | Contact | Status |
|-------|-------------|----------|--------|---------|--------|
| Arm-B0 | Protocol audit | no | no | no | ✓ This document |
| Arm-B1 | Plan only, validate sequences | no | no | no | ✓ Dry-run complete |
| Arm-B2 | Single-servo small-angle (±100 pulse) | yes | yes | no | Pending calibration |
| Arm-B3 | Full no-load sequence | yes | yes | no | Pending B2 |
| Arm-C | No-load motion with return home | yes | yes | no | Pending B3 |
| Arm-D | Soft foam light contact | yes | yes | foam only | Pending C |
| Arm-E | Real obstacle removal | yes | yes | yes | Pending D |

**Before Arm-B2 can proceed**:
1. Global `hardware_access_allowed` must be set to `true` (intentionally)
2. Global `serial_write_allowed` must be set to `true` (intentionally)
3. Joint home_pulse values must be calibrated and verified
4. Soft limit values must be updated from calibration data
5. Single-joint pulse direction must be confirmed

---

## 6. Candidate Sequence

**File**: `configs/arm_b_no_load_sample_v0_candidate.json`

Contains 8 steps transcribed from manual calibration notes, including:
- Step 1: flat/level starting pose
- Step 2: raised/bow pose
- Steps 3a-3b: approach and full reach
- Step 4: grasp pose
- Step 5: open gripper for release
- Step 6a-6b: return to safe idle home

**Status**: `validated=false`, `hardware_executed=false`

**Warnings identified**:
- ID2=250 in steps 3b/5 appears near lower mechanical limit
- ID2=771 in step 6b is far above default center (verify safe stow position)
- Pulse-to-angle mapping and direction not confirmed for any joint

---

## 7. Next Steps

1. **Manual calibration** (operator with Bus Servo Control V3.2):
   - Confirm servo ID → joint mapping
   - Record home_pulse for each joint
   - Record near-limit pulse values (lower and upper)
   - Record pulse increase direction per joint
   - Determine safe soft limits with margin

2. **Config update** (after calibration):
   - Update `joints.*.home_pulse` in arm_safety_config.json
   - Update `joints.*.soft_limit_lower_pulse` and `soft_limit_upper_pulse`
   - Update `poses.safe_idle_home_like_6b` if measured pose differs

3. **Gate enable** (intentional, explicit):
   - Set global `hardware_access_allowed=true`
   - Set global `serial_write_allowed=true`
   - Keep `contact_allowed=false`, `obstacle_removal_allowed=false`

4. **Arm-B2**: Single servo, ±100 pulse, no contact

5. **Arm-B3**: Full no-load sequence with intermediate steps
