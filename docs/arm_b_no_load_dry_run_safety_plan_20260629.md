# Arm-B No-Load Dry-Run Safety Plan

**Date**: 2026-06-29
**Protocol**: arm_safety_v1
**Status**: Plan generated, hardware NOT executed

---

## Overview

Arm-B is the second phase of K1 5-DOF bus servo arm integration. It defines the
safety framework, command validation, and dry-run motion plans **before** any
physical arm movement. This phase does not require the arm hardware to be connected.

### Safety Gates (Arm-B: ALL FALSE)

| Gate | Value | Reason |
|------|-------|--------|
| `arm_enabled` | false | Arm not yet connected |
| `hardware_access_allowed` | false | No serial port open |
| `serial_write_allowed` | false | No bytes sent to bus |
| `contact_allowed` | false | No physical contact |
| `obstacle_removal_allowed` | false | Not removing objects |
| `dry_run` | true | Validation only |

### Files

| File | Purpose |
|------|---------|
| `configs/arm_safety_config.json` | Joint limits, workspace boundaries, safety gates |
| `src/arm_safety.py` | Command validator, kinematics, phase management |
| `tools/generate_arm_b_no_load_dry_run_plan.py` | Plan generator and validator |
| `docs/arm_b_no_load_dry_run_safety_plan_20260629.md` | This document |

---

## Arm Kinematics Summary

```
ID1 base (yaw, z-axis, at chassis center -0.5cm)
  └── L1 = 19cm vertical arm (z-direction)
      └── ID2 shoulder (pitch, y-axis)
          └── L2 = 4cm horizontal link (x-direction)
              └── ID3 elbow (pitch, y-axis)
                  └── L3 = 19cm vertical arm (z-direction)
                      └── ID4 wrist (pitch, y-axis)
                          └── L4 = 5.5cm wrist + 6cm fingers
                              └── ID5 gripper (open/close, z-axis)
```

- Arm base position: x=-0.005m, y=0m, z=0.13m (relative to chassis center)
- Max horizontal reach: ~45cm forward from chassis center
- Max vertical reach: ~55cm above ground

---

## Safety Layers (7-layer architecture)

| Layer | Name | Description | Blocking |
|-------|------|-------------|----------|
| L1 | Phase Gate | Which phases allow hardware access | Yes |
| L2 | Protocol Validation | Servo ID, pulse [0,1000], time [0,30000]ms | Yes |
| L3 | Joint Limits | Soft limit (warning) + hard limit (block) | Yes |
| L4 | Step Size | Max 300 pulse change per step, phase-specific dev | Yes |
| L5 | Workspace | End-effector within safe zone, keep-out zones | Warn |
| L6 | Robot Safety | Base must be zero, robot must not be driving | Yes |
| L7 | Emergency | E-stop active, heartbeat timeout | Yes |

---

## Bus Servo Protocol

- **Transport**: UART, 9600 baud, 8N1
- **Frame**: `0x55 0x55 <length> <command> <parameters...>`
- **CMD 3 (SERVO_MOVE)**: `<count> <time_lo> <time_hi> [<id> <pulse_lo> <pulse_hi>]...`
- **Pulse range**: 0-1000 (bus servo mode)
- **Pulse center**: 500
- **Time range**: 0-30000 ms

---

## Phase Gates

### Arm-B1: Plan Only
```
arm_enabled = false
All sequences validated in dry-run mode.
No serial port opened. No bytes transmitted.
```

Sequences:
- 5× single-joint full-range traversal (plan only)
- 1× home-all cycle (plan only)

### Arm-B2: Single Joint, Small Angle
```
arm_enabled = true
hardware_access_allowed = true
serial_write_allowed = true
contact_allowed = false
max_pulse_deviation_from_center = 100 (±10% pulse range)
max_single_joint_count = 1
```

Sequences:
- 5 joints × 2 directions × 2 steps (center→±100→center) = 20 steps

Acceptance criteria:
- All 10 sequences (20 steps) complete without error
- No joint exceeds soft limit
- Base stays zero throughout
- Robot does not move

### Arm-B3: Full No-Load Sequence
```
arm_enabled = true
hardware_access_allowed = true
serial_write_allowed = true
contact_allowed = false
obstacle_removal_allowed = false
```

Sequences:
- 1× full removal trajectory (home → reach → grasp → lift → place → home)
- 4× individual joint sweeps (full soft range, low speed)

Acceptance criteria:
- Removal trajectory completes full cycle
- All joints return to home position
- No collisions with chassis, lidar, or D435 keep-out zones
- No hard limit violations

### Arm-C: No-Load Home Cycle
```
contact_allowed = false
Arm cycles: home → reach → home, no payload
```

### Arm-D: Light Contact (Foam Only)
```
contact_allowed = true
contact_material = foam_only
max_contact_force_n = 2.0
```

### Arm-E: Real Obstacle Removal
```
All gates open.
Real debris removal with full safety monitoring.
```

---

## Robot-Level Safety Requirements

During any arm movement:
- **Base must be zero** (base_zero_ok = true)
- **Robot must not be driving** (cmd_vel = 0)
- **Heartbeat active** (every 500ms, timeout 2.0s)
- **E-stop not active**
- **Front clearance** ≥ 0.40m (front_p10)

On heartbeat loss or E-stop:
- Auto-home all joints to center position
- Lock arm until manual reset

---

## Workspace Keep-Out Zones

| Zone | Center (x, z) | Size | Reason |
|------|---------------|------|--------|
| N10P Lidar | (0.075, 0.13) | radius 8cm | Protect lidar from arm strike |
| D435 Camera | (0.105, 0.11) | 12×6×6cm box | Protect camera from arm strike |
| Chassis edges | — | 2cm clearance | Prevent arm hitting chassis |

---

## Missing Data (Pending Physical Measurement)

The following values in `arm_safety_config.json` are **conservative placeholders**.
They must be replaced with measured values before Arm-B2 hardware execution:

| Parameter | Current Placeholder | How to Measure |
|-----------|-------------------|----------------|
| ID1 angle range | ±180° | Check servo datasheet or test via Bus Servo Control V3.2 |
| ID2 angle range | ±90° | Same |
| ID3 angle range | ±105° | Same |
| ID4 angle range | ±90° | Same |
| ID5 open/close angle | 0-60° | Measure jaw opening at pulse 0 and pulse 1000 |
| Servo current limits | 1000-1800mA | Check servo datasheet |
| Joint soft limits (pulse) | conservative | Calibrate via Bus Servo Control V3.2, find mechanical stops |

---

## Next Steps

1. **Now**: Review and validate generated dry-run plans
2. **When arm hardware connected**: Arm-B2 → single joint, small angle
3. **After B2 passes**: Arm-B3 → full no-load sequence
4. **After B3 passes**: Arm-C → home cycle validation
5. **After C passes**: Arm-D → foam contact test
6. **After D passes**: Arm-E → real obstacle removal
