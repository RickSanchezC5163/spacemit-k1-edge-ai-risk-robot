# Mechanical Arm 1 SW2URDF Import Audit - 2026-07-11

## Imported Files

Source folder:

```text
/Users/Zhuanz/Library/Containers/com.tencent.xinWeChat/Data/Documents/xwechat_files/wxid_2rbkv01sac4h22_f1f3/msg/file/2026-07/机械臂1/
```

Repository copies:

```text
assets/sw2urdf_raw/mechanical_arm_1/
sim/mechanical_arm_1_description/
```

The export contains:

- `base_link`, `Link1`, `Link2`, `Link3`, `Link4`
- revolute joints `j1`, `j2`, `j3`, `j4`
- STL meshes for each link
- ROS 1 catkin package metadata and launch files

## URDF Findings

The SW2URDF export is useful, but not yet motion-ready:

| Item | Finding | Impact |
| --- | --- | --- |
| Package name | Original package name is `机械臂1` | Risky under ROS 2 tooling; replaced with `mechanical_arm_1_description` |
| ROS version | Original package is catkin/ROS 1 | Converted to a ROS 2 `ament_cmake` description package |
| Joint limits | `j1`-`j4` all exported as `lower=0 upper=0 effort=0 velocity=0` | Joints are effectively locked |
| Mesh path | Original URDF uses `package://机械臂1/...` | Patched to `package://mechanical_arm_1_description/...` |
| Collision | Collision meshes equal visual STL meshes | Too heavy for reliable physics and MoveIt planning |
| Servo mapping | Not present in export | Must be confirmed before hardware control |

## Current Temporary Patch

`sim/mechanical_arm_1_description/urdf/mechanical_arm_1_visual.urdf` keeps the
original geometry and axes, but gives each joint temporary simulation-only
limits so RViz sliders can move the model:

```text
j1: [-90 deg, +90 deg]
j2: [-70 deg, +70 deg]
j3: [-70 deg, +70 deg]
j4: [-90 deg, +90 deg]
```

These are not hardware limits.

## Leakage Response Implication

For the risk pipeline, this arm can now be used as a visual and planning
placeholder:

```text
leakage risk point
-> approach observation pose
-> stop and confirm
-> generate arm leakage response candidate
-> MoveIt/no-load simulation after joint limits are confirmed
```

Do not use this import for physical leakage contact or obstacle removal yet.

## Next Required Work

1. Open the package in RViz and verify scale/orientation.
2. Confirm joint axes match expected servo movement.
3. Obtain real servo ID mapping and pulse-to-angle ranges.
4. Replace collision STL meshes with simplified collision shapes.
5. Generate MoveIt configuration after joint limits are validated.
6. Add a no-load `LEAKAGE_RESPONSE_CANDIDATE` primitive before any contact task.
