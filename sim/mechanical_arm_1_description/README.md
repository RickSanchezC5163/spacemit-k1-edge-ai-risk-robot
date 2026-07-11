# mechanical_arm_1_description

ROS 2 description package generated from the SolidWorks-to-URDF export provided
on 2026-07-11.

This package is for visual inspection and separated arm simulation first. It is
not yet a MoveIt-ready or hardware-ready arm model.

## Source

The raw SW2URDF export is preserved under:

```text
assets/sw2urdf_raw/mechanical_arm_1/
```

The ROS 2 package uses an ASCII package name and patched mesh paths:

```text
sim/mechanical_arm_1_description/
```

## Known Issues

- The original export was a ROS 1 catkin package named `机械臂1`.
- The original joint limits were all zero:
  `lower=0 upper=0 effort=0 velocity=0`.
- The current `mechanical_arm_1_visual.urdf` uses temporary simulation-only
  limits so RViz sliders can move the model.
- The true servo-to-joint mapping, home pose, joint limits, and gripper
  semantics still need mechanical/electrical confirmation.
- Collision geometry currently reuses visual STL meshes. Replace with simple
  boxes/cylinders before physics-heavy Gazebo or MoveIt planning.

## Build

```bash
cd /path/to/spacemit-k1-edge-ai-risk-robot
source /opt/ros/humble/setup.bash
colcon build --base-paths sim --symlink-install
source install/setup.bash
```

## Display

```bash
ros2 launch mechanical_arm_1_description display.launch.py
```

## Integration Plan

1. Validate link scale and joint axes in RViz.
2. Confirm real joint limits and servo ID mapping.
3. Replace mesh collisions with simplified collision geometry.
4. Add a MoveIt config package.
5. Connect leakage response as a no-load candidate before any contact action.
