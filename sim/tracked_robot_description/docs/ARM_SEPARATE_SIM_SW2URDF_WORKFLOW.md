# Mechanical Arm Separate Simulation Workflow

The default Gazebo vehicle model excludes the mechanical arm. Keep the vehicle
simulation focused on the mobile base, lidar, and D435-style camera. Import and
validate the arm as a separate simulation package after the mechanical CAD/URDF
export is confirmed.

## Scope

- `tracked_robot.urdf.xacro`: mobile base, lidar, and D435-style camera only.
- Mechanical arm: separate SW2URDF-exported package and separate RViz/Gazebo/
  MoveIt validation.
- Do not mount the arm back on the mobile base until its mass, joint axes,
  collision meshes, and controller interface are verified independently.

## SolidWorks To URDF Checklist

Use the mechanical team's SolidWorks assembly as the source of truth.

1. Install the SolidWorks URDF exporter matching the SolidWorks version.
2. Define one reference axis per actuated joint and name axes clearly, for
   example `joint_1`, `joint_2`, `joint_3`.
3. Add reference points at each joint origin and at the end-effector frame.
4. Create coordinate systems with ROS-friendly conventions:
   - `z` along the joint axis where applicable.
   - `x` forward from the parent link.
   - the base frame aligned with the intended RViz/Gazebo base pose.
5. In the exporter, define the parent-child link tree from base to tip.
6. Set realistic joint limits, effort, velocity, mass, and inertia values.
7. Export with a clean package name; avoid names inherited from `.SLDASM`.
8. Keep generated meshes together with the generated URDF package.

## First Validation

After copying the exported arm package into an Ubuntu ROS workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Check the generated model:

```bash
check_urdf path/to/arm.urdf
ros2 launch <arm_description_pkg> display.launch.py
```

In RViz:

- Set `Fixed Frame` to the arm base link.
- Add `RobotModel`.
- Move joints with `joint_state_publisher_gui`.
- Confirm every joint rotates around the expected physical axis.

## MoveIt Bring-Up Notes

Use MoveIt Setup Assistant only after the raw URDF is correct.

- Add a fixed virtual joint from `world` to the arm base link.
- Generate the self-collision matrix.
- Create one planning group for the arm kinematic chain.
- Add named poses such as `home`, `up`, and `stowed`.
- Use a `FollowJointTrajectory` controller interface for the first simulation
  pass.

## Before Rejoining Vehicle Simulation

Only mount the arm back on the mobile base after these checks pass:

- arm base frame, tip frame, and joint axes are correct
- no mesh path errors
- no extreme mass/inertia values
- Gazebo does not fall, explode, or drift with the arm alone
- MoveIt can plan between named poses
- mounting transform on the chassis is confirmed by the mechanical team
