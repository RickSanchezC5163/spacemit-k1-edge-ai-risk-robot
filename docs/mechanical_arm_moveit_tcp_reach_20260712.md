# Mechanical Arm MoveIt TCP Reach Test 2026-07-12

## Scope

This note records the ROS 2 Humble / MoveIt RViz simulation check for the
mechanical arm ground-task workspace. The tested TCP is the invisible
`link4_tip_link` fixed to the end of `Link4`; no USB camera model is included in
this version.

## Key Result

Final 1000-sample run:

```text
requested_count: 1000
completed_count: 1000
planned_count: 1000
reached_count: 1000
failed_plan_count: 0
failed_reach_count: 0
max_tcp_error_m: 0.007177084987072778
mean_tcp_error_m: 0.0025150740320270797
skipped_by_height: 6
```

The test publishes the arm state directly to `/joint_states` for RViz visual
execution. It uses no final settle compensation; the red endpoint marker is the
actual final `link4_tip_link` pose after replaying the MoveIt trajectory.

## Visualization

Marker topic:

```text
/visual_reach_stress_markers_world
```

Marker colors:

- green: start TCP
- blue: target TCP
- red: actual final TCP
- yellow: start-to-final line

The marker is published directly by `tools/visual_moveit_arm_reach_stress.py`
to avoid relay lag.

## Local Evidence Paths

The following outputs are intentionally under `outputs/` and ignored by git:

```text
outputs/moveit_arm_visual_ground_tcp_direct_marker_1000/run.log
outputs/moveit_arm_visual_ground_tcp_direct_marker_1000/visual_reach_stress.json
outputs/moveit_arm_visual_ground_tcp_direct_marker_1000/visual_reach_stress.md
```

Pulled Ubuntu screen recordings are stored locally and ignored by git:

```text
artifacts/recordings/ubuntu/
```

## Re-run Command

```bash
cd /home/ubuntu/Documents/GitHub/spacemit-k1-edge-ai-risk-robot
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 tools/visual_moveit_arm_reach_stress.py \
  --count 1000 \
  --profile ground \
  --output-dir outputs/moveit_arm_visual_ground_tcp_direct_marker_1000 \
  --allowed-planning-time-s 0.8 \
  --call-timeout-s 3.0 \
  --joint-tolerance-rad 0.001 \
  --reach-tolerance-m 0.04 \
  --tcp-z-min -0.05 \
  --tcp-z-max 0.22 \
  --publish-hz 60 \
  --playback-scale 0.10 \
  --min-point-dt-s 0.003 \
  --dwell-s 0.01 \
  --settle-s 0 \
  --final-hold-s 0.08 \
  --checkpoint-every 25
```
