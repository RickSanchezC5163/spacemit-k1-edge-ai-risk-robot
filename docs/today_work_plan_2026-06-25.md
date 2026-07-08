# Today Work Plan - 2026-06-25

Morning bring-up already exposed several hardware and safety issues. The
afternoon plan focuses on a non-arm closed loop that can be tested without
moving the robot automatically.

## Afternoon Goals

- Build passes on K1.
- `non_arm_bringup.launch.py` starts with safe defaults.
- Mock event input produces risk outputs.
- `event_logger_node` writes JSONL logs.
- `gpio37_light_node` is available as a ROS2 node.
- GPIO37 boot low guard remains installed and verified.

## Afternoon Order

1. Confirm battery and light power path.
2. Pull or upload this branch to the K1.
3. Build:

```bash
cd ~/edge-ai-robot-k1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

4. Start static stack:

```bash
cd ~/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch k1_system_bringup non_arm_bringup.launch.py
```

5. Publish mock events and verify risk topics.
6. Check `logs/events/*.jsonl`.
7. Test light brightness 0/30/60 only if power and temperature are safe.

## Evening Goals

- Draft local LLM deployment options.
- Define first risk dataset labels and JSON formats.
- Define safe hazard scene simulation plan.
- Write tomorrow test checklist.

## Safety Notes

- No automatic `/cmd_vel`.
- No arm action.
- No servo movement.
- Light starts off and must be turned off after tests.
