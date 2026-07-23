# Discrete Arm Clearance Implementation Audit - 2026-07-23

## Scope

This audit covers the first fail-closed implementation of:

```text
repeatable arm observation pose
-> near-camera object verification
-> LEFT/CENTER/RIGHT classification with reject bands
-> complete pre-verified sequence selection
-> whole-sequence ArmSafety preflight
-> optional one-shot supervised hardware execution
```

It does not implement visual servoing, pulse interpolation, arbitrary inverse
kinematics, automatic retry, or automatic home after a fault.

The near camera is mounted at the arm tip. Its configured mode is therefore
`ARM_MOVING_FIXED_OBSERVATION`, not `BODY_FIXED`. Pixel regions and marker
coordinates are valid only after the base is locked and the arm has reached
the calibrated observation pose. They must not be reused after the arm leaves
that pose.

## Autonomous Mission Handoff

The upstream mission uses SLAM with Frontier/RRT goal selection and Nav2 path
execution. When autonomous exploration raises an obstacle-clearance event,
the required handoff is:

```text
autonomous exploration reports an obstacle-clearance event
-> Nav2 goal cancellation and STOP_SAFE
-> fresh base-zero confirmation
-> pause D435 inference
-> move arm and arm-tip camera to the calibrated observation pose
-> discard warm-up frames
-> run near-camera YOLO class verification and marker localization
-> select LEFT/CENTER/RIGHT or reject
```

Earlier D435 detections may be retained as coarse exploration evidence, but they do
not satisfy the close-range classification gate. The obstacle announcement
must either be based on that earlier valid evidence or occur after the arm-tip
camera verifies the object. Autonomous navigation must remain paused while
this handoff is in progress.

## Reusable Existing Code

### USB capture and YOLO

`tools/run_real_k1_risk_approach_from_event.py` already provides a proven
pattern for:

- opening a V4L2 USB camera;
- discarding warm-up frames;
- saving one close-confirm image;
- invoking `tools/run_yolo_inference_once.py`;
- parsing `risk_detection.json`;
- recording capture and inference failures.

The new script reuses this execution pattern and the existing one-shot YOLO
tool. It does not introduce a camera topic multiplexer.

### Arm validation and frame building

`src/arm_safety.py` provides:

- `ArmSafety.set_phase()`;
- `ArmSafety.validate_multi()` and `validate_all()`;
- soft/hard pulse limits and step-size checks;
- the ID1/ID2 coupled interlock;
- base-zero, driving, heartbeat, and emergency-stop checks;
- review-only and executable servo frames;
- the controller action-group stop frame.

The new path treats failed soft limits as blocking even though the shared
validator currently reports them as warnings. In hardware mode, a failed
workspace estimate is also blocking unless a future explicitly enabled and
evidenced pre-verified override is configured.

### Existing serial helpers

The staged Arm-B/Arm-C scripts contain serial-port audit, voltage-query, and
single-frame write helpers. The new executor keeps one serial handle open for
the selected sequence so it can attempt the reviewed stop frame on a failure.

## Automatic Home Audit

Searches were performed for `move_home`, `send_home`, `return_home`,
`auto_home`, and `finally` in `src/`, `tools/`, and `ros2_ws/`.

Findings:

1. `configs/arm_safety_config.json` contains:
   - `auto_home_on_estop=true`
   - `auto_home_on_heartbeat_loss=true`
2. `src/arm_safety.py` does not consume either field. They are currently
   declarative configuration, not active automatic behavior.
3. Arm-B2, Arm-B3, and Arm-C1 scripts include return-home as a planned normal
   sequence step.
4. No existing arm script was found with an unconditional `move_home()` in a
   generic `finally` block.
5. The new discrete executor never calls home from `except` or `finally`.
   On a runtime failure it stops new steps, attempts the configured controller
   stop frame, records `failed_safe`, and leaves recovery to the operator.

The two `auto_home_*` fields should be changed or removed only after the wider
repository has agreed on their semantics. The new configuration independently
requires `automatic_home_on_fault=false`.

## Safety Configuration Findings

Current relevant fields in `configs/arm_safety_config.json`:

- global hardware, serial, contact, and obstacle-removal gates default false;
- gripper `gripper_open_pulse=0` conflicts with its soft limit `[100, 900]`;
- ID1 rotation requires ID2 to remain at or above 600 pulse;
- emergency stop state blocks subsequent validation;
- heartbeat timeout is 2 seconds after a heartbeat has first been supplied;
- workspace failure is currently a warning in the shared validator;
- controller stop command is available as action-group command 7;
- no joint-position feedback loop is currently implemented by `ArmSafety`.

The gripper conflict is not auto-corrected. It blocks discrete-sequence audit
until a real calibrated open pulse is written after supervised testing.

## Added Files

```text
src/discrete_arm_clearance.py
tools/run_real_k1_discrete_arm_clearance.py
configs/arm_discrete_clearance_config.json
configs/arm_discrete_sequences/left.json
configs/arm_discrete_sequences/center.json
configs/arm_discrete_sequences/right.json
tests/test_discrete_arm_clearance.py
docs/discrete_arm_clearance_implementation_audit_20260723.md
```

The three sequence files contain no pulse placeholders. They are deliberately
empty and marked `verified=false`, so they cannot be executed before field
calibration.

## Modified File

`tools/generate_moveit_gripper_sim_pick_place_plan.py` now emits:

- `moveit_joint_targets_rad`: canonical field for the current estimated radian targets;
- `moveit_joint_targets_rad_calibration`: an explicit warning that the conversion
  is not hardware-calibrated;
- `legacy_moveit_joint_targets_pulse`: explicit legacy pulse values;
- the old `moveit_joint_targets` field with deprecation and unit metadata for
  compatibility.

## Commands

All commands run from the repository root.

### 1. Configuration/dry-run audit

```bash
python3 tools/run_real_k1_discrete_arm_clearance.py audit \
  --output outputs/arm_clearance/config_audit/audit.json
```

The committed configuration intentionally returns `blocked_unconfigured`
until the camera ROIs, reject bands, HSV marker range, gripper pulse, and
sequences have been calibrated.

After CENTER calibration, a no-hardware sequence dry-run is:

```bash
python3 tools/run_real_k1_discrete_arm_clearance.py run \
  --grid CENTER \
  --manual-grid-for-no-load \
  --execution-stage no_load \
  --output-dir outputs/arm_clearance/center_dryrun
```

### 2. Camera-only three-grid classification

```bash
python3 tools/run_real_k1_discrete_arm_clearance.py classify \
  --base-zero-evidence outputs/live_base_zero_evidence.json \
  --confirm-arm-at-safe-observation \
  --confirm-d435-inference-paused \
  --output-dir outputs/arm_clearance/camera_three_grid
```

For offline frame development, repeat `--input-image` with at least the
configured number of frames. Marker-only development can add
`--skip-yolo-development`; that result is marked `DEVELOPMENT_ONLY` and cannot
be used for hardware execution.

### 3. CENTER no-load plan from accepted camera evidence

```bash
python3 tools/run_real_k1_discrete_arm_clearance.py run \
  --detection-evidence outputs/arm_clearance/camera_three_grid/near_detection.json \
  --execution-stage no_load \
  --output-dir outputs/arm_clearance/center_no_load_plan
```

### 4. One-shot no-load hardware execution

This remains blocked until all calibration fields are completed,
`hardware.enabled=true` is deliberately reviewed, the CENTER sequence is
verified, and fresh live base-zero evidence exists.

```bash
python3 tools/run_real_k1_discrete_arm_clearance.py run \
  --detection-evidence outputs/arm_clearance/camera_three_grid/near_detection.json \
  --execution-stage no_load \
  --base-zero-evidence outputs/live_base_zero_evidence.json \
  --serial-port /dev/arm_bus \
  --enable-hardware-write \
  --confirm-discrete-clearance-once \
  --confirm-operator-supervision \
  --confirm-d435-inference-paused \
  --confirm-stop-interface-ready \
  --confirm-no-auto-home \
  --confirmation-text I_UNDERSTAND_THIS_MOVES_THE_ARM_ONCE \
  --output-dir outputs/arm_clearance/center_no_load_once
```

For a later supervised foam stage, change `--execution-stage` to
`foam_contact` and add `--confirm-foam-only`. Execution pauses after
`short_lift` and requires the operator to type `GRASP_CONFIRMED`. No automatic
retry or automatic release is performed.

## Evidence Layout

Classification output:

```text
<output_dir>/near_before.jpg
<output_dir>/near_detection.json
<output_dir>/yolo/risk_detection.json
```

Sequence audit/execution output:

```text
<output_dir>/near_detection.json
<output_dir>/selected_sequence.json
<output_dir>/plan.json
<output_dir>/execution_log.jsonl
<output_dir>/result.json
```

`execution_log.jsonl` is created only after real serial execution begins.

## Current Blocking Items

1. The near camera is known to be arm-tip mounted, but its calibrated
   observation pose and stable device identity are not frozen.
2. HSV marker limits and image ROIs are uncalibrated.
3. LEFT/CENTER/RIGHT reject bands are uncalibrated.
4. All three arm sequences are empty and unverified.
5. The real gripper open pulse conflicts with the committed soft limit.
6. Real pulse-to-joint calibration remains incomplete.
7. Full link/vehicle/D435/N10P collision validation is not available.
8. The stop command exists, but its behavior during a direct servo move still
   requires a supervised hardware validation.
9. Live joint-position, current, and temperature feedback are not integrated.
10. Simultaneous D435 and USB camera bandwidth/power stability is unverified.
11. Autonomous-mission-to-arm-observation-pose orchestration is not implemented
    by this standalone tool; the caller must cancel navigation, lock the base,
    place the arm-tip camera at the reviewed observation pose, and supply fresh
    base-zero evidence.
12. The new hardware path also remains blocked by
    `validated_stop_interface=false` and `live_base_monitor_verified=false`.

These blockers intentionally prevent the committed repository from moving the
arm through the new path.

## Automated Verification

```bash
python3 -m unittest discover -s tests -p "test_discrete_arm_clearance.py" -v
python3 -m py_compile \
  src/discrete_arm_clearance.py \
  tools/run_real_k1_discrete_arm_clearance.py \
  tools/generate_moveit_gripper_sim_pick_place_plan.py
```

The tests cover three-grid classification, reject bands, marker stability,
YOLO single-object gating, placeholder rejection, live base-zero evidence,
ArmSafety sequence preflight, and a calibrated-fixture CENTER dry-run.
