# Step7-C Guarded D435 Mock-Risk Arm No-Load Validation

## Scope

Step7-C connects the live guarded stationary evidence chain into the map-gated
arm no-load response path:

```text
guarded stack live base-zero evidence
-> D435 live HOLD_CAPTURE
-> deterministic mock risk point
-> Map-A0 live projection
-> Arm-C0 map-gated candidate
-> Arm-C1 no-load gate
-> deterministic LLM-A report
```

The default mode is Step7-C0 dry-run. In that mode the Arm-C1 gate is executed
without serial access or hardware motion.

Step7-C1 hardware mode is a separate, explicitly confirmed one-shot no-load
run. It must not be used for contact, grasping, payload handling, or obstacle
removal.

## Runner

```powershell
python tools\run_step7c_guarded_d435_mockrisk_arm_noload.py --dry-run-arm
```

Default output root:

```text
outputs/step7c_guarded_d435_mockrisk_arm_noload_v1/
```

The runner creates the next available directory such as:

```text
dryrun_001/
hw_001/
```

Each run writes:

```text
episode_report.json
step7c_report.md
base_zero_live/
d435_hold_capture/
mock_risk/
map_projection/
arm_candidate/
arm_execution/
llm_a_report/
errors.json
README.md
```

## Step7-C0 Dry-Run Acceptance

- `base_zero_ok_before_capture=true`
- `d435_live_capture_executed=true`
- `risk_point_generated=true`
- `mock_risk_triggered=true`
- `risk_map_points>=1`
- `arm_candidate_selected=true`
- `hardware_executed=false`
- `serial_port_opened=false`
- `serial_bytes_written=0`
- `published_cmd_vel=false`
- `contact_allowed=false`
- `obstacle_removed=false`
- `errors=[]`
- LLM-A deterministic report generated

## Step7-C1 Hardware Gate

Hardware no-load mode must be run only once per validation and requires all
explicit flags:

```powershell
python tools\run_step7c_guarded_d435_mockrisk_arm_noload.py `
  --enable-hardware-write `
  --confirm-map-gated-no-load `
  --confirm-no-contact `
  --confirm-base-zero-live `
  --confirm-no-cmd-vel
```

The selected arm response remains:

```text
selected_sequence=arm_b3_8_step_safety_adjusted_no_load_sample
```

It must finish at the planned safe idle/home-like 6b pose.

## Claim Boundary

Allowed:

- guarded stationary live integration
- D435 evidence capture
- deterministic mock anomaly trigger
- approximate Map-A0 risk point projection
- map-gated no-load arm candidate dry-run
- one explicitly confirmed no-load hardware response only when evidence shows
  `hardware_executed=true`
- deterministic LLM-A report generation from `episode_report.json`

Disallowed:

- autonomous navigation
- path planning success
- SLAM or high-precision map claims
- real visual detection accuracy
- grasping
- contact
- payload handling
- physical obstacle removal
- LLM control of the robot

## Next Step

Run Step7-C0 first and freeze its evidence. Step7-C1 should only be attempted
after C0 passes and the operator confirms no-contact/no-load hardware
conditions at the vehicle.
