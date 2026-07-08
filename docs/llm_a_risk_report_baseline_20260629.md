# LLM-A Risk Report Baseline

Date: 2026-06-29

## Scope

LLM-A v1 is a deterministic report-generation baseline. It reads an existing
`episode_report.json` and writes a Markdown report plus structured JSON report.

It does not:

- start ROS
- modify P4-V safety code
- rewrite or backfill P4-X evidence
- access the real bus-servo controller
- call an online LLM/API
- issue control commands

## Tool

```text
tools/generate_llm_a_risk_report.py
```

Example usage:

```powershell
python tools\generate_llm_a_risk_report.py --episode-report outputs\p4x_d435_hold_capture_v1\episode_report.json --output-dir outputs\llm_a_risk_report_v1\p4x

python tools\generate_llm_a_risk_report.py --episode-report outputs\arm_a_mock_remove_obstacle_v1\episode_report.json --output-dir outputs\llm_a_risk_report_v1\arm_a

python tools\generate_llm_a_risk_report.py --episode-report outputs\step7e2_guarded_motion_red_rule_flow_v1\e2_guarded_red_rule_arm_hw_fastdemo_002\episode_report.json --output-dir outputs\step7e2_guarded_motion_red_rule_flow_v1\e2_guarded_red_rule_arm_hw_fastdemo_002\llm_a_report
```

Each output directory contains:

- `risk_report.md`
- `risk_report.json`
- `claim_boundary.md`
- `README.md`

## Report Sections

The Markdown report includes:

- Episode Summary
- Safety Summary
- Action Trace
- Evidence Summary
- Risk Point Summary
- Claim Boundary
- Next Recommended Step

The JSON report keeps the same information in structured form for later UI or
real LLM integration.

## Claim Boundaries

For P4-X reports, LLM-A may only claim safe stationary visual evidence capture.
It must not claim visual detection accuracy, arm manipulation, or autonomous
semantic reasoning.

For Arm-A mock reports, LLM-A may only claim mock `ARM_REMOVE_OBSTACLE` action
chain validation. It must not claim real mechanical-arm control or bus-servo
hardware validation.

For Step7-E2 reports, LLM-A may only claim guarded motion through the existing
P4/N10P safety chain, deterministic D435 HSV red-rule trigger, approximate
risk map projection, and no-load arm response or dry-run status as recorded in
the source episode report. It must not claim trained visual recognition
accuracy, autonomous navigation, path planning, high-precision SLAM, grasping,
contact, payload handling, obstacle clearing, or LLM control of the robot.

## Missing Fields

The generator does not infer or backfill missing fields. If `depth_scale_m` or
`bbox_valid_depth_ratio` is absent from a risk point, the report marks it as
missing rather than deriving it from notes or sibling records.

## Next Steps

- P4-X: run P4-X3 real ROS/D435 validation for header/frame/depth-ratio fields.
- Arm-A: before real arm integration, run bus-servo dry-run / no-load tests.
- Step7-E2: keep the fastdemo reproduction command and acceptance evidence
  frozen before recording teacher-facing demo video.
