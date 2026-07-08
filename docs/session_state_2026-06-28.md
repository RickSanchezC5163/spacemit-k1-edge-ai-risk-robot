# Session State 2026-06-28

## Current Capability

- N10P `/scan` + Python tank base `/odom` + `base_footprint -> laser` + `slam_toolbox` can generate and save a map.
- Manual guarded mapping is the current safe operating mode.
- Nav2/RRT autonomous exploration remains paused.

## Current Safety Position

- Do not run Nav2 or RRT without a human guard.
- Do not treat `/cmd_vel_guarded = 0` alone as a successful stop.
- A valid stop chain is:
  - motion command
  - STOP_REQUEST or active zero command
  - base node `stop_kick_start`
  - reverse brake command
  - `stop_kick_end`
  - serial command returns to zero
  - feedback velocity approaches zero

## Tank Base Parameters Used In Latest Test

```text
cmd_vel_topic=/cmd_vel_guarded
stop_request_topic=/chassis/stop_request
max_linear=0.45
max_angular=0.80
cruise_linear_limit=0.45
cruise_angular_limit=0.80
brake_duration=0.30
stop_kick_match_cmd=true
stop_kick_match_duration=false
stop_kick_speed_gain=1.50
stop_kick_duration_mode=fixed
stop_kick_duration=0.55
stop_kick_max_duration=0.75
stop_kick_min_duration=0.12
stop_kick_until_stopped=false
odom_linear_scale=60.0
odom_angular_scale=1.0
```

## Latest Mapping Result

Initial map without motion:

```text
maps_preview/mapping_static_20260628_031259.pgm
size: 114 x 102
resolution: 0.05 m/pixel
```

After adding `odom_linear_scale=60.0` and running a forward mapping test:

```text
maps_preview/mapping_scaled_forward_20260628_032434.pgm
size: 117 x 225
resolution: 0.05 m/pixel
```

The map size changed, so `slam_toolbox` accepted odometry motion after scaling.

## Latest Motion Test Result

Command:

```text
0.30 m/s x 2.0 s
primary brake: -0.45 m/s x 0.55 s
supplemental brake: up to 2 x (-0.45 m/s x 0.22 s)
```

Observed scan data:

```text
start front_p10:        3.296 m
STOP_REQUEST front_p10: 2.749 m
final front_p10:        0.150 m
approach before stop:   0.547 m
post-stop approach:     2.599 m
```

Conclusion:

- Mapping updated, but the 2 second forward run was not safe enough.
- The current brake setup is acceptable for shorter motion but not for longer mapping runs.
- Future mapping should use shorter segments or a stronger validated stop model before expanding the map.

## Odom Finding

Previous wall calibration showed raw odom underreports real forward motion by roughly 60x in the forward segment. `odom_linear_scale=60.0` is a temporary engineering correction for SLAM testing, not a final calibrated model.

## Next Session

1. Confirm robot is stationary and reconnect SSH.
2. Keep `odom_linear_scale=60.0` for SLAM experiments unless recalibrated.
3. Re-test braking using shorter mapping segments:

```text
0.30 m/s x 0.8 s
0.30 m/s x 1.0 s
```

4. Save maps after each successful segment.
5. Only after reliable stop distance is measured, resume guarded mapping expansion.

