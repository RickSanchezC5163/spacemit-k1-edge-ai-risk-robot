# Arm-D Clearance Staging - 2026-07-01

Arm-D is the future obstacle-clearance path. It must not be confused with the
validated Arm-C1 no-load response.

Stages:

- Arm-D0: clearance candidate dry-run
- Arm-D1: no-load clearance trajectory plan
- Arm-D2: soft-object contact test
- Arm-D3: controlled obstacle displacement

Current implementation covers only:

- Arm-D0 dry-run candidate
- Arm-D1 no-load trajectory planning language

Current dry-run boundary:

```text
contact_allowed=false
obstacle_removal_allowed=false
hardware_executed=false
serial_port_opened=false
serial_bytes_written=0
```

D2/D3 require a separate hardware gate, soft-object test plan, emergency stop
procedure, current/temperature observation, and operator confirmation. They are
not validated in this stage.
