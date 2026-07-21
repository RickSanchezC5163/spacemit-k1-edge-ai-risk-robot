# Firmware Artifacts

This directory contains STM32 hex artifacts used during C30D Tank bring-up.

## Files

- `WHEELTEC_tank_pi_reset_20260722.hex`
  - Tested Tank firmware with explicit incremental-PI state and hard-stop reset.
  - Keil rebuild result: `0 Error(s), 0 Warning(s)`.
  - SHA-256: `cb618b304d95ccb6c110943a0b633ff2fc43563299acf00b8c7547bf996e879f`.

- `WHEELTEC_tank_pi_reset_20260722.patch`
  - Reviewable patch against the vendor `BALANCE/balance.c` and
    `BALANCE/balance.h` files.
  - Keeps the PI formula, gains, sign conventions, PWM limit, and integer
    return behavior unchanged.
  - Resets the A/B PI accumulator and error history only at an explicit hard
    stop, and skips the normal PI update for that stop cycle.

- `WHEELTEC_tank_safe_stop_20260624.hex`
  - Named stable artifact after reverting the incorrect Tank PWM-A polarity test.
  - Includes the stop watchdog / zero command handling work used during ground tests.

- `WHEELTEC_current.hex`
  - Latest copied local build artifact at the time this repository snapshot was created.

## Firmware Notes

- The tested C30D mode display is `Tank`.
- The incorrect `MOTOR_A` polarity inversion build made APP direction mapping wrong and should not be used.
- The current usable behavior is:
  - forward/backward directions are correct
  - left/right turn directions are correct
  - equal reverse command followed by zero speed gives stable stopping

The full vendor STM32 source tree is intentionally not stored in this repository.

The 2026-07-22 firmware was validated together with the ROS serial-driver
change that sends security-enable frames once at startup. Repeating those
zero-speed frames during an action would create periodic hard-stop boundaries
and prevent low-speed PI accumulation.
