# Firmware Artifacts

This directory contains STM32 hex artifacts used during C30D Tank bring-up.

## Files

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
