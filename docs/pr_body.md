# Summary

This PR adds the current K1 Muse Pi Pro edge robot bring-up snapshot for the private competition repository.

It includes:

- ROS 2 Humble base driver package for the WHEELTEC Tank/C30D chassis.
- Tested brake-profile launch file for short motion commands and active reverse braking.
- N10P lidar + tank base mapping launch scaffold.
- D435 RealSense bring-up and quality retest notes.
- Bus servo controller wiring and smoke-test tool.
- GPIO37/PWM7 light control tools, including smooth brightness ramp control.
- Current vehicle status notes and bring-up command reference.
- Current safe-stop STM32 firmware artifacts used during chassis testing.

# Why

The robot has reached a usable hardware bring-up checkpoint. Capturing this state now gives the team a recoverable baseline before continuing with mapping, lighting integration, D435 perception, servo arm behavior, and edge AI deployment.

# Impact

- Keeps verified chassis, lidar, light, and firmware state in version control.
- Provides reproducible commands for the Bianbu LXQT v2.3.3 / ROS 2 Humble environment.
- Separates deployment-critical files from vendor archives and temporary local experiments.

# Validation

- C30D Tank chassis tested on ground with forward, backward, left, and right commands.
- Stable stop profile confirmed with `0.45 m/s` linear motion, `2.4 rad/s` angular motion, equal-duration reverse braking, and zero hold.
- N10P lidar was previously verified publishing data at about 12 Hz.
- D435 RGB/depth capture was verified, though depth quality still needs later evaluation.
- Bus servo controller accepted commands after common GND and TX/RX wiring were fixed.
- GPIO37 light control was verified for on/off and smooth brightness adjustment.

# Notes

Do not commit board passwords, proxy subscription URLs, API keys, or private network credentials.
