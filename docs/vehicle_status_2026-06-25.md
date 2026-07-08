# Vehicle Status - 2026-06-25

## Platform

- Main board: SpacemiT K1 Muse Pi Pro
- OS: Bianbu LXQT v2.3.3 class image
- Kernel observed: Linux 6.6.63 riscv64
- ROS: ROS 2 Humble
- Board user: `soc`
- Active SSH address during bring-up: `192.168.43.40`

No passwords, API keys, or proxy subscription links are stored in this repo.

## Chassis

- Chassis: WHEELTEC L150Pro Tank
- Controller: C30D
- C30D display/mode: Tank
- ROS serial device: `/dev/base_controller`
- Current symlink observed: `/dev/base_controller -> ttyACM1`
- Baud: `115200`
- Firmware: ROS protocol firmware with stop watchdog behavior

Ground-tested command profile:

- Linear speed limit: `0.45 m/s`
- Angular speed limit: `2.40 rad/s`
- Send rate: `50 Hz`
- Command timeout: `0.25 s`
- Reverse brake ratio: `1.0`
- Reverse brake duration: `min(motion_duration, 0.50 s)`
- Zero speed is sent continuously after braking when no fresh `/cmd_vel` is received.

Validated ground tests:

- Forward: correct direction, about 11 cm for `0.45 m/s` during `0.5 s`
- Backward: correct direction
- Left turn: correct direction
- Right turn: correct direction
- Braking: stable with equal reverse command and zero hold

## N10P Lidar

- Sensor: Leishen/WHEELTEC N10P/N10Plus class lidar
- Serial baud: `460800`
- ROS driver: `lslidar_driver`
- Previous runtime observation: serial opened successfully and lidar output rate was around 12 Hz.
- Mapping launch now uses N10P + Tank base + `slam_toolbox`.

## Intel RealSense D435

- D435 RGB/depth capture has been verified.
- Depth quality still needs a longer controlled scene test.
- Recommended cable: real USB 3.x data cable, Type-C to host USB 3.x, not a charge-only phone cable.
- Mechanical mount thread: `1/4-20 UNC`.
- Dedicated bring-up notes: `docs/d435_realsense.md`.

## Bus Servos

- Bus servo controller tested.
- Stable path: controller micro-B connection.
- USART path required TX-to-controller-RX and common GND before it accepted commands.
- External power: 12 V on controller side; logic/reference wiring must share GND.
- Servo IDs observed/configured: 1, 2, 3, 4.
- Safe movement:
  - IDs 1 and 4 are safer for small demonstrations.
  - IDs 2 and 3 can collide and must be moved cautiously.
- Dedicated wiring and smoke-test notes: `docs/bus_servo_controller.md`.
- Smoke-test tool: `tools/arm_bus_servo_smoke.py`.

## Light Control

- Light driver accepts servo-style PWM range: `1100-1900 us`.
- Practical control line: `GPIO_37_3V3`, Linux GPIO `37`.
- `gpio37=0`: light off
- `gpio37=1`: light on
- Hardware PWM7 was mapped to `/sys/class/pwm/pwmchip1`, but low-frequency periods were rejected.
- Current software solution: GPIO37 soft 50 Hz pulse control.
- Smooth brightness script: `tools/gpio37_light_smooth.py`
- Boot-time software guard: `scripts/install_gpio37_boot_low_service.sh`

Known light notes:

- The earlier sinusoidal 50/60 Hz waveform was likely floating/no-common-ground pickup.
- The GPIO line must be held low by software when the light should remain off.
- Real D435 tests showed that this 20 W lamp can overexpose RGB frames at
  10-15 percent depending on angle and distance. Default automatic brightness
  is therefore capped at 5 percent until a diffuser or new lamp angle is used.
- A gate pulldown resistor is still recommended on the MOSFET/driver input.
- Systemd can only pull the pin low after Linux userspace starts; use a hardware
  pulldown if the lamp turns on immediately at power application.

## Power

- A 3S battery previously showed one cell as low as about `2.8 V`.
- Do not continue high-load testing with an imbalanced or damaged pack.
- Light load can be high; do not run at full brightness for long until current and thermal behavior are measured.

## Current Risks

- Long-duration mapping stability still needs testing.
- D435 depth needs better lighting/scene validation.
- Lidar + chassis + lighting together may stress power rails.
- PWM/light driver electrical interface should be finalized with a pulldown and current limit.
- Firmware source changes are documented by artifacts, but the full vendor STM32 source tree is not included here.
