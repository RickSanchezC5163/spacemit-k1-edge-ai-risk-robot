# Yaw Angle Source Audit - 2026-06-28

## Question

P4 yaw calibration produced very small yaw deltas:

```text
angular.z=+0.40 x 1.0s -> +0.86 deg
angular.z=-0.40 x 1.0s -> -0.11 deg
angular.z=+0.80 x 1.0s -> +4.83 deg
angular.z=-0.80 x 1.0s -> -3.85 deg
```

This audit checks how yaw is calculated and whether the source code has a likely issue.

## ROS Yaw Calculation

File:

```text
ros2_ws/src/turn_on_wheeltec_robot/scripts/wheeltec_tank_base_safe.py
```

Relevant path:

```python
wz = s16_to_float_mmps(frame[6], frame[7])
odom_wz = wz * self.odom_angular_scale
self.yaw += odom_wz * dt
qx, qy, qz, qw = yaw_to_quat(self.yaw)
odom.twist.twist.angular.z = odom_wz
```

`s16_to_float_mmps()` reconstructs a signed 16-bit integer and divides by `1000.0`.
For `frame[6:8]`, the value is angular speed in `rad/s * 1000`, not linear mm/s.

Conclusion for ROS side:

- The yaw integration formula is straightforward: `yaw += wz * dt`.
- Quaternion generation for planar yaw is standard: `z=sin(yaw/2), w=cos(yaw/2)`.
- ROS currently does not integrate IMU `gyro[2]` for yaw.
- ROS yaw depends directly on firmware-reported `Z_speed`.

## Firmware Feedback Chain

Firmware source:

```text
User-provided STM32 firmware source directory:
K:\risc-vCar\<ros-related>\Mini_STM32_source_2025.01.13_GMR
```

### Command Input

`wheeltec_tank_base_safe.py` sends:

```python
vx = int(round(vx_mps * 1000.0))
wz = int(round(wz_radps * 1000.0))
data += bytes_s16(vx)
data += bytes_s16(vy)
data += bytes_s16(wz)
```

The firmware receives `Z_speed` and converts it back:

```c
Move_Z = XYZ_Target_Speed_transition(Receive_Data.buffer[7], Receive_Data.buffer[8]);
```

`XYZ_Target_Speed_transition()` reconstructs signed 16-bit data and divides by `1000`, so command-side angular speed units are consistent.

### Tank Kinematics

`BALANCE/balance.c`:

```c
case Tank_Car:
    MOTOR_A.Target = Vx - Vz * Wheel_spacing / 2.0f;
    MOTOR_B.Target = Vx + Vz * Wheel_spacing / 2.0f;
    MOTOR_C.Target = 0;
    MOTOR_D.Target = 0;
```

For an in-place turn (`Vx=0`), A and B should receive opposite target speeds.

### Encoder To Wheel Speed

`BALANCE/balance.c`:

```c
OriginalEncoder.A = Read_Encoder(2);
OriginalEncoder.B = Read_Encoder(3);
OriginalEncoder.C = Read_Encoder(4);
OriginalEncoder.D = Read_Encoder(5);

case Tank_Car:
    Encoder_A_pr =  OriginalEncoder.A;
    Encoder_B_pr = -OriginalEncoder.B;
    Encoder_C_pr =  OriginalEncoder.C;
    Encoder_D_pr =  OriginalEncoder.D;

MOTOR_A.Encoder = Encoder_A_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
MOTOR_B.Encoder = Encoder_B_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
```

### Firmware Z Speed Output

`HARDWARE/usartx.c`:

```c
case Tank_Car:
    Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder) / 2) * 1000;
    Send_Data.Sensor_Str.Y_speed = 0;
    Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder - MOTOR_A.Encoder) / Wheel_spacing * 1000);
```

`Z_speed` is then packed into the outgoing serial frame:

```c
Send_Data.buffer[6] = Send_Data.Sensor_Str.Z_speed >> 8;
Send_Data.buffer[7] = Send_Data.Sensor_Str.Z_speed;
```

## Critical Source Finding

`BALANCE/system.c` comments define the physical encoder mapping:

```c
// Encoder A is initialized to read the real time speed of motor C
Encoder_Init_TIM2();

// Encoder B is initialized to read the real time speed of motor D
Encoder_Init_TIM3();

// Encoder C is initialized to read the real time speed of motor B
Encoder_Init_TIM4();

// Encoder D is initialized to read the real time speed of motor A
Encoder_Init_TIM5();
```

But `Tank_Car` uses:

```c
Encoder_A_pr =  OriginalEncoder.A;   // TIM2, documented as motor C encoder
Encoder_B_pr = -OriginalEncoder.B;   // TIM3, documented as motor D encoder
```

For this tank chassis, control and odometry use `MOTOR_A` and `MOTOR_B`, but the documented physical encoder channels for those motors are:

```text
motor A encoder -> Encoder D -> TIM5 -> OriginalEncoder.D
motor B encoder -> Encoder C -> TIM4 -> OriginalEncoder.C
```

Therefore the firmware appears to compute `MOTOR_A.Encoder`, `MOTOR_B.Encoder`, and outgoing `Z_speed` from the wrong physical encoder channels in `Tank_Car`.

## Expected Fix Direction

Do not treat this as final without one hardware confirmation run, but the source-backed candidate mapping is:

```c
case Tank_Car:
    Encoder_A_pr =  OriginalEncoder.D;   // TIM5 -> motor A encoder
    Encoder_B_pr = -OriginalEncoder.C;   // TIM4 -> motor B encoder, polarity to verify
    Encoder_C_pr =  OriginalEncoder.A;   // unused by Tank_Car
    Encoder_D_pr =  OriginalEncoder.B;   // unused by Tank_Car
    break;
```

The sign on `Encoder_B_pr` still needs a physical polarity check. Keep the existing negative sign first because the stock `Tank_Car` mapping uses `-OriginalEncoder.B` for the right-side encoder, but verify by running one low-speed forward command and checking that both `MOTOR_A.Encoder` and `MOTOR_B.Encoder` are positive when `Vx > 0`.

## Recommended Verification

Add a temporary raw encoder diagnostic before changing behavior:

```c
if (Car_Mode == Tank_Car) {
    printf("ENC_RAW A_TIM2=%d B_TIM3=%d C_TIM4=%d D_TIM5=%d\r\n",
           OriginalEncoder.A, OriginalEncoder.B,
           OriginalEncoder.C, OriginalEncoder.D);
}
```

Run only no-forward or lifted/supervised tests:

```text
angular.z=+0.40 x 1.0s
angular.z=-0.40 x 1.0s
angular.z=+0.80 x 1.0s
angular.z=-0.80 x 1.0s
```

Expected confirmation:

- Current firmware: large counts on TIM4/TIM5 and weak/zero counts on TIM2/TIM3 would prove the mapping bug.
- After mapping fix: `Z_speed` should track commanded angular speed more closely, and ROS yaw delta should become much larger and more symmetric.

## Decision For P4

Do not enter guarded forward micro mapping until this is fixed or disproven.

The weak yaw calibration is likely not caused by the Python yaw formula. It is most likely caused by firmware `Tank_Car` using encoder channels that the same source tree documents as belonging to unused motor C/D paths.
