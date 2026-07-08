# Arm-B0 串口设备审计与 dry-run 安全门禁验证

日期：2026-06-30

## 结论

Arm-B0 通过。

Arm-B0 定义为：

```text
K1 SSH 在线
+ 串口设备只读审计
+ dry-run 动作帧审计
+ global safety gate 覆盖 phase-level serial write permission
+ serial_bytes_written=0
```

## K1 只读检查

- K1 在线：PASS
- 主机名：`nwpu-soc`
- 架构：`riscv64`
- 用户：`soc`
- Python：`3.12.3`
- `pyserial`：available
- ROS / realsense / wheeltec / servo 相关进程：未发现

## 串口设备审计

候选设备：

```text
/dev/ttyUSB0
/dev/ttyS0
/dev/ttyS2
```

确认的 USB 串口：

```text
/dev/ttyUSB0
vendor: 1a86
model: USB_Serial
driver: ch341
database: QinHeng Electronics CH340 serial converter
permission: root:dialout 660
```

`soc` 用户在 `dialout` 组内。未发现 `/dev/ttyUSB0` 被进程占用。

## Dry-Run 验证

K1 上执行：

```bash
python3 -m py_compile src/arm_safety.py tools/arm_b_sample_action_frame_audit.py tools/generate_arm_b_no_load_dry_run_plan.py
python3 tools/arm_b_sample_action_frame_audit.py
python3 tools/generate_arm_b_no_load_dry_run_plan.py
```

结果：

- `arm_b_sample_action_frame_audit`: `all_valid=True`
- `max_step_delta=275`
- `errors.json=[]`
- `hardware_executed=false`
- `serial_bytes_written=0`
- Arm-B2：9 个单舵机小角度 dry-run 序列 PASS
- Arm-B3：5 个 no-load dry-run 序列 PASS
- Arm-B1 plan-only：phase gate 预期拦截

## Safety Gate 验证

K1 上确认：

```text
arm_b2_single_joint_small_angle:
  arm_enabled=True
  serial_write_allowed_global=False
  serial_write_allowed_phase=True
  serial_write_allowed_effective=False
  frame_bytes_is_none=True

arm_b3_full_no_load_sequence:
  arm_enabled=True
  serial_write_allowed_global=False
  serial_write_allowed_phase=True
  serial_write_allowed_effective=False
  frame_bytes_is_none=True
```

这说明 phase gate 不能绕过 global safety gate。当前默认配置不会真实写串口。

## Claim Boundary

可以声明：

- K1 可 SSH 连接。
- CH340 串口设备可见。
- 当前用户具备串口权限组。
- Arm-B dry-run 安全审计通过。
- global safety gate 正确阻止串口写入。

不能声明：

- 真实机械臂已经动作。
- ROS arm executor 已经运行。
- 6b home 已经通过 K1 串口发送。
- 任何抓取、接触或清障能力。

## 下一步

Arm-B1：

```text
send safe idle home 6b once
```

必须使用专用脚本，并同时带：

```bash
--enable-hardware-write --confirm-send-home-6b
```

默认配置仍保持 `serial_write_allowed=false`。
