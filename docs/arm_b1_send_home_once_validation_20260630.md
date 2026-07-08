# Arm-B1 Send Home Once Validation

日期：2026-06-30

## 结论

Arm-B1 已完成 K1 到总线舵机控制器的真实通信与单帧安全归位验证。系统在未启动 ROS、未发布 `cmd_vel`、未执行障碍物接触或夹取的条件下，通过 `/dev/ttyUSB0` 向控制板发送 `safe_idle_home_like_6b` 姿态帧，并在控制板上电后确认机械臂完成归位动作。该阶段仅声明通信链路与单帧 home 动作通过，不声明完整机械臂动作序列、清障能力或自主执行能力。

## 验证范围

- 阶段：Arm-B1
- 动作类型：`ARM_SEND_HOME_ONCE`
- 目标姿态：`safe_idle_home_like_6b`
- 姿态值：`#1 P510 #2 P771 #3 P426 #4 P503 #5 P497`
- 动作时间：`2000 ms`
- 串口：`/dev/ttyUSB0`
- 波特率：`9600`
- 控制器：CH340 USB serial path, `/dev/arm_bus -> ttyUSB0`

## 执行结果

最终上电后执行结果：

- `status=succeeded`
- `dry_run=false`
- `hardware_executed=true`
- `serial_port_opened=true`
- `serial_bytes_written=22`
- `base_zero_ok_before=true`
- `published_cmd_vel=false`
- `errors=[]`

发送帧：

```text
5555140305d00701fe0102030303aa0104f70105f101
```

控制板上电后的非运动电压查询：

- 查询帧：`5555020f`
- 回包：`5555040f122d`
- 解析电压：`11538 mV`
- 结论：`/dev/ttyUSB0 -> 控制板` 通信链路有真实回包。

现场人工确认：

- 初次写入时机械臂无动作，后确认原因是舵机控制板开关未打开。
- 控制板开关打开后，重新执行 Arm-B1，机械臂已回到 `6b` safe idle/home 姿态。

## Evidence

K1 evidence 目录：

```text
/home/soc/edge-ai-robot-k1/outputs/arm_b1_send_home_once_v1/
```

文件清单：

```text
episode_report.json
action_result.json
arm_b1_status.json
sent_frame_hex.txt
errors.json
README.md
physical_observation.json
physical_observation_after_switch_on.json
physical_actuation_confirmation.json
```

关键证据含义：

- `physical_observation.json`：记录第一次串口写入成功但现场无可见动作；后续定位为控制板开关未开。
- `physical_observation_after_switch_on.json`：记录控制板开关打开后电压查询有回包，且只重发一次 6b。
- `physical_actuation_confirmation.json`：记录操作员现场确认机械臂已归位。

## Claim Boundary

本阶段可以声明：

- K1 可以通过 `/dev/ttyUSB0` 与总线舵机控制器通信。
- 控制板上电后，K1 发送的 `safe_idle_home_like_6b` 单帧可以触发机械臂归位。
- Arm-B1 未启动 ROS。
- Arm-B1 未发布 `cmd_vel`。
- Arm-B1 未执行接触、夹取或清障。

本阶段不能声明：

- 不能声明 Arm-B2 单舵机小角度验证已完成。
- 不能声明完整动作组验证已完成。
- 不能声明机械臂可执行障碍物移除。
- 不能声明夹爪接触、抓取或负载能力。
- 不能声明自主语义决策或在线模型控制闭环。

## Arm-B2 Entry Criteria

进入 Arm-B2 前必须保持以下条件：

- 不直接运行完整动作组。
- 每次只测试一个舵机目标动作，并强制回 `6b`。
- 真实写串口必须显式带 `--enable-hardware-write` 和 `--confirm-single-servo-no-load`。
- 每步必须现场确认无异响、无卡滞、无顶死。
- 失败必须写入 `failed_safe`。

建议 B2 顺序：

1. ID5 夹爪：`497 -> 360 -> 497`
2. ID4 腕部：`503 -> 470 -> 503`
3. ID3 肘部：`426 -> 526 -> 426`
4. ID2 肩部：`771 -> 671 -> 771`
5. ID1 底座：仅在 `ID2 >= 600` 时，`510 -> 610 -> 510`

额外安全约束：

- ID2 不允许超过 `771`。
- ID1 旋转必须满足 `ID2 >= 600`。
- 当 `250 < ID2 < 650` 时，ID3 必须在 `400..700`，ID4 必须在 `300..470`。
