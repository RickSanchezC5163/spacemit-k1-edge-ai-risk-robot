# Arm ROS 与总线舵机通信协议研究

日期：2026-06-30

## 结论

总线舵机控制器本身不是 ROS 协议。正确架构应是：

```text
Policy / Arm action
  -> ROS 2 arm executor node
  -> safety gate + base_zero check
  -> Lobot bus-servo serial frame
  -> bus servo controller
```

也就是说，ROS 只负责动作请求、状态反馈和证据记录；真正发给控制器的是串口二进制帧。

当前阶段建议先做 Arm-B no-load 验证链路，形态对齐 P4-X/D435：

- 先做 protocol/frame audit，不发串口。
- 再做只读串口/设备 audit。
- 再做单舵机小角度 no-load。
- 最后做这组 8 步 sample action no-load。

不要直接把机械臂接入自动清障闭环。

## 资料来源

本次只读取本地资料，不连接硬件、不启动 ROS、不打开串口。

资料目录：

```text
K:\risc-vCar\总线舵机控制器
```

关键示例：

```text
1.教程资料/2.总线舵机控制器二次开发教程/06 Jetson版本开发/02 源码教程/案例3 控制多个舵机转动/ServoControl.py
1.教程资料/2.总线舵机控制器二次开发教程/06 Jetson版本开发/02 源码教程/案例3 控制多个舵机转动/BusServoMoveByArray.py
```

## Lobot 串口协议

Jetson 示例中默认串口：

```text
/dev/ttyTHS1
baudrate = 9600
```

K1 上真实端口仍需单独确认，候选为：

```text
/dev/ttyS1
/dev/ttyUSB0
/dev/ttyTHS1
```

总线舵机多舵机同步移动帧：

```text
55 55 <len> 03 <count> <time_lo> <time_hi> [<id> <pulse_lo> <pulse_hi>]...
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `55 55` | 帧头 |
| `len` | 数据长度，等于 `servo_count * 3 + 5` |
| `03` | `CMD_SERVO_MOVE` |
| `count` | 本帧控制的舵机数量 |
| `time_lo time_hi` | 小端序移动时间，单位 ms |
| `id` | 舵机 ID |
| `pulse_lo pulse_hi` | 小端序 pulse，范围 0-1000 |

单舵机移动也是同一条 `CMD_SERVO_MOVE`，只是 `count=1`。

动作组命令也存在，但当前不建议优先使用：

| 命令 | 值 | 说明 |
| --- | ---: | --- |
| `CMD_ACTION_GROUP_RUN` | 6 | 控制器内置动作组运行 |
| `CMD_ACTION_GROUP_STOP` | 7 | 停止动作组 |
| `CMD_ACTION_GROUP_SPEED` | 11 | 设置动作组速度 |

当前建议使用 ROS 侧逐步下发多舵机同步帧，因为这样每一步都能写 ActionResult 和 evidence。

## ROS 2 包装建议

第一版不建议自定义复杂 ROS msg。为了 8 天交付，建议使用“标准 ROS transport + JSON payload”：

### 输入

Topic 或 service：

```text
/arm/action_request
type: std_msgs/msg/String
payload: JSON
```

示例：

```json
{
  "action_id": "arm_b_sample_no_load_001",
  "action_type": "ARM_SAMPLE_NO_LOAD",
  "requires_base_zero": true,
  "publishes_cmd_vel": false,
  "dry_run": false,
  "steps": [
    {"label": "step_1", "time_ms": 1500, "servos": {"1": 499, "2": 770, "3": 457, "4": 500, "5": 494}}
  ]
}
```

### 输出

Topic：

```text
/arm/action_result
type: std_msgs/msg/String
payload: JSON
```

必须包含：

- `action_id`
- `action_type`
- `status`
- `base_zero_ok_before`
- `published_cmd_vel=false`
- `hardware_executed`
- `serial_port`
- `serial_bytes_written`
- `step_results[]`
- `errors[]`

### 状态

Topic：

```text
/arm/status
type: std_msgs/msg/String
payload: JSON
```

建议包含：

- 当前 phase
- serial connected
- last action id
- estop state
- heartbeat state
- base_zero state

## 必须保留的安全门

执行任何真实串口写入前，必须同时满足：

- `requires_base_zero=true`
- `base_zero_ok_before=true`
- robot not driving
- `published_cmd_vel=false`
- arm phase 允许 hardware access
- 顶层 global safety gate 允许 hardware access
- serial write gate 允许
- action 明确 no-load 或更高阶段
- E-stop 未触发
- heartbeat 正常
- 每一步 pulse 在 soft limit 内
- 每一步 delta 在限制内

当前 `src/arm_safety.py` 已经能构造 Lobot 帧，但还需要先修正一个安全语义：

```text
global safety_gates 必须作为 phase_gates 的 AND gate。
```

否则 B2/B3 phase 里 `serial_write_allowed=true` 时，代码可以构造硬件帧。虽然当前 dry-run 脚本没有串口写入，但正式 ROS 节点前必须修。

## 你的 8 步动作样例

当前样例适合作为 no-load sample action 候选：

```text
1: 1500ms  #1 P499 #2 P770 #3 P457 #4 P500 #5 P494
2: 1500ms  #1 P498 #2 P600 #3 P540 #4 P470 #5 P498
3a: 1500ms #1 P498 #2 P400 #3 P590 #4 P470 #5 P496
3b: 2000ms #1 P498 #2 P250 #3 P646 #4 P470 #5 P494
4: 1500ms  #1 P498 #2 P291 #3 P644 #4 P470 #5 P495
5: 1500ms  #1 P498 #2 P290 #3 P642 #4 P470 #5 P220
6a: 1500ms #1 P498 #2 P500 #3 P540 #4 P470 #5 P360
6b: 2000ms #1 P510 #2 P771 #3 P426 #4 P503 #5 P497
```

其中 6b 同时作为机械臂未启动或动作结束后的安全保持姿态：

```text
safe_idle_home_like_6b = #1 P510 #2 P771 #3 P426 #4 P503 #5 P497
```

`configs/arm_safety_config.json` 中各关节 `home_pulse` 应与该姿态一致。`pulse_center=500` 只表示舵机数值中位，不再表示 K1 机械臂安全 home 姿态。

注意：B2 小角度测试不能简单理解成每个关节都围绕 home 做 `±100 pulse`。ID2 的 6b home 已经贴近上位机安装板，正方向 `home+100` 不安全，因此 B2 只允许 ID2 做 `home-100` 方向测试。配置中通过 `test_offsets_from_home` 显式列出允许方向，通过 `blocked_offsets_from_home` 记录被禁用方向。

补充机械耦合约束：

- ID1 旋转只允许在 ID2 至少等于 600 时进行。
- 当 `250 < ID2 < 650` 时，ID3 必须保持在 `400-700`，ID4 必须保持在 `300-470`。

因此 sample action 的低 ID2 段已经调整为受约束版本：ID2 低于 600 时 ID1 不再改变；ID2 在 250-650 区间时 ID4 不超过 470。

静态观察：

- 舵机 ID 集合完整：1-5。
- pulse 均在 0-1000。
- time 均在 0-30000 ms。
- 最大相邻步进 delta 为 275，小于当前 300 pulse 限制。
- `3b` 已将 ID2 从 186 改成 250，比原始候选更保守。

因此它可以作为：

```text
ARM_SAMPLE_NO_LOAD
```

不能作为：

```text
ARM_REMOVE_OBSTACLE
```

除非后续完成真实障碍物验证。

## 类似 P4-X/D435 的 Arm-B 验证链路

建议输出目录：

```text
outputs/arm_b_bus_servo_validation_v1/
```

### Arm-B0: protocol frame audit

不连接硬件。

输出：

- `arm_b_sample_action_frame_audit.json`
- `arm_b_sample_action_frame_audit.md`
- `arm_b_sample_action_frame_audit.csv`
- `errors.json`

验收：

- 能生成每一步 Lobot frame hex。
- 每一步 pulse/time/id 通过静态检查。
- `hardware_executed=false`
- `serial_bytes_written=0`

### Arm-B1: serial device audit

只读检查设备，不写串口。

输出：

- `arm_b_serial_audit.json`
- `arm_b_serial_audit.md`
- `errors.json`

验收：

- 找到候选串口。
- 记录权限、用户组、波特率配置。
- 不发送控制帧。

### Arm-B2: single-servo no-load test

真实硬件，小角度，单舵机。

要求：

- base_zero 前置成立。
- 不发布 `cmd_vel`。
- 一次只动一个舵机。
- 每次动作写 ActionResult。
- 失败必须 `failed_safe`。

### Arm-B3: sample no-load action

真实硬件，多舵机同步帧，但无负载、无接触。

要求：

- 使用上面的 8 步动作。
- 执行前必须回安全 home-like 姿态。
- 10 次连续验证可以作为后续目标，但第一次只做 1 次人工监督。
- 每一步记录 frame、时间、状态、错误。

输出：

- `episode_report.json`
- `arm_b_no_load_status.csv`
- `errors.json`
- 每次动作目录下保存 `action_result.json`

## Claim Boundary

完成 Arm-B0 只能声明：

- 已研究 Lobot 总线舵机串口帧格式。
- 已完成 sample action 的 dry-run 帧生成和静态安全检查。

不能声明：

- 真实机械臂动作成功。
- ROS 硬件闭环已完成。
- 清障能力。

完成 Arm-B3 后也只能声明：

- no-load sample action 可在安全约束下执行。

仍不能声明：

- 真实清障。
- 有负载抓取。
- 自动语义决策。
