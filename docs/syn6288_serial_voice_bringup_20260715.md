# SYN6288 串口语音播报接入记录 - 2026-07-15

## 目标

复赛现场需要在“自主建图 + 风险识别 + 机械臂清障”链路中加入可听见的状态播报。当前采用 SYN6288 串口语音合成模块，K1 通过 USB-TTL 或板载串口发送文本帧，模块本地合成中文语音，不依赖云端 TTS。

目标播报语句：

```text
发现障碍风险，正在生成风险点。
已到达处置距离，底盘锁定。
机械臂开始执行清障动作。
清障完成，正在复核风险状态。
风险报告已生成。
```

## 资料来源

参考资料路径：

```text
K:\chrome\1778751105332853\7.4、STM32F103C8T6应用程序(播报变量例程)\HARDWARE\SYN6288\syn6288.c
K:\chrome\1778751105332853\7.4、STM32F103C8T6应用程序(播报变量例程)\USER\main.c
```

关键协议：

```text
默认波特率：9600 bps
帧格式：FD LEN_H LEN_L CMD PARAM TEXT... XOR
CMD：0x01 表示合成播放
PARAM：0x01 | (music << 4)
LEN：len(TEXT) + 3
XOR：从 FD 到 TEXT 末尾逐字节异或
文本前缀示例：[d][V12][m15][t5]
```

## 接线

推荐先使用 USB-TTL 转串口模块调试：

```text
SYN6288 VCC  -> 5V
SYN6288 GND  -> K1/USB-TTL GND
SYN6288 RXD  -> USB-TTL TXD
SYN6288 TXD  -> USB-TTL RXD
```

注意：

- 必须共地。
- 先确认模块供电电压和串口电平；USB-TTL 优先使用 3.3V TTL 信号。
- 下载/烧录其他单片机程序时不要把 SYN6288 接在同一串口上。

## 文件

新增工具：

```text
tools/syn6288_serial_tts.py
tools/prelim_voice_event_bridge.py
tools/start_prelim_syn6288_voice_k1.sh
```

`syn6288_serial_tts.py` 负责直接构造 SYN6288 帧并写串口。  
`prelim_voice_event_bridge.py` 是 ROS2 节点，订阅风险报警和阶段 cue。  
`start_prelim_syn6288_voice_k1.sh` 是 K1 上的启动脚本。

## 单独调试

在 K1 上确认串口：

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
dmesg | tail -50
```

确认 Python 串口库：

```bash
python3 -c "import serial; print(serial.__file__)" || sudo apt install -y python3-serial
```

如果没有权限，临时用 `sudo` 运行；长期可把 `soc` 加入 `dialout` 组。

先做 dry-run，看帧是否正确：

```bash
cd /home/soc/edge-ai-robot-k1
python3 tools/syn6288_serial_tts.py --dry-run --cue blockage_detected
```

真正播报：

```bash
python3 tools/syn6288_serial_tts.py \
  --port /dev/ttyUSB0 \
  --baud 9600 \
  --cue blockage_detected
```

连续播报复赛清障流程：

```bash
python3 tools/syn6288_serial_tts.py \
  --port /dev/ttyUSB0 \
  --sequence prelim_clearance \
  --delay-s 1.2
```

停止当前合成：

```bash
python3 tools/syn6288_serial_tts.py --port /dev/ttyUSB0 --command stop
```

## 接入风险识别流程

启动语音桥：

```bash
cd /home/soc/edge-ai-robot-k1
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

bash tools/start_prelim_syn6288_voice_k1.sh /dev/ttyUSB0 9600
```

语音桥会订阅：

```text
/perception/risk_alarm
/prelim_demo/voice_cue
```

当 YOLO 风险识别节点发布 `class_name=blockage` 的报警时，自动播报：

```text
发现障碍风险，正在生成风险点。
```

其他阶段由流程脚本或现场命令发布 cue：

```bash
ros2 topic pub --once /prelim_demo/voice_cue std_msgs/msg/String "{data: 'risk_point_generated'}"
ros2 topic pub --once /prelim_demo/voice_cue std_msgs/msg/String "{data: 'approach_reached'}"
ros2 topic pub --once /prelim_demo/voice_cue std_msgs/msg/String "{data: 'arm_clear_start'}"
ros2 topic pub --once /prelim_demo/voice_cue std_msgs/msg/String "{data: 'clear_done'}"
ros2 topic pub --once /prelim_demo/voice_cue std_msgs/msg/String "{data: 'report_ready'}"
```

对应文本：

```text
risk_point_generated -> 风险点已生成，正在规划靠近位置。
approach_reached     -> 已到达处置距离，底盘锁定。
arm_clear_start      -> 机械臂开始执行清障动作。
clear_done           -> 清障完成，正在复核风险状态。
report_ready         -> 风险报告已生成。
```

也可以发送 JSON，自定义一句话：

```bash
ros2 topic pub --once /prelim_demo/voice_cue std_msgs/msg/String \
  "{data: '{\"cue\":\"custom\",\"text\":\"机械臂已回到安全位置。\",\"force\":true}'}"
```

## 复赛现场建议流程

1. 先启动 SYN6288 语音桥，确认播报“语音播报模块已启动”。
2. 启动建图/风险识别/Dashboard。
3. YOLO 识别 `blockage` 后自动播报“发现障碍风险，正在生成风险点”。
4. 底盘靠近处置距离并 `base_zero` 后，发布 `approach_reached`。
5. 机械臂夹取/移动/放下开始前发布 `arm_clear_start`。
6. 放下并回安全位后发布 `clear_done`。
7. D435 复核和 LLM 报告生成后发布 `report_ready`。

这样现场呈现为：

```text
看见障碍 -> 地图定位 -> 靠近停车 -> 机械臂排障 -> 复核 -> 报告
```

## 故障排查

- 没声音：先确认供电、喇叭、功放、模块 TX/RX 是否交叉。
- 中文乱码：当前脚本默认 GBK 编码，和资料例程一致。
- 串口打不开：检查 `/dev/ttyUSB0` 是否存在，或用 `sudo` 临时运行。
- 播报过于频繁：语音桥默认同一 cue 4 秒冷却，可用 `--cooldown-s` 调整。
- 需要调音量/语速：调整 `--volume`、`--background-volume`、`--speed`。
