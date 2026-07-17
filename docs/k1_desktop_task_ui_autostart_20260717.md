# K1 桌面任务控制台与自动登录 - 2026-07-17

目标：

```text
K1 开机 -> SDDM 自动登录 soc -> LXQt 桌面 -> 自动打开 K1 任务控制台
```

## 1. 已完成的用户级自启动

任务控制台服务：

```bash
/home/soc/edge-ai-robot-k1/tools/k1_task_desktop_dashboard.py
```

启动脚本：

```bash
/home/soc/edge-ai-robot-k1/tools/start_k1_task_desktop_ui.sh
```

LXQt 用户级自启动：

```text
/home/soc/.config/autostart/k1-task-dashboard.desktop
```

桌面快捷方式：

```text
/home/soc/Desktop/K1任务控制台.desktop
```

控制台访问地址：

```text
K1 本机:     http://127.0.0.1:8780/
Windows:    http://192.168.43.40:8780/
```

控制台显示：

```text
电池电压 /battery_voltage
前向安全距离 /safety/front_obstacle
里程计 /odom
风险告警 /perception/risk_alarm
D435、YOLO、Nav2/SLAM、RRT 进程状态
当前 run 目录和风险事件数
```

电压显示使用补偿值：

```text
display_voltage = /battery_voltage + 0.83 V
```

原因：当前底盘回传 `/battery_voltage` 相比平衡口实测偏低。2026-07-17 实测平衡口约 `12.16 V`，UI raw 值约 `11.33 V`，差值约 `0.83 V`。

默认低电压提醒阈值：

```text
3.7 V x 3 = 11.10 V
```

可通过环境变量调整：

```bash
export K1_BATTERY_VOLTAGE_OFFSET_V=0.83
export K1_BATTERY_WARN_V=11.10
bash tools/start_k1_task_desktop_ui.sh
```

任务控制台启动时默认会同时启动“底盘待命模式”：

```text
ros2 launch turn_on_wheeltec_robot tank_base_safe.launch.py
cmd_vel_topic:=/k1_boot_zero_cmd
```

`/k1_boot_zero_cmd` 默认没有发布者，因此底盘驱动只保持零速/超时零速，但会打开底盘串口并发布：

```text
/battery_voltage
/odom
```

正式进入手动建图或 Nav2 探图前，执行：

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh clean
```

这样会关闭开机待命底盘节点，避免正式建图流程抢占 `/dev/base_controller`。

## 2. 需要 sudo 的 SDDM 自动登录

当前 K1 的 SDDM 配置只有：

```ini
[Autologin]
Session=bianbu-lite
```

这还不是完整自动登录。需要补：

```ini
User=soc
Relogin=true
```

执行：

```bash
cd /home/soc/edge-ai-robot-k1
sudo bash tools/configure_k1_sddm_autologin.sh soc bianbu-lite
```

成功后应看到：

```ini
[Autologin]
Session=bianbu-lite
User=soc
Relogin=true
```

重启验证：

```bash
sudo reboot
```

重启后应自动进入 `soc` 桌面并打开 K1 任务控制台。

## 3. 手动启动/排查

不重启时可手动启动：

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_k1_task_desktop_ui.sh
```

检查服务：

```bash
pgrep -af k1_task_desktop_dashboard.py
ss -ltnp | grep 8780
```

检查日志：

```bash
cat /home/soc/edge-ai-robot-k1/outputs/k1_task_desktop_ui/server.log
cat /home/soc/edge-ai-robot-k1/outputs/k1_task_desktop_ui/chromium.log
```

关闭：

```bash
pkill -f tank_base_safe.launch.py
pkill -f wheeltec_tank_base_safe.py
pkill -f k1_task_desktop_dashboard.py
pkill -f 'chromium.*8780'
```
