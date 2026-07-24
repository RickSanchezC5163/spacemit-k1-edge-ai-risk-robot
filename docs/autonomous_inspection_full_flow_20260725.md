# K1 自主巡检、自动补光与本地报告完整流程

## 1. 运行边界

底盘运动目标由运行时的实时地图和传感器数据生成：

1. N10P 与里程计持续更新 `slam_toolbox` 占据栅格；
2. Frontier/WFD 从当前 `/map` 提取已知空间与未知空间边界；
3. RRT 对候选目标做可达性和净空检查；
4. Nav2 规划并执行路径，雷达安全守护过滤最终速度；
5. 地图更新后重新计算探索目标。

只有当运行目录同时存在 RRT 探索报告和已保存 SLAM 地图时，报告生成器才把该轮标记为“自主探索建图证据完整”。

## 2. PWM7 / GPIO37 补光

实机补光使用 K1 `d401bc00.pwm`（PWM7）映射到 GPIO37，周期为 20 ms。系统服务在 Linux 启动时把占空比设为 0，避免车辆上电后灯保持点亮。

安装硬件灯控服务：

```bash
sudo REPO_DIR=/home/soc/edge-ai-robot-k1 \
  bash scripts/install_k1_pwm7_light_service.sh

python3 tools/k1_pwm7_light.py status
python3 tools/k1_pwm7_light.py set 100
python3 tools/k1_pwm7_light.py off
```

设备树工具先检查当前状态并只构建候选 DTB：

```bash
bash tools/k1_pwm7_50hz_dtb_trial.sh status
bash tools/k1_pwm7_50hz_dtb_trial.sh build
```

核对候选文件、当前 DTB 和备份路径后，才执行安装并重启：

```bash
sudo bash tools/k1_pwm7_50hz_dtb_trial.sh install
sudo reboot
```

设备树修改仅适用于已核对 DTB 版本的 MUSE Pi Pro。先执行状态和构建检查，再人工确认安装及重启；不应把其他板卡的 DTB 直接覆盖到当前系统。

自主流程同时启动：

- `adaptive_light_controller_node`：根据 D435 实时亮度输出 `/light/brightness_cmd`；
- `pwm7_light_node`：把亮度的开/关状态映射到经过验证的 PWM7 特权 helper；
- 任务启动、停止和异常退出时均显式执行 `off`。

## 3. 一键自主流程

首次部署后编译新增 ROS 节点：

```bash
cd /home/soc/edge-ai-robot-k1/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select k1_light_control
```

确认场地安全并把机器人放置在未知环境起点后执行：

```bash
cd /home/soc/edge-ai-robot-k1
K1_AUTONOMOUS_RUNTIME_S=240 \
  bash tools/run_k1_autonomous_inspection.sh run
```

编排器按以下顺序运行：

```text
PWM7 强制熄灯
-> 启动 SLAM + Nav2 + 雷达安全守护
-> 等待实时 /map
-> 启动 D435 + SpaceMIT EP YOLO + 风险接近处理
-> 启动 D435 亮度驱动的 PWM7 自动补光
-> 启动 Frontier/RRT 自主目标选择和 Nav2 执行
-> 到达运行时限后停车
-> 保存 SLAM 地图和 RRT/风险证据
-> 关闭底盘、雷达、D435、YOLO 与补光
-> K1 本地 Qwen 读取结构化风险结果
-> 生成 JSON、HTML 和 PDF 报告
```

查询和安全停止：

```bash
bash tools/run_k1_autonomous_inspection.sh status
bash tools/run_k1_autonomous_inspection.sh stop
```

## 4. 报告生成

编排器默认调用：

```bash
python3 tools/generate_k1_autonomous_report_bundle.py \
  --run-dir <run_dir> \
  --reports-root outputs/k1_autonomous_reports
```

当本机存在 Qwen2.5-0.5B GGUF 与 `llama-cli` 时，`auto` 后端运行本地 Qwen；否则生成确定性摘要，并在报告元数据中明确 `local_llm_used=false`。两种模式都不调用在线 API。

输出包括：

- `report.json`：自主建图证据、风险点和模型元数据；
- `index.html`：适合浏览器展示的风险表格；
- `report.pdf`：Chromium 无头渲染结果；
- `local_llm_prompt.txt`：提供给本地模型的结构化输入；
- `outputs/k1_autonomous_reports/latest`：最近一轮报告链接。

报告地址：

```text
http://<K1_IP>:8780/latest/
```

## 5. 证据口径

- “自主建图”指 SLAM 实时更新地图、Frontier/RRT 实时选择未知区域目标、Nav2 执行路径；
- D435 + YOLO 负责视觉检测，本地 Qwen读取结构化检测、深度和地图结果，当前模型不是视觉语言模型；
- 风险坐标是传感器和 TF 推算结果，不等同于工程测量或缺陷确诊；
- 补光由 D435 实时亮度触发；
- 机械臂实机动作仍受独立标定和安全门约束。
