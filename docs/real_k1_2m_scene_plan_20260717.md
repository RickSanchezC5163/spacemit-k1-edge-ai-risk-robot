# K1 2m x 2m 复赛复刻场景布置 - 2026-07-17

目标是在 2m x 2m 小场景内复刻“建图 + 风险识别 + 风险点落图 + RRT/Nav2 探图预览/执行”的现场演示。当前阶段优先保证底盘 odom、二维雷达 SLAM 和 D435 YOLO 风险识别稳定，机械臂处置另起流程。

## 1. 场地尺度

建议外框：

```text
2.0 m x 2.0 m
最小通道宽度 >= 0.60 m
风险牌/障碍物与相机典型距离 0.45-0.75 m
底盘起点留出至少 0.35 m 转向空间
```

不要把风险点贴得过近。当前自动落图边界是：

```text
crack:    confidence >= 0.29, 0.60 m <= depth <= 0.80 m
blockage: confidence >= 0.23, 0.35 m <= depth <= 0.75 m
```

因此：

- `blockage` 适合放在车前 0.45-0.65 m。
- `crack` 适合放在车前 0.65-0.75 m。
- 如果要放 `corrosion/leakage` 作为报告展示点，先用 YOLO overlay 看稳定识别距离，再决定是否加入 gate。

## 2. 推荐布局

```text
┌──────────────────── 2.0 m ────────────────────┐
│                                                │
│   crack / corrosion 展示板       侧边障碍       │
│                                                │
│        ┌────────── 观察通道 ──────────┐        │
│        │                              │        │
│ 起点   │        blockage 处置点        │  leakage│
│  S     │                              │  展示板 │
│        └──────────────────────────────┘        │
│                                                │
└────────────────────────────────────────────────┘
```

关键原则：

- 起点不要贴墙，避免 SLAM 初始几帧只有近距离墙面。
- 雷达能看到清晰边界，但不要用太多黑色吸光材料。
- YOLO 风险牌尽量朝向 D435，不要和雷达墙体完全重叠。
- `blockage` 放在可接近区域，后续机械臂演示可以让底盘停在处置距离。

## 3. 演示启动顺序

先做保底手动建图：

```powershell
Set-Location K:\risc-vCar\edge-ai-robot-k1
powershell -ExecutionPolicy Bypass -File tools\win_start_real_k1_rrt_nav2_mapping.ps1 -Mode manual -CleanFirst
```

确认：

```text
/scan 正常
/odom 正常
/map 增量生成
键盘 i/,/j/l/k 能控制底盘
```

保存第一版地图：

```bash
cd /home/soc/edge-ai-robot-k1
bash tools/start_real_k1_rrt_nav2_mapping.sh save-map
```

再做 2m 场景 RRT 预览：

```powershell
Set-Location K:\risc-vCar\edge-ai-robot-k1
powershell -ExecutionPolicy Bypass -File tools\win_start_real_k1_rrt_nav2_mapping.ps1 -Mode nav2-preview-2m -CleanFirst
```

`nav2-preview-2m` 只发布 `/rrt_preview_goal_pose`，不会发送 Nav2 action。确认目标点合理后，再进入执行：

```powershell
powershell -ExecutionPolicy Bypass -File tools\win_start_real_k1_rrt_nav2_mapping.ps1 -Mode nav2-run-2m -CleanFirst
```

如果当前目标是先验证自动探图，不需要同时启动 YOLO，可用纯 RRT 长运行模式：

```powershell
powershell -ExecutionPolicy Bypass -File tools\win_start_real_k1_rrt_nav2_mapping.ps1 -Mode nav2-run-2m-unlimited -CleanFirst
```

## 4. 现场判断标准

第一轮只要求：

```text
地图边界与 2m x 2m 场地基本一致
front_min 随障碍变化正常
odom 连续、无大跳变
D435 overlay 能显示 infer_fps
至少一个 blockage 或 crack 正式通过 gate
风险点能写入 risk_map_points.json
```

## 5. 侧边擦碰处理

现场发现侧边偶尔蹭到地图框架时，主要不是 Nav2 完全把车当成质点，而是 RRT 目标筛选和 Nav2 footprint/实际外廓之间留量不够。尤其不要使用类似 `--inflation-m 0.02`、`--map-edge-margin-m 0.00` 的临时放松参数做实车长跑；这会让 frontier 目标贴近边界，履带车再叠加里程计误差和局部控制跟踪误差，就容易侧面擦到框架。

当前 2m 实车默认改为保守参数：

```text
RRT_INFLATION_M=0.24
RRT_FRONTIER_STANDOFF_M=0.30
RRT_GOAL_SEPARATION_M=0.30
RRT_MAP_EDGE_MARGIN_M=0.16
Nav2 footprint=0.50 m x 0.44 m approximate outer envelope
```

如果 RRT 目标仍偏到边界：

```bash
export RRT_SAMPLE_RADIUS_M=1.0
export RRT_MAX_GOALS=4
export RRT_MAP_EDGE_MARGIN_M=0.20
bash tools/start_real_k1_rrt_nav2_mapping.sh rrt-preview-2m
```

如果目标过近：

```bash
export RRT_MIN_GOAL_DISTANCE_M=0.35
export RRT_GOAL_SEPARATION_M=0.35
bash tools/start_real_k1_rrt_nav2_mapping.sh rrt-preview-2m
```

如果路径过保守：

```bash
export RRT_INFLATION_M=0.18
bash tools/start_real_k1_rrt_nav2_mapping.sh rrt-preview-2m
```
