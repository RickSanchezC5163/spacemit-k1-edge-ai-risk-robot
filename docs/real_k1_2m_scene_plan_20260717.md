# K1 受限空间自主建图场景布置 - 2026-07-17

目标是在小型受限空间内验证“自主建图 + 风险识别 + 风险点落图 + RRT/Nav2 探索执行”的完整链路。当前阶段优先保证底盘 odom、二维雷达 SLAM 和 D435 YOLO 风险识别稳定，机械臂处置另起流程。

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

当前长运行的 goal 计算默认是 `RRT_FRONTIER_MODE=hybrid`，不是纯随机 RRT：先用 WFD 在当前可达自由区里找 frontier cluster 并评分，找不到再回退到 RRT/free-roam。这样能减少“随机采样很久才找到目标”和“目标合法但 Nav2 吃不下”的情况。

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

当前 2m 实车默认改为更大胆但仍保留边界的自由探图参数：

```text
RRT_SAMPLE_RADIUS_M=1.00
RRT_INFLATION_M=0.12
RRT_FRONTIER_STANDOFF_M=0.10
RRT_GOAL_SEPARATION_M=0.12
RRT_MAP_EDGE_MARGIN_M=0.15
RRT_FRONTIER_MODE=hybrid
RRT_WFD_MAX_CELLS=12000
RRT_MIN_FRONTIER_CLUSTER_CELLS=2
RRT_FRONTIER_BACKOFFS_M=0.10,0.18,0.25,0.35
RRT_GOAL_CLEARANCE_CHECK_M=0.50
RRT_FRONTIER_DISTANCE_WEIGHT=1.0
RRT_FRONTIER_SIZE_WEIGHT=0.05
RRT_REJECTED_GOAL_SEPARATION_M=0.25
RRT_GOAL_PROGRESS_TIMEOUT_S=12
RRT_GOAL_PROGRESS_GRACE_S=5
RRT_GOAL_PROGRESS_EPSILON_M=0.03
RRT_FAILURE_BACKOFF_AFTER=8
RRT_FAILURE_BACKOFF_S=5
Nav2 footprint=0.50 m x 0.44 m approximate outer envelope
```

窄通道入口优先按“frontier -> 未知方向 -> 向已知 free 区回退”生成 goal，而不是直接把 frontier cell 发给 Nav2。每个 frontier 会尝试 `RRT_FRONTIER_BACKOFFS_M` 里的多个回退距离，筛掉不在当前 WFD 可达区、非 free、inflation 不安全、重复过近的点，再按 `goal_clearance_m` 选更宽的入口点。`RRT_GOAL` 日志会输出 `frontier_id`、`frontier_size`、`candidate_backoff_m`、`goal_clearance_m`、`candidate_type`，现场排查时优先看这些字段。

2026-07-17 实机验证：`rolling_window: false` 加 `static_layer` 会被 SLAM `/map` 尺寸反向 resize，RRT 目标在 `/map` 内合法但仍可能落到 Nav2 global costmap 外沿，触发 `worldToMap failed`。当前 K1 可启动配置使用 4m rolling global costmap：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      rolling_window: true
      width: 4
      height: 4
      inflation_layer:
        inflation_radius: 0.15
```

薄壁或贴墙场景不要把微扇区直接当 hard stop。`scan_safety_guard_node` 使用左右各 `45°` 微扇区：任一侧进入 `MICRO_ADJUST_TRIGGER_M=0.22m` 时，安全层把前进量压成 0，并以约 `0.22rad/s` 朝更空的一侧旋转；右侧近则左转，左侧近则右转。hard stop 只看车体正前方走廊，默认 `FRONT_COLLISION_CORRIDOR_HALF_WIDTH_M=0.26`、`FRONT_COLLISION_MIN_X_M=0.02`，日志同时输出 `front_*` 和 `corridor_*`。2026-07-18 实机卡边读数为右前 `45°` 扇区 `min=0.150m`、`p10=0.154m`，因此新增 `escape_reverse`：`ESCAPE_REVERSE_TRIGGER_M=0.16m`、`ESCAPE_REVERSE_CLEAR_M=0.24m`、`ESCAPE_REVERSE_LINEAR_X=-0.08m/s`、`ESCAPE_REVERSE_ANGULAR_Z=0.20rad/s`，仅在贴边过近时短时后退并远离近侧。

如果 RRT 目标仍偏到边界：

```bash
export RRT_SAMPLE_RADIUS_M=1.0
export RRT_MAX_GOALS=4
export RRT_INFLATION_M=0.18
export RRT_FRONTIER_STANDOFF_M=0.18
export RRT_MAP_EDGE_MARGIN_M=0.15
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
export RRT_INFLATION_M=0.12
export RRT_FRONTIER_STANDOFF_M=0.10
export RRT_GOAL_SEPARATION_M=0.12
export RRT_MAP_EDGE_MARGIN_M=0.15
bash tools/start_real_k1_rrt_nav2_mapping.sh rrt-preview-2m
```

## 6. 2026-07-18 收车状态

本轮 K1 实机跑的是纯 RRT/Nav2/SLAM，不开 YOLO/EP。场地已经扩大到约 `2.5m x 2.5m`，最终 run 目录：

```text
/home/soc/edge-ai-robot-k1/outputs/real_k1_rrt_nav2_mapping_20260718_024536
```

最终地图已经在 K1 保存：

```text
/home/soc/edge-ai-robot-k1/outputs/real_k1_rrt_nav2_mapping_20260718_024536/maps/map_after_rrt_free_roam_stop_20260718_030446.yaml
```

Mac 本地渲染快照：

```text
outputs/k1_map_snapshots/20260718_030446/map_after_rrt_free_roam_stop_20260718_030446.png
```

今天关键结论：

- 45 度侧向微调逻辑已经实际生效，进入 `micro_adjust` 后会把线速度压成 0，并按更空旷的一侧旋转。
- 触发阈值从 `0.22/0.30` 调整到 `MICRO_ADJUST_TRIGGER_M=0.28`、`MICRO_ADJUST_CLEAR_M=0.34`，避免薄壁/角落里等到履带已经贴边才反应。
- `FRONT_COLLISION_MIN_X_M=0.12`，避免把激光雷达近侧噪声误判成正前方硬停。
- `ESCAPE_REVERSE_TRIGGER_M=0.16` 仍保留，只在真的贴得过近时短后退。
- RRT 前期能推动建图；后期 frontier 基本耗尽后开始大量 `wfd_free_roam`，不少目标只有 `goal_clearance_m=0.15~0.22`，Nav2 多次 `progress_timeout/status_6`，底盘多数时间实际 `cmd=(0,0)`。因此本轮在 goal_count 约 70 后主动停止并保存地图。
- 收车时已发零速，RRT/Nav2/SLAM/雷达进程已停，K1 上只剩用户终端 `tail -f` 日志，不再控制底盘。

明天优先做两件事：

1. 让 late-stage free-roam 更克制：提高 free-roam 的最小 clearance，或在连续失败后禁用 free-roam，只保留 frontier / entrance goal。
2. 针对当前地图做可视化复盘：把 RRT goal、Nav2 成败、guard micro_adjust 事件叠到地图上，看哪些区域导致 progress timeout。
