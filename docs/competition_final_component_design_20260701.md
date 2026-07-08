# 竞赛成品级四组件设计方案

**日期**: 2026-07-01
**目标**: 7/8 前补齐 AI 视觉模型、本地 LLM 推理、Gazebo RL 闭环、真实排障原语

---

## 设计原则

```
所有新组件必须:
1. 复用现有冻结的操作原语 (不改底盘/N10P/D435/机械臂底层)
2. 保持接口兼容 (替换模块只改内部实现, 不改外部调用)
3. 分层可降级 (每个组件有 A/B/C 方案, 时间不够就降级)
4. Claim boundary 清晰 (能做什么、不能做什么, 写在代码注释里)
```

---

## 组件一：AI 视觉模型 (替换 HSV 规则)

### 当前接口

```python
# tools/d435_red_rule_detector.py
def detect(rgb_path, depth_npy_path, camera_info, ...) -> dict:
    return {
        "red_object_detected": True/False,
        "bbox_xywh": [...],
        "depth_median_m": 0.541,
        "bbox_valid_depth_ratio": 0.844,
        "camera_point_xyz_m": {...},
        "detection_mode": "hsv_rule_based_red_color",  # 当前
        "model_used": False,       # 当前
        "accuracy_claimed": False, # 当前
    }
```

### 替换方案

**方案 A (推荐): YOLOv8n ONNX + ONNX Runtime**

```
训练 (x86 服务器):
  数据集: 风险图片 5 类, 每类 20-30 张, 标注 bbox
  模型: YOLOv8n (~3.2M 参数)
  导出: ONNX (FP32) + ONNX (INT8 量化)

推理 (K1 ARM CPU):
  ONNX Runtime 加载模型
  输入: D435 RGB 640×480
  输出: bbox + class + confidence
  帧率: 预计 1-3 FPS (ARM CPU)

接口不变:
  detection_mode: "yolov8n_onnx_cpu"
  model_used: True
  accuracy_claimed: False (因为是少量数据训练)
```

**方案 B (降级): MobileNet-SSD ONNX**

```
更小更快 (约 1.5M 参数)
精度更低但帧率更高 (3-5 FPS)
适合"证明模型在边缘端运行"的最低要求
```

**方案 C (保底): 如果 K1 编译 ONNX Runtime 失败**

```
在 x86 服务器上跑 YOLOv8n 推理
K1 通过网络把图片发到服务器
但声明: "当前验证在伴生计算机, 架构支持 RISC-V 部署"
这不够好但比没有强
```

### 新增文件

```
tools/d435_yolo_detector.py     ← 新检测器, 接口兼容 red_rule_detector
configs/yolo_risk_classes.yaml  ← 风险类别映射
models/yolov8n_risk.onnx        ← 训练好的模型
models/yolov8n_risk_int8.onnx   ← 量化版本
docs/yolo_risk_model_report_2026070X.md  ← 训练报告
```

### 验收标准

```
✓ 替换 red_rule_detector 后 Step7-E1A 仍能跑通
✓ red_object_detected=true 时 detection_mode="yolov8n_onnx_cpu"
✓ model_used=true
✓ 记录推理耗时 (FPS)
✓ 记录模型大小 (FP32 vs INT8)
✓ 正例负例都能正确处理
```

---

## 组件二：本地 LLM 推理 (替换模板报告)

### 当前接口

```python
# tools/generate_llm_a_risk_report.py
def generate_report(episode_report_path, output_dir) -> dict:
    # 读取 episode_report.json
    # 用 Python 模板拼接生成 Markdown 和 JSON
    return {
        "report_version": "llm_a_deterministic_risk_report_v1",
        "llm_model": "none (deterministic template)",  # 当前
        ...
    }
```

### 替换方案

**方案 A (推荐): llama.cpp + Qwen2-0.5B-Instruct Q4_K_M**

```
编译:
  git clone https://github.com/ggerganov/llama.cpp
  cd llama.cpp && make -j4  # ARM Linux

模型:
  Qwen2-0.5B-Instruct (约 400MB Q4_K_M)
  中文指令微调, 适合巡检报告生成

输入 (构造 prompt):
  将 episode_report.json 的结构化字段填入模板:

  "你是一个管廊巡检机器人。请根据以下巡检数据生成风险报告。

  巡检结果:
  - 执行步数: 3
  - 累计前进: 0.127m
  - 停止原因: max_consecutive_fast_arc_reached
  - 发现风险: 1 处
  - 风险类型: 红色异常标记
  - 风险位置: odom(x=0.32, y=0.11)
  - 机械臂响应: 已执行 no-load 序列, 回到安全位
  - 安全状态: base_zero=ok, hard_stop=未触发

  请生成:
  1. 巡检总结
  2. 风险列表
  3. 处置建议"

输出:
  Markdown 巡检报告

指标:
  tokens/s: 预计 2-8 (ARM CPU, Q4 量化)
  TTFT: 预计 1-3 秒
  内存: 预计 500-800MB
```

**方案 B (降级): TinyLlama-1.1B 或 Phi-3-mini**

```
更小更快的模型, 但中文能力弱
可以用英文 prompt 再翻译
```

**方案 C (保底): 保留 LLM-A 模板, 但声明架构可替换**

```
当前 LLM-A deterministic 报告 + llama.cpp 架构说明
在文档中写清楚替换路径和预期性能
```

### 新增文件

```
tools/generate_llm_local_risk_report.py  ← 新报告生成器
tools/llm_prompt_template.txt            ← prompt 模板
models/qwen2-0.5b-instruct-q4_k_m.gguf   ← 量化模型
docs/llm_local_inference_report_2026070X.md  ← 推理报告
```

### 验收标准

```
✓ llama.cpp 在 K1 ARM 上编译成功
✓ 能加载 Qwen2-0.5B Q4 模型
✓ 从 episode_report.json 生成中文巡检报告
✓ 测量 tokens/s, TTFT, 内存占用
✓ 报告内容合理 (人工评判)
✓ 不调用在线 API
```

---

## 组件三：Gazebo RL 闭环

### 当前状态

```yaml
# rl/configs/rl_a0_mock_ppo.yaml — mock training only
action_space: [hold, forward_0p15, arc_fast_left, arc_fast_right]
ros_started: false
hardware_connected: false
```

### RL 架构设计

**状态空间 (PolicyState)**

```python
observation = {
    # 激光雷达扇区
    "front_p10": float,      # 前向 35° P10 距离
    "front_min": float,      # 前向最近点
    "left_p10": float,       # 左侧 60° P10
    "right_p10": float,      # 右侧 60° P10
    "sector_counts": [5],    # 5 扇区有效点数

    # 里程计
    "odom_x": float,
    "odom_y": float,
    "odom_yaw": float,
    "total_forward": float,  # 累计前进

    # 建图
    "map_coverage_ratio": float,  # 已探索比例

    # 风险
    "risk_visible": bool,
    "risk_type": str,
    "risk_distance": float,

    # 状态
    "base_zero_ok": bool,
    "consecutive_fast_arc": int,
    "step_index": int,
}
```

**动作空间 (PolicyAction) — 7 个离散原语**

```python
actions = [
    "HOLD",                    # 停稳观察
    "FORWARD_0P15",            # 前进 0.15m
    "ARC_FAST_LEFT",           # 快速左弧 (~25°)
    "ARC_FAST_RIGHT",          # 快速右弧 (~28°)
    "HOLD_CAPTURE",            # 停稳拍照 (触发 D435)
    "ARM_NO_LOAD_RESPONSE",    # 机械臂 no-load 响应
    "STOP_SAFE",               # 安全终止
]
```

**奖励函数**

```python
reward = (
    + 1.0  * map_coverage_delta         # 新增探索面积
    + 2.0  * risk_detected              # 发现风险
    + 3.0  * risk_successfully_mapped   # 风险成功定位
    + 0.5  * forward_progress           # 有效前进 (front_p10 > 0.5)
    + 1.0  * obstacle_avoided           # 成功绕障
    + 1.0  * safe_stop                  # 合理停止

    - 5.0  * hard_stop_triggered        # 触发急停
    - 10.0 * collision                  # 碰撞
    - 2.0  * repeated_arc_no_progress   # 连续绕圈无改善
    - 0.1  * time_step                  # 时间惩罚 (鼓励高效)
)
```

**训练架构**

```
8×4090 服务器:
  GPU 0-5: 6 个 Gazebo 实例 (headless, 不同 seed)
  GPU 6: PPO 训练进程 (PyTorch, 从 replay buffer 采样)
  GPU 7: 周期性评估实例

每轮:
  6 个 env 并行 step → 收集 (obs, action, reward, next_obs)
  → 写入 replay buffer
  → PPO 更新策略网络
  → 每 100 episode 评估一次
```

**真车部署**

```
训练完导出: policy_weights.pt
在 K1 上加载: torch.jit.load()
替换 select_policy_action 的阈值逻辑:

  def select_policy_action(self, pre):
      obs = self._build_observation(pre)
      action_idx = self.policy_net(obs).argmax()
      return self.ACTIONS[action_idx]

  但安全 Guard 仍然在策略之后:
    RL 输出 → safety guard 过滤 → 底盘执行
```

### 新增文件

```
rl/train_ppo_gazebo.py           ← Gazebo 集成 PPO 训练
rl/gazebo_env.py                 ← Gym wrapper for Gazebo
rl/export_policy_for_k1.py       ← 导出策略到 K1 格式
rl/policy_net.py                 ← 轻量策略网络 (MLP, ~50K 参数)
rl/configs/rl_b0_gazebo_ppo.yaml ← 训练超参
sim/tracked_robot_description/worlds/pipe_corridor.sdf  ← 管廊 world
sim/tracked_robot_description/models/risk_marker/        ← 风险标记模型
```

### 验收标准

```
✓ Gazebo 中能跑完一次完整 episode (探图 → 发现风险 → 响应 → 终止)
✓ PPO 训练收敛 (reward curve 上升)
✓ 导出策略在 Gazebo 评估中表现优于随机策略
✓ 导出策略在 K1 dry-run 中不产生非法动作
✓ safety guard 始终在策略之后 (不被绕过)
```

---

## 组件四：真实排障原语 (Arm-D)

### 当前状态

```
Arm-C1: no-load sequence (8 步, 180 bytes 串行, 回到 6b)
已验证: serial_bytes_written=180, contact_allowed=false, obstacle_removed=false
```

### 排障原语设计

**Arm-D0: 排障 dry-run 计划**

```
生成 grasp-and-clear 候选序列
不做硬件执行
验证序列在安全门禁内
输出 candidate plan JSON
```

**Arm-D1: 空载排障轨迹 (no-load clear trajectory)**

```
和 B3 类似, 但轨迹更像真实排障:
  home → reach_forward → grasp_pose → lift → side_place → open → home
不做夹取, 不接触物体
确认轨迹在工作空间内
```

**Arm-D2: 轻接触泡沫测试**

```
contact_allowed=true (仅 foam)
放置软泡沫块在固定位置
  home → reach → grasp (轻夹泡沫) → lift → side_place → release → home
记录: 是否成功位移物体, 夹持力是否足够
```

**Arm-D3: 受控障碍物移除**

```
contact_allowed=true
obstacle_removal_allowed=true (首次)
真实移除轻障碍物 (纸箱片/泡沫块)
每次只移除一个
人在旁边监督
```

### 安全约束 (严格递增)

```
Arm-D0: 所有安全门关闭
Arm-D1: hardware_access=true, contact=false
Arm-D2: contact=true (foam only), force<2N
Arm-D3: obstacle_removal=true, 全程人工监督
```

### 新增文件

```
configs/arm_d_clear_config.json             ← 排障安全配置
tools/generate_arm_d0_clear_dryrun_plan.py  ← D0 计划生成
tools/run_arm_d1_no_load_clear_trajectory.py ← D1 空载轨迹
tools/run_arm_d2_foam_contact_test.py       ← D2 泡沫接触
docs/arm_d_clear_validation_2026070X.md     ← 排障验证文档
```

### 验收标准

```
✓ D0 dry-run 计划通过安全验证
✓ D1 空载轨迹在工作空间内, 无碰撞
✓ D2 能夹起并移动泡沫块 (≥80% 成功率)
✓ D3 受控移除一个障碍物, 全程无异常
✗ 不声明自主清障 (AI 不控制机械臂细节)
```

---

## 整体时间线

```
7/1 (今天):
  [x] 四组件设计文档完成
  [ ] 收集风险图片, 开始标注
  [ ] 开始编译 K1 上的 ONNX Runtime

7/2:
  [ ] YOLOv8n 训练完成, 导出 ONNX
  [ ] ONNX Runtime 在 K1 上跑通
  [ ] llama.cpp 在 K1 上编译

7/3:
  [ ] YOLO 推理替代 HSV, Step7-E1A 跑通
  [ ] llama.cpp 加载模型, 生成第一份中文报告
  [ ] Gazebo 管廊 world 构建完成

7/4:
  [ ] INT8 量化对比 (FP32 vs INT8)
  [ ] LLM tokens/s, TTFT 测量
  [ ] Gazebo RL 单实例跑通

7/5:
  [ ] Gazebo RL 多实例并行训练启动
  [ ] Arm-D1 空载排障轨迹验证

7/6:
  [ ] RL 训练 24h checkpoint 评估
  [ ] Arm-D2 泡沫接触测试
  [ ] 性能数据汇总

7/7:
  [ ] Arm-D3 真实排障 (如果 D2 通过)
  [ ] 录 Gazebo + 真机 演示视频
  [ ] 写竞赛报告

7/8:
  [ ] 最终收口检查
  [ ] 提交
```

---

## 降级策略

如果时间不够, 按以下顺序降级:

```
必须保的 (赛题硬性要求):
  1. YOLO 模型推理 (至少 ONNX CPU 跑通, 有 FPS 数据)
  2. LLM 本地推理 (至少 llama.cpp 跑通, 有 token/s 数据)

可以降级的 (加分项):
  3. Gazebo RL → 如果训练来不及, 至少跑通单实例 Gazebo pipeline
  4. Arm-D → 如果 D2/D3 来不及, D0 dry-run + 架构图声明可扩展
```
