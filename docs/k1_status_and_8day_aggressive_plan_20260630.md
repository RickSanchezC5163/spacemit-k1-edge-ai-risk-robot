# K1 当前状态与 8 天成品交付计划

日期：2026-06-30  
交付日：2026-07-08  
目标：8 天后交付一个可演示、可说明、证据链完整的边缘智能终端作品，而不是只交付单点实验。

## 一、作品最终定位

本作品面向 GPS 拒止、通信受限、低照度及遮挡干扰等复杂受限空间场景，基于 K1 Muse Pi Pro 平台构建一套具备离线认知与本地风险闭环能力的边缘智能终端。

系统融合视觉感知、激光测距与惯性信息辅助，在弱特征、局部退化环境下实现异常识别、风险分级与任务反馈，并支持在断网条件下完成从感知输入、模型推理到执行输出的本地闭环。

作品可应用于地下管廊、涵洞、设备舱、狭窄检修通道等典型场景，并可拓展至抢险救灾、紧急情况处置等对离线认知与本地决策能力要求较高的任务环境。

作品重点突出：

- RISC-V 平台上的轻量化模型部署与本地运行能力。
- D435 视觉、N10P 激光、里程计/惯性辅助的多源信息融合。
- 在复杂受限空间下的稳定运行、风险证据采集和本地闭环反馈。
- 断网条件下的本地推理、本地报告、本地动作协议闭环。

## 二、8 天后必须交付的成品形态

2026-07-08 的交付物应是一个完整演示系统，至少包含以下能力：

1. K1 实车安全底座可用  
   底盘、N10P、safety guard、base_zero 约束成立，不展示危险探索，不破坏 P4-V 冻结链路。

2. D435 安全静止视觉证据链可用  
   支持在停稳后采集 RGB、depth、camera_info、odom、meta、risk_point，并生成 episode_report。

3. 本地风险闭环可用  
   从 episode_report 生成本地 Markdown/JSON 风险报告，不依赖在线大模型 API。

4. 离线认知展示可用  
   如果真实模型来不及稳定部署，允许先用 deterministic baseline + 轻量 mock risk point 作为可解释基线，但必须清楚标注“不声明真实识别准确率”。

5. Gazebo 仿真补强可用  
   仿真 world、URDF/frame tree、camera/depth/odom topics、sim risk_point、sim validation runner 至少完成一条可复现实验链路，用来展示扩展能力和调试能力。

6. 机械臂按实际状态交付  
   如果机械臂维修/拼装完成并通过安全门禁，再进入 no-load 小角度验证。若未完成，则只交付 Arm-A mock 和 Arm-B safety prep，不声明真实机械臂控制。

7. 文档与证据可交付  
   必须有 README、验证报告、claim boundary、evidence manifest、演示流程说明，做到评审能看懂系统边界。

## 三、当前已经完成

### 1. P4-V 底盘安全链路

状态：冻结为当前稳定运动底座。

已确认：

- 底盘 + N10P + safety guard 链路可作为当前稳定基线。
- 不继续扩展底盘探索逻辑。
- 不修改 P4-V 主 safety guard 链路。

### 2. P4-X D435 HOLD_CAPTURE

状态：真实 K1 / D435 验证完成并冻结。

证据目录：

`outputs/p4x_d435_hold_capture_v1/`

已确认：

- P4-X0 topic audit：PASS。
- P4-X1 capture once：PASS。
- P4-X2 连续 HOLD_CAPTURE：10/10 succeeded。
- 每次 HOLD_CAPTURE 前 `base_zero_ok_before=True`。
- HOLD_CAPTURE 未发布 `cmd_vel`。
- RGB/depth/camera_info/odom/meta/risk_point 证据链完整。
- 不声明视觉检测准确率。
- 不声明机械臂操作。
- 不声明自主语义推理。

### 3. P4-Z lite 协议

状态：已完成最小协议收口。

已有文件：

- `src/edge_robot_protocol.py`
- `tools/validate_episode_report_schema.py`

已完成：

- 能解释现有 P4-X `episode_report.json`。
- 覆盖 EpisodeReport、PolicyState、PolicyAction、ActionResult、EvidencePaths、RiskPoint。

### 4. LLM-A deterministic baseline

状态：已完成本地规则报告生成，不接真实 LLM。

已有文件：

- `tools/generate_llm_a_risk_report.py`
- `docs/llm_a_risk_report_baseline_20260629.md`

输出目录：

`outputs/llm_a_risk_report_v1/`

已确认：

- 支持 P4-X episode_report 输入。
- 支持 Arm-A mock episode_report 输入。
- 输出 Markdown 和 JSON。
- 不使用在线 API。
- 不使用本地大模型。
- 不进入控制闭环。

### 5. Arm-A mock

状态：mock 动作协议验证完成。

证据目录：

`outputs/arm_a_mock_remove_obstacle_v1/`

已确认：

- 10/10 succeeded。
- 0 failed_safe。
- 全部 `base_zero_ok_before=True`。
- 全部 `published_cmd_vel=false`。
- 全部 `mock=True`。
- 全部 `obstacle_removed=True`。

边界：

- 只声明 ARM_REMOVE_OBSTACLE mock 动作链路。
- 不声明真实机械臂控制。
- 不声明真实清障。

### 6. P4-X3 字段补丁

状态：代码准备完成，但未做真实 ROS/D435 硬件验证。

未来新字段：

- `rgb_header_stamp`
- `depth_header_stamp`
- `rgb_frame_id`
- `depth_frame_id`
- `depth_encoding`
- `depth_scale_m`
- `valid_depth_ratio`
- `bbox_valid_depth_ratio`

边界：

- 不回填旧 P4-X evidence。
- 不声明 P4-X3 hardware-validated。

### 7. Gazebo 仿真资产

状态：已有仿真包，适合下一步做 Sim-A。

目录：

`sim/tracked_robot_description/`

包含：

- `urdf/`
- `meshes/`
- `config/`
- `worlds/`
- `launch/`
- `docs/`
- `MODEL_MEASUREMENTS.md`
- `README.md`

当前定位：

- 用于 Ubuntu 上的 ROS 2 Jazzy + Gazebo Harmonic 仿真。
- 用于补强 frame tree、传感器 topic、仿真 risk_point、仿真 episode_report。
- 不用于替代 K1 实车验证结论。

## 四、当前必须暂停的事情

机械臂真实硬件暂停。

原因：

- 机械臂仍在打印、拼装或维修。
- 真实总线舵机控制器暂不访问。
- 暂不发送串口指令。

暂停项：

- 不做真实 ARM_REMOVE_OBSTACLE。
- 不做真实 no-load 小角度动作。
- 不做真实接触测试。
- 不做真实清障。

Arm-B safety prep 只允许做代码审查和安全门禁修复。

已知问题：

- `configs/arm_safety_config.json` 里顶层 `safety_gates` 尚未作为全局 AND gate 强制覆盖 phase gate。
- `src/arm_safety.py` 在 B2/B3 phase 下可以构造舵机帧，虽然当前 dry-run 脚本没有串口写入，但接近实机前必须修复。
- 部分 Arm-B 文档/注释存在编码乱码，提交前需要清理。

## 五、8 天激进倒排计划

### 第 1 天：2026-07-01，Sim-A0 静态审计 + 成品脚本收口

目标：不启动 ROS，先把仿真和交付边界理清。

任务：

- 静态审计 `sim/tracked_robot_description`。
- 检查 package.xml、CMakeLists.txt、Xacro、URDF、world、launch。
- 梳理 frame tree 目标。
- 写清真实 topic 与 Gazebo topic 对照。
- 写 Sim-A 验收标准。
- 修本文档中所有交付边界。

交付物：

- `docs/sim_a_gazebo_validation_plan_20260701.md`
- `docs/final_demo_scope_20260701.md`

验收：

- 不启动 K1。
- 不启动真实机械臂。
- 不修改 P4-V/P4-X frozen evidence。

### 第 2 天：2026-07-02，Ubuntu Gazebo bring-up

目标：让仿真模型在 Ubuntu 上跑起来。

任务：

- 在 Ubuntu 24.04 上构建 `sim/`。
- 启动 Gazebo。
- 确认 robot model spawn。
- 确认 TF tree。
- 确认 `/scan`、RGB、depth、camera_info、odom topic。

交付物：

- `outputs/sim_a_gazebo_evidence_v1/sim_a0_topic_audit.json`
- `outputs/sim_a_gazebo_evidence_v1/sim_a0_topic_audit.md`
- `outputs/sim_a_gazebo_evidence_v1/errors.json`

验收：

- Gazebo 能启动。
- 传感器 topic 可读。
- 失败必须记录，不静默跳过。

### 第 3 天：2026-07-03，Sim-A1 capture once

目标：仿真版 D435 证据采集与 P4-X 格式对齐。

任务：

- 实现或复用 capture once 脚本。
- 从 Gazebo 读取 RGB、depth、camera_info、odom。
- 保存仿真 capture evidence。
- metadata 中明确 `source=simulation`。

交付物：

`outputs/sim_a_gazebo_evidence_v1/captures/<capture_id>/`

至少包含：

- `rgb.png`
- `depth_raw.npy`
- `depth_vis.png`
- `camera_info.json`
- `odom.json`
- `capture_meta.json`

验收：

- `depth_raw.npy` 可加载。
- depth dtype、depth_scale、frame_id、timestamp 记录清楚。
- 不与真实 P4-X evidence 混淆。

### 第 4 天：2026-07-04，Sim risk_point + episode protocol

目标：仿真中也能生成 risk_point，并进入 episode_report 协议。

任务：

- 复用 mock risk detector。
- 从 bbox 内计算 depth median。
- 反投影 `camera_point_xyz_m`。
- 输出 `risk_point.json`。
- 写入 episode_report 兼容结构。

交付物：

- `risk_point.json`
- `episode_report.json`
- `sim_a_capture_summary.md`

验收：

- `depth_median_m` 单位为米。
- `depth_scale_m` 明确。
- `valid_depth_ratio` 和 `bbox_valid_depth_ratio` 明确。
- 不声明视觉检测准确率。

### 第 5 天：2026-07-05，Sim-A2 连续验证 runner

目标：仿真证据链连续运行，形成稳定报告。

任务：

- 实现 `tools/run_sim_a_hold_capture_validation.py`。
- 连续运行 10 次 simulated HOLD_CAPTURE。
- 每次写 ActionResult。
- 汇总 `episode_report.json`。
- 写 `errors.json` 和 status CSV。

交付物：

`outputs/sim_a_gazebo_evidence_v1/`

至少包含：

- `sim_a_hold_capture_status.csv`
- `episode_report.json`
- `errors.json`
- `README.md`

验收：

- 10 次中至少 9 次成功。
- 每次都有 ActionResult。
- 每次都标记 simulation source。
- 不发布真实机器人 `cmd_vel`。

### 第 6 天：2026-07-06，本地风险闭环成品化

目标：把 P4-X、Arm-A、Sim-A 都接入本地风险报告链。

任务：

- 运行 LLM-A deterministic baseline。
- 对 P4-X 输出最终风险报告。
- 对 Arm-A mock 输出最终风险报告。
- 对 Sim-A 输出最终风险报告。
- 写统一 evidence manifest。

交付物：

- `outputs/final_risk_reports_v1/p4x/`
- `outputs/final_risk_reports_v1/arm_a/`
- `outputs/final_risk_reports_v1/sim_a/`
- `docs/final_evidence_manifest_20260706.md`

验收：

- 每个报告都有 Markdown 和 JSON。
- claim boundary 清楚。
- 不接在线 API。
- 不让 LLM 进入控制闭环。

### 第 7 天：2026-07-07，成品演示脚本 + 离线展示包

目标：把系统变成评审能看懂的成品演示。

任务：

- 写最终演示流程。
- 写一分钟、三分钟、五分钟三个版本讲稿。
- 准备断网演示步骤。
- 准备 K1 实车安全演示步骤。
- 准备 Gazebo 备份演示步骤。
- 清理文档中的乱码和边界不清项。

交付物：

- `docs/final_demo_script_20260707.md`
- `docs/final_claim_boundary_20260707.md`
- `docs/offline_demo_runbook_20260707.md`
- `docs/reviewer_quick_start_20260707.md`

验收：

- 断网时仍可展示本地 evidence -> report 闭环。
- 实车演示不依赖机械臂。
- Gazebo 可作为可控备份。
- 文档不夸大能力。

### 第 8 天：2026-07-08，最终交付日

目标：交付完整成品包。

任务：

- 最终跑一遍只读验证。
- 确认 P4-V/P4-X frozen evidence 未被改写。
- 确认输出目录完整。
- 确认 README 指向正确。
- 根据 git 状态分组提交，不混提交。
- 准备最终交付说明。

最终交付物：

- K1 实车安全底座说明。
- P4-X D435 HOLD_CAPTURE 真实 evidence。
- P4-Z 协议定义。
- LLM-A 本地风险报告 baseline。
- Arm-A mock action chain。
- Sim-A Gazebo evidence chain。
- 成品演示脚本。
- claim boundary。
- evidence manifest。
- 离线运行说明。

最终验收：

- 能解释“感知输入 -> 风险点 -> episode_report -> 本地风险报告 -> 动作协议反馈”闭环。
- 能展示 D435 与 N10P/底盘安全链路的关系。
- 能说明为什么系统适合 GPS 拒止、通信受限、低照度和遮挡干扰场景。
- 能清楚说明哪些已经实车验证，哪些只是仿真或 mock。
- 不把 mock、仿真、真实硬件验证混为一谈。

## 六、最终成品边界

可以声明：

- 已建立 K1 上的安全静止视觉证据采集链路。
- 已建立 episode_report 到本地风险报告的离线闭环。
- 已建立 mock 动作协议闭环。
- 已建立 Gazebo 仿真验证链路。
- 已形成面向复杂受限空间的边缘智能终端原型。

不能声明：

- 不能声明成熟视觉识别准确率。
- 不能声明真实机械臂清障能力，除非 2026-07-08 前完成独立硬件验证。
- 不能声明完全自主语义决策。
- 不能声明仿真结果等价于真实 K1 实车。
- 不能声明 RL 训练成果，除非后续确实完成训练和验证。

## 七、明天第一步

2026-07-01 首先做 Sim-A0，不做机械臂。

推荐顺序：

1. 检查 git 状态，确认当前工作树分组。
2. 修正本计划涉及的中文文档乱码。
3. 静态审计 `sim/tracked_robot_description`。
4. 写 Sim-A 验收标准。
5. 再决定是否进入 Ubuntu Gazebo bring-up。

