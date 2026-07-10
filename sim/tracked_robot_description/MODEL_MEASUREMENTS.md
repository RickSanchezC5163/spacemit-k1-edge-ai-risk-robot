# 车型尺寸记录

单位：模型文件统一使用米。这里同时记录原始厘米数据，方便之后核对。

## 坐标约定

- 底盘中心为 `x/y` 原点。
- 前方为 `+x`，左方为 `+y`，向上为 `+z`。
- 27 cm 底盘的前边缘是 `x = +0.135 m`。
- “距离底盘最前面 N cm”按“从前边缘向后 N cm 的中心点”换算。

## 已录入尺寸

| 项目 | 原始数据 | 模型值 |
| --- | --- | --- |
| 底盘长 | 27 cm | `chassis_length = 0.27` |
| 底盘宽 | 27 cm | `chassis_width = 0.27` |
| 底盘平面距地 | 10 cm | `chassis_top_z = 0.10` |
| 底盘厚度 | 暂估 3 cm | `chassis_height = 0.03` |
| 履带单条宽度 | 4 cm | `track_width = 0.04` |
| 履带高度 | 8 cm | `track_height = 0.08` |
| 可见主动轮直径 | 4 cm | `drive_sprocket_radius = 0.02` |
| 下方承重轮 | 每侧 4 个，带减震支柱外观 | `road_wheel_1..4`，`suspension_struts_link` |
| 左右履带中心距 | 23 cm | `wheel_separation = 0.23`，`track_y = +/-0.115` |
| Gazebo 等效履带半径 | 履带高度一半 4 cm | `virtual_drive_radius = 0.04` |
| D435 高度 | 距地 11 cm | `d435_z_abs = 0.11` |
| D435 前后位置 | 距底盘最前面 3 cm | `d435_x = 0.105` |
| D435 左右位置 | 居中 | `d435_y = 0` |
| 雷达型号 | N10P | `front_lidar` |
| 雷达高度 | 距地 13 cm | `lidar_z_abs = 0.13` |
| 雷达前后位置 | 中心距底盘最前面 6 cm | `lidar_x = 0.075` |
| 雷达左右位置 | 居中 | `lidar_y = 0` |
| ID1 转轴高度 | 距地 13 cm | `arm_base_z_abs = 0.13` |
| ID1 前后位置 | 中心距底盘最前面 14 cm | `arm_base_x = -0.005` |
| ID1 左右位置 | 暂按居中 | `arm_base_y = 0` |
| ID1 到 ID2 | 19 cm | `arm_id1_to_id2 = 0.19` |
| ID2 到 ID3 | 4 cm | `arm_id2_to_id3 = 0.04` |
| ID3 到 ID4 | 19 cm，双平行管水平臂 | `arm_id3_to_id4 = 0.19` |
| ID5 | 控制爪子 | `arm_joint_5_gripper` |

## 当前假设

- 底盘厚度没有实测，先按 3 cm。
- 底盘质量没有实测，当前按 2.5 kg。
- 主动轮按坦克式履带结构建模为前侧悬空轮；Gazebo 差速插件使用隐藏等效驱动轮近似履带接地速度。
- 机械臂按示意图建成：ID1 立轴 yaw，ID1->ID2 竖直长臂，ID2->ID3 顶部短连杆，ID3->ID4 第二根竖直长臂，ID5 控制夹爪。
- 机械臂各舵机角度范围还没有实测，当前是安全占位范围。

## 还需要补充

| 项目 | 用途 |
| --- | --- |
| 底盘质量 | 修正惯量和动力学 |
| ID1-ID4 各舵机角度范围 | 修正关节限位 |
| 夹爪开合角度/最大开口 | 修正 ID5 夹爪模型 |
| 末端夹爪长度 | 修正工作空间 |

## 参考图

机械臂标注图已保存为：

```text
docs/arm_reference.png
```

多视角整车参考图已保存为：

```text
docs/reference_views/
```

整车参考图已保存为：

```text
docs/vehicle_reference.png
```

整车示意图已保存为：

```text
docs/vehicle_schematic.svg
```
