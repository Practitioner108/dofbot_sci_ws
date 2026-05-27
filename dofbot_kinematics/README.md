# dofbot_kinematics

DOFBOT 5-DOF 机械臂的 **闭式解析逆运动学插件**。基于 MoveIt `KinematicsBase` 接口，替代默认的 KDL 数值迭代求解器，提供亚毫秒级求解速度。

## 算法概要

1. 将 `wrist_roll_link` 目标位姿分解为位置和姿态
2. 在底座旋转轴投影平面上求解 **q1**（偏航角），处理天顶奇异点
3. 将三维目标退推至二维二连杆平面，用**余弦定理**求解 **q2、q3、q4**
4. 从目标姿态矩阵剥离旋转分量，求解 **q5**（腕部滚转）
5. 对每个关节在其限位范围内搜索所有 `2π` 等效解
6. 从全部可行解中选择距种子关节角最近的最优构型

## 运动学常量

| 参数 | 值 | 说明 |
|------|-----|------|
| `L_BASE` | 0.1075 m | 底座高度（base_rotation → upper_arm） |
| `L2` | 0.08285 m | 大臂长度（upper_arm → forearm） |
| `L3` | 0.08285 m | 小臂长度（forearm → wrist_pitch） |

## 配置

在 MoveIt `kinematics.yaml` 中启用：

```yaml
arm:
  kinematics_solver: dofbot_kinematics/DofbotIKPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  position_only_ik: True        # 5-DOF 机械臂必须设为 True
gripper:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```

## 许可证

Apache License, Version 2.0
