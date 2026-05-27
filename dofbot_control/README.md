# dofbot_control

DOFBOT 5-DOF 机械臂的 ros_control 硬件接口与控制系统。同时支持 **MuJoCo 仿真**（`DofbotSimInterface`）和 **真实 I2C 舵机**（`DofbotHWInterface`）。

---

## 硬件背景

### 平台构成

- **上位机：** Raspberry Pi 4B，运行 ROS 1 Noetic（Docker）
- **扩展板：** Yahboom 定制 MCU，I2C 地址 `0x15`，桥接上位机与舵机 UART 总线
- **舵机：** DS-SY15A 串口舵机 × 6，DS Power 协议，半双工 UART
- **机械臂：** 5-DOF + 夹爪，桌面级

### 问题一：出厂未标定

`dofbot_description` 中的舵机参数（URDF 关节限位、`ServoParams` 默认值）与实物严重不符：

| 参数 | 出厂假设 | 实测 | 偏差 |
|------|---------|------|------|
| 舵机行程 | 270°（±135°） | **300°（±150°）** | 1.11× |
| base_rotation 中位 | 0 raw | −47 raw | — |
| upper_arm 中位 | 0 raw | −47 raw | — |
| forearm 中位 | 0 raw | −50 raw | — |
| wrist_pitch 中位 | 0 raw | −47 raw | — |
| wrist_roll 中位 | 0 raw | **−562 raw** | 最严重 |
| 角度映射系数 | 1.0× | 1.11× ~ 1.22× | 指令 90° 实际转 100°~120° |

**结论：不标定直接用，机械臂根本无法到达指定位置。**

> **注意：** 上表中的实测值为本作者机械臂的个体数据。DS-SY15A 舵机出厂一致性较差，**每台机械臂的中位偏差和行程系数均不同**。若在你的硬件上使用本代码，务必自行标定以下四项参数：
> - `angle_min_deg` / `angle_max_deg` — 舵机实际行程
> - `mid_deviation` — 中位偏差（手机量角器或激光标定）
> - `direction` — 旋转方向（±1）
>
> 标定数据写入 `DofbotHWInterface::init()` 中的 `ServoParams` 即可。URDF 关节限位也需同步更新。

### 问题二：扩展板 MCU 固件的三个致命限制

**1. 全局唯一运行时间寄存器**

MCU 固件对所有 6 路舵机只提供一个 `move_time` 寄存器（`0x1E`）。标准 ROS 轨迹中，每关节在同一时间段内位移量不同——走的多的关节应该快、走的少的应该慢。但 MCU 强迫所有关节在**完全相同的时间**内完成各自位移。结果：走的多的关节暴冲，走的少的关节蠕动，多关节运动不同步。

**2. 逐个单播而非广播同步**

MCU 通过 UART 逐个向 6 个舵机发送指令（非 SYNC WRITE 广播）。第一个舵机和最后一个舵机之间启动时差约 12ms，多关节协调运动产生可感知的"时差"。

**3. MCU 总线独占**

MCU 固件在 I2C 指令停止后仍然持续轮询舵机位置，占满 UART 总线。TXD 空闲高电平在物理层压制外部信号，即使将外部 USB-TTL 模块接入舵机总线，也只能监听、无法插入控制指令。**这一限制使得任何绕过 MCU 的外部控制方案都无法实施。**

---

## 两套控制方案

### 方案一：优化 FollowJointTrajectory（ros_control 标准路线）

**思路：** 保留标准 ROS 工作流（MoveIt → JointTrajectoryController → ros_control），通过参数调优适配舵机特性。

**改动：**
- `move_time` 固定 20ms，消除 Linux 控制周期抖动
- 指令死区 ±0.005 rad，过滤低于舵机精度（~0.017 rad）的无效微调
- 轨迹跟踪容差放宽至 0.25~0.5 rad，容忍机械滞后
- I2C 读取延迟降至 5ms（手册最低值）

**启动方式：** 即 `dofbot_hw.launch` 的默认行为

```bash
roslaunch dofbot_control dofbot_hw.launch
```

**解决了什么：**
- 角度标定后指令位置准确
- 卡顿感较最初大幅减轻
- 标准 ROS 接口，与 MoveIt 无缝集成

**没解决什么：**
- 舵机内部梯形曲线被 50Hz 指令反复打断——`move_time=20ms` 等于控制周期，舵机在每步加速阶段即被下一指令中断
- 多关节运动时机械共振显著
- MCU 全局单一 `move_time` 破坏同步性

**为什么无法继续优化：** `FollowJointTrajectory` 的设计假设——舵机能以控制频率精确追踪位置——被 MCU 的单一 `move_time` 打破。每个关节被迫以同一时间跑不同距离，同步性是靠牺牲平滑性换来的。这是固件层面的瓶颈，不是参数调优能解决的。

---

### 方案二：Waypoint Executor（稀疏逐点直连路线）

**思路：** 绕过 `FollowJointTrajectory`，提取轨迹关键路径点，逐个发给 MCU，给舵机足够时间完整展开内部梯形曲线。

**改动：**
- 路径点稀疏化（只取位移 ≥ 5° 的关键点），大幅减少指令密度
- 动态 `move_time`——大步给长时间、小步给短时间，适配每个路径点的实际需求
- 用 MoveIt 自带的时序（`time_from_start`）而非自行估算，保留轨迹的时间逻辑
- 尝试用 HW-597 USB-TTL 模块直连舵机 UART 总线，完全脱离 MCU

**启动方式：**

```bash
roslaunch dofbot_control dofbot_waypoint.launch
```

**解决了什么：**
- 不再打断舵机内部梯形曲线——每个指令都能完整执行
- 单步运动平滑，无高频抖动

**没解决什么：**
- 多关节同步问题仍未解决——MCU 的单一 `move_time` 导致"走得多的关节冲、走得少的关节蠕"
- HW-597 直连方案被硬件卡住——HW-597 是全双工芯片，空闲高电平压制舵机应答信号，能听不能发

**为什么无法继续优化：** 要实现"每关节独立时间 + 广播同步启动"，必须使用 DS Power 协议的 **SYNC WRITE** 指令。但此指令只能通过舵机原生 UART 总线发出——MCU 在硬件层独占了这条总线（拔掉跳线帽也只能监听，无法替代）。

---

## 根因总结

两条技术路径的交集在同一个瓶颈上：

```
方案一：50Hz 密集追踪 ──→ 打断舵机内部曲线 ──→ 卡顿
                           ↓
                MCU 全局单一 move_time
                + 无 SYNC WRITE 固件
                           ↓
方案二：稀疏逐点驱动 ──→ 不同步 + 暴冲
```

换一块带半双工自动切换的 USB-TTL 模块可以解锁方案二的部分限制，但要真正做到**同步且平滑**的多关节运动，最终必须修改 MCU 固件，使其支持：

1. **每关节独立运行时间**
2. **SYNC WRITE 广播同步启动**

---

## 当前状态

| 模式 | 状态 | 可用性 |
|------|------|--------|
| **MuJoCo 仿真** | 完整可用 | `roslaunch dofbot_moveit_config demo_mujoco.launch` |
| 真机 FollowJointTrajectory | 功能可用但不够平滑 | `roslaunch dofbot_control dofbot_hw.launch` |
| 真机 Waypoint Executor | 功能可用但不同步 | `roslaunch dofbot_control dofbot_waypoint.launch` |

**仿真模式不受上述硬件限制影响**，是当前最主要的研究平台。

---

## 代码结构

```
dofbot_control/
├── config/
│   ├── dofbot_control.yaml           # dofbot_trajectory_controller 配置
│   └── joint_state_controller.yaml   # JointStateController 配置（仅真机用）
├── include/dofbot_control/
│   ├── dofbot_hw_interface.h         # DofbotHWInterface (I2C 真机)
│   └── dofbot_sim_interface.h        # DofbotSimInterface (MuJoCo 仿真桥)
├── launch/
│   ├── dofbot_sim.launch             # 仿真模式：MuJoCo + ros_control 闭环
│   ├── dofbot_hw.launch              # 真机模式：I2C + FollowJointTrajectory
│   ├── dofbot_waypoint.launch        # 真机模式：Waypoint Executor + MoveIt
│   ├── mujoco_sim.launch             # 仅启动 MuJoCo 仿真节点（不含 ros_control）
│   └── camera.launch                 # USB 摄像头驱动（眼在手上）
├── scripts/
│   ├── mujoco_sim_node.py            # MuJoCo 仿真节点 (PD 力矩控制)
│   └── waypoint_executor.py          # Waypoint 执行器 (MoveIt 轨迹 → 逐点 I2C)
└── src/
    ├── dofbot_hw_interface.cpp       # 真机 HW 接口实现
    ├── dofbot_sim_interface.cpp      # 仿真 HW 接口实现
    └── dofbot_hw_node.cpp            # 主节点 (sim_mode 选择)
```

## 许可证

Apache License, Version 2.0 — 版权所有 (c) 2025 Tu Zhichao
