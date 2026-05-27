# DOFBOT SCI Workspace

5-DOF 桌面级机械臂（含夹爪）的 ROS 1 Noetic 工作空间。支持 **MuJoCo 仿真** 与 **真实硬件（I2C 舵机）** 双模式运行，集成 MoveIt 运动规划。

## 系统架构

```
┌─ MoveIt (运动规划) ──────────────────────────────────────┐
│  IK 求解: dofbot_kinematics (闭式解析解)                    │
│  轨迹执行: dofbot_trajectory_controller (ros_control)      │
└─────────────────┬───────────────────────────────────────┘
                  │ FollowJointTrajectory action
                  ↓
┌─ ros_control Hardware Interface ─────────────────────────┐
│  ┌──────────────────┐   ┌──────────────────────────┐     │
│  │ DofbotSimInterface│   │ DofbotHWInterface        │     │
│  │ (MuJoCo 仿真桥)   │   │ (I2C → MCU → 舵机)       │     │
│  └────────┬─────────┘   └────────────┬─────────────┘     │
└───────────┼──────────────────────────┼───────────────────┘
            ↓                          ↓
   MuJoCo 物理引擎             DS-SY15A 串口舵机 × 6
```

## 功能包

| 包 | 说明 |
|----|------|
| [dofbot_description](dofbot_description/) | URDF/XACRO 模型、STL/OBJ 网格、MJCF MuJoCo 模型 |
| [dofbot_control](dofbot_control/) | ros_control 硬件接口（仿真 + I2C 真机）、MuJoCo 仿真节点 |
| [dofbot_moveit_config](dofbot_moveit_config/) | MoveIt 配置（运动学、控制器、关节限位、RViz） |
| [dofbot_kinematics](dofbot_kinematics/) | 5-DOF 机械臂闭式解析 IK 插件 |

## 快速开始

### 仿真模式

```bash
# MuJoCo 仿真 + MoveIt + RViz 一键启动
roslaunch dofbot_moveit_config demo_mujoco.launch

# 无头模式（不显示 MuJoCo 查看器）
roslaunch dofbot_moveit_config demo_mujoco.launch headless:=true
```

### 真实硬件模式

```bash
# Waypoint 执行器模式（逐点 I2C 驱动舵机）
roslaunch dofbot_control dofbot_waypoint.launch
```

### 仅查看 URDF 模型

```bash
roslaunch dofbot_description display.launch
```

## 依赖

- ROS 1 Noetic
- MuJoCo ≥ 3.1.0 (Python bindings)
- MoveIt 1
- catkin tools

## 许可证

本工作空间所有功能包均基于 [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0) 开源。

原始 URDF 基础模型和网格文件来源于 [YahboomTechnology/dofbot-Pi](https://github.com/YahboomTechnology/dofbot-Pi)。

## 作者

**Tu Zhichao** — [3954058022@qq.com](mailto:3954058022@qq.com)
