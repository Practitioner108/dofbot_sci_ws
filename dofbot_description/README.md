# dofbot_description

5-DOF 桌面级机械臂（含夹爪）的 URDF 描述文件、3D 网格和仿真资源。

## 快速开始

在 RViz 中查看机器人模型：

```bash
roslaunch dofbot_description display.launch
```

加载其他 xacro 文件（例如备份版本）：

```bash
roslaunch dofbot_description display.launch xacro:=<路径>
```

## 文件结构

```
dofbot_description/
  urdf/
    dofbot.xacro           # 主模型（xacro 自包含，含宏定义）
    dofbot.urdf            # 旧版纯 URDF（备份，不再引用）
    dofbot_orgin.urdf      # Yahboom 官方原始 URDF（备份，不再引用）
  meshes/
    visual/                # 8 个高精度视觉 STL 网格
    collision/             # 20 个简化碰撞 STL 网格
    source_files/          # Blender 源文件（.blend）
  mjcf/
    dofbot_mjcf.xml        # MuJoCo 仿真模型
  launch/
    display.launch         # 一键可视化（RViz + joint_state_publisher_gui）
  rviz/
    urdf.rviz              # 默认 RViz 配置
  doc/
    images/                # 产品图片和尺寸参考图
    hardware/              # 舵机规格书、引脚映射表
    protocol/              # I2C 寄存器映射、串口通信协议
```

## 归属声明

本包的 URDF 基础模型和网格文件来源于 Yahboom DOFBOT 开源项目
（[YahboomTechnology/dofbot-Pi](https://github.com/YahboomTechnology/dofbot-Pi)）。

在原版基础上做了以下改进：
- 转为 xacro 格式，提取参数化宏（`dofbot_inertial`、`dofbot_visual`）
- 描述性关节/连杆命名（`base_rotation_joint`、`forearm_link` 等）
- 完整的碰撞几何体（20 个简化碰撞网格）
- 所有 6 个驱动关节的 ros_control transmission 定义
- Gazebo + MuJoCo 双仿真支持
- 完整的安装规则，支持 `catkin build --install` 部署
- 眼在手上 USB 摄像头 URDF/SRDF 定义

## 许可证

本包基于 [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0) 开源。

版权所有 (c) 2025 Tu Zhichao

Yahboom 原始基础数据同样为开源协议。详见上游项目。
