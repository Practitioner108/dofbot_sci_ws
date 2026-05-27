# dofbot_moveit_config

DOFBOT 5-DOF 机械臂的 MoveIt 配置包。由 MoveIt Setup Assistant 生成，经手工调整适配 MuJoCo 仿真和真实硬件。

## 启动方式

```bash
# MuJoCo 仿真 + MoveIt + RViz（推荐）
roslaunch dofbot_moveit_config demo_mujoco.launch

# Gazebo 仿真 + MoveIt + RViz（旧版，需安装 Gazebo）
roslaunch dofbot_moveit_config demo_gazebo.launch

# 无头仿真（不显示 MuJoCo 查看器，节省资源）
roslaunch dofbot_moveit_config demo_mujoco.launch headless:=true
```

## 双控制器配置

通过 `controller_config` 参数切换控制器模式：

| 配置文件 | 控制器 | 用途 |
|----------|--------|------|
| `ros_controllers.yaml` | `/dofbot_trajectory_controller` | 仿真模式（ros_control + MuJoCo） |
| `ros_controllers_waypoint.yaml` | `/waypoint_executor` | 真机模式（逐点 I2C 驱动舵机） |

## 目录结构

```
config/
  dofbot.srdf                   # 机器人语义描述（规划组、碰撞矩阵、位姿）
  kinematics.yaml               # IK 求解器配置（dofbot_kinematics 闭式解）
  joint_limits.yaml             # 关节速度和加速度限位
  ompl_planning.yaml            # OMPL 规划器参数
  ros_controllers.yaml          # 仿真控制器列表
  ros_controllers_waypoint.yaml # 真机控制器列表
  moveit.rviz / moveit_mujoco.rviz  # RViz 配置文件
launch/
  demo_mujoco.launch            # MuJoCo 仿真一键启动
  demo_gazebo.launch            # Gazebo 仿真一键启动
  move_group.launch             # MoveIt move_group 节点
  moveit_rviz.launch            # RViz 可视化启动
  trajectory_execution.launch.xml  # 轨迹执行配置
```

## IK 求解器

本包配置使用 `dofbot_kinematics` 闭式解析 IK 插件（替代默认的 KDL 数值迭代），`position_only_ik: True` 适配 5-DOF 机械臂。

## 许可证

Apache License, Version 2.0
