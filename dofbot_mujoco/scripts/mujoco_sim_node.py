#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 仿真节点：加载 DOFBOT 模型、步进物理、发布关节状态。

当前版本 (Phase 1)：
  - 加载 dofbot_description/mjcf/dofbot_mjcf.xml
  - 步进 MuJoCo 物理引擎
  - 发布 /joint_states
  - 接收 /joint_command（Float64MultiArray，7 个关节位置）

未来 (Phase 2)：
  - ros_control hardware_interface 桥接
  - MoveIt FollowJointTrajectory 对接
"""

import os
import sys
import numpy as np

import rospy
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    import mujoco
    import mujoco.viewer
    HAS_MUJOCO = True
except ImportError:
    HAS_MUJOCO = False


class DofbotMujocoSim:
    def __init__(self, headless=False):
        rospy.init_node('dofbot_mujoco_sim', anonymous=False)

        if not HAS_MUJOCO:
            rospy.logerr("MuJoCo Python bindings not found. Install 'mujoco' package.")
            sys.exit(1)

        # 加载 MJCF 模型
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('dofbot_description')
        xml_path = os.path.join(pkg_path, 'mjcf', 'dofbot_mjcf.xml')

        if not os.path.exists(xml_path):
            rospy.logerr("Model file not found: %s", xml_path)
            sys.exit(1)

        rospy.loginfo("Loading MuJoCo model: %s", xml_path)
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # 关节映射：MJCF 中的关节名 → ROS joint_states 中的名称
        # 注意：gripper_right_joint 由 MJCF equality 约束自动镜像，不在此列表
        self.joint_names = [
            'base_rotation_joint',
            'upper_arm_joint',
            'forearm_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
            'gripper_left_joint',
        ]

        # 获取 MuJoCo 关节 ID
        self.mj_joint_ids = []
        for name in self.joint_names:
            try:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                self.mj_joint_ids.append(jid)
                rospy.loginfo("  Mapped joint %s -> MuJoCo id %d", name, jid)
            except Exception:
                rospy.logerr("Joint '%s' not found in MuJoCo model", name)
                sys.exit(1)

        self.n_joints = len(self.joint_names)

        # 控制输入缓冲
        self.cmd_positions = np.zeros(self.n_joints)

        # ROS 接口
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/joint_command', Float64MultiArray, self._cmd_callback)

        # MuJoCo 查看器
        self.viewer = None
        self.headless = headless
        if not headless:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    def _cmd_callback(self, msg):
        if len(msg.data) >= self.n_joints:
            self.cmd_positions = np.array(msg.data[:self.n_joints])
        else:
            rospy.logwarn_throttle(5, "Received joint_command with insufficient dims (%d < %d)",
                                   len(msg.data), self.n_joints)

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = [float(self.data.qpos[jid]) for jid in self.mj_joint_ids]
        msg.velocity = [float(self.data.qvel[jid]) for jid in self.mj_joint_ids]
        self.joint_state_pub.publish(msg)

    def _apply_control(self):
        """将 /joint_command 中的目标位置写入 MuJoCo 执行器。"""
        for i, jid in enumerate(self.mj_joint_ids):
            self.data.ctrl[i] = self.cmd_positions[i]

    def run(self):
        rate = rospy.Rate(int(1.0 / self.model.opt.timestep))
        rospy.loginfo("MuJoCo simulation running at %.0f Hz", 1.0 / self.model.opt.timestep)

        viewer_running = self.viewer is not None

        while not rospy.is_shutdown():
            if not self.headless:
                if not self.viewer.is_running():
                    break

            self._apply_control()
            mujoco.mj_step(self.model, self.data)
            self._publish_joint_state()

            if not self.headless:
                self.viewer.sync()

            rate.sleep()

        if self.viewer:
            self.viewer.close()

        rospy.loginfo("MuJoCo simulation stopped.")


def main():
    headless = rospy.get_param('~headless', False)
    sim = DofbotMujocoSim(headless=headless)
    sim.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
