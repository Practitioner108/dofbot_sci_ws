#!/usr/bin/env python3
"""Waypoint Executor: MoveIt trajectory → 逐点 I2C 驱动串口舵机。

替代 FollowJointTrajectory，适配 DS-SY15A 的"目标点+时间"控制范式。
每个关键路径点给予完整 move_time，让舵机内部梯形曲线完整展开。
"""
import math
import time
import threading
from collections import namedtuple

import rospy
import actionlib
import smbus
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

# ── 舵机参数（与 dofbot_hw_interface.cpp 完全一致） ──
ServoParams = namedtuple("ServoParams", [
    "raw_min", "raw_max", "angle_min_deg", "angle_max_deg",
    "mid_deviation", "direction"
])

SERVO_PARAMS = [
    # idx 0: base_rotation
    ServoParams(96, 4000, -150.0, 150.0, -47, 1),
    # idx 1: upper_arm
    ServoParams(96, 4000, -162.0, 162.0, -47, -1),
    # idx 2: forearm
    ServoParams(96, 4000, -163.0, 163.0, -50, -1),
    # idx 3: wrist_pitch
    ServoParams(96, 4000, -162.0, 162.0, -47, -1),
    # idx 4: wrist_roll
    ServoParams(96, 4000, -165.0, 165.0, -562, 1),
    # idx 5: gripper (使用 gripperToRaw 分段映射, 此处参数仅占位)
    ServoParams(96, 4000, -90.0,  90.0, -10, 1),
]

JOINT_NAMES = [
    "base_rotation_joint", "upper_arm_joint", "forearm_joint",
    "wrist_pitch_joint", "wrist_roll_joint", "gripper_left_joint"
]

# ── I2C 常量 ──
I2C_BUS = 1
I2C_ADDR = 0x15
REG_TIME = 0x1E
REG_POS = 0x1D
REG_POS_BASE = 0x31  # 舵机 0~5 位置寄存器基址

# ── 运动参数 ──
MIN_STEP = 0.122       # 最小选取步长 rad (7°)，减少步数降低暴冲
EFFECTIVE_SPEED = 2.45  # 舵机有效追踪速度 rad/s (200°/s × 0.7 梯形系数)
MIN_MOVE_TIME = 80      # 最小运行时间 ms
MAX_MOVE_TIME = 800     # 最大运行时间 ms
GOAL_TOLERANCE = 0.05   # 到位容差 rad (~2.9°)


# ── 转换函数（与 C++ radianToRaw / gripperToRaw 完全一致） ──

def radian_to_raw(rad, p):
    deg = rad * p.direction * 180.0 / math.pi
    ratio = (deg - p.angle_min_deg) / (p.angle_max_deg - p.angle_min_deg)
    raw_d = p.raw_min + ratio * (p.raw_max - p.raw_min) + p.mid_deviation
    return int(max(p.raw_min, min(p.raw_max, raw_d)))


def raw_to_radian(raw, p):
    ratio = float(raw - p.raw_min - p.mid_deviation) / (p.raw_max - p.raw_min)
    deg = p.angle_min_deg + ratio * (p.angle_max_deg - p.angle_min_deg)
    return p.direction * deg * math.pi / 180.0


def gripper_to_raw(pos_m, p):
    pos = pos_m * p.direction
    pos_touch = -0.0015
    pos_open  = 0.03
    pos_close = -0.03
    raw_open  = 1039
    raw_touch = 2477
    raw_close = 2992
    if pos < pos_touch:
        ratio = (pos - pos_close) / (pos_touch - pos_close)
        raw_d = raw_close + ratio * (raw_touch - raw_close)
    else:
        ratio = (pos - pos_touch) / (pos_open - pos_touch)
        raw_d = raw_touch + ratio * (raw_open - raw_touch)
    return int(max(raw_open, min(raw_close, raw_d)))


def raw_to_gripper(raw, p):
    pos_touch = -0.0015
    pos_open  = 0.03
    pos_close = -0.03
    raw_open  = 1039
    raw_touch = 2477
    raw_close = 2992
    if raw < raw_touch:
        ratio = float(raw - raw_open) / (raw_touch - raw_open)
        pos = pos_open + ratio * (pos_touch - pos_open)
    else:
        ratio = float(raw - raw_touch) / (raw_close - raw_touch)
        pos = pos_touch + ratio * (pos_close - pos_touch)
    return p.direction * pos


# ── I2C 通信 ──

class I2CBridge:
    def __init__(self):
        self._lock = threading.Lock()
        self._bus = None

    def _ensure_bus(self):
        if self._bus is None:
            self._bus = smbus.SMBus(I2C_BUS)

    def write_servos(self, joints_rad, move_time_ms):
        """将 6 路关节角度(rad)写入 MCU。先写时间(0x1E)，再写位置(0x1D)。"""
        # 转换 raw
        raw = [0] * 6
        for i in range(5):
            raw[i] = radian_to_raw(joints_rad[i], SERVO_PARAMS[i])
        raw[5] = gripper_to_raw(joints_rad[5], SERVO_PARAMS[5])

        # 拼装 0x1D 数据 (12 字节)
        buf_pos = [0] * 12
        for i in range(6):
            buf_pos[2 * i]     = (raw[i] >> 8) & 0x0F
            buf_pos[2 * i + 1] = raw[i] & 0xFF

        # 拼装 0x1E 数据 (2 字节大端)
        buf_time = [(move_time_ms >> 8) & 0xFF, move_time_ms & 0xFF]

        with self._lock:
            self._ensure_bus()
            self._bus.write_i2c_block_data(I2C_ADDR, REG_TIME, buf_time)
            self._bus.write_i2c_block_data(I2C_ADDR, REG_POS, buf_pos)

    def read_positions(self):
        """读取全部 6 路舵机 raw 值，返回 rad 列表 (gripper 返回 m)。"""
        result = [0.0] * 6
        with self._lock:
            self._ensure_bus()
            for i in range(6):
                reg = REG_POS_BASE + i
                # 写触发器 [reg, 0x01]
                self._bus.write_i2c_block_data(I2C_ADDR, reg, [0x01])
                time.sleep(0.006)  # 6ms > 手册最低 5ms
                # 指针复位
                self._bus.write_byte_data(I2C_ADDR, reg, 0x00)
                # 读 2 字节
                data = self._bus.read_i2c_block_data(I2C_ADDR, reg, 2)
                raw = (data[0] << 8) | data[1]
                if 96 <= raw <= 4000:
                    if i < 5:
                        result[i] = raw_to_radian(raw, SERVO_PARAMS[i])
                    else:
                        result[i] = raw_to_gripper(raw, SERVO_PARAMS[i])
        return result


# ── 路径点提取 ──

def extract_keypoints(trajectory_points, joint_count=6):
    if len(trajectory_points) <= 2:
        return list(trajectory_points)

    keys = [trajectory_points[0]]
    prev = trajectory_points[0].positions

    for pt in trajectory_points[1:-1]:
        max_diff = max(abs(pt.positions[i] - prev[i]) for i in range(min(joint_count, 5)))
        if max_diff >= MIN_STEP:
            keys.append(pt)
            prev = pt.positions

    if keys[-1] != trajectory_points[-1]:
        keys.append(trajectory_points[-1])
    return keys


def compute_move_time(target, current, joint_count=6):
    max_diff = max(abs(target[i] - current[i]) for i in range(min(joint_count, 5)))
    if max_diff < 0.001:
        return MIN_MOVE_TIME
    mt = int(max_diff / EFFECTIVE_SPEED * 1000.0)
    return max(MIN_MOVE_TIME, min(MAX_MOVE_TIME, mt))


# ── ROS Action Server ──

class WaypointExecutor:
    def __init__(self):
        self._i2c = I2CBridge()
        self._server = actionlib.SimpleActionServer(
            "/waypoint_executor/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._server.start()
        self._joint_sub = rospy.Subscriber(
            "/joint_states", JointState, self._joint_cb
        )
        self._current_pos = [0.0] * 6
        rospy.loginfo("WaypointExecutor: ready")

    def _joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                idx = JOINT_NAMES.index(name)
                self._current_pos[idx] = pos

    def _read_current(self):
        """从 I2C 直接读当前位置，同时更新缓存。"""
        try:
            pos = self._i2c.read_positions()
            self._current_pos = pos
            return pos
        except Exception as e:
            rospy.logwarn("I2C read failed: %s, using cached", e)
            return self._current_pos

    def _execute(self, goal):
        result = FollowJointTrajectoryResult()
        feedback = FollowJointTrajectoryFeedback()

        traj = goal.trajectory
        if not traj.points:
            result.error_code = result.INVALID_GOAL
            result.error_string = "Empty trajectory"
            self._server.set_aborted(result)
            return

        # 匹配关节名 → 全局索引，MoveIt 可能只发部分关节（如仅 arm 不含 gripper）
        # gripper (index 5) 始终排除，由夹爪专用指令控制
        traj_indices = []   # traj_joint_idx → global_idx (JOINT_NAMES 中的位置)
        global_indices = []  # global_idx → traj_joint_idx
        for gi, jn in enumerate(JOINT_NAMES):
            if gi == 5:  # skip gripper
                continue
            try:
                ti = traj.joint_names.index(jn)
                traj_indices.append(ti)
                global_indices.append(gi)
            except ValueError:
                pass  # 该关节不在本次轨迹中，跳过

        if not traj_indices:
            rospy.logerr("No matching joints in trajectory")
            result.error_code = result.INVALID_JOINTS
            self._server.set_aborted(result)
            return

        n_active = len(traj_indices)
        rospy.loginfo("Executing %d joints: %s", n_active,
                      [JOINT_NAMES[gi] for gi in global_indices])

        # 提取关键路径点
        keypoints = extract_keypoints(traj.points, n_active)
        rospy.loginfo("Trajectory %d points → %d keypoints",
                      len(traj.points), len(keypoints))

        # 读取当前位置（完整 6 路，用于补全轨迹中未包含的关节）
        current = self._read_current()

        # 逐点执行
        prev_t = rospy.Duration(0)
        for idx, kp in enumerate(keypoints):
            if self._server.is_preempt_requested():
                result.error_code = result.SUCCESS
                self._server.set_preempted(result)
                return

            # 构建完整目标：轨迹中的关节取目标值，其余保持当前位置
            target = list(current)
            for ai in range(n_active):
                target[global_indices[ai]] = kp.positions[traj_indices[ai]]

            # 用 MoveIt 本身的时序，而非自己重新算时间
            t_diff = kp.time_from_start - prev_t
            move_time = int(t_diff.to_sec() * 1000.0)
            if move_time < MIN_MOVE_TIME:
                move_time = MIN_MOVE_TIME
            prev_t = kp.time_from_start

            rospy.loginfo("Keypoint %d/%d: move_time=%dms, joints=%s",
                          idx + 1, len(keypoints), move_time,
                          [round(target[gi], 2) for gi in global_indices])

            # 写 I2C（完整 6 路）
            try:
                self._i2c.write_servos(target, move_time)
            except Exception as e:
                rospy.logerr("I2C write failed: %s", e)
                result.error_code = result.PATH_TOLERANCE_VIOLATED
                self._server.set_aborted(result)
                return

            # 等 80% 的 move_time，留 20% 缓冲让舵机自然过渡
            wait_time = move_time * 0.8 / 1000.0
            if wait_time > 0:
                time.sleep(wait_time)

            # 读实际位置
            current = self._read_current()

            # 检查到位（仅活动关节, 不含 gripper）
            err = max(abs(target[gi] - current[gi]) for gi in global_indices)
            if err > GOAL_TOLERANCE:
                rospy.logwarn("Keypoint %d/%d: goal err %.4f rad > tolerance, "
                              "continuing", idx + 1, len(keypoints), err)

            # 发布 feedback
            feedback.header.stamp = rospy.Time.now()
            feedback.joint_names = traj.joint_names
            feedback.desired = kp
            feedback.actual.positions = [current[gi] for gi in global_indices]
            self._server.publish_feedback(feedback)

        rospy.loginfo("Trajectory complete: %d keypoints executed", len(keypoints))
        result.error_code = result.SUCCESS
        self._server.set_succeeded(result)


# ── main ──

def main():
    rospy.init_node("waypoint_executor")
    executor = WaypointExecutor()
    rospy.spin()


if __name__ == "__main__":
    main()
