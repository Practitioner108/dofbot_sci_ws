//  DOFBOT 5-DOF 机械臂解析逆运动学插件。
//  基于闭式代数解的 MoveIt KinematicsBase 插件实现，用于替代默认的数值迭代求解器。
//  5-DOF 机械臂无法满足任意 6D 位姿请求，因此需配合 position_only_ik: True 使用。
//
//  算法概要：
//    1. 将 wrist_roll_link 的目标位姿分解为位置和姿态两部分。
//    2. 在底座旋转轴投影平面上求解 q1（偏航），处理天顶奇异点。
//    3. 将三维目标退推至二维二连杆平面，用余弦定理求解 q2、q3、q4。
//    4. 从目标姿态矩阵中剥离旋转分量，求解 q5（腕部滚转）。
//    5. 对每个关节在其限位范围内搜索所有 2π 等效解，输出所有可行构型。
//    6. 从可行解中选择距种子关节角最近的解。

#include <ros/ros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <map>
#include <memory>

namespace dofbot_kinematics {

class DofbotIKPlugin : public kinematics::KinematicsBase {
private:
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::vector<double> joint_min_;
    std::vector<double> joint_max_;

    // wrist_roll_link 到自定义 TCP 的偏移矩阵
    Eigen::Isometry3d tip_to_wrist_offset_;

    // 运动学常量，与 dofbot_description/urdf/dofbot.xacro 中的关节原点一致
    const double L_BASE = 0.1075;   // 底座高度：base_rotation_joint(z=0.064) + upper_arm_joint(z=0.0435)
    const double L2 = 0.08285;      // 大臂长度：upper_arm_joint -> forearm_joint
    const double L3 = 0.08285;      // 小臂长度：forearm_joint -> wrist_pitch_joint

    // wrist_pitch_joint 到 wrist_roll_joint 的平移分量
    const double L4_X = -0.18385;
    const double L4_Y = -0.00215;
    const double L4_Z = -0.000605;

    // 将角度归一化到 [-pi, pi]
    double normalizeAngle(double angle) const {
        double a = std::fmod(angle, 2.0 * M_PI);
        if (a > M_PI) a -= 2.0 * M_PI;
        if (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // 在关节限位内搜索给定角度的所有 2π 等效解，去除重复值
    std::vector<double> getValidAngles(double angle, int joint_idx) const {
        std::vector<double> valid_angles;
        double min_limit = joint_min_[joint_idx];
        double max_limit = joint_max_[joint_idx];

        double norm_angle = normalizeAngle(angle);
        double k_vals[] = {0.0, 2.0 * M_PI, -2.0 * M_PI, 4.0 * M_PI, -4.0 * M_PI};

        for (double k : k_vals) {
            double test_angle = norm_angle + k;
            if (test_angle >= min_limit - 1e-4 && test_angle <= max_limit + 1e-4) {
                test_angle = std::max(min_limit, std::min(max_limit, test_angle));
                bool duplicate = false;
                for (double existing : valid_angles) {
                    if (std::abs(existing - test_angle) < 1e-3) {
                        duplicate = true; break;
                    }
                }
                if (!duplicate) valid_angles.push_back(test_angle);
            }
        }
        return valid_angles;
    }

    // 解析 IK 主计算：给定 wrist_roll_link 目标位姿和底座旋转 seed，返回所有可行解
    std::vector<std::vector<double>> calculateIK(const Eigen::Isometry3d& target_wrist_pose,
                                                  double seed_q1) const {
        std::vector<std::vector<double>> solutions;
        double px = target_wrist_pose.translation().x();
        double py = target_wrist_pose.translation().y();
        double pz = target_wrist_pose.translation().z();
        Eigen::Matrix3d R = target_wrist_pose.rotation();

        double dz = L4_Z;
        double val = px * px + py * py - dz * dz;
        if (val < 1e-8) val = 0.0;

        // 正负两个手臂构型分支（前探 / 后探）
        double Rp_sols[2] = {std::sqrt(val), -std::sqrt(val)};

        for (int i = 0; i < 2; ++i) {
            double Rp = Rp_sols[i];

            // q1: 底座偏航，从 XY 投影平面求解
            double y_q1 = py * dz + px * Rp;
            double x_q1 = px * dz - py * Rp;
            double q1 = 0.0;

            // 天顶奇异点：末端在底座旋转轴正上方时，q1 任意解均成立。
            // 此时锁定 q1 为 seed 值，避免 180° 跳变。
            if (val < 1e-8 || (std::abs(y_q1) < 1e-6 && std::abs(x_q1) < 1e-6)) {
                q1 = seed_q1;
            } else {
                q1 = std::atan2(y_q1, x_q1);
            }

            // 剥离 q1 旋转分量：M = Rz(-q1) * R，从 M 中提取俯仰 theta 和腕部滚转 q5
            Eigen::Matrix3d M = Eigen::AngleAxisd(-q1, Eigen::Vector3d::UnitZ())
                                    .toRotationMatrix() * R;
            double theta = std::atan2(-M(1, 2), M(2, 2));
            double q5 = std::atan2(-M(0, 1), M(0, 0));

            // 将三维目标位置退推至二维连杆平面 (XZ -> xy)
            double Y_w = Rp + L4_X * std::sin(theta) + L4_Y * std::cos(theta);
            double Z_w = pz - L_BASE + L4_X * std::cos(theta) - L4_Y * std::sin(theta);

            // upper_arm_joint 的 rpy 预设旋转导致坐标系倒置，
            // 将 Y 坐标恢复为正方向，避免臂向后弯折
            double x_plane = Z_w;
            double y_plane = Y_w;

            // 余弦定理求解 q3（肘关节）
            double D2 = x_plane * x_plane + y_plane * y_plane;
            double cos_q3 = (D2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
            if (cos_q3 > 1.0) cos_q3 = 1.0;
            if (cos_q3 < -1.0) cos_q3 = -1.0;

            double acos_q3 = std::acos(cos_q3);
            double q3_candidates[2] = {acos_q3, -acos_q3};   // 肘部朝上 / 肘部朝下

            for (int j = 0; j < 2; ++j) {
                double q3 = q3_candidates[j];
                double q2 = std::atan2(y_plane, x_plane)
                            - std::atan2(L3 * std::sin(q3), L2 + L3 * std::cos(q3));
                double q4 = theta - q2 - q3;

                // 对每个关节搜索限位内的所有等效角度
                std::vector<double> v_q1 = getValidAngles(q1, 0);
                std::vector<double> v_q2 = getValidAngles(q2, 1);
                std::vector<double> v_q3 = getValidAngles(q3, 2);
                std::vector<double> v_q4 = getValidAngles(q4, 3);
                std::vector<double> v_q5 = getValidAngles(q5, 4);

                for (double sq1 : v_q1) for (double sq2 : v_q2) for (double sq3 : v_q3)
                for (double sq4 : v_q4) for (double sq5 : v_q5)
                    solutions.push_back({sq1, sq2, sq3, sq4, sq5});
            }
        }
        return solutions;
    }

    // 正运动学：关节角 → 各连杆变换矩阵
    std::map<std::string, Eigen::Isometry3d> computeAllFK(const std::vector<double>& q) const {
        std::map<std::string, Eigen::Isometry3d> poses;
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        poses[getBaseFrame()] = T;

        T = T * Eigen::Translation3d(0, 0, 0.064)
              * Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitZ());
        poses["base_rotation_link"] = T;

        T = T * Eigen::Translation3d(0, 0, 0.0435)
              * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitZ());
        poses["upper_arm_link"] = T;

        T = T * Eigen::Translation3d(-0.08285, 0, 0)
              * Eigen::AngleAxisd(q[2], Eigen::Vector3d::UnitZ());
        poses["forearm_link"] = T;

        T = T * Eigen::Translation3d(-0.08285, 0, 0)
              * Eigen::AngleAxisd(q[3], Eigen::Vector3d::UnitZ());
        poses["wrist_link"] = T;

        T = T * Eigen::Translation3d(-0.18385, -0.00215, -0.000605)
              * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitZ());
        poses["wrist_roll_link"] = T;
        return poses;
    }

public:
    bool initialize(const moveit::core::RobotModel& robot_model,
                    const std::string& group_name,
                    const std::string& base_frame,
                    const std::vector<std::string>& tip_frames,
                    double search_discretization) override {
        storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
        joint_names_ = {"base_rotation_joint", "upper_arm_joint", "forearm_joint",
                        "wrist_pitch_joint", "wrist_roll_joint"};
        link_names_ = tip_frames_;

        const moveit::core::JointModelGroup* jmg = robot_model.getJointModelGroup(group_name);
        if (!jmg) {
            ROS_ERROR_NAMED("dofbot_ik",
                            "Failed to retrieve JointModelGroup from RobotModel.");
            return false;
        }

        for (const std::string& jn : joint_names_) {
            const moveit::core::JointModel* jm = jmg->getJointModel(jn);
            if (jm && !jm->getVariableBounds().empty()) {
                joint_min_.push_back(jm->getVariableBounds()[0].min_position_);
                joint_max_.push_back(jm->getVariableBounds()[0].max_position_);
            } else {
                // 读取失败时使用 URDF 中的默认限位
                if (jn == "wrist_roll_joint") {
                    joint_min_.push_back(-1.5708); joint_max_.push_back(3.1416);
                } else {
                    joint_min_.push_back(-1.5708); joint_max_.push_back(1.5708);
                }
            }
        }

        // RobotState 构造函数在 Noetic 中要求 shared_ptr<RobotModel>。
        // 此处的 robot_model 由 MoveIt KinematicsPluginLoader 管理生命周期，
        // 使用空删除器避免二次释放 MoveIt 持有的内存。
        //
        // SAFETY: robot_model 是 const 引用，其生命周期由 MoveIt 插件加载器保证。
        // KinematicsPluginLoader 在插件卸载时销毁 RobotModel，而插件卸载发生在
        // 所有使用该 RobotModel 的 RobotState 对象销毁之后——MoveIt 保证此时序。
        // 因此由本 shared_ptr 创建的临时 RobotState 不会出现 use-after-free。
        std::shared_ptr<const moveit::core::RobotModel> robot_model_ptr(
            &robot_model, [](const moveit::core::RobotModel*){});
        moveit::core::RobotState state(robot_model_ptr);
        state.setToDefaultValues();

        // 计算 TCP 相对于腕部的偏移，供 IK 中从 TCP 位姿反推到腕部使用
        if (!tip_frames_.empty()) {
            Eigen::Isometry3d T_wrist = state.getGlobalLinkTransform("wrist_roll_link");
            Eigen::Isometry3d T_tip   = state.getGlobalLinkTransform(tip_frames_[0]);
            tip_to_wrist_offset_ = T_wrist.inverse() * T_tip;
        } else {
            tip_to_wrist_offset_ = Eigen::Isometry3d::Identity();
        }

        ROS_INFO_NAMED("dofbot_ik", "DOFBOT Exact Analytic IK Plugin Initialized.");
        ROS_WARN_ONCE_NAMED("dofbot_ik",
            "[IMPORTANT] This is a 5-DOF arm. "
            "Set 'position_only_ik: True' in your kinematics.yaml.");

        return true;
    }

    const std::vector<std::string>& getJointNames() const override {
        return joint_names_;
    }
    const std::vector<std::string>& getLinkNames() const override {
        return link_names_;
    }

    // 正运动学：返回请求的连杆位姿。
    // 对于臂部连杆使用代数 FK；对于 TCP 通过 wrist_roll_link + 偏移计算；
    // 对于夹爪等非臂部连杆返回 false，由 MoveIt 的 RobotState 树形 FK 处理。
    bool getPositionFK(const std::vector<std::string>& req_link_names,
                       const std::vector<double>& joint_angles,
                       std::vector<geometry_msgs::Pose>& poses) const override {
        if (joint_angles.size() != 5) return false;
        poses.resize(req_link_names.size());

        std::map<std::string, Eigen::Isometry3d> all_fk = computeAllFK(joint_angles);

        for (size_t i = 0; i < req_link_names.size(); ++i) {
            const std::string& name = req_link_names[i];

            if (all_fk.count(name)) {
                poses[i] = tf2::toMsg(all_fk.at(name));
            } else if (!tip_frames_.empty() && name == tip_frames_[0]) {
                poses[i] = tf2::toMsg(all_fk.at("wrist_roll_link") * tip_to_wrist_offset_);
            } else {
                return false;
            }
        }
        return true;
    }

    // 以下四个重载将调用逐级委托到最终的 searchPositionIK 实现
    bool getPositionIK(const geometry_msgs::Pose& ik_pose,
                       const std::vector<double>& ik_seed_state,
                       std::vector<double>& solution,
                       moveit_msgs::MoveItErrorCodes& error_code,
                       const kinematics::KinematicsQueryOptions& options) const override {
        return searchPositionIK(ik_pose, ik_seed_state, 0.1, std::vector<double>(),
                                solution, IKCallbackFn(), error_code, options);
    }
    bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
                          const std::vector<double>& ik_seed_state,
                          double timeout, std::vector<double>& solution,
                          moveit_msgs::MoveItErrorCodes& error_code,
                          const kinematics::KinematicsQueryOptions& options) const override {
        return searchPositionIK(ik_pose, ik_seed_state, timeout, std::vector<double>(),
                                solution, IKCallbackFn(), error_code, options);
    }
    bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
                          const std::vector<double>& ik_seed_state,
                          double timeout,
                          const std::vector<double>& consistency_limits,
                          std::vector<double>& solution,
                          moveit_msgs::MoveItErrorCodes& error_code,
                          const kinematics::KinematicsQueryOptions& options) const override {
        return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                                solution, IKCallbackFn(), error_code, options);
    }
    bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
                          const std::vector<double>& ik_seed_state,
                          double timeout, std::vector<double>& solution,
                          const IKCallbackFn& solution_callback,
                          moveit_msgs::MoveItErrorCodes& error_code,
                          const kinematics::KinematicsQueryOptions& options) const override {
        return searchPositionIK(ik_pose, ik_seed_state, timeout, std::vector<double>(),
                                solution, solution_callback, error_code, options);
    }

    // IK 求解入口：对外暴露的唯一 searchPositionIK 实现
    bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
                          const std::vector<double>& ik_seed_state,
                          double /*timeout*/,
                          const std::vector<double>& consistency_limits,
                          std::vector<double>& solution,
                          const IKCallbackFn& solution_callback,
                          moveit_msgs::MoveItErrorCodes& error_code,
                          const kinematics::KinematicsQueryOptions& options) const override {

        Eigen::Isometry3d target_tip_pose;
        tf2::fromMsg(ik_pose, target_tip_pose);

        // 从 TCP 位姿反推到 wrist_roll_link
        Eigen::Isometry3d target_wrist_pose = target_tip_pose * tip_to_wrist_offset_.inverse();

        std::vector<double> safe_seed = (ik_seed_state.size() == 5)
            ? ik_seed_state : std::vector<double>(5, 0.0);
        std::vector<std::vector<double>> solutions = calculateIK(target_wrist_pose, safe_seed[0]);

        int best_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < solutions.size(); ++i) {
            // 连续解一致性约束：相邻解的关节变化不得超过限制
            if (!consistency_limits.empty() && consistency_limits.size() == 5) {
                bool consistent = true;
                for (size_t j = 0; j < 5; ++j) {
                    double diff = std::abs(solutions[i][j] - safe_seed[j]);
                    if (diff > consistency_limits[j]) { consistent = false; break; }
                }
                if (!consistent) continue;
            }

            // 严格模式：IK 解经 FK 验证，位置误差 < 1mm 且角度误差 < 0.05rad
            if (!options.return_approximate_solution) {
                std::map<std::string, Eigen::Isometry3d> actual_fk = computeAllFK(solutions[i]);
                Eigen::Isometry3d actual_wrist = actual_fk.at("wrist_roll_link");
                Eigen::AngleAxisd err_rot(
                    target_wrist_pose.rotation().inverse() * actual_wrist.rotation());
                Eigen::Vector3d err_pos =
                    target_wrist_pose.translation() - actual_wrist.translation();

                if (err_pos.norm() > 1e-3 || std::abs(err_rot.angle()) > 0.05) {
                    continue;
                }
            }

            // 碰撞检测
            if (solution_callback) {
                solution_callback(ik_pose, solutions[i], error_code);
                if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) continue;
            }

            // 选择距 seed 最近（L2 范数最小）的解
            double d = 0;
            for (size_t j = 0; j < 5; ++j) {
                double diff = std::abs(solutions[i][j] - safe_seed[j]);
                d += diff * diff;
            }

            if (d < min_dist) {
                min_dist = d;
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx >= 0) {
            solution = solutions[best_idx];
            error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
            return true;
        }

        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
    }
};

} // namespace dofbot_kinematics

PLUGINLIB_EXPORT_CLASS(dofbot_kinematics::DofbotIKPlugin, kinematics::KinematicsBase)
