#include <ros/ros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <vector>

namespace dofbot_kinematics {

class DofbotIKPlugin : public kinematics::KinematicsBase {
private:
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::vector<double> joint_min_;
    std::vector<double> joint_max_;

    // 检查解是否位于软硬限位内部
    bool isValid(const std::vector<double>& q) const {
        if (q.size() != 5) return false;
        for (size_t i = 0; i < 5; ++i) {
            if (q[i] < joint_min_[i] - 1e-4 || q[i] > joint_max_[i] + 1e-4) {
                return false;
            }
        }
        return true;
    }

    // 评估解与当前机械臂状态(Seed)的角距离，用于优选平滑动作
    double distance(const std::vector<double>& q1, const std::vector<double>& q2) const {
        double d = 0;
        for (size_t i = 0; i < 5; ++i) {
            double diff = q1[i] - q2[i];
            while (diff > M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;
            d += diff * diff;
        }
        return d;
    }

    // 代数降维法核心逆向运动学解析
    std::vector<std::vector<double>> calculateIK(const Eigen::Isometry3d& target_pose) const {
        std::vector<std::vector<double>> solutions;
        double px = target_pose.translation().x();
        double py = target_pose.translation().y();
        double pz = target_pose.translation().z();
        Eigen::Matrix3d R = target_pose.rotation();

        // 吸收 Joint 5 的微小轴外偏置 dz = -0.000605
        double dz = -0.000605;
        double val = px * px + py * py - dz * dz;
        if (val < 0) val = 0.0; 

        double Rp_sols[2] = {std::sqrt(val), -std::sqrt(val)};

        for (int i = 0; i < 2; ++i) {
            double Rp = Rp_sols[i];
            
            // 1. 求解 Joint 1 (Yaw), 利用偏移圆投影解耦死锁
            double est_q1 = std::atan2(px * Rp - py * dz, -px * dz - py * Rp);

            double R00 = R(0, 0), R01 = R(0, 1), R02 = R(0, 2);
            double R12 = R(1, 2);
            double R20 = R(2, 0), R21 = R(2, 1), R22 = R(2, 2);
            
            // 2. 将目标的 Z 轴（接近向量）向机械臂物理平面投影，提取可达的最佳俯仰角 (theta = q2+q3+q4)
            double theta = std::atan2(R02 * std::sin(est_q1) - R12 * std::cos(est_q1), R22);

            // 3. 剥离末端执行器的偏差量 y = -0.00215 和连杆长度，映射至腕关节(Joint 4)平面点
            double Pz_wrist = pz - 0.1075;
            double x_wrist = Pz_wrist - 0.18385 * std::cos(theta) + 0.00215 * std::sin(theta);
            double y_wrist = Rp - 0.18385 * std::sin(theta) - 0.00215 * std::cos(theta);

            // 4. 标准平面 2-Link 逆向解 (针对 Joint 2 和 Joint 3)
            double L = 0.08285;
            double D2 = x_wrist * x_wrist + y_wrist * y_wrist;
            double cos_q3 = (D2 - 2 * L * L) / (2 * L * L);

            // 容差限幅，保证即使用户强行拖拽超出臂长，也能达到最远距离
            if (cos_q3 > 1.0) cos_q3 = 1.0;
            if (cos_q3 < -1.0) cos_q3 = -1.0;

            double acos_q3 = std::acos(cos_q3);
            double q3_candidates[2] = {acos_q3, -acos_q3}; // 涵盖肘部向上和向下的解

            for (int j = 0; j < 2; ++j) {
                double est_q3 = q3_candidates[j];
                double est_q2 = std::atan2(y_wrist, x_wrist) - std::atan2(L * std::sin(est_q3), L + L * std::cos(est_q3));
                double est_q4 = theta - est_q2 - est_q3;

                // 5. 反向投影提取 Joint 5 的滚转角 (Roll)
                double est_q5;
                if (std::abs(std::sin(theta)) > 1e-4) {
                    est_q5 = std::atan2(R20 * std::sin(theta), R21 * std::sin(theta));
                } else {
                    // 处理指向正下/正上的万向锁退化
                    if (R22 > 0) est_q5 = std::atan2(-R01, R00) - est_q1;
                    else         est_q5 = est_q1 - std::atan2(R01, R00);
                }

                // 统一约束角周期至 [-PI, PI]
                auto normalize_angle = [](double a) {
                    a = std::fmod(a, 2.0 * M_PI);
                    if (a > M_PI) a -= 2.0 * M_PI;
                    if (a < -M_PI) a += 2.0 * M_PI;
                    return a;
                };

                solutions.push_back({normalize_angle(est_q1), 
                                     normalize_angle(est_q2), 
                                     normalize_angle(est_q3), 
                                     normalize_angle(est_q4), 
                                     normalize_angle(est_q5)});
            }
        }
        return solutions;
    }

public:
    virtual bool initialize(const moveit::core::RobotModel& robot_model,
                            const std::string& group_name,
                            const std::string& base_frame,
                            const std::vector<std::string>& tip_frames,
                            double search_discretization) override {
        storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5"};
        link_names_ = tip_frames_; 
        
        // 动态读取机器人的 URDF 限位，若读取失败则设置安全兜底
        const moveit::core::JointModelGroup* jmg = robot_model.getJointModelGroup(group_name);
        if (jmg) {
            for (const std::string& jn : joint_names_) {
                const moveit::core::JointModel* jm = jmg->getJointModel(jn);
                if (jm && !jm->getVariableBounds().empty()) {
                    joint_min_.push_back(jm->getVariableBounds()[0].min_position_);
                    joint_max_.push_back(jm->getVariableBounds()[0].max_position_);
                } else {
                    joint_min_.push_back(-M_PI);
                    joint_max_.push_back(M_PI);
                }
            }
        } else {
            joint_min_ = {-1.5708, -1.5708, -1.5708, -1.5708, -1.5708};
            joint_max_ = { 1.5708,  1.5708,  1.5708,  1.5708,  3.1416};
        }
        return true;
    }

    const std::vector<std::string>& getJointNames() const override { return joint_names_; }
    const std::vector<std::string>& getLinkNames() const override { return link_names_; }

    // 正向运动学重写以兼容环境计算
    bool getPositionFK(const std::vector<std::string>& req_link_names,
                       const std::vector<double>& joint_angles,
                       std::vector<geometry_msgs::Pose>& poses) const override {
        if (joint_angles.size() != 5) return false;

        poses.resize(req_link_names.size());
        for (size_t i = 0; i < req_link_names.size(); ++i) {
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            if (req_link_names[i] == "base_link") { poses[i] = tf2::toMsg(T); continue; }

            T = T * Eigen::Translation3d(0, 0, 0.064) * Eigen::AngleAxisd(joint_angles[0], Eigen::Vector3d::UnitZ());
            if (req_link_names[i] == "link1") { poses[i] = tf2::toMsg(T); continue; }

            T = T * Eigen::Translation3d(0, 0, 0.0435) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(joint_angles[1], Eigen::Vector3d::UnitZ());
            if (req_link_names[i] == "link2") { poses[i] = tf2::toMsg(T); continue; }

            T = T * Eigen::Translation3d(-0.08285, 0, 0) * Eigen::AngleAxisd(joint_angles[2], Eigen::Vector3d::UnitZ());
            if (req_link_names[i] == "link3") { poses[i] = tf2::toMsg(T); continue; }

            T = T * Eigen::Translation3d(-0.08285, 0, 0) * Eigen::AngleAxisd(joint_angles[3], Eigen::Vector3d::UnitZ());
            if (req_link_names[i] == "link4") { poses[i] = tf2::toMsg(T); continue; }

            T = T * Eigen::Translation3d(-0.18385, -0.00215, -0.000605) * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(joint_angles[4], Eigen::Vector3d::UnitZ());
            poses[i] = tf2::toMsg(T); 
        }
        return true;
    }

    // ==== IK 方法包装映射 ====
    bool getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override {
        return searchPositionIK(ik_pose, ik_seed_state, 0.1, std::vector<double>(), solution, IKCallbackFn(), error_code, options);
    }
    bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override {
        return searchPositionIK(ik_pose, ik_seed_state, timeout, std::vector<double>(), solution, IKCallbackFn(), error_code, options);
    }
    bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override {
        return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code, options);
    }
    bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override {
        return searchPositionIK(ik_pose, ik_seed_state, timeout, std::vector<double>(), solution, solution_callback, error_code, options);
    }

    bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override {
        
        Eigen::Isometry3d target_pose;
        tf2::fromMsg(ik_pose, target_pose);

        std::vector<std::vector<double>> solutions = calculateIK(target_pose);

        int best_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        // 筛选解空间中最近似种子状态的最优解
        for (size_t i = 0; i < solutions.size(); ++i) {
            if (!isValid(solutions[i])) continue;

            if (!consistency_limits.empty()) {
                bool consistent = true;
                for (size_t j = 0; j < 5; ++j) {
                    double diff = std::abs(solutions[i][j] - ik_seed_state[j]);
                    diff = std::min(diff, 2 * M_PI - diff); 
                    if (diff > consistency_limits[j]) { consistent = false; break; }
                }
                if (!consistent) continue;
            }

            if (solution_callback) {
                solution_callback(ik_pose, solutions[i], error_code);
                if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) continue;
            }

            double dist = distance(solutions[i], ik_seed_state);
            if (dist < min_dist) {
                min_dist = dist;
                best_idx = i;
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

// 向 MoveIt 系统注册并导出插件类
PLUGINLIB_EXPORT_CLASS(dofbot_kinematics::DofbotIKPlugin, kinematics::KinematicsBase)