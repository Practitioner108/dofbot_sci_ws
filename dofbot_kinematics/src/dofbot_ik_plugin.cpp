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
#include <memory> // ✅ 引入智能指针支持

namespace dofbot_kinematics {

class DofbotIKPlugin : public kinematics::KinematicsBase {
private:
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::vector<double> joint_min_;
    std::vector<double> joint_max_;

    // 【工业规范】：TCP 动态偏移矩阵，支持无缝解耦并挂载任何末端工具
    Eigen::Isometry3d tip_to_wrist_offset_;

    // 绝对精准匹配 URDF 尺寸的代数常量 (提取自原生的平移变换)
    const double L_BASE = 0.1075;   // joint1(0.064) + joint2(0.0435) 的绝对物理高度
    const double L2 = 0.08285;      // joint2 -> joint3 等效平面轴长绝对值
    const double L3 = 0.08285;      // joint3 -> joint4 等效平面轴长绝对值
    
    // 腕部参数严格遵循原生 URDF 负向矢量体系
    const double L4_X = -0.18385;   // joint4 -> joint5 X 向延伸
    const double L4_Y = -0.00215;   // joint4 -> joint5 Y 侧向微偏差
    const double L4_Z = -0.000605;  // joint4 -> joint5 Z 向微偏置

    // 周期同余映射 (Normalize to [-pi, pi])
    double normalizeAngle(double angle) const {
        double a = std::fmod(angle, 2.0 * M_PI);
        if (a > M_PI) a -= 2.0 * M_PI;
        if (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // 【完备性规范】：搜索物理限位内的所有同余解，杜绝多态流形丢解
    std::vector<double> getValidAngles(double angle, int joint_idx) const {
        std::vector<double> valid_angles;
        double min_limit = joint_min_[joint_idx];
        double max_limit = joint_max_[joint_idx];
        
        double norm_angle = normalizeAngle(angle);
        // 跨周期测试域，涵盖空间全流形
        double k_vals[] = {0.0, 2.0 * M_PI, -2.0 * M_PI, 4.0 * M_PI, -4.0 * M_PI};
        
        for (double k : k_vals) {
            double test_angle = norm_angle + k;
            if (test_angle >= min_limit - 1e-4 && test_angle <= max_limit + 1e-4) {
                test_angle = std::max(min_limit, std::min(max_limit, test_angle));
                // 精准容差去重
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

    // 纯代数流形逆向核心 (严丝合缝闭环推导，彻底规避万向锁)
    std::vector<std::vector<double>> calculateIK(const Eigen::Isometry3d& target_link5_pose, double seed_q1) const {
        std::vector<std::vector<double>> solutions;
        double px = target_link5_pose.translation().x();
        double py = target_link5_pose.translation().y();
        double pz = target_link5_pose.translation().z();
        Eigen::Matrix3d R = target_link5_pose.rotation();

        // 吸收底座死区圆柱面微偏误差
        double dz = L4_Z; 
        double val = px * px + py * py - dz * dz;
        
        // 【防微小浮点溢出钳制】
        if (val < 1e-8) val = 0.0;

        // 向前探及向后探的两组物理姿态分支
        double Rp_sols[2] = {std::sqrt(val), -std::sqrt(val)};

        for (int i = 0; i < 2; ++i) {
            double Rp = Rp_sols[i];
            
            // 严密的 Yaw (q1) 方程求解，完美规避四象限符号位反转 Bug
            double y_q1 = py * dz + px * Rp;
            double x_q1 = px * dz - py * Rp;
            double q1 = 0.0;

            // 【死区奇异点平滑过渡】：
            // 当落入正上方天顶极点死区（val < 1e-8）或计算极值趋零时，
            // 彻底冻结底座偏航，自适应沿用上层 Seed 的连续平滑态，彻底杜绝底座 180° 剧烈抽搐。
            if (val < 1e-8 || (std::abs(y_q1) < 1e-6 && std::abs(x_q1) < 1e-6)) {
                q1 = seed_q1;
            } else {
                q1 = std::atan2(y_q1, x_q1);
            }

            // 欧拉旋量矩阵剥离法：M = Rz(-q1) * target_R
            // 完美精准分离全局俯仰 (Theta) 与 腕部滚转 (q5)
            Eigen::Matrix3d M = Eigen::AngleAxisd(-q1, Eigen::Vector3d::UnitZ()).toRotationMatrix() * R;
            double theta = std::atan2(-M(1, 2), M(2, 2));
            double q5 = std::atan2(-M(0, 1), M(0, 0));

            // 将空间三维靶坐标系反向退推至二维二连杆虚拟平面
            double Y_w = Rp + L4_X * std::sin(theta) + L4_Y * std::cos(theta);
            double Z_w = pz - L_BASE + L4_X * std::cos(theta) - L4_Y * std::sin(theta);

            // 【致命漏洞修复：双重空间镜像映射补偿】
            // 依据 DOFBOT 的 URDF 构型，joint2 存在的预设旋转导致了倒置轴系。
            // 为了正确驱动关节产生正向的平面延伸（不向后弯折），映射的平面 Y 坐标应当恢复为正向的 Y_w。
            double x_plane = Z_w;
            double y_plane = Y_w; 
            
            double D2 = x_plane * x_plane + y_plane * y_plane;
            double cos_q3 = (D2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);

            // 余弦定理越界钳制，当要求坐标超出臂展时返回安全拉直极限解（改善拖拽示教手感）
            if (cos_q3 > 1.0) cos_q3 = 1.0;
            if (cos_q3 < -1.0) cos_q3 = -1.0;

            double acos_q3 = std::acos(cos_q3);
            double q3_candidates[2] = {acos_q3, -acos_q3}; // 覆盖“肘部朝上”与“朝下”形态

            for (int j = 0; j < 2; ++j) {
                double q3 = q3_candidates[j];
                double q2 = std::atan2(y_plane, x_plane) - std::atan2(L3 * std::sin(q3), L2 + L3 * std::cos(q3));
                double q4 = theta - q2 - q3;

                // 交叉排列出所有处于实际限位阈值域内的可用流形集合
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

    // 与原生 URDF 数据轴向完全对齐的正向运动学矩阵流 (支持 FCL 中间连杆避障推演)
    std::map<std::string, Eigen::Isometry3d> computeAllFK(const std::vector<double>& q) const {
        std::map<std::string, Eigen::Isometry3d> poses;
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        poses[getBaseFrame()] = T;

        T = T * Eigen::Translation3d(0, 0, 0.064) * Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitZ());
        poses["link1"] = T;

        T = T * Eigen::Translation3d(0, 0, 0.0435) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(q[1], Eigen::Vector3d::UnitZ());
        poses["link2"] = T;

        T = T * Eigen::Translation3d(-0.08285, 0, 0) * Eigen::AngleAxisd(q[2], Eigen::Vector3d::UnitZ());
        poses["link3"] = T;

        T = T * Eigen::Translation3d(-0.08285, 0, 0) * Eigen::AngleAxisd(q[3], Eigen::Vector3d::UnitZ());
        poses["link4"] = T;

        T = T * Eigen::Translation3d(-0.18385, -0.00215, -0.000605) * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitZ());
        poses["link5"] = T;
        return poses;
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
        
        const moveit::core::JointModelGroup* jmg = robot_model.getJointModelGroup(group_name);
        if (!jmg) {
            ROS_ERROR_NAMED("dofbot_ik", "Failed to retrieve JointModelGroup from RobotModel.");
            return false;
        }

        // 精确载入真实的物理安全界限，若读取异常设置准确的 URDF 兜底
        for (const std::string& jn : joint_names_) {
            const moveit::core::JointModel* jm = jmg->getJointModel(jn);
            if (jm && !jm->getVariableBounds().empty()) {
                joint_min_.push_back(jm->getVariableBounds()[0].min_position_);
                joint_max_.push_back(jm->getVariableBounds()[0].max_position_);
            } else {
                if (jn == "joint5") {
                    joint_min_.push_back(-1.5708); joint_max_.push_back(3.1416);
                } else {
                    joint_min_.push_back(-1.5708); joint_max_.push_back(1.5708);
                }
            }
        }
        
        // =========================================================================================
        // ✅ 解决 Noetic 编译报错的核心修复：利用空删除器 (Empty Deleter) 技巧
        // 由于 RobotState 在 Noetic 中强行要求 shared_ptr，我们将 robot_model 引用包装起来，
        // 并传入空的 Lambda 表达式作为 Deleter，保证智能指针结束时不会误删除 MoveIt 正在使用的内存。
        // =========================================================================================
        std::shared_ptr<const moveit::core::RobotModel> robot_model_ptr(&robot_model, [](const moveit::core::RobotModel*){});
        moveit::core::RobotState state(robot_model_ptr);
        state.setToDefaultValues();

        // 提取任意自定义 TCP 夹具产生的偏置并反求变换阵
        if (!tip_frames_.empty()) {
            Eigen::Isometry3d T_link5 = state.getGlobalLinkTransform("link5");
            Eigen::Isometry3d T_tip = state.getGlobalLinkTransform(tip_frames_[0]);
            tip_to_wrist_offset_ = T_link5.inverse() * T_tip;
        } else {
            tip_to_wrist_offset_ = Eigen::Isometry3d::Identity();
        }

        ROS_INFO_NAMED("dofbot_ik", "DOFBOT Exact Analytic IK Plugin Final V6 Initialized.");
        ROS_WARN_ONCE_NAMED("dofbot_ik", "\033[1;33m[IMPORTANT] Please ensure 'position_only_ik: True' is set in your kinematics.yaml for this 5-DOF arm!\033[0m");

        return true;
    }

    const std::vector<std::string>& getJointNames() const override { return joint_names_; }
    const std::vector<std::string>& getLinkNames() const override { return link_names_; }

    // 正向运动学重写以兼容 MoveIt C-Space 环境检测计算
    bool getPositionFK(const std::vector<std::string>& req_link_names,
                       const std::vector<double>& joint_angles,
                       std::vector<geometry_msgs::Pose>& poses) const override {
        if (joint_angles.size() != 5) return false;
        poses.resize(req_link_names.size());
        
        std::map<std::string, Eigen::Isometry3d> all_fk = computeAllFK(joint_angles);
        
        for (size_t i = 0; i < req_link_names.size(); ++i) {
            const std::string& name = req_link_names[i];
            
            if (all_fk.count(name)) {
                // 如果是已知骨架，直接返回我们精确代数正演的结果
                poses[i] = tf2::toMsg(all_fk.at(name));
            } else if (!tip_frames_.empty() && name == tip_frames_[0]) {
                // 动态工具 TCP 正算补偿
                poses[i] = tf2::toMsg(all_fk.at("link5") * tip_to_wrist_offset_);
            } else {
                // ✅ 【绝杀修复：防 FCL 环境毒化拦截】
                // 遇到无法解析的附加连杆（如 left_finger/right_finger），直接认怂返回 false！
                // 这将迫使 MoveIt 接管，使用内部 RobotState 树形正算推算出所有夹爪的绝对位置。
                return false; 
            }
        }
        return true;
    }

    // ==== 纯虚方法族全实现，100% 消除 ROS1 Noetic 的隐式隐藏编译告警 ====
    bool getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const override {
        return searchPositionIK(ik_pose, ik_seed_state, 0.1, std::vector<double>(), solution, IKCallbackFn(), error_code, options);
    }
    bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const override {
        return searchPositionIK(ik_pose, ik_seed_state, timeout, std::vector<double>(), solution, IKCallbackFn(), error_code, options);
    }
    bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const override {
        return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code, options);
    }
    bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout, std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const override {
        return searchPositionIK(ik_pose, ik_seed_state, timeout, std::vector<double>(), solution, solution_callback, error_code, options);
    }

    // ====== MoveIt! 插件架构检索解析器主干入口 ======
    bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double /*timeout*/, const std::vector<double>& consistency_limits, std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const override {
        
        Eigen::Isometry3d target_tip_pose;
        tf2::fromMsg(ik_pose, target_tip_pose);

        // TCP 逆演退回：将外界送入的含末端夹具偏移的目标空间位姿，数学反演推回纯法兰面
        Eigen::Isometry3d target_link5_pose = target_tip_pose * tip_to_wrist_offset_.inverse();

        std::vector<double> safe_seed = (ik_seed_state.size() == 5) ? ik_seed_state : std::vector<double>(5, 0.0);
        std::vector<std::vector<double>> solutions = calculateIK(target_link5_pose, safe_seed[0]);

        int best_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < solutions.size(); ++i) {
            
            // 执行 C-Space 平滑连续性审查
            if (!consistency_limits.empty() && consistency_limits.size() == 5) {
                bool consistent = true;
                for (size_t j = 0; j < 5; ++j) {
                    // 我们直接使用真实的物理转角差，避免因 normalizeAngle 归一化掩盖导致机械臂绕圈的危险
                    double diff = std::abs(solutions[i][j] - safe_seed[j]);
                    if (diff > consistency_limits[j]) { consistent = false; break; }
                }
                if (!consistent) continue;
            }

            // 【刚性合规拦截】：对于 5-DOF 机械臂必然存在无法满足的 6D 无理姿态请求，依据标签进行位姿超限拦截过滤
            if (!options.return_approximate_solution) {
                std::map<std::string, Eigen::Isometry3d> actual_fk = computeAllFK(solutions[i]);
                Eigen::Isometry3d actual_wrist = actual_fk.at("link5");
                Eigen::AngleAxisd err_rot(target_link5_pose.rotation().inverse() * actual_wrist.rotation());
                Eigen::Vector3d err_pos = target_link5_pose.translation() - actual_wrist.translation();
                
                if (err_pos.norm() > 1e-3 || std::abs(err_rot.angle()) > 0.05) {
                    continue; 
                }
            }

            // 【场景安全验证】：交由 MoveIt 规划场景进行自碰撞干涉与 Octomap 障碍物检验
            if (solution_callback) {
                solution_callback(ik_pose, solutions[i], error_code);
                if (error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) continue;
            }

            // L2 范数能量最优评判：优先返回物理运动行程最小的低耗能解
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

// 注册并向系统级 Pluginlib 暴露宏绑定
PLUGINLIB_EXPORT_CLASS(dofbot_kinematics::DofbotIKPlugin, kinematics::KinematicsBase)