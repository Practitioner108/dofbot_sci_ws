#include "dofbot_control/dofbot_sim_interface.h"

namespace dofbot_control {

DofbotSimInterface::DofbotSimInterface()
{}

DofbotSimInterface::~DofbotSimInterface() {
}

bool DofbotSimInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    joint_names_ = {
        "base_rotation_joint",
        "upper_arm_joint",
        "forearm_joint",
        "wrist_pitch_joint",
        "wrist_roll_joint",
        "gripper_left_joint"
    };

    const size_t n = joint_names_.size();
    joint_position_.resize(n, 0.0);
    joint_velocity_.resize(n, 0.0);
    joint_effort_.resize(n, 0.0);
    joint_position_command_.resize(n, 0.0);

    // ros_control 接口注册
    for (size_t i = 0; i < n; ++i) {
        hardware_interface::JointStateHandle js_handle(
            joint_names_[i],
            &joint_position_[i],
            &joint_velocity_[i],
            &joint_effort_[i]
        );
        js_interface_.registerHandle(js_handle);

        hardware_interface::JointHandle pj_handle(
            js_interface_.getHandle(joint_names_[i]),
            &joint_position_command_[i]
        );
        pj_interface_.registerHandle(pj_handle);
    }

    registerInterface(&js_interface_);
    registerInterface(&pj_interface_);

    // 订阅 MuJoCo 仿真发布的 /joint_states_raw，发布 /joint_command
    joint_state_sub_ = root_nh.subscribe("/joint_states_raw", 1,
                                         &DofbotSimInterface::jointStateCallback, this);
    joint_cmd_pub_ = root_nh.advertise<std_msgs::Float64MultiArray>("/joint_command", 1);

    ROS_INFO_STREAM("DofbotSimInterface: " << n << " joints registered");
    ROS_INFO("  Subscribing to /joint_states, publishing to /joint_command");
    return true;
}

void DofbotSimInterface::read(const ros::Time& time, const ros::Duration& period) {
    // 数据在 jointStateCallback 中异步更新，此处不做阻塞等待
    // ros_control 的 read() 只需要保证 joint_position_ 是最新的即可
}

void DofbotSimInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < joint_names_.size() && i < msg->name.size(); ++i) {
        // 按关节名匹配
        for (size_t j = 0; j < msg->name.size(); ++j) {
            if (msg->name[j] == joint_names_[i]) {
                joint_position_[i] = msg->position[j];
                if (j < msg->velocity.size())
                    joint_velocity_[i] = msg->velocity[j];
                if (j < msg->effort.size())
                    joint_effort_[i] = msg->effort[j];
                break;
            }
        }
    }
}

void DofbotSimInterface::write(const ros::Time& time, const ros::Duration& period) {
    // 将 ros_control 的命令位置发布到 /joint_command，由 mujoco_sim_node.py 接收并写入 MuJoCo
    std_msgs::Float64MultiArray cmd;
    cmd.data.resize(joint_names_.size());
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            cmd.data[i] = joint_position_command_[i];
        }
    }
    joint_cmd_pub_.publish(cmd);
}

} // namespace dofbot_control
