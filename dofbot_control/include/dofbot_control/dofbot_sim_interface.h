#ifndef DOFBOT_SIM_INTERFACE_H
#define DOFBOT_SIM_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <vector>
#include <mutex>

namespace dofbot_control {

class DofbotSimInterface : public hardware_interface::RobotHW {
public:
    DofbotSimInterface();
    ~DofbotSimInterface();

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
    void read(const ros::Time& time, const ros::Duration& period) override;
    void write(const ros::Time& time, const ros::Duration& period) override;

private:
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

    std::vector<std::string> joint_names_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;

    hardware_interface::JointStateInterface    js_interface_;
    hardware_interface::PositionJointInterface pj_interface_;

    ros::Subscriber joint_state_sub_;
    ros::Publisher  joint_cmd_pub_;

    std::mutex data_mutex_;
};

} // namespace dofbot_control

#endif // DOFBOT_SIM_INTERFACE_H
