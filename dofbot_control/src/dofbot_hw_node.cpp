#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "dofbot_control/dofbot_hw_interface.h"
#include "dofbot_control/dofbot_sim_interface.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "dofbot_hw_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 选择真实硬件或仿真模式
    bool sim_mode = false;
    pnh.param<bool>("sim_mode", sim_mode, "false");

    std::unique_ptr<hardware_interface::RobotHW> robot;

    if (sim_mode) {
        auto sim = std::make_unique<dofbot_control::DofbotSimInterface>();
        if (!sim->init(nh, pnh)) {
            ROS_FATAL("Failed to initialize DofbotSimInterface");
            return 1;
        }
        robot = std::move(sim);
        ROS_INFO("Running in SIMULATION mode");
    } else {
        auto hw = std::make_unique<dofbot_control::DofbotHWInterface>();
        if (!hw->init(nh, pnh)) {
            ROS_FATAL("Failed to initialize DofbotHWInterface");
            return 1;
        }
        robot = std::move(hw);
        ROS_INFO("Running in REAL HARDWARE mode");
    }

    controller_manager::ControllerManager cm(robot.get(), nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    double control_freq = 50.0;
    pnh.param<double>("control_freq", control_freq, 50.0);
    ros::Rate rate(control_freq);

    ros::Time last = ros::Time::now();
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        ros::Duration period = now - last;
        last = now;

        robot->read(now, period);
        cm.update(now, period);
        robot->write(now, period);

        rate.sleep();
    }

    spinner.stop();
    return 0;
}
