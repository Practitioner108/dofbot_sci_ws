#ifndef DOFBOT_HW_INTERFACE_H
#define DOFBOT_HW_INTERFACE_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace dofbot_control {

// 舵机物理参数结构，方便后续调参
struct ServoParams {
    int raw_min   = 96;     // 原始值下限
    int raw_max   = 4000;   // 原始值上限
    int raw_mid   = 2048;   // 原始值中位
    double angle_min_deg = -135.0;  // 对应最小角度 (度)
    double angle_max_deg =  135.0;  // 对应最大角度 (度)
    double mid_deviation = 0.0;     // 中位偏差 (flash 储存值)

    ServoParams() = default;

    // 第 6 轴 (gripper) 特殊参数
    void setGripper() {
        raw_min = 96;
        raw_max = 4000;
        raw_mid = 3100;      // 夹爪中位不同
        angle_min_deg = -90.0;
        angle_max_deg =  90.0;
    }
};

// 弧度 → raw value 转换
inline int radianToRaw(double rad, const ServoParams& p) {
    double deg = rad * 180.0 / M_PI;
    double ratio = (deg - p.angle_min_deg) / (p.angle_max_deg - p.angle_min_deg);
    double raw_d = p.raw_min + ratio * (p.raw_max - p.raw_min) + p.mid_deviation;
    if (raw_d < p.raw_min) raw_d = p.raw_min;
    if (raw_d > p.raw_max) raw_d = p.raw_max;
    return static_cast<int>(raw_d);
}

// raw value → 弧度转换
inline double rawToRadian(int raw, const ServoParams& p) {
    double ratio = static_cast<double>(raw - p.raw_min - p.mid_deviation)
                   / (p.raw_max - p.raw_min);
    double deg = p.angle_min_deg + ratio * (p.angle_max_deg - p.angle_min_deg);
    return deg * M_PI / 180.0;
}

// 夹爪位置 (米) → raw value 转换 (prismatic joint)
inline int gripperToRaw(double position_m, const ServoParams& p) {
    double stroke_half = 0.03;  // meter
    double ratio = (position_m + stroke_half) / (2.0 * stroke_half);
    double raw_d = p.raw_min + ratio * (p.raw_max - p.raw_min) + p.mid_deviation;
    if (raw_d < p.raw_min) raw_d = p.raw_min;
    if (raw_d > p.raw_max) raw_d = p.raw_max;
    return static_cast<int>(raw_d);
}

// raw value → 夹爪位置 (米) 转换
inline double rawToGripper(int raw, const ServoParams& p) {
    double stroke_half = 0.03;
    double ratio = static_cast<double>(raw - p.raw_min - p.mid_deviation)
                   / (p.raw_max - p.raw_min);
    return (ratio * 2.0 - 1.0) * stroke_half;
}

class DofbotHWInterface : public hardware_interface::RobotHW {
public:
    DofbotHWInterface();
    ~DofbotHWInterface();

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
    void read(const ros::Time& time, const ros::Duration& period) override;
    void write(const ros::Time& time, const ros::Duration& period) override;

private:
    // MCU 初始化 (主控板型号 + 重启 + 扭矩使能)
    bool initMCU();

    // I2C 底层操作
    bool i2cOpen();
    void i2cClose();
    bool i2cWriteBlock(uint8_t reg, const uint8_t* data, size_t len);
    bool i2cReadBlock(uint8_t reg, uint8_t* data, size_t len, int delay_ms = 10);

    // 关节数据
    std::vector<std::string> joint_names_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;

    // 舵机参数
    std::vector<ServoParams> servo_params_;

    // ros_control 接口
    hardware_interface::JointStateInterface    js_interface_;
    hardware_interface::PositionJointInterface pj_interface_;

    // I2C 参数
    std::string i2c_device_;
    int i2c_addr_;
    int i2c_fd_;

    // 安全与速率控制
    int read_phase_;            // 0..2 分三轮读取 6 路舵机，避免 I2C 阻塞
    int consecutive_errors_;    // 连续 I2C 错误计数，达到阈值触发恢复
    int warmup_cycles_;         // 启动热身计数，期间容忍 raw=0（MCU 首次轮询需时间）
    bool mcu_initialized_;      // MCU 初始化是否完成
};

} // namespace dofbot_control

#endif // DOFBOT_HW_INTERFACE_H
