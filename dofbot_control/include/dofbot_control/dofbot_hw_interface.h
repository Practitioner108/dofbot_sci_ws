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
    int    direction     = 1;      // 旋转方向: 1=正向, -1=反向

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

// 弧度 → raw value 转换 (direction 处理旋转方向)
inline int radianToRaw(double rad, const ServoParams& p) {
    double deg = rad * p.direction * 180.0 / M_PI;
    double ratio = (deg - p.angle_min_deg) / (p.angle_max_deg - p.angle_min_deg);
    double raw_d = p.raw_min + ratio * (p.raw_max - p.raw_min) + p.mid_deviation;
    if (raw_d < p.raw_min) raw_d = p.raw_min;
    if (raw_d > p.raw_max) raw_d = p.raw_max;
    return static_cast<int>(raw_d);
}

// raw value → 弧度转换 (direction 处理旋转方向)
inline double rawToRadian(int raw, const ServoParams& p) {
    double ratio = static_cast<double>(raw - p.raw_min - p.mid_deviation)
                   / (p.raw_max - p.raw_min);
    double deg = p.angle_min_deg + ratio * (p.angle_max_deg - p.angle_min_deg);
    return p.direction * deg * M_PI / 180.0;
}

// 夹爪位置 (米) → raw value 转换 (三点实测标定: 全开6.1cm / 轻触2.9cm / 全闭0cm)
// 安全钳位在实测物理极限 [1039, 2992] 内，超出将堵转
inline int gripperToRaw(double position_m, const ServoParams& p) {
    double pos = position_m * p.direction;
    // 分段点: pos=-0.0015 为轻触 2.9cm 积木位
    const double pos_touch = -0.0015;
    const double pos_open  =  0.03;   // 全开
    const double pos_close = -0.03;   // 全闭
    const double raw_open  =  1039;   // pos_open 时的 raw
    const double raw_touch =  2477;   // pos_touch 时的 raw
    const double raw_close =  2992;   // pos_close 时的 raw

    double raw_d;
    if (pos < pos_touch) {
        // 闭合段 [pos_close, pos_touch] → [raw_close, raw_touch]
        double ratio = (pos - pos_close) / (pos_touch - pos_close);
        raw_d = raw_close + ratio * (raw_touch - raw_close);
    } else {
        // 张开段 [pos_touch, pos_open] → [raw_touch, raw_open]
        double ratio = (pos - pos_touch) / (pos_open - pos_touch);
        raw_d = raw_touch + ratio * (raw_open - raw_touch);
    }
    if (raw_d < raw_open)  raw_d = raw_open;   // 物理全开限位
    if (raw_d > raw_close) raw_d = raw_close;  // 物理全闭限位
    return static_cast<int>(raw_d);
}

// raw value → 夹爪位置 (米) 转换 (三点实测反向解析)

// raw value → 夹爪位置 (米) 转换 (三点实测反向解析)
inline double rawToGripper(int raw, const ServoParams& p) {
    const double pos_touch = -0.0015;
    const double pos_open  =  0.03;
    const double pos_close = -0.03;
    const double raw_open  =  1039;
    const double raw_touch =  2477;
    const double raw_close =  2992;

    double pos;
    if (raw < raw_touch) {
        // 张开段 [raw_open, raw_touch] → [pos_open, pos_touch]
        double ratio = static_cast<double>(raw - raw_open) / (raw_touch - raw_open);
        pos = pos_open + ratio * (pos_touch - pos_open);
    } else {
        // 闭合段 [raw_touch, raw_close] → [pos_touch, pos_close]
        double ratio = static_cast<double>(raw - raw_touch) / (raw_close - raw_touch);
        pos = pos_touch + ratio * (pos_close - pos_touch);
    }
    return p.direction * pos;
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

    // 堵转检测
    void checkStall();
    struct StallWatch {
        ros::Time error_start;
    };
    std::vector<StallWatch> stall_watch_;
    bool stall_shutdown_;

    // 关节数据
    std::vector<std::string> joint_names_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;
    std::vector<double> last_written_command_;  // 死区过滤用，记录上次实际写入的指令值

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
