#include "dofbot_control/dofbot_hw_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

namespace dofbot_control {

DofbotHWInterface::DofbotHWInterface()
    : i2c_fd_(-1)
{}

DofbotHWInterface::~DofbotHWInterface() {
    i2cClose();
}

bool DofbotHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
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

    servo_params_.resize(n);
    for (size_t i = 0; i < 5; ++i) {
        servo_params_[i] = ServoParams();
    }
    servo_params_[5].setGripper();

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

    robot_hw_nh.param<std::string>("i2c_device", i2c_device_, "/dev/i2c-1");
    robot_hw_nh.param<int>("i2c_address", i2c_addr_, 0x15);

    ROS_INFO_STREAM("DofbotHWInterface: " << n << " joints registered");
    ROS_INFO_STREAM("  I2C device: " << i2c_device_ << ", addr: 0x" << std::hex << i2c_addr_);

    if (!i2cOpen()) {
        ROS_ERROR("Failed to open I2C device");
        return false;
    }

    return true;
}

void DofbotHWInterface::read(const ros::Time& time, const ros::Duration& period) {
    // 逐个读取 6 路舵机位置 (寄存器 0x31~0x36)。
    // 每路需要: write(reg) → usleep(5ms) → read(2 bytes)。
    // 总耗时约 30ms，在 50Hz control loop 中会超周期，
    // 后续需改为非阻塞/异步读取或降低读取频率。
    for (int i = 0; i < 6; ++i) {
        uint8_t buf[2] = {0};
        if (i2cReadBlock(0x31 + i, buf, 2)) {
            int raw = (buf[0] << 8) | buf[1];
            if (i < 5)
                joint_position_[i] = rawToRadian(raw, servo_params_[i]);
            else
                joint_position_[i] = rawToGripper(raw, servo_params_[i]);
        }
    }
}

void DofbotHWInterface::write(const ros::Time& time, const ros::Duration& period) {
    const size_t n = joint_names_.size();

    int raw[6];
    for (size_t i = 0; i < 5; ++i) {
        raw[i] = radianToRaw(joint_position_command_[i], servo_params_[i]);
    }
    raw[5] = gripperToRaw(joint_position_command_[5], servo_params_[5]);

    // 拼装 12 字节 (H, L 拆分) 用于寄存器 0x1D 批量写入
    uint8_t buf_0x1d[12];
    for (size_t i = 0; i < n; ++i) {
        buf_0x1d[2 * i]     = static_cast<uint8_t>((raw[i] >> 8) & 0x0F);
        buf_0x1d[2 * i + 1] = static_cast<uint8_t>(raw[i] & 0xFF);
    }

    // 运行时间 (ms)，不小于 20ms
    int move_time_ms = static_cast<int>(period.toSec() * 1000.0);
    if (move_time_ms < 20) move_time_ms = 20;
    uint8_t buf_0x1e[2] = {
        static_cast<uint8_t>((move_time_ms >> 8) & 0xFF),
        static_cast<uint8_t>(move_time_ms & 0xFF)
    };

    // I2C 批量写入：先写 0x1E 时间，再写 0x1D 位置
    i2cWriteBlock(0x1E, buf_0x1e, 2);
    i2cWriteBlock(0x1D, buf_0x1d, 12);

    ROS_DEBUG_STREAM_THROTTLE(1.0, "HW write: "
        << raw[0] << " " << raw[1] << " " << raw[2] << " "
        << raw[3] << " " << raw[4] << " " << raw[5]
        << " | time=" << move_time_ms << "ms");
}

// ========== Linux I2C 底层驱动 ==========

bool DofbotHWInterface::i2cOpen() {
    i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        ROS_ERROR("Failed to open %s: %s", i2c_device_.c_str(), strerror(errno));
        return false;
    }
    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr_) < 0) {
        ROS_ERROR("Failed to set I2C slave addr 0x%02x: %s", i2c_addr_, strerror(errno));
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }
    ROS_INFO("I2C opened: %s @ 0x%02x", i2c_device_.c_str(), i2c_addr_);
    return true;
}

void DofbotHWInterface::i2cClose() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

bool DofbotHWInterface::i2cWriteBlock(uint8_t reg, const uint8_t* data, size_t len) {
    if (i2c_fd_ < 0) return false;
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    ssize_t ret = ::write(i2c_fd_, buf, len + 1);
    if (ret != static_cast<ssize_t>(len + 1)) {
        ROS_WARN("I2C write failed @ reg 0x%02x: %s", reg, strerror(errno));
        return false;
    }
    return true;
}

bool DofbotHWInterface::i2cReadBlock(uint8_t reg, uint8_t* data, size_t len, int delay_ms) {
    if (i2c_fd_ < 0) return false;
    // 先写入寄存器地址触发 MCU 准备数据
    if (::write(i2c_fd_, &reg, 1) != 1) {
        ROS_WARN("I2C read trigger failed @ reg 0x%02x: %s", reg, strerror(errno));
        return false;
    }
    // 等待 MCU 锁存舵机位置数据
    if (delay_ms > 0) {
        usleep(delay_ms * 1000);
    }
    // 读回数据
    ssize_t ret = ::read(i2c_fd_, data, len);
    if (ret != static_cast<ssize_t>(len)) {
        ROS_WARN("I2C read failed @ reg 0x%02x: %s", reg, strerror(errno));
        return false;
    }
    return true;
}

} // namespace dofbot_control
