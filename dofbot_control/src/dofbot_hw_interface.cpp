#include "dofbot_control/dofbot_hw_interface.h"
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

namespace dofbot_control {

DofbotHWInterface::DofbotHWInterface()
    : i2c_fd_(-1)
    , read_phase_(0)
    , consecutive_errors_(0)
    , warmup_cycles_(0)
    , mcu_initialized_(false)
    , stall_shutdown_(false)
{}

DofbotHWInterface::~DofbotHWInterface() {
    i2cClose();
}

bool DofbotHWInterface::initMCU() {
    ROS_INFO("Initializing MCU...");

    if (i2c_fd_ < 0) {
        ROS_ERROR("Cannot init MCU: I2C not open");
        return false;
    }

    // 1. 设置主控板型号为树莓派 (手册寄存器 0x04)
    uint8_t board = 0x05;  // raspberryPi
    if (!i2cWriteBlock(0x04, &board, 1)) {
        ROS_WARN("Failed to set board type (0x04), continuing anyway");
    } else {
        ROS_INFO("  Board type set to raspberryPi (0x04=0x05)");
    }

    // 2. 重启 MCU 使设置生效 (手册寄存器 0x05)
    uint8_t reset_cmd = 0x01;
    if (!i2cWriteBlock(0x05, &reset_cmd, 1)) {
        ROS_WARN("Failed to send reset command (0x05)");
    } else {
        ROS_INFO("  MCU reset command sent (0x05=0x01)");
    }

    // 3. 等待 MCU 重启完成 (手册说 MCU 启动需 ~100ms，留余量)
    usleep(200000);  // 200ms

    // 4. 重启后 I2C 状态可能已重置，关闭并重新打开
    i2cClose();
    if (!i2cOpen()) {
        ROS_ERROR("Failed to re-open I2C after MCU reset");
        return false;
    }

    // 5. 开启总线舵机扭矩 (手册寄存器 0x1A)
    uint8_t torque_on = 0x01;  // 非0 = 开启
    if (!i2cWriteBlock(0x1A, &torque_on, 1)) {
        ROS_WARN("Failed to enable torque (0x1A)");
    } else {
        ROS_INFO("  Servo torque enabled (0x1A=0x01)");
    }

    // 6. 等待 MCU 完成首次舵机轮询（上电后约需 200-500ms）
    usleep(500000);

    ROS_INFO("MCU initialization complete");
    return true;
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
    // 关节 1/2/3 旋转方向与 URDF 的右手定则相反
    servo_params_[1].direction = -1;  // upper_arm
    servo_params_[2].direction = -1;  // forearm
    servo_params_[3].direction = -1;  // wrist_pitch

    // 中位偏差校准: 竖直位姿下各舵机的 raw 偏移量
    servo_params_[0].mid_deviation = -47;   // base_rotation
    servo_params_[1].mid_deviation = -47;   // upper_arm
    servo_params_[2].mid_deviation = -50;   // forearm
    servo_params_[3].mid_deviation = -47;   // wrist_pitch
    servo_params_[4].mid_deviation = -562;  // wrist_roll
    servo_params_[5].mid_deviation = -10;   // gripper

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

    stall_watch_.resize(n);
    stall_shutdown_ = false;

    robot_hw_nh.param<std::string>("i2c_device", i2c_device_, "/dev/i2c-1");
    robot_hw_nh.param<int>("i2c_address", i2c_addr_, 0x15);

    ROS_INFO_STREAM("DofbotHWInterface: " << n << " joints registered");
    ROS_INFO_STREAM("  I2C device: " << i2c_device_ << ", addr: 0x" << std::hex << i2c_addr_);

    if (!i2cOpen()) {
        ROS_ERROR("Failed to open I2C device");
        return false;
    }

    // MCU 初始化（型号设置 + 重启 + 扭矩使能）
    if (!initMCU()) {
        ROS_ERROR("MCU initialization failed");
        return false;
    }
    mcu_initialized_ = true;

    // 首次读取全部 6 路舵机位置，防止 write() 写入数学零导致机械臂跳动
    for (int phase = 0; phase < 3; ++phase) {
        for (int i = phase * 2; i < (phase + 1) * 2 && i < 6; ++i) {
            uint8_t buf[2] = {0};
            if (i2cReadBlock(0x31 + i, buf, 2, 15)) {
                int raw = (buf[0] << 8) | buf[1];
                if (raw >= 96 && raw <= 4000) {
                    if (i < 5)
                        joint_position_[i] = rawToRadian(raw, servo_params_[i]);
                    else
                        joint_position_[i] = rawToGripper(raw, servo_params_[i]);
                    joint_position_command_[i] = joint_position_[i];
                }
            }
        }
    }
    warmup_cycles_ = 6;  // 跳过 write() 的热身等待
    read_phase_ = 0;

    ROS_INFO("Running in REAL HARDWARE mode (initial positions read)");
    return true;
}

void DofbotHWInterface::read(const ros::Time& time, const ros::Duration& period) {
    if (!mcu_initialized_) return;

    // 分 3 个 phase 轮流读取，每次只读 2 路舵机，避免 I2C 阻塞
    // Phase 0: 舵机 0,1 (寄存器 0x31, 0x32)
    // Phase 1: 舵机 2,3 (寄存器 0x33, 0x34)
    // Phase 2: 舵机 4,5 (寄存器 0x35, 0x36)
    // 每轮 2×10ms = 20ms，适配 50Hz 控制周期

    int start_servo = read_phase_ * 2;  // 0, 2, or 4
    read_phase_ = (read_phase_ + 1) % 3;

    bool any_error = false;
    for (int i = start_servo; i < start_servo + 2 && i < 6; ++i) {
        uint8_t buf[2] = {0};
        // 手册要求：先写触发字节，延迟 ≥5ms（取 10ms 更可靠），再读 2 字节
        if (i2cReadBlock(0x31 + i, buf, 2, 10)) {
            int raw = (buf[0] << 8) | buf[1];
            // 校验 raw 值范围
            if (raw >= 96 && raw <= 4000) {
                if (i < 5)
                    joint_position_[i] = rawToRadian(raw, servo_params_[i]);
                else
                    joint_position_[i] = rawToGripper(raw, servo_params_[i]);
            } else if (raw == 0) {
                // 部分舵机寄存器（尤其是 0x31）首次触发后需二次采样才就绪
                // 重试一次，延长时间到 15ms
                uint8_t retry_buf[2] = {0};
                if (i2cReadBlock(0x31 + i, retry_buf, 2, 15)) {
                    int retry_raw = (retry_buf[0] << 8) | retry_buf[1];
                    if (retry_raw >= 96 && retry_raw <= 4000) {
                        raw = retry_raw;
                        if (i < 5)
                            joint_position_[i] = rawToRadian(raw, servo_params_[i]);
                        else
                            joint_position_[i] = rawToGripper(raw, servo_params_[i]);
                        goto read_done;
                    }
                }
                // 重试失败，容忍策略：热身期放过，否则报错
                if (warmup_cycles_ < 12) {
                    ROS_DEBUG_THROTTLE(1.0, "Servo %d raw=0 during warmup, tolerated", i);
                } else {
                    ROS_WARN_THROTTLE(5.0, "Servo %d retry also returned 0", i);
                    any_error = true;
                }
            } else {
                ROS_WARN_THROTTLE(5.0, "Servo %d raw value out of range: %d", i, raw);
                any_error = true;
            }
            read_done: ;
        } else {
            any_error = true;
        }
    }

    if (any_error) {
        consecutive_errors_++;
        if (consecutive_errors_ >= 20) {
            ROS_WARN_THROTTLE(5.0,
                "I2C consecutive errors: %d. Attempting I2C recovery...",
                consecutive_errors_);
            i2cClose();
            if (i2cOpen()) {
                ROS_INFO("I2C reconnected after errors");
                consecutive_errors_ = 0;
                warmup_cycles_ = 0;
            }
        }
    } else {
        // 热身计数：前 6 个 phase（完整 2 轮 = 120ms）记录 MCU 舵机轮询就绪状态
        if (warmup_cycles_ < 6)
            warmup_cycles_++;
        // 仅有明确错误时才递减，避免偶尔单次失败触发恢复
        if (consecutive_errors_ > 0 && consecutive_errors_ < 20)
            consecutive_errors_--;
    }
}

void DofbotHWInterface::write(const ros::Time& time, const ros::Duration& period) {
    if (!mcu_initialized_) return;
    if (stall_shutdown_) return;

    // 启动热身期内不写
    if (warmup_cycles_ < 6) {
        return;
    }
    // 连续 I2C 错误频繁时不写入
    if (consecutive_errors_ >= 20) {
        ROS_WARN_THROTTLE(1.0, "Skipping write: too many I2C errors");
        return;
    }

    const size_t n = joint_names_.size();

    int raw[6];
    for (size_t i = 0; i < 5; ++i) {
        raw[i] = radianToRaw(joint_position_command_[i], servo_params_[i]);
    }
    raw[5] = gripperToRaw(joint_position_command_[5], servo_params_[5]);

    // 最终安全检查：确保 raw 值在有效范围
    for (size_t i = 0; i < n; ++i) {
        if (raw[i] < servo_params_[i].raw_min || raw[i] > servo_params_[i].raw_max) {
            ROS_WARN_THROTTLE(1.0, "Joint %zu raw %d out of [%d,%d], clamping",
                i, raw[i], servo_params_[i].raw_min, servo_params_[i].raw_max);
            if (raw[i] < servo_params_[i].raw_min) raw[i] = servo_params_[i].raw_min;
            if (raw[i] > servo_params_[i].raw_max) raw[i] = servo_params_[i].raw_max;
        }
    }

    // 拼装 12 字节 (H, L 拆分) 用于寄存器 0x1D 批量写入
    uint8_t buf_0x1d[12];
    for (size_t i = 0; i < n; ++i) {
        buf_0x1d[2 * i]     = static_cast<uint8_t>((raw[i] >> 8) & 0x0F);
        buf_0x1d[2 * i + 1] = static_cast<uint8_t>(raw[i] & 0xFF);
    }

    // 运行时间 (ms)，不小于 30ms（手册默认 500ms，最低 20ms，取 30ms 安全）
    int move_time_ms = static_cast<int>(period.toSec() * 1000.0);
    if (move_time_ms < 30) move_time_ms = 30;
    uint8_t buf_0x1e[2] = {
        static_cast<uint8_t>((move_time_ms >> 8) & 0xFF),
        static_cast<uint8_t>(move_time_ms & 0xFF)
    };

    // 手册明确要求：先写 0x1E（时间），再写 0x1D（位置）
    if (!i2cWriteBlock(0x1E, buf_0x1e, 2)) {
        consecutive_errors_++;
        return;
    }
    if (!i2cWriteBlock(0x1D, buf_0x1d, 12)) {
        consecutive_errors_++;
        return;
    }

    ROS_DEBUG_STREAM_THROTTLE(1.0, "HW write: "
        << raw[0] << " " << raw[1] << " " << raw[2] << " "
        << raw[3] << " " << raw[4] << " " << raw[5]
        << " | time=" << move_time_ms << "ms");

    checkStall();
}

// ========== 堵转保护 ==========

void DofbotHWInterface::checkStall() {
    const size_t n = joint_names_.size();

    for (size_t i = 0; i < n; ++i) {
        double error = std::fabs(joint_position_command_[i] - joint_position_[i]);

        if (i == 5) {
            // --- 夹爪：仅限位保护（夹紧物体时误差持续存在是正常的）---
            // 开限位 raw≈1039, 闭限位 raw≈2992, ±50 余量
            double gap = rawToGripper(0, servo_params_[5]);  // 当前位置的 gap
            double cmd = joint_position_command_[5];
            bool at_open_limit  = (gap > 0.025) && (cmd > gap);       // 已全开，还要开
            bool at_close_limit = (gap < -0.025) && (cmd < gap);      // 已全闭，还要闭

            if ((at_open_limit || at_close_limit) && error > 0.003) {
                if (stall_watch_[i].error_start.isZero()) {
                    stall_watch_[i].error_start = ros::Time::now();
                } else if ((ros::Time::now() - stall_watch_[i].error_start).toSec() > 0.4) {
                    uint8_t off = 0x00;
                    i2cWriteBlock(0x1A, &off, 1);
                    stall_shutdown_ = true;
                    ROS_FATAL("GRIPPER STALLED at limit (gap=%.4fm, cmd=%.4fm)! Torque disabled.",
                              gap, cmd);
                    return;
                }
            } else {
                stall_watch_[i].error_start = ros::Time(0);
            }
        } else {
            // --- 旋转关节：位置跟踪误差检测 ---
            const double kErrThreshold = 0.12;  // rad ≈ 6.9°
            const double kTimeWindow   = 0.4;   // 秒

            if (error > kErrThreshold) {
                if (stall_watch_[i].error_start.isZero()) {
                    stall_watch_[i].error_start = ros::Time::now();
                } else if ((ros::Time::now() - stall_watch_[i].error_start).toSec() > kTimeWindow) {
                    uint8_t off = 0x00;
                    i2cWriteBlock(0x1A, &off, 1);
                    stall_shutdown_ = true;
                    ROS_FATAL("JOINT %s STALLED! cmd=%.3f actual=%.3f error=%.3f rad. "
                              "Torque disabled.",
                              joint_names_[i].c_str(),
                              joint_position_command_[i],
                              joint_position_[i], error);
                    return;
                }
            } else {
                stall_watch_[i].error_start = ros::Time(0);
            }
        }
    }
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
    // 手册协议：写 2 字节（寄存器地址 + 触发值 0x01），MCU 锁存舵机数据
    uint8_t trigger[2] = {reg, 0x01};
    if (::write(i2c_fd_, trigger, 2) != 2) {
        ROS_WARN("I2C read trigger failed @ reg 0x%02x: %s", reg, strerror(errno));
        return false;
    }
    // 等待 MCU 从 UART 舵机总线读取数据
    if (delay_ms > 0) {
        usleep(delay_ms * 1000);
    }
    // 重置 I2C 寄存器指针 (trigger write [reg,0x01] 可能使 MCU 内部指针偏移)
    uint8_t ptr[1] = {reg};
    if (::write(i2c_fd_, ptr, 1) != 1) {
        ROS_WARN("I2C pointer reset failed @ reg 0x%02x: %s", reg, strerror(errno));
        return false;
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
