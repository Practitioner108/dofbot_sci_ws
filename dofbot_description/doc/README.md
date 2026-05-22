# DOFBOT 硬件文档

本目录包含 DOFBOT 5-DOF 机械臂的底层硬件手册、通信协议和参考文档。

## protocol/ — 通信协议

| 文件 | 内容 |
|------|------|
| `i2c_register_map.xlsx` | 树莓派 ↔ 扩展板MCU I2C寄存器映射 (从机地址 0x15)，含RGB灯/蜂鸣器/舵机控制/动作组/中位偏差等全部寄存器 |
| `pc_app_protocol.xlsx` | PC上位机/手机APP ↔ 下层 ASCII 通信协议 ($...# 帧格式, 26条指令)，含错误码和应答格式 |
| `desheng_servo_protocol_v4.01.pdf` | 德晟串口总线智能舵机协议 v4.01 (UART 115200bps, 半双工主从, 6种指令类型, 完整寄存器表) |

## hardware/ — 硬件参考

| 文件 | 内容 |
|------|------|
| `DS-SY15A_datasheet.pdf` | DS-SY15A 15kg串口舵机规格书 (扭矩/速度/尺寸/接口/电气参数) |
| `hardware_quick_reference.xlsx` | 硬件速查手册 (树莓派/STC51/STM32/Arduino/microbit 五平台引脚映射, 按键功能, RGB灯状态) |
| `raspberryPi-pins-40.png` | 树莓派40Pin GPIO 引脚分布图 |

## 通信架构

```
[PC/手机APP]  ←→  [树莓派]  ←→  [扩展板MCU]  ←→  [6× DS-SY15A 总线舵机]
   ASCII串口        I2C (0x15)      UART (德晟协议)
   $...# 帧格式      寄存器映射        115200,8N1
```
