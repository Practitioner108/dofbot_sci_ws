# Eye-in-Hand 摄像头

## 硬件参数

| 项目 | 值 |
|------|-----|
| 型号 | Microdia USB Camera (ID 0c45:6340) |
| 像素 | 30 万 (640×480) |
| 帧率 | 30 fps (最高) |
| 视场角 | 110° 广角 |
| 接口 | USB |
| 对焦 | 手动调节 |

## 安装位置

固定在 `wrist_roll_link` 下方，眼在手上（eye-in-hand）布局：

| 相对 wrist_roll_link | 值 |
|---------------------|------|
| X | 0 |
| Y | 0.05 m |
| Z | -0.05 m |

## v4l2 参数

```
设备:       /dev/video0
像素格式:   YUYV (YUYV 4:2:2)
分辨率:     640×480 @ 30/25/20/15/10/5 fps
            352×288 (支持)
```

不支持 MJPG 压缩，不支持自动白平衡和自动对焦。

## ROS 驱动

- 包: `usb_cam` (ros-noetic-usb-cam)
- launch: `dofbot_control/launch/camera.launch`
- 话题: `/usb_cam/image_raw` (sensor_msgs/Image, rgb8)
- 话题: `/usb_cam/camera_info` (sensor_msgs/CameraInfo)
- TF frame: `camera_link`
