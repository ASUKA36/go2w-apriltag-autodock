# go2w_vision

真机自动泊狗使用的最小 D435i AprilTag 检测包，只负责发布
`/tag_detections`，不包含运动控制器。

默认配置：

- Tag family：`tag36h11`
- Tag ID：`0`
- Tag 名称：`tag36h11_0`
- 黑色边框尺寸：`0.080 m`
- 仅开启 D435i 彩色流
- 默认显示检测框

单独测试：

```bash
roslaunch go2w_vision detect_tag.launch
```

完整真机泊狗请使用：

```bash
roslaunch go2w_parking auto_dock_real.launch
```

若相机驱动已由其他程序启动：

```bash
roslaunch go2w_vision detect_tag.launch start_realsense:=false
```

请根据打印 Tag 的实际黑色边框修改 `tag_size`。D435i 深度、对齐和 IMU 默认关闭，
避免在 WSL/usbipd 下触发不必要的 USB 带宽和深度流启动错误。
