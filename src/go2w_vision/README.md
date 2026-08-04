# go2w_vision

Go2-W 的 ROS1 AprilTag 视觉包，支持真实 D435i 和 `go2w_sim` Gazebo 相机。

只启动检测（真实 D435i）：

```bash
roslaunch go2w_vision detect_tag.launch target_id:=0 tag_size:=0.080
```

使用已经启动的 Gazebo 相机：

```bash
roslaunch go2w_vision detect_tag.launch start_realsense:=false
```

启动检测和 30 cm 视觉跟随：

```bash
roslaunch go2w_vision follow_tag.launch target_id:=0 tag_size:=0.080
```

检测输出为 `/tag_detections`，跟随控制输出默认为 `/cmd_vel`。
