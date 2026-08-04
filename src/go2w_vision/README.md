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

仿真中推荐直接运行完整流程（内置 ID 0、16 cm 标签参数）：

```bash
roslaunch go2w_vision follow_tag_sim.launch
```

控制器识别到目标后先水平居中并接近到30 cm，在目标区稳定保持1秒后停止并向
`/go2w/posture_cmd` 发布 `lie_down`。到达判定使用低通滤波、进入/退出滞回和近距离
短暂丢帧容错；视觉丢失时不会继续前进。

启动检测和 30 cm 视觉跟随：

```bash
roslaunch go2w_vision follow_tag.launch target_id:=0 tag_size:=0.080
```

检测输出为 `/tag_detections`，跟随控制输出默认为 `/cmd_vel`。
