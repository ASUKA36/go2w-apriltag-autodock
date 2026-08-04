# go2w_ws

面向 ROS1 Noetic 的 Go2-W 导航与视觉算法工作空间，针对 Ubuntu 20.04/WSL2 优化。
模型结构参考
[`jj7258/unitree_go2w_ros2`](https://github.com/jj7258/unitree_go2w_ros2/tree/main/src/go2w_description/urdf)，
使用基础几何体实现轻量 Gazebo 3D 模型。

## 目录结构

```text
go2w_ws/
├── src/
│   ├── go2w_sim/         # Gazebo、Go2-W、RGB-D、激光和仿真地图
│   ├── go2w_navigation/  # move_base 参数与导航验证
│   └── go2w_vision/      # D435i/Gazebo 相机 AprilTag 检测与跟随
├── LICENSE
└── NOTICE
```

三个包按标准 ROS 接口解耦：

```text
go2w_sim ── /map /scan /odom /tf /camera/* ──► navigation / vision
go2w_navigation ── /cmd_vel ──► go2w_sim 或真实机器狗
go2w_vision ── /tag_detections，跟随模式下输出 /cmd_vel
```

## 安装与编译

```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins \
  ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz \
  ros-noetic-move-base ros-noetic-apriltag-ros ros-noetic-realsense2-camera

cd ~/go2w_ws
catkin_make
source devel/setup.bash
```

## 分终端启动

终端 1，仅启动仿真环境：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_sim sim.launch
```

终端 2，仅启动导航：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_navigation navigation.launch
```

终端 3，使用 Gazebo 相机启动 AprilTag 检测：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_vision detect_tag.launch start_realsense:=false tag_size:=0.46
```

真实 D435i 检测时使用：

```bash
roslaunch go2w_vision detect_tag.launch start_realsense:=true \
  target_id:=0 tag_size:=0.080
```

## 自动验证

```bash
roslaunch go2w_sim verify.launch
roslaunch go2w_sim vision_verify.launch
roslaunch go2w_navigation navigation_verify.launch
```

验证 launch 中测试节点完成后会主动关闭所有节点；出现 `process has finished cleanly`
和 `PASS` 表示成功。

## 许可证

工作空间采用 Apache-2.0。Go2-W 简化模型的来源和修改说明见 `NOTICE`。
