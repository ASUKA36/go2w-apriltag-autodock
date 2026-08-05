# go2w_ws

面向 ROS1 Noetic 的 Go2-W 导航与视觉算法工作空间，针对 Ubuntu 20.04/WSL2 优化。
模型结构和网格来自
[宇树官方 `unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2w_description)，
使用基础几何体实现轻量 Gazebo 3D 模型。

## 目录结构

```text
go2w_ws/
├── src/
│   ├── go2w_sim/         # Gazebo、Go2-W、RGB-D、激光和仿真地图
│   ├── go2w_navigation/  # move_base 参数与导航验证
│   ├── go2w_vision/      # D435i/Gazebo 相机 AprilTag 检测与跟随
│   └── go2w_parking/     # 全向矩形机器狗二维停车/入窝仿真
├── LICENSE
└── NOTICE
```

四个包按标准 ROS 接口解耦：

```text
go2w_sim ── /map /scan /odom /tf /camera/* ──► navigation / vision
go2w_navigation ── /cmd_vel ──► go2w_sim 或真实机器狗
go2w_vision ── /tag_detections，跟随模式下输出 /cmd_vel
```

独立的二维停车场景：

```bash
roslaunch go2w_parking parking_sim.launch
```

不使用地图的自动泊狗分两个终端启动：

```bash
# 终端 1：仿真环境、Mid-360 扫描、D435i 画面和 RViz
roslaunch go2w_parking parking_sim.launch

# 终端 2：停车控制器
roslaunch go2w_parking auto_dock.launch
```

方块机器狗支持 `linear.x` 前后、`linear.y` 横移和 `angular.z` 原地转向，
并发布 `/parking/map`、`/scan`、`/odom`、`/parking/goal` 与 `/parking/parked`。

## 安装与编译

```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins \
  ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz \
  ros-noetic-move-base ros-noetic-apriltag-ros ros-noetic-realsense2-camera

cd ~
git clone https://gitee.com/alaurb/go2w_nav_sim.git go2w_ws
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

需要球棍轮腿动力学和对角相位腿轨迹时，可改用：

```bash
roslaunch go2w_sim physics_sim.launch
```

需要对比官方 Go2-W 网格外观时，启动独立版本：

```bash
roslaunch go2w_sim unitree_sim.launch
```

两个动力学启动入口使用相同的关节、简化碰撞体、控制插件和 `/cmd_vel` 接口；
`unitree_sim.launch` 只把渲染外观换成宇树官方模型网格。若要打开 RViz，添加
`rviz:=true`。

终端 2，仅启动导航：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_navigation navigation.launch
```

终端 3，使用 Gazebo 相机启动 AprilTag 检测：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_vision detect_tag.launch start_realsense:=false tag_size:=0.16
```

仿真专用的自动接近与趴下流程：

```bash
roslaunch go2w_vision follow_tag_sim.launch
```

键盘遥控需要在独立终端中直接运行（W/S 前后、A/D 横移、Q/E 转向、Z 趴下、X 站立）：

```bash
rosrun go2w_sim teleop_keyboard.py
```

真实 D435i 检测时使用：

```bash
roslaunch go2w_vision follow_tag.launch
```

真实相机默认仅启动彩色流，不启动深度、深度对齐和 IMU，以避免 D435i
在 WSL USB 转发下出现 `Depth stream start failure`。

## 自动验证

```bash
roslaunch go2w_sim verify.launch
roslaunch go2w_sim vision_verify.launch
roslaunch go2w_navigation navigation_verify.launch
```

验证 launch 中测试节点完成后会主动关闭所有节点；出现 `process has finished cleanly`
和 `PASS` 表示成功。

## 许可证

工作空间自有代码采用 Apache-2.0；宇树官方 Go2-W 网格按其 BSD-3-Clause
许可证分发。模型来源和修改说明见 `NOTICE`。
