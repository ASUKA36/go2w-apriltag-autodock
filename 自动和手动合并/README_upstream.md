# Go2-W 仿真与无地图自动泊狗

面向 ROS1 Noetic、Ubuntu 20.04 和 WSL2 的轻量工作空间，集中保留两项主要功能：

1. Go2-W 三维 Gazebo 仿真；
2. 基于 Mid-360 与 D435i/`tag36h11_0` 的无地图自动泊狗。

Go2-W 外观网格来自[宇树官方 unitree_ros](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2w_description)。

## 目录

```text
go2w_ws/
├── src/
│   ├── go2w_sim/      # Go2-W三维模型、Gazebo环境、传感器和键盘控制
│   ├── go2w_parking/  # 二维停车仿真、预测轨迹和自动泊狗控制器
│   └── go2w_vision/   # 真机D435i的tag36h11_0检测
├── LICENSE
└── NOTICE
```

核心接口：

```text
Mid-360 /scan ───────────────┐
                             ├─> go2w_parking/auto_dock ─> /cmd_vel
D435i /tag_detections ───────┘
```

自动泊狗控制器不订阅全局地图和里程计，只使用当前激光扫描与 Tag 相对位姿。

## 安装

```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins \
  ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz \
  ros-noetic-apriltag-ros ros-noetic-realsense2-camera ros-noetic-image-view

cd ~
git clone https://gitee.com/alaurb/go2w_nav_sim.git go2w_ws
cd ~/go2w_ws
catkin_make
source devel/setup.bash
```

## 1. 二维自动泊狗仿真

一键启动（仿真 + RViz + 自动泊狗 + 键盘；默认手动，按 `R` 切换自动/手动）：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_sim parking2_full.launch
```

| 按键 | 作用 |
|------|------|
| R | 切换 MANUAL ↔ AUTO |
| W/S A/D Q/E | 手动遥控（仅 MANUAL） |
| Space | 停止 |
| +/- | 调速 |

也可分终端启动。终端 1，只启动仿真、模拟 Mid-360、模拟 D435i 和 RViz：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_sim parking2_sim.launch
```

RViz 显示：

- 狗窝、充电桩和矩形机器狗；
- `/scan` 激光点；
- `/camera/color/image_raw` 相机画面；
- 绿色 Mid-360 狗窝中心线；
- 橙色 `/auto_dock/predicted_path` 三秒预测轨迹。

终端 2，只启动停车控制器：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_parking auto_dock.launch
```

二维相机会对“相机到 Tag”的线段执行墙体遮挡检测。Tag 被任意狗窝墙遮住时，
模拟图像不再绘制充电板和 Tag，`/tag_detections` 也会同步变为空数组。

状态流程：

```text
SEARCH_TAG / LIDAR_GARAGE_SEEK
  -> PRE_ALIGN_OUTSIDE
  -> APPROACH
  -> LIDAR_CENTERED_ENTRY
  -> FINE_ALIGN
  -> HOLD
  -> PARKED
```

监视状态：

```bash
rostopic echo /auto_dock/state
rostopic echo /auto_dock/success
rostopic echo /auto_dock/lidar_garage_detected
rostopic echo /control_mode
rostopic echo /cmd_vel
```

重置二维场景：

```bash
rosservice call /parking/reset
```

手动测试时不要启动 `auto_dock.launch`，单独运行：

```bash
rosrun go2w_sim teleop_keyboard.py
```

W/S 前后、A/D 横移、Q/E 原地转向，松开按键停止。

## 2. 真机自动泊狗

先启动 Mid-360 驱动并确认存在 `/scan`，然后运行：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_parking auto_dock_real.launch
```

该入口会启动：

- D435i 彩色流；
- `tag36h11` ID 0 检测；
- 检测框窗口；
- 自动泊狗控制器。

D435i 默认关闭深度、红外和 IMU，以减少 WSL USB 转发错误。真机使用前必须标定：

- D435i、Mid-360 到 `base_link` 的外参；
- Tag 实际黑色边框尺寸；
- 充电位相机到 Tag 的目标距离；
- `/cmd_vel` 到宇树 SDK2 `Move(vx, vy, vyaw)` 的适配关系。

主要参数：

- `src/go2w_parking/config/auto_dock.yaml`
- `src/go2w_vision/launch/detect_tag.launch`

充电完成最终应由 BMS/充电电流确认；当前版本以位姿稳定保持后发布
`/auto_dock/success=true` 作为仿真成功条件。

## 3. Go2-W 三维仿真

三维自动泊车场景使用宇树官方 Go2-W 网格外观、带碰撞体的 U 形狗窝、
Gazebo 激光和 RGB-D 相机。一键启动（Gazebo + 传感器 + Tag 检测 + 自动泊狗 + 键盘）：

```bash
roslaunch go2w_sim parking3_full.launch
```

默认手动，按 `R` 切换自动/手动，操作与二维相同。也可分终端：终端 1 启动场景与传感器：

```bash
roslaunch go2w_sim parking3_sim.launch
```

终端 2 启动同一个无地图泊车控制器：

```bash
roslaunch go2w_parking auto_dock.launch
```

默认使用支持 `linear.x/linear.y/angular.z` 的轻量平面运动插件，适合验证泊车算法；
如需比较固定轮接触动力学，可运行：

```bash
roslaunch go2w_sim parking3_sim.launch physics:=true
# 或一键版：
roslaunch go2w_sim parking3_full.launch physics:=true
```

物理模式的固定轮不会执行横移指令，因此不建议用它验证依赖 `linear.y` 的完整泊车流程。

轻量球棍动力学模型：

```bash
roslaunch go2w_sim simple_sim.launch
```

宇树官方网格外观版本：

```bash
roslaunch go2w_sim unitree_sim.launch
```

需要 RViz 时添加：

```bash
roslaunch go2w_sim unitree_sim.launch rviz:=true
```

`go2w_sim/launch` 只保留四个仿真入口：`simple_sim.launch`、
`unitree_sim.launch`、`parking2_sim.launch`、`parking2_full.launch`、
`parking3_sim.launch` 和 `parking3_full.launch`。

## 许可证

工作空间自有代码采用 Apache-2.0。宇树官方 Go2-W 网格按 BSD-3-Clause
许可证分发，来源和修改说明见 `NOTICE`。
