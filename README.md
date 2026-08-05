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
│   ├── go2w_parking/  # 泊车场景节点、预测轨迹和无地图自动泊狗控制器
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
# 公司仓库
git clone https://gitee.com/scenex/parking.git go2w_ws
# 或个人镜像：git clone https://gitee.com/alaurb/go2w_nav_sim.git go2w_ws
cd ~/go2w_ws
catkin_make
source devel/setup.bash
```

## 仿真入口

仿真入口统一放在 `go2w_sim/launch`：

| 启动命令 | 模型与用途 |
|---|---|
| `roslaunch go2w_sim simple_sim.launch` | 球棍外观、接触驱动轮式动力学 |
| `roslaunch go2w_sim unitree_sim.launch` | 宇树官方外观、接触驱动轮式动力学 |
| `roslaunch go2w_sim parking2_sim.launch` | 最轻量的二维泊车算法验证 |
| `roslaunch go2w_sim parking3_sim.launch` | 官方外观、三维狗窝、相机与激光泊车验证 |

`parking2_sim` 和 `parking3_sim` 只负责环境与传感器，停车算法始终通过
`roslaunch go2w_parking auto_dock.launch` 在另一个终端独立启动。
停车算法不依赖 `/control_mode`，启动后自动接管 `/cmd_vel`。默认出生位姿正对
Tag；如果手动启用随机出生且相机看不到 Tag，算法会停在 `WAIT_TAG`。

## 1. 二维自动泊狗仿真

终端 1，只启动仿真、模拟 Mid-360、模拟 D435i 和 RViz：

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
PREALIGN_GATE -> ENTER_BAY -> CROUCH
```

- `PREALIGN_GATE`：在窝外按 Tag 法向、横向误差和前方激光距离完成预对准；
- `ENTER_BAY`：相机与左右墙激光共同引导入窝，近距离时转为激光终段；
- `CROUCH`：停车并发布趴下指令，可按配置自动复位后进行下一轮。

监视状态：

```bash
rostopic echo /auto_dock/state
rostopic echo /auto_dock/success
rostopic echo /auto_dock/lidar_garage_detected
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
Gazebo 激光和 RGB-D 相机。终端 1 启动场景与传感器：

```bash
roslaunch go2w_sim parking3_sim.launch
```

终端 2 启动同一个无地图泊车控制器：

```bash
roslaunch go2w_parking auto_dock.launch
```

默认使用支持 `linear.x/linear.y/angular.z` 的轻量平面运动插件，适合验证泊车算法；
该模式已禁用各链接的独立物理解算，静止时不会因接触冲量出现腿部抖动或模型漂移。
如需比较固定轮接触动力学，可运行：

```bash
roslaunch go2w_sim parking3_sim.launch physics:=true
```

物理模式的固定轮不会执行横移指令，因此不建议用它验证依赖 `linear.y` 的完整泊车流程。

当前自动泊车默认限速为：接近 `0.38 m/s`、最终入库 `0.12 m/s`、横移
`0.18 m/s`、转向 `0.60 rad/s`。门外预对准距离默认为 `1.50 m`，最终
激光停车距离默认为 `0.35 m`。可在
`src/go2w_parking/config/auto_dock.yaml` 中继续调整。

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

## 许可证

工作空间自有代码采用 Apache-2.0。宇树官方 Go2-W 网格按 BSD-3-Clause
许可证分发，来源和修改说明见 `NOTICE`。
