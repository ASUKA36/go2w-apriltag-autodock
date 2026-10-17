# go2w-apriltag-autodock

面向 **Unitree Go2-W** 的无地图自动入库（对接充电）方案：用 **AprilTag (`tag36h11`)** 相对位姿引导轮足机器狗进入狭窄充电仓，可选 Mid-360 激光辅助终段对准。

仓库提供：

- Go2-W 的 ROS1 / Gazebo 仿真（2D 轻量验证 + 3D 官方外观狗窝）
- 无地图自动泊狗状态机（预对准 → 入仓 → 趴下）
- 真机 D435i + AprilTag 检测入口

> GitHub: https://github.com/ASUKA36/go2w-apriltag-autodock

## 特性

- **无全局地图 / 无里程计依赖**：控制器主要订阅 `/tag_detections`（及可选 `/scan`），适合固定充电仓场景
- **分阶段对接**：`PREALIGN_GATE`（仓外预对准）→ `ENTER_BAY`（入仓）→ `CROUCH`（趴下）
- **可横移底盘控制**：同时使用 `linear.x / linear.y / angular.z`
- **仿真与真机同一控制包**：`go2w_parking`，通过不同 launch 切换

## 系统结构

```text
go2w-apriltag-autodock/
├── src/
│   ├── go2w_sim/       # Go2-W 模型、Gazebo 世界、传感器与键盘遥控
│   ├── go2w_parking/   # 泊车场景、预测轨迹、无地图自动对接控制器
│   └── go2w_vision/    # 真机 D435i + tag36h11 检测
├── LICENSE             # Apache-2.0（本仓库自有代码）
└── NOTICE              # 宇树官方网格来源说明（BSD-3-Clause）
```

数据流（示意）：

```text
Mid-360 /scan  ──────────────┐
                             ├─> go2w_parking/auto_dock ─> /cmd_vel
D435i /tag_detections ───────┘
```

## 环境依赖

- Ubuntu 20.04（或 WSL2）
- ROS1 Noetic
- Gazebo、RViz、`apriltag_ros`、`realsense2_camera`（真机）

```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins \
  ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz \
  ros-noetic-apriltag-ros ros-noetic-realsense2-camera ros-noetic-image-view
```

## 编译

```bash
git clone https://github.com/ASUKA36/go2w-apriltag-autodock.git go2w_ws
cd go2w_ws
catkin_make
source devel/setup.bash
```

## 快速开始

### 1) 二维自动泊狗仿真（推荐先跑这个）

终端 1 — 仿真 + 模拟 Mid-360 / D435i + RViz：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_sim parking2_sim.launch
```

终端 2 — 对接控制器：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_parking auto_dock.launch
```

状态流程：

```text
PREALIGN_GATE -> ENTER_BAY -> CROUCH
```

| 阶段 | 作用 |
|------|------|
| `PREALIGN_GATE` | 仓外按 Tag 法向 / 横向 / 前方距离完成预对准 |
| `ENTER_BAY` | Tag + 左右墙激光引导入仓，近距可切激光终段 |
| `CROUCH` | 停车并发布趴下指令 |

常用监视：

```bash
rostopic echo /auto_dock/state
rostopic echo /auto_dock/success
rostopic echo /cmd_vel
```

重置二维场景：

```bash
rosservice call /parking/reset
```

手动遥控（不要同时开 `auto_dock`）：

```bash
rosrun go2w_sim teleop_keyboard.py
```

按键：`W/S` 前后，`A/D` 横移，`Q/E` 转向，松键停止。

### 2) 三维狗窝仿真

```bash
# 终端 1
roslaunch go2w_sim parking3_sim.launch

# 终端 2
roslaunch go2w_parking auto_dock.launch
```

默认使用平面运动插件（支持横移），适合验证算法。若要对比接触动力学：

```bash
roslaunch go2w_sim parking3_sim.launch physics:=true
```

> 物理模式下固定轮不执行横移，不适合完整验证依赖 `linear.y` 的流程。

其它仿真入口：

| 命令 | 说明 |
|------|------|
| `roslaunch go2w_sim simple_sim.launch` | 球棍外观、接触驱动 |
| `roslaunch go2w_sim unitree_sim.launch` | 宇树官方外观 |
| `roslaunch go2w_sim unitree_sim.launch rviz:=true` | 官方外观 + RViz |

### 3) 真机自动对接

先确认 Mid-360 已提供 `/scan`，再：

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_parking auto_dock_real.launch
```

会启动：D435i 彩色流、`tag36h11` ID 0 检测、检测可视化、自动对接控制器。

上真机前请完成：

- D435i / Mid-360 到 `base_link` 外参
- Tag 黑色边框实际尺寸（默认配置见 `go2w_vision`）
- 充电位目标距离与 `/cmd_vel` ↔ 宇树 SDK `Move(vx, vy, vyaw)` 适配

主要参数文件：

- `src/go2w_parking/config/auto_dock.yaml`
- `src/go2w_vision/launch/detect_tag.launch`
- `src/go2w_vision/config/settings.yaml`

## 默认关键参数（可改）

来自 `auto_dock.yaml`：

| 参数 | 默认 | 含义 |
|------|------|------|
| `gate_front_distance` | 1.50 m | 仓外预对准前距 |
| `target_distance` | 0.35 m | 最终停车距离 |
| `max_approach_speed` | 0.38 m/s | 接近限速 |
| `max_final_speed` | 0.12 m/s | 终段限速 |
| `max_lateral_speed` | 0.18 m/s | 横移限速 |
| `max_angular_speed` | 0.60 rad/s | 转向限速 |

仿真成功条件当前以位姿稳定后发布 `/auto_dock/success=true` 为准；真机充电建议再结合 BMS / 充电电流做最终确认。

## 网格与许可证

- 本仓库自有代码：**Apache-2.0**（见 `LICENSE`）
- Go2-W 外观网格来自 [Unitree unitree_ros](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2w_description)，按 **BSD-3-Clause** 分发（见 `NOTICE` 与 `src/go2w_sim/meshes/go2w_official/LICENSE`）

## 说明与边界

- 新流程从**已能看见 Tag** 的仓外预对准位开始，不包含远距离搜索 Tag。
- Tag 丢失时控制器停车，不会在仓内盲目搜索。
- 二维模拟相机会做墙体遮挡：视线被狗窝墙挡住时，`/tag_detections` 为空。

欢迎 Issue / PR 反馈仿真与真机部署问题。
