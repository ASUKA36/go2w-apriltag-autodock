# 自动和手动合并

本目录是 **Go2-W 3D/2D 停车仿真 + 手动/自动合并控制** 的代码快照，对应仓库提交时的完整功能包（`go2w_sim` / `go2w_parking` / `go2w_vision`）。

与仓库根目录 `src/` 的关系：根目录为持续开发树；**本文件夹为可单独拷贝使用的版本快照**。

---

## 功能概览

| 模块 | 作用 |
|------|------|
| `mode_control` | 键盘手动遥控；按 **R** 在 MANUAL ↔ AUTO 间切换 |
| `auto_dock` | 无地图自动入仓（AprilTag + Mid-360） |
| `parking_sim` | 2D 轻量泊车仿真（含充电板、出生区、LOS 遮挡） |
| `parking_3d.world` | 3D 狗窝、充电板、AprilTag、出生区 |
| `parking2/3_full.launch` | 一键启动：仿真 + Tag + auto_dock + 键盘 |

---

## 环境

- Ubuntu 20.04 / WSL2
- ROS1 Noetic
- Gazebo（3D）

依赖示例：

```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins \
  ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz \
  ros-noetic-apriltag-ros ros-noetic-image-view
```

---

## 安装到工作空间

```bash
# 假设仓库已克隆为 ~/go2w_ws（或从本目录拷贝包）
cp -r 自动和手动合并/src/* ~/go2w_ws/src/
cd ~/go2w_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

也可直接使用仓库根目录 `src/`（若已与本快照同步）。

---

## 一键启动

```bash
source ~/go2w_ws/devel/setup.bash

# 3D：Gazebo + 相机/激光 + Tag 检测 + 自动泊车 + 键盘
roslaunch go2w_sim parking3_full.launch

# 2D：RViz 平面仿真 + 同上
roslaunch go2w_sim parking2_full.launch
```

仅仿真（不含 auto_dock / 键盘）：

```bash
roslaunch go2w_sim parking3_sim.launch
roslaunch go2w_sim parking2_sim.launch
```

---

## 键盘操作

默认 **MANUAL**。焦点在终端或 WSL 可读键盘状态时生效。

| 按键 | 作用 |
|------|------|
| **R** | 切换 MANUAL ↔ AUTO |
| W / S | 前进 / 后退 |
| A / D | 左移 / 右移 |
| Q / E | 左转 / 右转 |
| Space | 急停 |
| + / - | 调速 |
| Z | 趴下 |
| X | 站立 |

切到 AUTO 后由 `auto_dock` 接管 `/cmd_vel`；再按 R 回到手动。

---

## 自动入仓流程（三阶段）

看到目标 Tag（`tag36h11` id 0）后开始；**无搜 Tag / 雷达寻库前置步骤**。

1. **PREALIGN_GATE（预对准）**  
   - 横移使 Tag 居中（相机 `x`）  
   - 航向对齐 Tag **法向** `facing`（**不使用 bearing**）  
   - 前方雷达稳到 **1.5 m ± 5 cm**，条件连续满足 **2 s** 后进入下一阶段  

2. **ENTER_BAY（入仓）**  
   - 相机 + 雷达走廊保证直行靠近  
   - Tag 落到画面下方后改为纯雷达终段  
   - 前方约 **0.35 m**、左右误差 **&lt; 5 cm** 进入下一阶段  

3. **CROUCH（蹲下）**  
   - 停车并发 `lie_down`  
   - 可选延迟后仿真复位到出生区并再开一轮  

状态话题：`/auto_dock/state`  
模式话题：`/control_mode`（`manual` / `auto`）

主要参数见：`src/go2w_parking/config/auto_dock.yaml`。

---

## 场景几何（当前快照）

| 项目 | 数值 |
|------|------|
| 充电板 | 长 1.2 m × 宽 0.7 m |
| 板中心距 Tag | 0.55 m |
| Tag 边长 / 离地 | 0.11 m / 0.30 m |
| 仓墙高度 | 1.2 m |
| 出生区 | 仓前约 2 m 处，2×2 m 随机位姿 |
| 趴下目标距 Tag | 约 0.35 m（雷达前方） |

---

## 目录结构

```text
自动和手动合并/
├── README.md                 # 本说明
├── README_upstream.md        # 上游仓库说明备份
└── src/
    ├── go2w_parking/         # 泊车控制与 2D 仿真
    ├── go2w_sim/             # Gazebo / launch / 插件 / 模型
    └── go2w_vision/          # AprilTag 检测 launch
```

常用 launch：

- `go2w_sim/launch/parking3_full.launch`
- `go2w_sim/launch/parking2_full.launch`
- `go2w_sim/launch/parking3_sim.launch`
- `go2w_sim/launch/parking2_sim.launch`

---

## 说明

- 自动泊车**不依赖**全局地图与里程计，只使用 `/scan` 与 `/tag_detections`。  
- 3D 外观网格来自宇树官方 Go2-W，许可证见包内 `NOTICE` / meshes 说明。  
- 若进程残留导致端口占用，可在 Windows 执行：`wsl --terminate Ubuntu-20.04`。
