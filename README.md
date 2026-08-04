# nav_sim

`nav_sim` 是面向 ROS1 Noetic/WSL2 的轻量 3D 导航与视觉算法仿真环境。模型结构参考
[`jj7258/unitree_go2w_ros2`](https://github.com/jj7258/unitree_go2w_ros2/tree/main/src/go2w_description/urdf)
中的 Go2-W URDF，保留原模型的主要尺寸、关节名称、关节层级、关节轴和轮腿特征，
将 DAE/STL 网格替换为低面数箱体和圆柱体。

本项目不是宇树官方模型，不用于验证真实动力学或腿部控制。底盘采用稳定的平面运动
插件，适合验证导航、视觉感知、轨迹生成和多传感器算法。

## 模型与传感器

- 躯干、头部和四条轮腿；
- 每条腿包含 hip、thigh、calf 和连续 foot/wheel 关节，共 16 个活动关节；
- 关节使用参考 URDF 的 `FL_*`、`FR_*`、`RL_*`、`RR_*` 命名；
- 顶部 MID-360 简化安装体和 240 束二维激光；
- 前置轻量 RGB-D 相机，默认 424×240、15 FPS；
- Gazebo 3D 障碍物、不同颜色/高度目标和视觉坡道；
- `/cmd_vel`、`/odom`、`/scan`、`/joint_states` 和完整 TF；
- RGB、深度图、CameraInfo 和按需点云；
- 已知二维 OccupancyGrid 及 `move_base` 示例。

平面插件固定腿部为站立姿态并让四个轮子随速度旋转。它直接更新模型平面位姿，
因此不会模拟倾倒、悬挂、打滑、越障和腿部受力。

## 安装

```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins \
  ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz \
  ros-noetic-move-base

cd ~/catkin_ws/src
git clone https://gitee.com/alaurb/go2w_nav_sim.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 启动

启动 Gazebo、机器人和 RViz：

```bash
roslaunch nav_sim sim.launch
```

启动 `move_base` 导航演示：

```bash
roslaunch nav_sim navigation.launch
```

在 RViz 中用 **2D Nav Goal** 设置目标。仿真使用精确 `map -> odom` 关系，默认不需要
AMCL，便于专注规划器、控制器和视觉轨迹算法。

WSL 低负载模式：

```bash
roslaunch nav_sim sim.launch gui:=false rviz:=true camera:=true
```

仅测试导航、关闭相机：

```bash
roslaunch nav_sim navigation.launch gui:=false camera:=false
```

## 视觉话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 图像 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RGB 内参 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度内参 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 深度点云，存在订阅者时生成 |

点云计算最消耗性能；不需要时不要在 RViz 中启用 `Depth Points` 显示。

## 导航接口

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 输入 | 平面速度命令 |
| `/map` | `nav_msgs/OccupancyGrid` | 输出 | 与 3D 世界障碍物对应的二维地图 |
| `/scan` | `sensor_msgs/LaserScan` | 输出 | 顶部二维激光 |
| `/odom` | `nav_msgs/Odometry` | 输出 | 仿真里程计 |
| `/joint_states` | `sensor_msgs/JointState` | 输出 | 12 个腿关节和 4 个轮关节 |

TF 主链：`map -> odom -> base_footprint -> base -> legs/wheels/camera/lidar/imu`。

## 自动验证

```bash
roslaunch nav_sim verify.launch
roslaunch nav_sim vision_verify.launch
roslaunch nav_sim navigation_verify.launch
```

三项测试依次检查基础导航接口、RGB-D 图像/内参，以及真实 `move_base` Action 到点。

## 自定义环境

3D 世界位于 `worlds/nav_world.world`，二维导航地图参数位于 `config/world.yaml`。
添加或移动障碍物时，需要同步修改两个文件中的矩形范围，确保规划地图与 Gazebo 世界一致。

相机分辨率和帧率在 `urdf/go2w_stick.urdf.xacro` 中配置。WSL 卡顿时建议按顺序：

1. 禁用 RViz 点云显示；
2. 使用 `gui:=false` 关闭 Gazebo 图形界面；
3. 将相机更新率降至 10 FPS；
4. 只验证导航时使用 `camera:=false`。

## 上传 Gitee

目录本身是完整 catkin 包。上传前修改 `package.xml` 的维护者信息和 README 中的仓库地址。
项目采用 Apache-2.0 License；请保留 `LICENSE`、`NOTICE` 和上方参考仓库来源说明。
