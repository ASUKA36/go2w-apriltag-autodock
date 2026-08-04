# go2w_sim

轻量 Gazebo 11 三维仿真包，包含简化 Go2-W、240 束二维激光、424×240 RGB-D
相机、测试世界、二维 OccupancyGrid 和 `/cmd_vel` 平面运动插件。

```bash
roslaunch go2w_sim sim.launch
```

常用参数：`gui`、`rviz`、`camera`、`paused`、`world`、`map_config`。

```bash
roslaunch go2w_sim sim.launch gui:=false rviz:=true camera:=true
```

## 球棍轮腿动力学模型

原有 `sim.launch` 平面快速模型保持不变。实验性的球棍轮腿动力学版本使用：

```bash
roslaunch go2w_sim physics_sim.launch
```

使用官方 Go2-W 网格外观进行对比（动力学、碰撞体、步态和控制话题与球棍版相同）：

```bash
roslaunch go2w_sim unitree_sim.launch
```

需要同时打开 RViz 时添加 `rviz:=true`。官方网格仅用于显示，Gazebo 碰撞仍采用
轻量基础几何体，因此不会把高面数 DAE/STL 当作碰撞网格。官方外观版渲染负担高于
球棍版；WSL 较卡时可使用 `gui:=false rviz:=true`，或继续使用默认球棍版。

官方网格与关节尺寸来自 Unitree Robotics 的 `unitree_ros/robots/go2w_description`。
官方 Go2-W URDF 没有 `transmission` 或 Go2-W 高层步态控制器；本包因此保留
自有的 ROS1 Gazebo 控制插件。`unitree_ros` 的通用 `unitree_controller` 可作为
位置/速度/力矩级关节控制参考，但不能直接提供 Go2-W 行走步态。

该版本保留16个 Go2-W 关节、质量和惯量、四轮碰撞/摩擦、轮速PID、机身高度/姿态/
速度反馈，以及 FL+RR、FR+RL 对角相位腿轨迹。模型通过暂停生成、设置站立关节角后
再解除物理，避免初始穿地。机身与轮子由 Gazebo 物理积分；为适配低惯量球棍模型，
腿轨迹采用30 Hz混合关节伺服，并非宇树闭源固件的力矩级步态控制器。

固定轮动力学模式不支持 `linear.y` 横移。当前 `StandDown` 会安全停车并保持站立，
因为完整折腿还需要接触感知的足端逆运动学；完整趴下仍可在原 `sim.launch` 中使用。

机器人初始朝向正前方的墙面带有 `tag36h11`、ID 0 的 AprilTag。墙上黑色编码
方形的仿真边长为 `0.16 m`，可这样启动 Gazebo 相机识别：

```bash
roslaunch go2w_vision detect_tag.launch start_realsense:=false tag_size:=0.16
```

键盘遥控请在独立终端运行：

```bash
rosrun go2w_sim teleop_keyboard.py
```

W/S 控制前后，A/D 控制左右横移，Q/E 控制朝向，空格停车，Z 执行与宇树
`StandDown` 对应的趴下状态，X 执行 `StandUp`。姿态话题为 `/go2w/posture_cmd`。
在 WSL 中节点直接读取 Windows 的 key-down/key-up 状态：按住持续运动，松开立即发布
零速度，并支持 `W+Q` 这类组合键。非 WSL 终端自动使用按键重复看门狗作为兼容模式。
