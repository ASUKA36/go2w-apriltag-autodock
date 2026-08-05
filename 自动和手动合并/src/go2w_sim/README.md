# go2w_sim

Go2-W 的 ROS1/Gazebo 仿真包。模型包含必要的躯干、腿部关节、轮子、Mid-360
二维扫描和 RGB-D 相机；详细外观使用宇树官方 Go2-W 网格。

`launch` 目录只保留四个入口：

| 启动文件 | 用途 |
|---|---|
| `simple_sim.launch` | 球棍外观与接触驱动轮式动力学 |
| `unitree_sim.launch` | 宇树官方外观与接触驱动轮式动力学 |
| `parking2_sim.launch` | 轻量二维自动泊车场景 |
| `parking3_sim.launch` | 官方外观、三维狗窝、相机和激光泊车场景 |

## 普通三维仿真

```bash
roslaunch go2w_sim simple_sim.launch
roslaunch go2w_sim unitree_sim.launch
```

需要 RViz 时：

```bash
roslaunch go2w_sim unitree_sim.launch rviz:=true
```

## 自动泊车仿真

二维：

```bash
roslaunch go2w_sim parking2_sim.launch
```

三维：

```bash
roslaunch go2w_sim parking3_sim.launch
```

停车算法保持独立启动：

```bash
roslaunch go2w_parking auto_dock.launch
```

`parking3_sim.launch` 默认使用稳定的运动学平面插件，支持前后、横移和原地转向。
如需观察固定轮接触动力学，可增加 `physics:=true`；该模式不会执行横移指令。

键盘控制节点仍可直接运行：

```bash
rosrun go2w_sim teleop_keyboard.py
```
