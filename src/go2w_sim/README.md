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

机器人初始朝向正前方的墙面带有 `tag36h11`、ID 0 的 AprilTag。墙上黑色编码
方形的仿真边长约为 `0.46 m`，可这样启动 Gazebo 相机识别：

```bash
roslaunch go2w_vision detect_tag.launch start_realsense:=false tag_size:=0.46
```
