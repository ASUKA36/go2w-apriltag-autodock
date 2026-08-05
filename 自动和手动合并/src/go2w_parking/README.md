# go2w_parking

用于测试机器狗“停车/入窝”算法的轻量 ROS1 二维仿真。机器狗用矩形表示，按 Go2-W
运动方式接受 `geometry_msgs/Twist`：

- `linear.x`：前后；
- `linear.y`：左右横移；
- `angular.z`：原地转向。

启动（一键，含手动/自动切换）：

```bash
roslaunch go2w_sim parking2_full.launch
```

按 `R` 在手动与自动入库之间切换。三维一键启动：

```bash
roslaunch go2w_sim parking3_full.launch
```

也可只启动二维仿真：

```bash
roslaunch go2w_sim parking2_sim.launch
```

手动测试可复用现有键盘节点（W/S 前后、A/D 横移、Q/E 转向）：

```bash
rosrun go2w_sim teleop_keyboard.py
```

算法输入/输出：

- 输入：`/cmd_vel`
- 位姿：`/odom` 和 `odom -> base_link` TF
- 地图：`/parking/map`
- 激光：`/scan`
- 目标：`/parking/goal`
- 结果：`/parking/parked` 和 `/parking/status`
- 重置：`rosservice call /parking/reset`

机器人进入目标位置与角度容差，且低速保持 1 秒后，`/parking/parked` 变为
`true`。狗窝三面墙和机器狗矩形脚印参与碰撞检测。

## 无地图自动泊狗

二维场景会按机器狗与充电桩 Tag 的相对位姿模拟 `/tag_detections`，并由狗窝墙壁
生成 `/scan`。先启动仿真与 RViz（不启动停车算法）：

```bash
roslaunch go2w_sim parking2_sim.launch
```

RViz 会同时显示 `/scan` 的 Mid-360 二维扫描点和 `/camera/color/image_raw`
的模拟 D435i 彩色画面。再打开第二个终端，单独启动停车控制器：

```bash
roslaunch go2w_parking auto_dock.launch
```

二维模拟相机具有墙体遮挡：相机到 Tag 的视线只要与狗窝墙体相交，图像中的 Tag
和 `/tag_detections` 会同时消失；重新获得无遮挡视线后才恢复。

`auto_dock.py` 只订阅 `/tag_detections` 和 `/scan`，不读取地图、里程计或仿真真值。状态为：

```text
SEARCH_TAG / LIDAR_GARAGE_SEEK -> PRE_ALIGN_OUTSIDE -> APPROACH -> LIDAR_CENTERED_ENTRY
           -> FINE_ALIGN -> HOLD -> PARKED
```

运行时可监视：

```bash
rostopic echo /auto_dock/state
rostopic echo /auto_dock/success
rostopic echo /cmd_vel
```

`PRE_ALIGN_OUTSIDE` 禁止向前进入，先用横移和原地转向把机器狗在窝外摆正。
入口后从 Mid-360 扫描点拟合左右墙面，用墙面中心偏差修正 `linear.y`，用墙面
方向修正 `angular.z`。这些都在 `base_link` 局部坐标中完成。

RViz 中的橙红色 `/auto_dock/predicted_path` 是按当前限速后的 `vx/vy/wz` 外推的
3 秒局部预测轨迹，用于直观检查下一段运动趋势；它不是全局规划路径。

当 `tag36h11 ID 0` 不可见但 Mid-360 拟合到左右狗窝墙时，控制器进入
`LIDAR_GARAGE_SEEK`：先对齐绿色雷达中心线，再低速向入口靠近以主动重新找到
Tag。前方距离过近时不会继续盲目前进。雷达估计状态可通过以下话题检查：

```bash
rostopic echo /auto_dock/lidar_garage_detected
rostopic echo /auto_dock/lidar_center_error
rostopic echo /auto_dock/lidar_heading_error
```

泊车目标不是 Tag 本身的位置，而是配置文件中标定的“相机到 Tag 的充电位距离”。
仿真默认为 `0.60 m`，真机使用前必须按充电板、相机和 Tag 安装关系重新标定。
控制参数见 `config/auto_dock.yaml`。

## 三维泊车场景

三维场景保留相同的 `/scan`、`/camera/color/image_raw`、`/tag_detections` 和
`/cmd_vel` 接口，并使用宇树官方 Go2-W 网格外观。Gazebo 墙体具有碰撞和真实射线
遮挡，相机看不到墙后的 Tag，激光也只能返回最先命中的表面。

终端 1：

```bash
roslaunch go2w_sim parking3_sim.launch
```

终端 2：

```bash
roslaunch go2w_parking auto_dock.launch
```

默认运动模式支持 Go2-W 式前后、横移和原地转向。使用以下命令可以切换到官方
外观搭配接触驱动轮式动力学的对比模式：

```bash
roslaunch go2w_sim parking3_sim.launch physics:=true
```

固定轮物理模式会忽略 `linear.y`，主要用于观察动力学效果，而不是运行依赖横移的
完整自动泊车流程。
