# go2w_parking

用于二维、三维仿真和真机的无地图自动泊狗控制包。二维场景中机器狗用矩形表示，
三维场景使用宇树官方 Go2-W 外观；两者都按 Go2-W 运动方式接受
`geometry_msgs/Twist`：

- `linear.x`：前后；
- `linear.y`：左右横移；
- `angular.z`：原地转向。

启动：

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
PREALIGN_GATE -> ENTER_BAY -> CROUCH
```

运行时可监视：

```bash
rostopic echo /auto_dock/state
rostopic echo /auto_dock/success
rostopic echo /cmd_vel
```

`PREALIGN_GATE` 先按 Tag 法向和横向位置摆正，并将前方激光距离稳定在
`1.50 ± 0.05 m`；条件持续满足 2 秒后进入 `ENTER_BAY`。入口后从 Mid-360
扫描点拟合左右墙面，用墙面中心偏差修正 `linear.y`，用墙面方向修正
`angular.z`。接近充电板后切换为激光终段，到达约 `0.35 m` 时进入 `CROUCH`。

RViz 中的橙红色 `/auto_dock/predicted_path` 是按当前限速后的 `vx/vy/wz` 外推的
3 秒局部预测轨迹，用于直观检查下一段运动趋势；它不是全局规划路径。

新版流程从能够看到 `tag36h11 ID 0` 的门外预对准位置开始，不包含原地搜索 Tag
或雷达寻库前置阶段。Tag 接近画面下方后允许转为纯雷达终段。雷达估计状态可通过
以下话题检查：

二维场景默认固定出生在 Tag 正前方，确保独立启动算法后能够直接运行。若将
`parking.yaml` 的 `spawn_yaw_random` 改为 `true`，随机朝向看不到 Tag 时会保持
`WAIT_TAG` 和零速度，这是无搜索阶段的预期行为。

```bash
rostopic echo /auto_dock/lidar_garage_detected
rostopic echo /auto_dock/lidar_center_error
rostopic echo /auto_dock/lidar_heading_error
```

泊车目标不是 Tag 本身的位置，而是配置文件中标定的激光最终停车距离。
仿真默认为 `0.35 m`，真机使用前必须按充电板、相机、雷达和 Tag 安装关系重新标定。
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

## 默认速度

当前参数采用偏保守的泊车速度：

| 参数 | 默认值 |
|---|---:|
| 最大接近速度 | `0.38 m/s` |
| 最大最终入库速度 | `0.12 m/s` |
| 最大横移速度 | `0.18 m/s` |
| 最大转向速度 | `0.60 rad/s` |
| 雷达寻窝速度 | `0.16 m/s` |

需要进一步减速时修改 `config/auto_dock.yaml`，无需改动控制器源码。
