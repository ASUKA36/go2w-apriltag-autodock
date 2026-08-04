# go2w_navigation

Go2-W 的 ROS1 `move_base` 参数包，只启动导航相关节点，不会启动 Gazebo 或 RViz。

```bash
roslaunch go2w_navigation navigation.launch
```

启动前应由 `go2w_sim` 或真实机器人提供 `/map`、`/scan`、`/odom` 和 TF，并接收
`/cmd_vel`。
