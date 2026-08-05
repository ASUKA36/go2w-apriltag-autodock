# 自动和手动合并

本目录为当前 3D 停车仿真 + 自动/手动合并控制（R 键切换）的代码快照。

## 主要内容
- `go2w_parking`：`auto_dock` 三阶段入仓、`mode_control` 键盘手动/自动、2D `parking_sim`
- `go2w_sim`：`parking_3d.world`、`parking3_full.launch` / `parking2_full.launch`、平面插件复位
- `go2w_vision`：AprilTag 检测

## 在 ROS 工作空间使用
将本目录下的 `src/*` 拷入 catkin 工作空间的 `src/` 后编译，或直接以本仓库根目录 `src/` 为准（根目录为持续开发树，本文件夹为本次提交快照）。

## 一键启动
```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_sim parking3_full.launch   # 3D
# roslaunch go2w_sim parking2_full.launch # 2D
```
R：MANUAL ↔ AUTO；W/S A/D Q/E 驾驶；Z 趴下 / X 站立。
