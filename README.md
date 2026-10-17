# go2w-apriltag-autodock

Map-free auto-docking for **Unitree Go2-W**. The robot uses **AprilTag (`tag36h11`)** relative pose to enter a narrow charging bay. Mid-360 lidar can assist the final approach.

This repo includes:

- ROS1 / Gazebo simulation of Go2-W (lightweight 2D tests plus a 3D official-mesh kennel)
- A map-free docking state machine (pre-align → enter bay → crouch)
- A real-robot entry point for D435i + AprilTag detection

> GitHub: https://github.com/ASUKA36/go2w-apriltag-autodock

## Features

- **No global map or odometry dependency.** The controller mainly subscribes to `/tag_detections` (and optionally `/scan`), which fits a fixed charging bay.
- **Staged docking:** `PREALIGN_GATE` (pre-align outside the bay) → `ENTER_BAY` → `CROUCH`.
- **Holonomic chassis commands:** `linear.x`, `linear.y`, and `angular.z` are used together.
- **Same control package for sim and hardware:** `go2w_parking`, selected by launch files.

## Layout

```text
go2w-apriltag-autodock/
├── src/
│   ├── go2w_sim/       # Go2-W model, Gazebo worlds, sensors, keyboard teleop
│   ├── go2w_parking/   # parking scene, predicted path, map-free docking controller
│   └── go2w_vision/    # real D435i + tag36h11 detection
├── LICENSE             # Apache-2.0 for this repo's own code
└── NOTICE              # Unitree mesh provenance (BSD-3-Clause)
```

Data flow:

```text
Mid-360 /scan  ──────────────┐
                             ├─> go2w_parking/auto_dock ─> /cmd_vel
D435i /tag_detections ───────┘
```

## Requirements

- Ubuntu 20.04 (or WSL2)
- ROS1 Noetic
- Gazebo, RViz, `apriltag_ros`, and `realsense2_camera` (hardware)

```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins \
  ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz \
  ros-noetic-apriltag-ros ros-noetic-realsense2-camera ros-noetic-image-view
```

## Build

```bash
git clone https://github.com/ASUKA36/go2w-apriltag-autodock.git go2w_ws
cd go2w_ws
catkin_make
source devel/setup.bash
```

## Quick start

### 1) 2D auto-dock simulation (recommended first)

Terminal 1 — simulation, mocked Mid-360 / D435i, and RViz:

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_sim parking2_sim.launch
```

Terminal 2 — docking controller:

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_parking auto_dock.launch
```

State flow:

```text
PREALIGN_GATE -> ENTER_BAY -> CROUCH
```

| Stage | Role |
|------|------|
| `PREALIGN_GATE` | Align outside the bay using Tag normal, lateral error, and front range |
| `ENTER_BAY` | Tag plus left/right wall lidar guide the robot in; near the board it can switch to a lidar-only final stage |
| `CROUCH` | Stop and publish a lie-down command |

Useful topics:

```bash
rostopic echo /auto_dock/state
rostopic echo /auto_dock/success
rostopic echo /cmd_vel
```

Reset the 2D scene:

```bash
rosservice call /parking/reset
```

Manual teleop (do not start `auto_dock` at the same time):

```bash
rosrun go2w_sim teleop_keyboard.py
```

Keys: `W/S` forward/back, `A/D` strafe, `Q/E` yaw. Release a key to stop.

### 2) 3D kennel simulation

```bash
# terminal 1
roslaunch go2w_sim parking3_sim.launch

# terminal 2
roslaunch go2w_parking auto_dock.launch
```

The default planar motion plugin supports strafing and is meant for algorithm checks. To compare contact dynamics:

```bash
roslaunch go2w_sim parking3_sim.launch physics:=true
```

> In physics mode the fixed wheels do not execute strafe, so it is not a full check of a `linear.y` docking flow.

Other sim entries:

| Command | Description |
|------|------|
| `roslaunch go2w_sim simple_sim.launch` | Stick model, contact-driven wheels |
| `roslaunch go2w_sim unitree_sim.launch` | Official Unitree mesh |
| `roslaunch go2w_sim unitree_sim.launch rviz:=true` | Official mesh + RViz |

### 3) Real-robot docking

Confirm Mid-360 is publishing `/scan`, then:

```bash
source ~/go2w_ws/devel/setup.bash
roslaunch go2w_parking auto_dock_real.launch
```

This starts the D435i color stream, `tag36h11` ID 0 detection, a detection view, and the docking controller.

Before running on hardware, set:

- D435i / Mid-360 extrinsics to `base_link`
- The real black-border size of the Tag (defaults live in `go2w_vision`)
- Charging-bay target range and `/cmd_vel` ↔ Unitree SDK `Move(vx, vy, vyaw)` mapping

Main parameter files:

- `src/go2w_parking/config/auto_dock.yaml`
- `src/go2w_vision/launch/detect_tag.launch`
- `src/go2w_vision/config/settings.yaml`

## Default parameters

From `auto_dock.yaml`:

| Parameter | Default | Meaning |
|------|------|------|
| `gate_front_distance` | 1.50 m | Front range at the outside pre-align gate |
| `target_distance` | 0.35 m | Final stop range |
| `max_approach_speed` | 0.38 m/s | Approach speed limit |
| `max_final_speed` | 0.12 m/s | Final-stage speed limit |
| `max_lateral_speed` | 0.18 m/s | Strafe speed limit |
| `max_angular_speed` | 0.60 rad/s | Yaw rate limit |

Simulation success is currently `/auto_dock/success=true` after the pose hold. On a real robot, confirm charging with BMS / charge current as well.

## Meshes and license

- Code in this repository: **Apache-2.0** (`LICENSE`)
- Go2-W visual meshes come from [Unitree unitree_ros](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2w_description) and are redistributed under **BSD-3-Clause** (see `NOTICE` and `src/go2w_sim/meshes/go2w_official/LICENSE`)

## Notes and limits

- The current flow starts from a pre-align pose **where the Tag is already visible**. It does not search for the Tag from far away.
- If the Tag is lost, the controller stops. It will not wander inside the bay.
- The 2D mock camera applies wall occlusion: `/tag_detections` is empty when a kennel wall blocks the line of sight.

Issues and PRs about simulation or hardware bring-up are welcome.
