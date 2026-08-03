# mecanum-manipulator-ros2

ROS2 simulation of a mecanum-wheeled AGV with an RV-3SDB robotic arm, integrating Webots, MoveIt2 and MoveIt Servo into a single ROS2 application.

---

## Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy
- [Webots R2025a](https://cyberbotics.com)
- [webots_ros2](https://github.com/cyberbotics/webots_ros2)
- [webots_ros2_control](https://github.com/cyberbotics/webots_ros2) (arm control)
- MoveIt2 (`moveit_py`, `moveit_servo`, `moveit_ros_move_group`, `moveit_planners_ompl`, ...)
- `ros-jazzy-joint-state-publisher-gui` (manual arm joint sliders)

---

## Quick Start

```bash
# Source ROS2 (add this to your ~/.bashrc to avoid repeating it every terminal)
source /opt/ros/jazzy/setup.bash

# Clone the repository
git clone https://github.com/PetraMicanovic/mecanum-manipulator-ros2.git
cd mecanum-manipulator-ros2

# Install dependencies
sudo rosdep init   # only if never run before on this machine
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install python3-scipy
sudo apt install ros-jazzy-joint-state-publisher-gui

# Build
colcon build --symlink-install
source install/setup.bash
```

You can now launch either the **platform-only** simulation or the **platform with arm** simulation,
depending on what you want to work with.

---

## Option A — Platform only

```bash
ros2 launch mobile_platform_sim mobile_platform.launch.py
ros2 launch mobile_platform_sim mobile_platform.launch.py rviz:=true   # optional RViz2
```

This brings up Webots with the bare mecanum platform: `MecanumRobotDriver`,
`robot_state_publisher`, `joint_state_publisher`, `odometry_publisher`, TF publishers and
(optionally) RViz2.

In a second terminal, drive the platform directly:

```bash
source install/setup.bash

# Keyboard control (default)
ros2 run mecanum_manipulator_control mecanum_platform_controller

# Predefined sequence
ros2 run mecanum_manipulator_control mecanum_platform_controller --ros-args -p mode:=sequence

# Terminal control
ros2 run mecanum_manipulator_control mecanum_platform_controller --ros-args -p mode:=terminal
```

---

## Option B — Platform and rv3sdb arm

```bash
ros2 launch mobile_platform_with_arm mobile_platform_with_arm.launch.py
```

Launch arguments (all optional):

| Argument | Default | Effect |
|---|---|---|
| `rviz` | `false` | Start a plain RViz2 instance alongside the simulation |
| `moveit` | `true` | Include `rv3sdb_moveit_config`'s `move_group.launch.py`, so the arm can be planned/executed through MoveIt2 |
| `servo` | `true` | Start `moveit_servo`'s `servo_node`, so the arm can be jogged in real time |

Both moveit and servo can be enabled independently.

Examples:

```bash
ros2 launch mobile_platform_with_arm mobile_platform_with_arm.launch.py rviz:=true
ros2 launch mobile_platform_with_arm mobile_platform_with_arm.launch.py servo:=false
ros2 launch mobile_platform_with_arm mobile_platform_with_arm.launch.py moveit:=false servo:=true
```

This brings up:
- Webots with the combined mobile platform with arm world
- the mecanum platform, fully drivable exactly as in Option A (`/cmd_vel`, wheel odometry, both lidars)
- `webots_ros2_control` exposing the arm's 6 joints, plus `joint_state_broadcaster` and `arm_controller`
- `robot_state_publisher`, building TF for the whole chassis and arm tree (wheel and arm joint states
  are merged into a single `/joint_states` feed by `joint_state_publisher`)
- MoveIt2's `move_group` (if `moveit:=true`), for planning/executing to named SRDF poses or interactive goals in RViz
- `moveit_servo`'s `servo_node` (if `servo:=true`), for real-time Cartesian/joint jogging

In a second terminal, you can then:

**Drive the platform** — same as Option A:
```bash
ros2 run mecanum_manipulator_control mecanum_platform_controller
```

**Jog the arm in real time** via `moveit_servo`:
```bash
ros2 run mecanum_manipulator_control arm_servo_keyboard
```

**Plan and execute the arm to a named SRDF pose** via MoveIt2:
```bash
ros2 run mecanum_manipulator_control plan_named_pose
ros2 run mecanum_manipulator_control plan_named_pose --ros-args -p target_pose:=capture_pose1
```

**Run the combined table-push demo** (platform approaches the table, arm moves to
`ready_to_push`, platform nudges forward to push a cube, then both return to their start state):
```bash
ros2 run mecanum_manipulator_control demo
```

---

## Controlling the Platform

The `mecanum_manipulator_control` package provides three ways to drive the platform via
`mecanum_platform_controller`, selected with the `mode` ROS2 parameter.

**Keyboard mode** (default) — press a key to move, any other key to stop:

| Key | Motion |
|-----|--------|
| `w` | Forward |
| `s` | Backward |
| `a` | Strafe left *(mecanum only)* |
| `d` | Strafe right *(mecanum only)* |
| `q` | Rotate left |
| `e` | Rotate right |
| any other | Stop |
| `Ctrl+C` | Quit |

**Sequence mode** — the platform runs through a fixed choreography automatically and exits when
done. Translational steps are specified in metres, rotational steps in degrees — duration is
computed automatically from the distance and the default velocity. Edit `PLATFORM_SEQUENCE` in
`mecanum_manipulator_control/mecanum_platform_controller.py` to change the steps.

**Terminal mode** — type a command followed by a distance in metres or an angle in degrees. The
platform executes the command and waits for the next input:

| Command | Argument | Effect |
|---------|----------|--------|
| `forward <m>` | metres | Move forward |
| `back <m>` | metres | Move backward |
| `left <m>` | metres | Strafe left *(mecanum only)* |
| `right <m>` | metres | Strafe right *(mecanum only)* |
| `rotate_l <deg>` | degrees | Rotate left |
| `rotate_r <deg>` | degrees | Rotate right |
| `stop` | — | Stop immediately |
| `quit` | — | Exit |

Example:
```
> forward 1.5
> left 0.5
> rotate_l 90
> stop
```

---

## Controlling the Arm

### Real-time jogging (`arm_servo_keyboard`)

Streams jog commands to `moveit_servo`. Requires the platform with arm simulation running with
`servo:=true` (the default). Press `m` to toggle between two modes:

**CARTESIAN mode** (default off, task-space velocity of `tool_link`, resolved to joint motion via IK):

| Key | Motion |
|---|---|
| `w` / `s` | +X / −X (forward / back) |
| `a` / `d` | +Y / −Y (left / right) |
| `r` / `f` | +Z / −Z (up / down) |
| `q` / `e` | yaw left / right |
| `u` / `o` | roll +/− |
| `j` / `l` | pitch +/− |

**JOINT mode** (direct per-joint velocity, no IK, no singularity checking — useful for moving a
joint like `joint5` away from a singular configuration before switching to CARTESIAN):

| Key | Motion |
|---|---|
| `1` / `!` | joint1 +/− |
| `2` / `@` | joint2 +/− |
| `3` / `#` | joint3 +/− |
| `4` / `$` | joint4 +/− |
| `5` / `%` | joint5 +/− |
| `6` / `^` | joint6 +/− |

`m` toggles mode, any other key stops, `Ctrl+C` quits.

### Planning to named poses (`plan_named_pose`)

Plans and executes the arm through MoveIt2 to a named `group_state` defined in
`rv3sdb_moveit_config`'s SRDF (`config/rv3sdb.srdf`):

| Pose name | Description |
|---|---|
| `zero_pose` | All joints at 0 |
| `start_pose` | Arm's resting/home configuration |
| `capture_pose1` | First camera-capture pose |
| `capture_pose2` | Second camera-capture pose |
| `ready_to_push` | Positioned to push an object on the table |

```bash
ros2 run mecanum_manipulator_control plan_named_pose --ros-args -p target_pose:=ready_to_push
```

---

## Main ROS2 Topics

The platform is controlled by publishing `geometry_msgs/Twist` on `/cmd_vel`:

| Field | Effect |
|---|---|
| `linear.x` | Forward / backward |
| `linear.y` | Left / right (sideways — mecanum only) |
| `angular.z` | Rotation |

Available topics:

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/cmd_vel` | sub | `geometry_msgs/Twist` | Platform velocity commands |
| `/wheel/odometry` | pub | `nav_msgs/Odometry` | Wheel encoder odometry |
| `/livox/lidar_front` | pub | `sensor_msgs/PointCloud2` | Front Livox MID-70 |
| `/livox/lidar_back` | pub | `sensor_msgs/PointCloud2` | Rear Livox MID-70 |
| `/joint_states` | pub | `sensor_msgs/JointState` | Combined wheel + arm joint states |
| `/servo_node/delta_twist_cmds` | pub | `geometry_msgs/TwistStamped` | Cartesian jog commands to moveit_servo |
| `/servo_node/delta_joint_cmds` | pub | `control_msgs/JointJog` | Per-joint jog commands to moveit_servo |

---

## Project Structure

Standard ROS2 `ament_python` / `ament_cmake` workspace — every package is a direct subfolder of `src/`:

```
src/
├── mobile_platform_sim/              Webots simulation of the mecanum platform (platform only)
│   ├── launch/
│   │   └── mobile_platform.launch.py
│   ├── meshes/
│   │   ├── chassis.stl
│   │   ├── livox_mount.stl
│   │   └── IntelRealSenseT265.STL    # mesh available, not yet in simulation
│   ├── mobile_platform_sim/
│   │   ├── mecanum_robot_driver.py
│   │   └── odometry_publisher.py
│   ├── resource/
│   │   └── mobile_platform.urdf
│   └── worlds/
│       └── platform.wbt
│
├── mobile_platform_with_arm/         Webots simulation of the platform with the rv3sdb arm
│   ├── launch/
│   │   └── mobile_platform_with_arm.launch.py   # Webots, drivers and optional MoveIt2/servo
│   ├── meshes/
│   │   ├── chassis.stl
│   │   ├── livox_mount.stl
│   │   └── arm/                      # rv3sdb meshes
│   ├── resource/
│   │   └── mobile_platform_with_arm.urdf
│   └── worlds/
│       └── mobile_platform_with_arm.wbt
│
├── rv3sdb_moveit_config/             MoveIt2 configuration for the rv3sdb arm
│   ├── config/
│   │   ├── rv3sdb.srdf               # arm group and named poses (zero_pose, start_pose, ...)
│   │   ├── kinematics.yaml
│   │   ├── joint_limits.yaml
│   │   ├── ompl_planning.yaml
│   │   ├── moveit_controllers.yaml
│   │   ├── ros2_controllers.yaml
│   │   └── servo_params.yaml
│   └── launch/
│       ├── move_group.launch.py      # move_group node (assumes platform launch already running)
│       └── moveit_rviz.launch.py     # RViz2 preconfigured for MotionPlanning
│
└── mecanum_manipulator_control/      Platform with arm control application
    └── mecanum_manipulator_control/
        ├── mecanum_platform_controller.py   # keyboard / sequence / terminal platform driving
        ├── arm_servo_keyboard.py            # real-time arm jogging via moveit_servo
        ├── plan_named_pose.py               # MoveIt2 planning to named SRDF poses
        ├── moveit_config_utils.py           # shared MoveItPy config builders
        └── demo.py                          # combined platform and arm table-push task
```

---

## Platform and Arm Geometry

### Platform

All dimensions taken from the original ROS1 project.

| Parameter | Value |
|---|---|
| L_x (half-wheelbase, longitudinal) | 0.2505 m |
| L_y (half-wheelbase, lateral) | 0.26536 m |
| Wheel radius | 0.1015 m |
| Wheel width | 0.077 m |

### Arm (rv3sdb / Mitsubishi Melfa RV-3SDB)

| Parameter | Value |
|---|---|
| DOF | 6 |
| Joint names | `joint1` .. `joint6` |
| Mount joint | `arm_mounting_joint` (fixed, child of `base_link`) |
| Mount position (`xyz`, relative to `base_link`) | `0.25 0 0.325` |
| End-effector frame | `tool_link` (fixed `tcp` joint off `link6`) |
| Source | ported from the original ROS1 `melfa_description` xacro |

**Known limitation**: the gripper mesh was not available when the arm was ported, so `tool_link`
is currently a bare frame with no visual/collision geometry.

---

## License

MIT — except `rv3sdb_moveit_config`, which retains the BSD license of the original
ROS1 `rv3sdb_moveit_config` package it was ported from.

---
