# mecanum-manipulator-ros2
ROS2 simulation of a mecanum AGV platform and an RV-3SDB robotic arm, driven from a single ROS2 application.

---

## Prerequisites
- Ubuntu 24.04
- ROS2 Jazzy
- [Webots R2025a](https://cyberbotics.com)
- [webots_ros2](https://github.com/cyberbotics/webots_ros2)
- [webots_ros2_control](https://github.com/cyberbotics/webots_ros2) (arm control)
- `ros-jazzy-joint-state-publisher-gui` (manual arm joint sliders)

---

## Quick Start
```bash
# Clone the repository
git clone https://github.com/PetraMicanovic/mecanum-manipulator-ros2.git 
cd mecanum-manipulator-ros2

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
sudo apt install python3-scipy
sudo apt install ros-jazzy-joint-state-publisher-gui

# Build
colcon build --symlink-install
source install/setup.bash

# Launch the simulation
ros2 launch mobile_platform_sim mobile_platform_launch.py
```

Once the simulation is running, open a second terminal and start the demo controller:

```bash
source install/setup.bash

# Keyboard control (default)
ros2 run demo_app demo_controller

# Predefined sequence
ros2 run demo_app demo_controller --ros-args -p mode:=sequence

# Terminal control
ros2 run demo_app demo_controller --ros-args -p mode:=terminal
```

---

## Controlling the Platform

The `demo_app` package provides three ways to drive the platform.

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

**Sequence mode** — the platform runs through a fixed choreography automatically and exits when done. Translational steps are specified in metres, rotational steps in degrees — duration is computed automatically from the distance and the default velocity. Edit `PLATFORM_SEQUENCE` in `demo_app/demo_controller.py` to change the steps.

**Terminal mode** — type a command followed by a distance in metres or an angle in degrees. The platform executes the command and waits for the next input:

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

## ROS2 Interface

The platform is controlled by publishing `geometry_msgs/Twist` on `/cmd_vel`:

| Field | Effect |
|---|---|
| `linear.x` | Forward / backward |
| `linear.y` | Left / right (sideways — mecanum only) |
| `angular.z` | Rotation |

Available topics:

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/cmd_vel` | sub | `geometry_msgs/Twist` | Velocity commands |
| `/wheel/odometry` | pub | `nav_msgs/Odometry` | Wheel encoder odometry |
| `/livox/lidar_front` | pub | `sensor_msgs/PointCloud2` | Front Livox MID-70 |
| `/livox/lidar_back` | pub | `sensor_msgs/PointCloud2` | Rear Livox MID-70 |
| `/joint_states` | pub | `sensor_msgs/JointState` | State of all 4 wheels |

---

## Platform + Arm (rv3sdb) — geometry preview

The `mobile_platform_with_arm` package mounts an rv3sdb (Mitsubishi Melfa RV-3SDB, 6 DOF)
manipulator on top of the mecanum platform, as a separate package from `mobile_platform_sim`
(the platform-only package above is untouched). This is currently a **geometry-only** preview.


```bash
ros2 launch mobile_platform_with_arm mobile_platform_with_arm_launch.py
```

This brings up:
- Webots with the combined platform+arm world
- the mecanum platform, fully drivable exactly as above (`/cmd_vel`, wheel odometry, both lidars)
- `robot_state_publisher`, building TF for the whole chassis+arm tree
- `joint_state_publisher_gui`, with sliders for the arm's 6 joints, so the mount position, mesh
  alignment, and joint limits can be checked visually

Note: moving the sliders updates TF/RViz only — the arm does **not** move inside the Webots
render yet, since there's no controller commanding the physical joints. That will change once
ros2_control + MoveIt2 are added (see Roadmap).

### Arm Geometry

| Parameter | Value |
|---|---|
| DOF | 6 |
| Joint names | `joint1` .. `joint6` |
| Mount joint | `arm_mounting_joint` (fixed, child of `base_link`) |
| Mount position (`xyz`, relative to `base_link`) | `0.25 0 0.325` |
| End-effector frame | `tool_link` (fixed `tcp` joint off `link6`) |
| Source | ported from the original ROS1 `melfa_description` xacro |

**Known limitation**: the gripper mesh was not available when the arm was
ported, so `tool_link` is currently a bare frame with no visual/collision geometry.

---

## Project Structure

Standard ROS2 `ament_python`/`ament_cmake` workspace — every package is a direct subfolder of `src/`:

```
src/
├── mobile_platform_sim/            Webots simulation of the mecanum platform (platform only)
│   ├── launch/
│   │   └── mobile_platform_launch.py
│   ├── meshes/
│   │   ├── chassis.stl
│   │   ├── livox_mount.stl
│   │   └── IntelRealSenseT265.STL   # mesh available, not yet in simulation
│   ├── mobile_platform_sim/
│   │   ├── mecanum_robot_driver.py
│   │   └── odometry_publisher.py
│   ├── resource/
│   │   └── mobile_platform.urdf
│   └── worlds/
│       └── platform.wbt
│
├── mobile_platform_with_arm/        Webots simulation of the platform WITH the rv3sdb arm
│   ├── launch/
│   │   ├── mobile_platform_with_arm_launch.py   # geometry-only, no MoveIt2 yet
│   ├── meshes/
│   │   ├── chassis.stl 
│   │   ├── livox_mount.stl               
│   │   └── arm/                     # rv3sdb meshes
│   ├── resource/
│   │   └── mobile_platform_with_arm.urdf
│   └── worlds/
│       └── mobile_platform_with_arm.wbt
│
│
└── demo_app/                         Platform control application
    └── demo_app/
        └── demo_controller.py        Keyboard, sequence, and terminal control modes
```

---

## Platform Geometry

All dimensions taken from the original ROS1 project.

| Parameter | Value |
|---|---|
| L_x (half-wheelbase, longitudinal) | 0.2505 m |
| L_y (half-wheelbase, lateral) | 0.26536 m |
| Wheel radius | 0.1015 m |
| Wheel width | 0.077 m |

---