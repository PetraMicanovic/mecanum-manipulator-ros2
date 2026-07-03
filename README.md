# mecanum-manipulator-ros2
ROS2 simulation of a mecanum AGV platform and an RV-3SDB robotic arm, driven from a single ROS2 application.

---

## Prerequisites
- Ubuntu 24.04
- ROS2 Jazzy
- [Webots R2025a](https://cyberbotics.com)
- [webots_ros2](https://github.com/cyberbotics/webots_ros2)

---

## Quick Start
```bash
# Clone the repository
git clone https://github.com/PetraMicanovic/mecanum-manipulator-ros2.git 
cd mecanum-manipulator-ros2

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
sudo apt install python3-scipy

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

## Project Structure

Standard ROS2 `ament_python` workspace — every package is a direct subfolder of `src/`:

```
src/
├── mobile_platform_sim/          Webots simulation of the mecanum platform
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