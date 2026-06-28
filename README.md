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
# Clone into your workspace
git clone https://github.com/PetraMicanovic/mecanum-manipulator-ros2.git 
cd mecanum-manipulator-ros2

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
sudo apt install python3-scipy ros-jazzy-teleop-twist-keyboard

# Build
colcon build --packages-select mobile_platform_sim --symlink-install
source install/setup.bash

# Launch
ros2 launch mobile_platform_sim mobile_platform_launch.py

# Manual control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
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
│      └── mobile_platform_launch.py
│   ├── meshes/     
│      ├── chassis.stl
│      ├── livox_mount.stl
│      └── IntelRealSenseT265.STL   # mesh available, not yet added to simulation
│   ├── mobile_platform_sim/      
│      ├── mecanum_robot_driver.py
│      └── odometry_publisher.py
│   ├── resource/          
│      └── mobile_platform.urdf
└── └── worlds/            
       └── platform.wbt

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