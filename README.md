# mecanum-manipulator-ros2

ROS2 simulation of a mecanum AGV platform and an RV-3SDB robotic
arm, driven from a single ROS2 application.

---

## Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy
- Webots + `webots_ros2`

---

## Quick Start

```bash
mkdir -p ~/ros2_ws
git clone https://github.com/PetraMicanovic/mecanum-manipulator-ros2.git ~/mecanum-manipulator-ros2
ln -s ~/mecanum-manipulator-ros2/src ~/ros2_ws/src

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install python3-scipy ros-jazzy-teleop-twist-keyboard

```
