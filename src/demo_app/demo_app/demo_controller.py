"""

ROS2 node for controlling the mecanum-wheeled mobile platform in three modes, selected via the ROS2 parameter 'mode' at launch time.

Modes:
    keyboard
        Single-key WASD-style control using raw terminal input. The platform moves while a recognised key is held and stops on any unrecognised key.
    sequence
        The platform executes a predefined sequence of movements autonomously.
        Translational steps are specified in metres, rotational steps in degrees.
    terminal
        The user types movement commands into the terminal. Each command runs for a given duration, then the platform stops and waits for the next input.

Usage:
    ros2 run demo_app demo_controller
    ros2 run demo_app demo_controller --ros-args -p mode:=keyboard
    ros2 run demo_app demo_controller --ros-args -p mode:=sequence

ROS2 topics
Published:
    /cmd_vel: geometry_msgs/Twist
        Velocity commands for the mecanum platform.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time
import sys
import tty
import termios
import math

# Default velocities
LINEAR_VELOCITY = 0.2  # [m/s]
ANGULAR_VELOCITY = 0.5  # [rad/s]

# Key -> (vx, vy, wz)
KEYBOARD_BINDINGS = {
    "w": ( LINEAR_VELOCITY,  0.0,               0.0),   # forward
    "s": (-LINEAR_VELOCITY,  0.0,               0.0),   # back
    "a": ( 0.0,              LINEAR_VELOCITY,   0.0),   # strafe left
    "d": ( 0.0,             -LINEAR_VELOCITY,   0.0),   # strafe right
    "q": ( 0.0,              0.0,               ANGULAR_VELOCITY),   # rotate left
    "e": ( 0.0,              0.0,              -ANGULAR_VELOCITY),   # rotate right
}

# Terminal mode command map: name -> (vx, vy, wz)
COMMANDS = {
    "forward":  ( LINEAR_VELOCITY,  0.0,               0.0),
    "back":     (-LINEAR_VELOCITY,  0.0,               0.0),
    "left":     ( 0.0,              LINEAR_VELOCITY,   0.0),
    "right":    ( 0.0,             -LINEAR_VELOCITY,   0.0),
    "rotate_l": ( 0.0,              0.0,               ANGULAR_VELOCITY),
    "rotate_r": ( 0.0,              0.0,              -ANGULAR_VELOCITY),
    "stop":     ( 0.0,              0.0,               0.0),
}

KEYBOARD_HELP = """
Keyboard controls:
  w forward
  s backward
  a strafe left  (mecanum only)
  d strafe right (mecanum only)
  q rotate left
  e rotate right
  (any other key) stop
  Ctrl+C quit
"""

# Predefined autonomous sequence.
# Translational steps: (type, vx, vy, wz, distance_m)  — distance in metres
# Rotational steps:    (type, vx, vy, wz, angle_deg)   — angle in degrees
# type: 'linear' or 'angular'
PLATFORM_SEQUENCE = [
    ("linear", LINEAR_VELOCITY, 0.0, 0.0, 1.0),  # forward 1 m
    ("linear", 0.0, LINEAR_VELOCITY, 0.0, 0.5),  # strafe right 0.5 m
    ("angular", 0.0, 0.0, ANGULAR_VELOCITY, 90.0),  # rotate left 90 deg
    ("linear", 0.0, 0.0, 0.0, 0.0),  # stop
]


class DemoController(Node):
    """
    ROS2 node for mecanum platform control in keyboard or sequence mode.
    The active mode is selected via the ROS2 parameter 'mode' at launch time.
    Both modes publish geometry_msgs/Twist on /cmd_vel.
    """

    def __init__(self):
        """
        Initialise the node, declare the mode parameter and create the publisher.
        """
        super().__init__("demo_controller")

        # Declare ROS2 parameter for mode selection
        self.declare_parameter("mode", "keyboard")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value

        # Publisher: send velocity commands to MecanumRobotDriver
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info(f"Demo controller started in [{self.mode}] mode.")

    def publish_twist(self, vx, vy, wz):
        """Publish a single Twist message.

        Args:
            vx (float): Forward/backward velocity [m/s].
            vy (float): Lateral (strafe) velocity [m/s].
            wz (float): Yaw rate [rad/s].
        """
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        """
        Publish a zero Twist to stop the platform.
        """
        self.publish_twist(0.0, 0.0, 0.0)

    def run_keyboard(self):
        """
        Single-key WASD-style control without requiring Enter. Reads one keypress at a time from stdin using raw terminal mode. The platform moves on
        a recognised key and stops on any other key. Exits on Ctrl+C.
        """
        print(KEYBOARD_HELP)
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while rclpy.ok():
                key = sys.stdin.read(1)

                # Ctrl+C
                if key == "\x03":
                    break

                if key in KEYBOARD_BINDINGS:
                    vx, vy, wz = KEYBOARD_BINDINGS[key]
                    self.publish_twist(vx, vy, wz)
                else:
                    self.stop()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.stop()

    def run_sequence(self):
        """
        Execute the predefined PLATFORM_SEQUENCE autonomously.
        Translational steps use distance [m], rotational steps use angle [deg].
        Duration is computed automatically from the distance/angle and the
        corresponding default velocity.
        """
        self.get_logger().info("Running predefined sequence...")
        rate_hz = 10

        for step in PLATFORM_SEQUENCE:
            if not rclpy.ok():
                break

            step_type, vx, vy, wz, value = step

            # Compute duration from distance or angle
            if step_type == "linear":
                # value is distance [m]; speed is the magnitude of (vx, vy)
                speed = math.sqrt(vx**2 + vy**2)
                duration = value / speed if speed > 0 else 0.0
                self.get_logger().info(f"Moving {value} m  (vx={vx}, vy={vy}, wz={wz})")
            elif step_type == "angular":
                # value is angle [deg]; convert to rad then divide by angular velocity
                angle_rad = math.radians(value)
                duration = angle_rad / abs(wz) if wz != 0 else 0.0
                self.get_logger().info(f"Rotating {value} deg  (wz={wz})")
            else:
                duration = 0.0

            steps = int(duration * rate_hz)
            for _ in range(steps):
                self.publish_twist(vx, vy, wz)
                time.sleep(1.0 / rate_hz)
            self.stop()

        self.get_logger().info("Sequence complete.")


def main(args=None):
    """
    Entry point for the demo_controller node. Initialises rclpy, spins the node in a background thread and runs keyboard control in the main thread.
    """
    rclpy.init(args=args)
    node = DemoController()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if node.mode == "keyboard":
            node.run_keyboard()
        elif node.mode == "sequence":
            node.run_sequence()
        else:
            node.get_logger().error(
                f"Unknown mode: {node.mode}. Use keyboard or sequence."
            )
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
