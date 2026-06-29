"""
ROS2 node for keyboard control of the mecanum-wheeled mobile platform. Run keyboard teleoperation using raw terminal input. The platform moves while a 
recognised key is held and stops on any unrecognised key. No Enter required.

Keyboard controls:
    w  forward
    s  backward
    a  strafe left  (mecanum only)
    d  strafe right (mecanum only)
    q  rotate left
    e  rotate right
    (any other key) stop
    Ctrl+C quit

ROS2 topics
Published:
    /cmd_vel: geometry_msgs/Twist
        Velocity commands for the mecanum platform.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import sys
import tty
import termios

# Default velocities
LINEAR_VELOCITY = 0.2  # [m/s]
ANGULAR_VELOCITY = 0.5  # [rad/s]

# Key -> (vx, vy, wz)
KEYBOARD_BINDINGS = {
    "w": (LINEAR_VELOCITY, 0.0, 0.0), # forward
    "s": (-LINEAR_VELOCITY, 0.0, 0.0), # back
    "a": (0.0, LINEAR_VELOCITY, 0.0), # strafe left
    "d": (0.0, -LINEAR_VELOCITY, 0.0), # strafe right
    "q": (0.0, 0.0, ANGULAR_VELOCITY), # rotate left
    "e": (0.0, 0.0, -ANGULAR_VELOCITY), # rotate right
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


class DemoController(Node):
    """ROS2 node for single-key keyboard control of the mecanum platform.

    Reads one keypress at a time from stdin using raw terminal mode. The platform moves on a recognised key and stops on any other key.
    """

    def __init__(self):
        """
        Initialise the node and /cmd_vel publisher.
        """
        super().__init__("demo_controller")

        # Publisher: send velocity commands to MecanumRobotDriver
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Demo controller started. Press Ctrl+C to quit.")

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


def main(args=None):
    """
    Entry point for the demo_controller node. Initialises rclpy, spins the node in a background thread, and runs keyboard control in the main thread.

    Args:
        args: Command-line arguments passed to rclpy.init().
    """
    rclpy.init(args=args)
    node = DemoController()

    # Spin ROS2 in a background thread so keyboard reading does not block callbacks
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_keyboard()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
