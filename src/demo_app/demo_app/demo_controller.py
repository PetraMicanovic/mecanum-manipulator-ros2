"""
ROS2 node for controlling the mecanum-wheeled mobile platform in three modes, selected via the ROS2 parameter 'mode' at launch time.

Modes:
    keyboard
        Single-key WASD-style control using raw terminal input. The platform moves while a recognised key is held and stops on any unrecognised key.
    sequence
        The platform executes a predefined sequence of movements autonomously. Translational steps are specified in metres, rotational steps in degrees.
    terminal
        The user types movement commands into the terminal. Translational commands take a distance in metres, rotational commands take an angle in degrees.

Usage:
    ros2 run demo_app demo_controller
    ros2 run demo_app demo_controller --ros-args -p mode:=keyboard
    ros2 run demo_app demo_controller --ros-args -p mode:=sequence
    ros2 run demo_app demo_controller --ros-args -p mode:=terminal

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
    "w": (LINEAR_VELOCITY, 0.0, 0.0),  # forward
    "s": (-LINEAR_VELOCITY, 0.0, 0.0),  # back
    "a": (0.0, LINEAR_VELOCITY, 0.0),  # strafe left
    "d": (0.0, -LINEAR_VELOCITY, 0.0),  # strafe right
    "q": (0.0, 0.0, ANGULAR_VELOCITY),  # rotate left
    "e": (0.0, 0.0, -ANGULAR_VELOCITY),  # rotate right
}

# Terminal mode command map: name -> (type, vx, vy, wz)
# type: 'linear' (value in metres) or 'angular' (value in degrees)
COMMANDS = {
    "forward": ("linear", LINEAR_VELOCITY, 0.0, 0.0),
    "back": ("linear", -LINEAR_VELOCITY, 0.0, 0.0),
    "left": ("linear", 0.0, LINEAR_VELOCITY, 0.0),
    "right": ("linear", 0.0, -LINEAR_VELOCITY, 0.0),
    "rotate_l": ("angular", 0.0, 0.0, ANGULAR_VELOCITY),
    "rotate_r": ("angular", 0.0, 0.0, -ANGULAR_VELOCITY),
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

TERMINAL_HELP = """
Terminal mode commands:
  forward  <m>   move forward   (metres)
  back     <m>   move backward  (metres)
  left     <m>   strafe left    (metres, mecanum only)
  right    <m>   strafe right   (metres, mecanum only)
  rotate_l <deg> rotate left    (degrees)
  rotate_r <deg> rotate right   (degrees)
  stop           stop immediately
  quit           exit
"""

# Predefined autonomous sequence.
# Translational steps: ('linear',  vx, vy, wz, distance_m)
# Rotational steps: ('angular', vx, vy, wz, angle_deg)
PLATFORM_SEQUENCE = [
    ("linear", LINEAR_VELOCITY, 0.0, 0.0, 1.0),  # forward 1 m
    ("linear", 0.0, LINEAR_VELOCITY, 0.0, 0.5),  # strafe right 0.5 m
    ("angular", 0.0, 0.0, ANGULAR_VELOCITY, 90.0),  # rotate left 90 deg
    ("linear", 0.0, 0.0, 0.0, 0.0),  # stop
]


class DemoController(Node):
    """
    ROS2 node for mecanum platform control in keyboard, sequence or terminal mode. The active mode is selected via the ROS2 parameter 'mode' at launch
    time. All modes publish geometry_msgs/Twist on /cmd_vel.
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
        """
        Publish a single Twist message.

        Args:
            vx: float
                Forward/backward velocity [m/s].
            vy: float
                Lateral (strafe) velocity [m/s].
            wz: float)
                Yaw rate [rad/s].
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

    def _duration_from_value(self, cmd_type, vx, vy, wz, value):
        """
        Compute duration from distance[m] or angle[deg].

        Args:
            cmd_type: str
                'linear' or 'angular'.
            vx, vy, wz: float
                Velocity components.
            value: float
                Distance [m] or angle [deg].

        Returns:
            float:
                Duration in seconds.
        """
        if cmd_type == "linear":
            speed = math.sqrt(vx**2 + vy**2)
            if speed > 0:
                return value / speed
            else:
                return 0.0
        elif cmd_type == "angular":
            angle_rad = math.radians(value)
            if wz != 0:
                return angle_rad / abs(wz)
            else:
                return 0.0
        return 0.0

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

    def run_sequence(self, sequence=None):
        """
        Execute a sequence of movement steps autonomously. Defaults to the module-level
        PLATFORM_SEQUENCE if no custom sequence is given. Translational steps use distance [m], 
        rotational steps use angle [deg].

        Args:
        sequence: list[tuple] or None
            List of movement steps to execute, where each step is a tuple:
            (cmd_type, vx, vy, wz, value)
                cmd_type: str
                    'linear' or 'angular'.
                vx, vy: float
                    Linear velocity components [m/s] (used when cmd_type == 'linear').
                wz: float
                    Angular velocity [rad/s] (used when cmd_type == 'angular').
                value: float
                    Distance [m] for 'linear' steps or angle [deg] for 'angular' steps.
            If None, the module-level PLATFORM_SEQUENCE is used instead.
        
        Returns:
            None
        """
        if sequence is not None:
            sequence = sequence
        else:
            sequence = PLATFORM_SEQUENCE
        self.get_logger().info("Running predefined sequence...")
        rate_hz = 10

        for step in sequence:
            if not rclpy.ok():
                break

            cmd_type, vx, vy, wz, value = step
            duration = self._duration_from_value(cmd_type, vx, vy, wz, value)

            if cmd_type == "linear":
                self.get_logger().info(f"Moving {value} m  (vx={vx}, vy={vy}, wz={wz})")
            elif cmd_type == "angular":
                self.get_logger().info(f"Rotating {value} deg  (wz={wz})")

            steps = int(duration * rate_hz)
            for _ in range(steps):
                self.publish_twist(vx, vy, wz)
                time.sleep(1.0 / rate_hz)
            self.stop()

        self.get_logger().info("Sequence complete.")

    def run_terminal(self):
        """
        Block and read movement commands from stdin interactively. Translational commands take a distance in metres, rotational commands take an angle in
        degrees. Duration is computed automatically.
        """
        print(TERMINAL_HELP)
        while rclpy.ok():
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not line:
                continue

            parts = line.split()
            command = parts[0].lower()

            if command == "quit":
                self.get_logger().info("Exiting.")
                break

            if command == "stop":
                self.stop()
                self.get_logger().info("Stopped.")
                continue

            if command not in COMMANDS:
                print(f"Unknown command: {command}")
                print("Available:", ", ".join(COMMANDS.keys()))
                continue

            if len(parts) < 2:
                cmd_type = COMMANDS[command][0]
                if cmd_type == "linear":
                    unit = "metres"
                else:
                    unit = "degrees"
                print(f"Usage: {command} <{unit}>")
                continue

            try:
                value = float(parts[1])
                if value <= 0:
                    raise ValueError
            except ValueError:
                print("Value must be a positive number.")
                continue

            cmd_type, vx, vy, wz = COMMANDS[command]
            duration = self._duration_from_value(cmd_type, vx, vy, wz, value)

            if cmd_type == "linear":
                unit = "m"
            else:
                unit = "deg"
            self.get_logger().info(f"Executing: {command} {value} {unit}")

            rate_hz = 10
            steps = int(duration * rate_hz)
            for _ in range(steps):
                self.publish_twist(vx, vy, wz)
                time.sleep(1.0 / rate_hz)

            self.stop()
            self.get_logger().info("Done. Waiting for next command...")


def main(args=None):
    """
    Entry point for the demo_controller node. Initialises rclpy, spins the node in a background thread and runs the selected control mode in the main
    thread.
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
        elif node.mode == "terminal":
            node.run_terminal()
        else:
            node.get_logger().error(
                f"Unknown mode: {node.mode}. Use keyboard, sequence or terminal."
            )
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
