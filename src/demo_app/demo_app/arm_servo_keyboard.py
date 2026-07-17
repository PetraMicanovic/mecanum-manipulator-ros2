"""
ROS2 node for real-time keyboard jogging of the rv3sdb arm via moveit_servo.

Unlike demo_controller.py, which drives the platform directly, this node publishes Cartesian velocity commands
to moveit_servo. The servo node converts them into joint trajectories in real time using
inverse kinematics while respecting joint limits and collision checking.

ROS2 topics
Published:
    /servo_node/delta_twist_cmds: geometry_msgs/TwistStamped
        Cartesian velocity commands for moveit_servo, expressed in the `base_link` frame
        (matches servo_params.yaml's robot_link_command_frame).
"""

import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node

# Default Cartesian jog velocities
# command_in_type in servo_params.yaml is "unitless", so these values scale the
# configured linear and rotational limits instead of representing absolute velocities.
LINEAR_VELOCITY = 0.3  # fraction of scale.linear
ANGULAR_VELOCITY = 0.5  # fraction of scale.rotational

# Key -> (vx, vy, vz, wx, wy, wz), all in the base_link frame
KEYBOARD_BINDINGS = {
    "w": (LINEAR_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0),  # +X
    "s": (-LINEAR_VELOCITY, 0.0, 0.0, 0.0, 0.0, 0.0),  # -X
    "a": (0.0, LINEAR_VELOCITY, 0.0, 0.0, 0.0, 0.0),  # +Y
    "d": (0.0, -LINEAR_VELOCITY, 0.0, 0.0, 0.0, 0.0),  # -Y
    "r": (0.0, 0.0, LINEAR_VELOCITY, 0.0, 0.0, 0.0),  # +Z (up)
    "f": (0.0, 0.0, -LINEAR_VELOCITY, 0.0, 0.0, 0.0),  # -Z (down)
    "q": (0.0, 0.0, 0.0, 0.0, 0.0, ANGULAR_VELOCITY),  # yaw left
    "e": (0.0, 0.0, 0.0, 0.0, 0.0, -ANGULAR_VELOCITY),  # yaw right
    "u": (0.0, 0.0, 0.0, ANGULAR_VELOCITY, 0.0, 0.0),  # roll +
    "o": (0.0, 0.0, 0.0, -ANGULAR_VELOCITY, 0.0, 0.0),  # roll -
    "j": (0.0, 0.0, 0.0, 0.0, ANGULAR_VELOCITY, 0.0),  # pitch +
    "l": (0.0, 0.0, 0.0, 0.0, -ANGULAR_VELOCITY, 0.0),  # pitch -
}

KEYBOARD_HELP = """
Arm jogging controls (Cartesian velocity of tool_link, expressed in base_link):
  w/s   +X / -X   (forward / back)
  a/d   +Y / -Y   (left / right)
  r/f   +Z / -Z   (up / down)
  q/e   yaw left / right
  u/o   roll +/-
  j/l   pitch +/-
  (any other key) stop
  Ctrl+C quit
"""


class ArmServoKeyboard(Node):
    """
    ROS2 node that streams TwistStamped Cartesian jog commands to moveit_servo based on raw
    single-key keyboard input, following the same input-handling pattern as demo_controller.py's
    keyboard mode.
    """

    def __init__(self):
        """
        Initialise the node and create the publisher to moveit_servo's Cartesian command topic.
        """
        super().__init__("arm_servo_keyboard")

        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )

        self._set_twist_command_type()

        self.get_logger().info("Arm servo keyboard jogging node started.")

    def _set_twist_command_type(self):
        """
        Tell moveit_servo's servo_node to accept Twist (Cartesian velocity) commands.

        servo_node ignores incoming commands until its command is type is set via /servo_node/switch_command_type.
        """
        client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "/servo_node/switch_command_type service not available; "
                "is servo_node running? Arm will not respond to jog commands."
            )
            return

        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.TWIST
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done() and future.result() is not None and future.result().success:
            self.get_logger().info("servo_node switched to TWIST command input.")
        else:
            self.get_logger().error(
                "Failed to switch servo_node to TWIST command input; arm jogging "
                "will not work."
            )

    def publish_twist(self, vx, vy, vz, wx, wy, wz):
        """
        Publish a single stamped Cartesian velocity command.

        Args:
            vx, vy, vz: float
                Linear velocity components in the base_link frame.
            wx, wy, wz: float
                Angular velocity components in the base_link frame.
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz
        self.twist_pub.publish(msg)

    def stop(self):
        """
        Publish a zero TwistStamped so moveit_servo halts arm motion.
        """
        self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def run_keyboard(self):
        """
        Single-key jog control without requiring Enter. Reads one keypress at a time from stdin
        using raw terminal mode. The arm jogs while a recognised key is held and stops on any
        other key. Exits on Ctrl+C.
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
                    vx, vy, vz, wx, wy, wz = KEYBOARD_BINDINGS[key]
                    self.publish_twist(vx, vy, vz, wx, wy, wz)
                else:
                    self.stop()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.stop()


def main(args=None):
    """
    Entry point for the arm_servo_keyboard node. Initialises rclpy and runs the keyboard jog
    loop directly (no background spin thread needed here, since this node only publishes and
    never needs to process incoming callbacks).
    """
    rclpy.init(args=args)
    node = ArmServoKeyboard()

    try:
        node.run_keyboard()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
