"""
ROS2 node for real-time keyboard jogging of the rv3sdb arm via moveit_servo.

Unlike mecanum_platform_controller.py, which drives the platform directly, this node publishes 
Cartesian velocity commands to moveit_servo. The servo node converts them into joint trajectories
in real time using inverse kinematics while respecting joint limits and collision checking.

Two jogging modes are supported, toggled at runtime with the 'm' key:
  - JOINT mode: drives individual joints directly via JointJog messages. No IK/Jacobian is
    involved, so there is no singularity checking -- useful for moving a joint (e.g. joint5)
    away from a singular configuration (like 0) before switching to Cartesian mode.
  - CARTESIAN mode: drives the end effector in task space via TwistStamped messages, converted
    to joint motion by moveit_servo's IK solver.

ROS2 topics
Published:
    /servo_node/delta_twist_cmds: geometry_msgs/TwistStamped
        Cartesian velocity commands for moveit_servo, expressed in the `base_link` frame
        (matches servo_params.yaml's robot_link_command_frame). Used in CARTESIAN mode.
    /servo_node/delta_joint_cmds: control_msgs/JointJog
        Per-joint velocity commands for moveit_servo. Used in JOINT mode.

Usage:
    ros2 run mecanum_manipulator_control arm_servo_keyboard
"""

import sys
import termios
import tty

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node

# Default Cartesian jog velocities
# command_in_type in servo_params.yaml is "unitless", so these values scale the
# configured linear and rotational limits instead of representing absolute velocities.
LINEAR_VELOCITY = 0.3  # fraction of scale.linear
ANGULAR_VELOCITY = 0.5  # fraction of scale.rotational

# Default joint jog velocity (fraction of scale.joint, same "unitless" convention as above).
JOINT_VELOCITY = 0.5

# Joint names in the order the driver/MoveIt group expects them
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

MODE_CARTESIAN = "cartesian"
MODE_JOINT = "joint"

# Key -> (vx, vy, vz, wx, wy, wz), all in the base_link frame. Used in CARTESIAN mode.
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

# Key -> (joint_index, sign). Used in JOINT mode: number keys jog the joint positively,
# the shifted symbol above each number jogs it negatively.
JOINT_KEYBOARD_BINDINGS = {
    "1": (0, 1.0), "!": (0, -1.0),
    "2": (1, 1.0), "@": (1, -1.0),
    "3": (2, 1.0), "#": (2, -1.0),
    "4": (3, 1.0), "$": (3, -1.0),
    "5": (4, 1.0), "%": (4, -1.0),
    "6": (5, 1.0), "^": (5, -1.0),
}

KEYBOARD_HELP = """
Arm jogging controls -- press 'm' to toggle between CARTESIAN and JOINT mode.

CARTESIAN mode (Cartesian velocity of tool_link, expressed in base_link, uses IK):
  w/s   +X / -X   (forward / back)
  a/d   +Y / -Y   (left / right)
  r/f   +Z / -Z   (up / down)
  q/e   yaw left / right
  u/o   roll +/-
  j/l   pitch +/-

JOINT mode (direct per-joint velocity, no IK, no singularity checking):
  1/!   joint1 +/-
  2/@   joint2 +/-
  3/#   joint3 +/-
  4/$   joint4 +/-
  5/%   joint5 +/-
  6/^   joint6 +/-

  m       toggle CARTESIAN <-> JOINT mode
  (any other key) stop
  Ctrl+C quit
"""


class ArmServoKeyboard(Node):
    """
    ROS2 node that streams jog commands to moveit_servo based on raw single-key keyboard input,
    following the same input-handling pattern as mecanum_platform_controller.py's keyboard mode.

    Supports two modes: JOINT (direct joint velocities, no IK) and CARTESIAN (task-space velocities 
    resolved to joint motion via moveit_servo's IK). The node starts in JOINT mode so a joint that
    would otherwise put the arm in a singular configuration (e.g. joint5 at 0) can be moved away
    from zero before switching to Cartesian jogging.
    """

    def __init__(self):
        """
        Initialise the node, create the publishers for both jog topics and switch moveit_servo
        into JOINT command mode to match the node's starting mode.
        """
        super().__init__("arm_servo_keyboard")

        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )
        self.joint_pub = self.create_publisher(
            JointJog, "/servo_node/delta_joint_cmds", 10
        )

        self.mode = MODE_JOINT
        self._switch_command_type(ServoCommandType.Request.JOINT_JOG)

        self.get_logger().info(
            f"Arm servo keyboard jogging node started in {self.mode.upper()} mode."
        )

    def _switch_command_type(self, command_type):
        """
        Tell moveit_servo's servo_node which command message type to accept.

        servo_node ignores incoming commands until its command type is set via
        /servo_node/switch_command_type.

        Args:
            command_type: int
                One of ServoCommandType.Request.JOINT_JOG or ServoCommandType.Request.TWIST.
        """
        client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "/servo_node/switch_command_type service not available; "
                "is servo_node running? Arm will not respond to jog commands."
            )
            return

        request = ServoCommandType.Request()
        request.command_type = command_type
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done() and future.result() is not None and future.result().success:
            self.get_logger().info(f"servo_node switched to command type {command_type}.")
        else:
            self.get_logger().error(
                f"Failed to switch servo_node to command type {command_type}; "
                "arm jogging will not work."
            )

    def toggle_mode(self):
        """
        Toggle between JOINT and CARTESIAN jogging mode and tell servo_node to switch its
        expected command message type to match.
        """
        self.stop()
        if self.mode == MODE_CARTESIAN:
            self.mode = MODE_JOINT
            self._switch_command_type(ServoCommandType.Request.JOINT_JOG)
        else:
            self.mode = MODE_CARTESIAN
            self._switch_command_type(ServoCommandType.Request.TWIST)
        self.get_logger().info(f"Switched to {self.mode.upper()} mode.")

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

    def publish_joint_jog(self, joint_index=None, velocity=0.0):
        """
        Publish a single JointJog command that jogs at most one joint at a time; all other
        joints get zero velocity so servo_node holds them in place.

        Args:
            joint_index: int or None
                Index into JOINT_NAMES of the joint to jog. None publishes an all-zero
                (stop) command.
            velocity: float
                Signed velocity fraction to apply to the selected joint.
        """
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        velocities = [0.0] * len(JOINT_NAMES)
        if joint_index is not None:
            velocities[joint_index] = velocity
        msg.velocities = velocities
        msg.duration = 0.0
        self.joint_pub.publish(msg)

    def stop(self):
        """
        Publish a zero command on whichever topic matches the current mode so moveit_servo
        halts arm motion.
        """
        if self.mode == MODE_CARTESIAN:
            self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            self.publish_joint_jog()

    def run_keyboard(self):
        """
        Single-key jog control without requiring Enter. Reads one keypress at a time from stdin
        using raw terminal mode. Reads one keypress at a time from stdin using raw terminal mode.
        Recognised keys publish jog commands, while any other key stops the arm. Exits on Ctrl+C.
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

                if key == "m":
                    self.toggle_mode()
                elif self.mode == MODE_CARTESIAN and key in KEYBOARD_BINDINGS:
                    vx, vy, vz, wx, wy, wz = KEYBOARD_BINDINGS[key]
                    self.publish_twist(vx, vy, vz, wx, wy, wz)
                elif self.mode == MODE_JOINT and key in JOINT_KEYBOARD_BINDINGS:
                    joint_index, sign = JOINT_KEYBOARD_BINDINGS[key]
                    self.publish_joint_jog(joint_index, sign * JOINT_VELOCITY)
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