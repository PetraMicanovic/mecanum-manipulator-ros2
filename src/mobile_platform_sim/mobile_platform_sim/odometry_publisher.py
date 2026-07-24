"""
ROS2 node that estimates the pose and velocity of the mecanum-wheeled mobile platform by integrating wheel encoder readings.

This module implements the OdometryPublisher class, a standard ROS2 node that subscribes to wheel encoder positions published
by MecanumRobotDriver, applies mecanum forward kinematics to compute body velocity, integrates it over time to estimate the
robot pose and publishes the result as a nav_msgs/Odometry message.

Usage:
    ros2 run mobile_platform_sim odometry_publisher

ROS2 topics
Subscribed:
    /wheels_encoders: sensor_msgs/JointState
        Cumulative angular position[rad] of each wheel published by MecanumRobotDriver at every simulation step.

Published:
    /odom: nav_msgs/Odometry
        Estimated robot pose (x, y, yaw) and velocity (Vx, Vy, wz) in the odom frame, published at 100 Hz.
"""

import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

# Mobile platform geometry
# Half the wheelbase along the X axis (front-to-back half-distance) [m]
L_X = 0.2505
# Half the track width along the Y axis (left-to-right half-distance) [m]
L_Y = 0.26536
# Wheel radius [m]
WHEEL_RADIUS = 0.203 / 2  # = 0.1015 m


class OdometryPublisher(Node):
    def __init__(self):
        """
        Initialise the node, subscriber, publisher, TF broadcaster and timer.
        """
        super().__init__("odometry_publisher")

        # Robot pose state vector [x(m), y(m), theta(rad)]
        self.q = np.zeros((3, 1))

        self.joint_states = JointState()
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0]

        self.odometry = Odometry()

        # Subscriber: receive wheel encoder positions from MecanumRobotDriver
        self.create_subscription(
            JointState, "wheels_encoders", self.joint_states_callback, 1
        )

        self.previous_wheel_positions = np.zeros(4)

        # Publisher: broadcast odometry estimate to the navigation stack
        self.odometry_publisher = self.create_publisher(Odometry, "odom", 1)

        # TF broadcaster: publishes the odom -> base_footprint transform
        self.tf_broadcaster = TransformBroadcaster(self)

        self.dt = 0.01
        self.create_timer(self.dt, self.odometry_callback)

    def joint_states_callback(self, joint_states_msg):
        """
        Store the latest encoder message for use in the next odometry update.
        Called automatically by rclpy whenever a new JointState message arrives on /wheels_encoders.

        Args:
            joint_states_msg: sensor_msgs.msg.JointState
                Message containing the cumulative angular position [rad] of all four wheels in the order [front-left, 
                front-right, back-left, back-right].
        """
        self.joint_states = joint_states_msg

    def compute_body_velocity(self):
        """
        Compute body velocity from wheel encoder deltas using forward kinematics.

        Estimates each wheel's angular velocity by finite difference between the current and previous encoder positions
        divided by the timer period dt. Applies the mecanum forward kinematics matrix J to map the four wheel velocities
        to the three-dimensional body velocity [Vx, Vy, wz].
        Mecanum forward kinematics matrix::
            J = (R/4) * [[ 1,  1,  1,  1],
                         [-1,  1,  1, -1],
                         [-1/d, 1/d, -1/d, 1/d]]
        where d = L_X + L_Y is the effective rotation moment arm [m].

        Returns:
            numpy.ndarray:
                Shape (3, 1) body velocity vector [Vx (m/s), Vy (m/s), wz (rad/s)]^T.
        """
        vel_motors = (
            np.array(self.joint_states.position) - self.previous_wheel_positions
        ) / self.dt

        # Update previous positions for the next call
        self.previous_wheel_positions = self.joint_states.position

        d = L_X + L_Y

        J = (WHEEL_RADIUS / 4) * np.array(
            [[1, 1, 1, 1], [-1, 1, 1, -1], [-1 / d, 1 / d, -1 / d, 1 / d]]
        )

        q_dot = np.dot(J, vel_motors.reshape((4, 1)))

        return q_dot

    def odometry_callback(self):
        """
        Integrate body velocity to update pose and publish odometry.

        Called at 100 Hz by the ROS2 timer. Performs the following steps:
        1. Calls get_robot_speed to obtain the current body velocity.
        2. Integrates velocity over dt using a rotation matrix to account for the current heading (theta), updating the
          pose state vector q.
        3. Broadcasts the odom -> base_footprint TF transform (currently disabled — uncomment 
        self.tf_broadcaster.sendTransform(t) to enable).
        4. Builds and publishes a nav_msgs/Odometry message with pose, twist and a fixed diagonal covariance.
        """
        q_dot = self.compute_body_velocity()

        # Pose increment in the body frame
        dq = q_dot * self.dt

        # Rotation matrix from body frame to odom frame (rotate by current yaw)
        rot_m = np.array(
            [
                [np.cos(self.q[2, 0]), -np.sin(self.q[2, 0]), 0],
                [np.sin(self.q[2, 0]), np.cos(self.q[2, 0]), 0],
                [0, 0, 1],
            ]
        )

        # Integrate pose in the odom frame
        self.q = self.q + np.dot(rot_m, dq)

        # TF transform: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = self.q[0, 0]
        t.transform.translation.y = self.q[1, 0]
        t.transform.translation.z = 0.0

        r = R.from_euler("xyz", [0, 0, self.q[2, 0]])
        quat = r.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # self.tf_broadcaster.sendTransform(t)

        # Odometry message
        self.odometry.header.stamp = self.get_clock().now().to_msg()
        self.odometry.header.frame_id = "odom"
        self.odometry.child_frame_id = "base_footprint"

        # Pose
        self.odometry.pose.pose.position.x = self.q[0, 0]
        self.odometry.pose.pose.position.y = self.q[1, 0]
        self.odometry.pose.pose.position.z = 0.0

        self.odometry.pose.pose.orientation.x = quat[0]
        self.odometry.pose.pose.orientation.y = quat[1]
        self.odometry.pose.pose.orientation.z = quat[2]
        self.odometry.pose.pose.orientation.w = quat[3]

        # Twist (body-frame velocity)
        self.odometry.twist.twist.linear.x = q_dot[0, 0]
        self.odometry.twist.twist.linear.y = q_dot[1, 0]
        self.odometry.twist.twist.angular.z = q_dot[2, 0]

        # Diagonal covariance — small fixed value; tune after real-world testing
        self.odometry.pose.covariance[0] = 0.01  # x
        self.odometry.pose.covariance[7] = 0.01  # y
        self.odometry.pose.covariance[14] = 0.01  # z
        self.odometry.pose.covariance[21] = 0.01  # roll
        self.odometry.pose.covariance[28] = 0.01  # pitch
        self.odometry.pose.covariance[-1] = 0.01  # yaw

        self.odometry_publisher.publish(self.odometry)


def main(args=None):
    """
    Entry point for the odometry_publisher node.

    Initialises rclpy, creates and spins the OdometryPublisher node until shutdown and calls rclpy.shutdown().

    Args:
        args:
          Command-line arguments passed to rclpy.init(). Defaults to None, in which case sys.argv is used.
    """
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)

    odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
