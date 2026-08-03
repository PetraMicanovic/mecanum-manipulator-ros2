"""
Webots ROS2 plugin (controller) for the mecanum-wheeled mobile platform.

This module implements the MecanumRobotDriver class, which serves as the bridge between the Webots simulation
and ROS2. This node is responsible for initializing the Webots motors, encoders and lidars. It receives velocity
commands from the /cmd_vel (geometry_msgs/Twist) topic, computes the required angular velocity for each mecanum 
wheel and sends these commands to the motors. In addition, it publishes wheel encoder readings on the /wheels_encoders
(sensor_msgs/JointState) topic, allowing odometry_publisher to estimate the robot's odometry.

ROS2 topics
Subscribed:
    /cmd_vel: geometry_msgs/Twist
        Desired body velocity: linear.x (forward), linear.y (strafe), angular.z (yaw rate).

Published:
    /wheels_encoders: sensor_msgs/JointState
        Cumulative angular position [rad] of each wheel, stamped with the current ROS2 clock time.
"""

import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

# Mobile platform geometry
# Half the wheelbase along the X axis (front-to-back half-distance) [m].
L_X = 0.2505
# Half the track width along the Y axis (left-to-right half-distance) [m].
L_Y = 0.26536
# Wheel radius [m].
WHEEL_RADIUS = 0.203 / 2
WHEEL_LENGTH = 0.077


class MecanumRobotDriver:
    def init(self, webots_node, properties):
        """
        Initialize devices, ROS2 node, publisher, and subscriber.

        Called once by webots_ros2_driver when the controller is loaded. Sets up all Webots devices and creates the 
        ROS2 node that handles velocity commands and joint-state publishing.

        Args:
            webots_node:
                The Webots controller node provided by webots_ros2_driver. Exposes webots_node.robot which is the 
                entry point for all device access.
            properties: dict
                Plugin properties
        """
        # Robot node
        self.__robot = webots_node.robot

        # Configure front and back Livox MID-70 Lidar
        self.__lidar = self.__robot.getDevice("lidar")
        self.__lidar.enable(32)
        self.__lidar.enablePointCloud()

        self.__lidar2 = self.__robot.getDevice("lidar2")
        self.__lidar2.enable(32)
        self.__lidar2.enablePointCloud()

        # Configure wheel encoders
        self.__encoder1 = self.__robot.getDevice("encoder1")
        self.__encoder1.enable(32)

        self.__encoder2 = self.__robot.getDevice("encoder2")
        self.__encoder2.enable(32)

        self.__encoder3 = self.__robot.getDevice("encoder3")
        self.__encoder3.enable(32)

        self.__encoder4 = self.__robot.getDevice("encoder4")
        self.__encoder4.enable(32)

        # Configure motors
        # motor1 = front-left       motor2 = front-right
        # motor3 = back-left        motor4 = back-right
        self.__front_left_motor = self.__robot.getDevice("motor1")
        self.__front_right_motor = self.__robot.getDevice("motor2")
        self.__back_left_motor = self.__robot.getDevice("motor3")
        self.__back_right_motor = self.__robot.getDevice("motor4")

        for motor in (
            self.__front_left_motor,
            self.__front_right_motor,
            self.__back_left_motor,
            self.__back_right_motor,
        ):
            # switches each motor to "velocity control" mode
            motor.setPosition(float("inf"))
            # ensures the robot is stationary at startup
            motor.setVelocity(0)

        self.__target_twist = Twist()

        # ROS2 node configuration
        rclpy.init(args=None)
        self.__node = rclpy.create_node("mecanum_robot_driver")

        # Subscriber: receive velocity commands from a teleop node, navigation stack
        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)
        
        # Publisher: send encoder positions to odometry_publisher
        self.__joint_states_publisher = self.__node.create_publisher(JointState, "wheels_encoders", 1)
    
        self.tf_broadcaster = TransformBroadcaster(self.__node)

    def __cmd_vel_callback(self, twist):
        """
        Store the latest velocity command for use in the next step.

        Args:
            twist: geometry_msgs.msg.Twist
                Desired body velocity
                    - linear.x: forward/backward velocity [m/s]
                    - linear.y: lateral (strafe) velocity [m/s]
                    - angular.z: yaw rate [rad/s]
        """
        self.__target_twist = twist

    def step(self):
        """
        Execute one simulation step: process ROS2 messages, compute and apply wheel velocities, and publish 
        encoder states.

        Called by Webots on every simulation timestep (every 32 ms). The method performs three tasks in order:
            1. Spin ROS2 — process any pending incoming messages so that __cmd_vel_callback can update __target_twist.
            2. Inverse kinematics — convert the desired body twist (Vx, Vy, ω) into four individual wheel 
            angular velocities using the standard mecanum wheel model:
                ω_FL = ( Vx - Vy - (Lx + Ly) * ω ) / R
                ω_FR = ( Vx + Vy + (Lx + Ly) * ω ) / R
                ω_BL = ( Vx + Vy - (Lx + Ly) * ω ) / R
                ω_BR = ( Vx - Vy + (Lx + Ly) * ω ) / R
            where R is the wheel radius and (Lx + Ly) is the effective moment arm for rotation.
            3. Publish JointState — read current encoder values and publish them on /wheels_encoders for the odometry node.
        """
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Mecanum inverse kinematics: body twist -> 4 wheel angular velocities
        V_x = self.__target_twist.linear.x
        V_y = self.__target_twist.linear.y
        thetap = self.__target_twist.angular.z

        front_left_wheel_velocity = (V_x - V_y - (L_X + L_Y) * thetap) / WHEEL_RADIUS
        front_right_wheel_velocity = (V_x + V_y + (L_X + L_Y) * thetap) / WHEEL_RADIUS
        back_left_wheel_velocity = (V_x + V_y - (L_X + L_Y) * thetap) / WHEEL_RADIUS
        back_right_wheel_velocity = (V_x - V_y + (L_X + L_Y) * thetap) / WHEEL_RADIUS

        self.__front_left_motor.setVelocity(front_left_wheel_velocity)
        self.__front_right_motor.setVelocity(front_right_wheel_velocity)
        self.__back_left_motor.setVelocity(back_left_wheel_velocity)
        self.__back_right_motor.setVelocity(back_right_wheel_velocity)

        # Publish wheel joint states (used by odometry_publisher)
        joint_states_header = Header()
        joint_states_header.stamp = self.__node.get_clock().now().to_msg()
        joint_states_header.frame_id = ""

        joint_states = JointState()
        joint_states.header = joint_states_header
        joint_states.name = ["motor1", "motor2", "motor3", "motor4"]
        joint_states.position = [
            float(self.__encoder1.getValue()),
            float(self.__encoder2.getValue()),
            float(self.__encoder3.getValue()),
            float(self.__encoder4.getValue()),
        ]

        self.__joint_states_publisher.publish(joint_states)
