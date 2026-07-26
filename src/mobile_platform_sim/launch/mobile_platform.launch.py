"""
ROS2 launch file for the mecanum-wheeled mobile platform simulation in Webots.

This launch file starts the Webots simulation together with all ROS2 nodes required for the mobile platform,
including the Webots controller, robot_state_publisher, joint_state_publisher, odometry_publisher, TF 
publishers and optionally RViz2.

Usage:
    ros2 launch mobile_platform_sim mobile_platform.launch.py
    ros2 launch mobile_platform_sim mobile_platform.launch.py rviz:=true

Launch arguments:
    rviz: bool
        If true, starts RViz2 alongside the simulation.
        default: false
"""
import os
import launch

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate the launch description for the mobile platform simulation.

    Loads the robot URDF from the installed package share directory and constructs a LaunchDescription 
    containing all required nodes.

    Returns:
        LaunchDescription
            Complete launch description for the simulation.
    """

    use_rviz = LaunchConfiguration("rviz", default=False)

    package_dir = get_package_share_directory("mobile_platform_sim")
    platform_description_path = os.path.join(package_dir, "resource", "mobile_platform.urdf")
    with open(platform_description_path, "r") as description:
        platform_description = description.read()

    # Starts the Webots simulator with the platform world
    webots = WebotsLauncher(world=os.path.join(package_dir, "worlds", "platform.wbt"))

    # WebotsController loads MecanumRobotDriver as an external controller
    platform_driver = WebotsController(
        robot_name="mobile_platform",
        parameters=[{"robot_description": platform_description_path}],
    )

    rviz_config_path = os.path.join(package_dir, "config", "mobile_platform.rviz")

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"robot_description": platform_description}],
        condition=launch.conditions.IfCondition(use_rviz),
    )

    # Static transform: base_footprint -> base_link
    footprint_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0.1015", "0", "0", "0", "base_footprint", "base_link"],
    )

    # Broadcasts the URDF and TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": platform_description}],
        arguments=[platform_description_path],
    )

    # Aggregates wheel encoder data from /wheels_encoders into /joint_states for the TF tree
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"source_list": ["wheels_encoders"]}],
    )

    # Computes and publishes wheel odometry
    # /odom is remapped to /wheel/odometry
    odometry_publisher = Node(
        package="mobile_platform_sim",
        executable="odometry_publisher",
        remappings=[("/odom", "/wheel/odometry")],
    )

    # Delays rviz2 and odometry_publisher until the WebotsController connection is established
    waiting_nodes = WaitForControllerConnection(
        target_driver=platform_driver,
        nodes_to_start=[
            rviz2,
            odometry_publisher,
        ],
    )

    return LaunchDescription(
        [
            webots,
            platform_driver,
            footprint_publisher,
            robot_state_publisher,
            joint_state_publisher,
            waiting_nodes,
            # Shut down all nodes when Webots exits
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )