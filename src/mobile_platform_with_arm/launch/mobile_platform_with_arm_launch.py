"""
ROS2 launch file for the mecanum platform with the rv3sdb arm in Webots.

This enables launching the mobile platform together with the arm in the Webots simulation. The next step will be adding the MoveIt2 layer for
actual arm control.
"""

import os
import launch

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Build the launch description for the platform+arm simulation.

    Starts Webots with the mobile_platform_with_arm world, spins up the mobile_platform WebotsController, publishes the combined URDF and TF tree through robot_state_publisher and exposes the arm's 6 joints as maual sliders through joint_state_publisher_gui.

    Returns:
        LaunchDescription: the assembled set of processes/nodes for this launch file.
    """
    use_rviz = LaunchConfiguration("rviz", default=False)

    package_dir = get_package_share_directory("mobile_platform_with_arm")
    platform_description_path = os.path.join(
        package_dir, "resource", "mobile_platform_with_arm.urdf"
    )
    with open(platform_description_path, "r") as description:
        platform_description = description.read()

    webots = launch.actions.ExecuteProcess(
        cmd=[
            "/usr/local/webots/webots",
            os.path.join(package_dir, "worlds", "mobile_platform_with_arm.wbt"),
        ],
        output="screen",
    )

    # Only the MecanumRobotDriver plugin runs here right now. The URDF's webots block has no ros2-plugin yet.
    platform_driver = WebotsController(
        robot_name="mobile_platform_with_arm",
        parameters=[{"robot_description": platform_description_path}],
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=launch.conditions.IfCondition(use_rviz),
    )

    # Static transform: base_footprint -> base_link (unchanged)
    footprint_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0.1015", "0", "0", "0", "base_footprint", "base_link"],
    )

    # Broadcasts the combined platform+arm URDF and the whole TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": platform_description}],
        arguments=[platform_description_path],
    )

    # Wheel joint states come from the real MecanumRobotDriver encoders (/wheels_encoders).
    # The 6 arm joints aren't in any source topic (no controller drives them yet), so joint_state_publisher_gui below supplies them via sliders; this node only handles wheels.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"source_list": ["wheels_encoders"]}],
    )

    # Sliders for the 6 arm joints -- move these by hand to sanity-check mesh alignment/limits while there's no real controller driving the arm yet.
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # Wheel odometry (unchanged)
    odometry_publisher = Node(
        package="mobile_platform_sim",
        executable="odometry_publisher",
        remappings=[("/odom", "/wheel/odometry")],
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=platform_driver,
        nodes_to_start=[rviz2, odometry_publisher],
    )

    return LaunchDescription(
        [
            webots,
            platform_driver,
            footprint_publisher,
            robot_state_publisher,
            joint_state_publisher,
            joint_state_publisher_gui,
            waiting_nodes,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
