"""
ROS2 launch file for the mecanum platform with the rv3sdb arm in Webots. Arm is driven end-to-end
through MoveIt2.

The wheel/lidar side (MecanumRobotDriver, wheel odometry, footprint TF) reuses code from mobile_platform_sim unchanged. This file loads
mobile_platform_with_arm.urdf, launches platform_with_arm.wbt, webots_ros2_control exposes for the arm's 6 joints and includes rv3sdb_moveit_config's
move_group.launch.py so MoveIt2 can plan/execute against the arm.

Wheel joint states (from /wheels_encoders) and arm joint states (from joint_state_broadcaster) are merged into a single /joint_states topic by
joint_state_publisher's source_list, since robot_state_publisher needs exactly one combined feed to build the whole TF tree (chassis + arm).
The arm's ros2_control node is remapped so its own "joint_states" topic doesn't collide with that final merged one.
"""

import os
import launch

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Build the launch description for the full platform and arm simulation.

    Starts Webots with the platform_with_arm world, brings up both drivers (mecanum wheels and the arm's ros2_control), spawns the joint_state_broadcaster 
    and arm controller and merges wheel and arm joint states into a single /joint_states feed for robot_state_publisher.

    Launch arguments:
        rviz: bool
            Start a plain rviz2 instance alongside the simulation. Default False
        moveit: bool
            Include rv3sdb_moveit_config's move_group.launch.py so the arm can be planned through MoveIt2. Default True

    Returns:
        launch.LaunchDescription
            Webots, both robot description publishers, the joint state merger and controller group(spawners, move_group, rviz) wrapped in 
            WaitForControllerConnection
    """
    use_rviz = LaunchConfiguration("rviz")
    use_moveit = LaunchConfiguration("moveit")

    package_dir = get_package_share_directory("mobile_platform_with_arm")
    platform_description_path = os.path.join(
        package_dir, "resource", "mobile_platform_with_arm.urdf"
    )
    with open(platform_description_path, "r") as description:
        platform_description = description.read()

    controllers_yaml_path = os.path.join(
        get_package_share_directory("rv3sdb_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    # Starts the Webots simulator with the platform+arm world
    webots = launch.actions.ExecuteProcess(
        cmd=[
            "/usr/local/webots/webots",
            os.path.join(package_dir, "worlds", "mobile_platform_with_arm.wbt"),
        ],
        output="screen",
    )

    # Single WebotsController for the "mobile_platform" robot. Its URDF <webots> block declares 2 plugins: MecanumRobotDriver (wheels, unchanged) and
    # webots_ros2_control::Ros2Control(arm).
    platform_driver = WebotsController(
        robot_name="mobile_platform_with_arm",
        parameters=[
            {"robot_description": platform_description_path},
            controllers_yaml_path,
        ],
        # ros2_control_node's own "joint_states" gets remapped here so it doesn't collide with the merged /joint_states that joint_state_publisher 
        # produces below.
        remappings=[("/joint_states", "/arm/joint_states")],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # Bare rviz2 with no MoveIt2 parameters. It is good for a quick look at TF/topics, but has no robot model or MotionPlanning display.
    # For actual arm planning, use moveit_rviz.launch.py alongside this file.
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

    # Merges wheel encoder joint states and the arm's joint_state_broadcaster output into one /joint_states feed for robot_state_publisher.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"source_list": ["wheels_encoders", "/arm/joint_states"]}],
    )

    # Wheel odometry (unchanged)
    odometry_publisher = Node(
        package="mobile_platform_sim",
        executable="odometry_publisher",
        remappings=[("/odom", "/wheel/odometry")],
    )

    # MoveIt2's move_group, planning/executing against the same URDF+SRDF
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rv3sdb_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        ),
        condition=launch.conditions.IfCondition(use_moveit),
    )

    # Delay everything that needs the arm's controllers/TF/move_group until the Webots controller connection is actually up
    waiting_nodes = WaitForControllerConnection(
        target_driver=platform_driver,
        nodes_to_start=[
            rviz2,
            odometry_publisher,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            move_group_launch,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start a plain rviz2 instance",
            ),
            DeclareLaunchArgument(
                "moveit",
                default_value="true",
                description="Include move_group for MoveIt2 planning",
            ),
            webots,
            platform_driver,
            footprint_publisher,
            robot_state_publisher,
            joint_state_publisher,
            waiting_nodes,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
