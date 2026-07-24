"""
Launches RViz2 configured for MoveIt2 motion planning.

This file does not start simulation and its own move_group. It assumes mobile_platform_with_arm_launch.py (with moveit:=true) is already running and has
its own move_group node up. This file only builds the same MoveIt2 config (so RViz's MotionPlanning display has robot_description/SRDF/kinematics/etc. to
show) and starts RViz2 alone.

Usage:
    ros2 launch mobile_platform_with_arm moveit_rviz.launch.py

Not intended to be launched standalone. Run mobile_platform_with_arm_launch.py
(with moveit:=true) first, since it provides the move_group node this RViz instance
connects to for motion planning.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _build_moveit_config():
    """
    Assemble the MoveIt2 config object for the rv3sdb arm.

    Keep in sync with move_group.launch.py's build_moveit_config().

    Returns:
        moveit_configs_utils.MoveItConfigs:
            MoveIt configuration containing robot_description, SRDF, kinematics, joint limits and trajectory execution settings.
    """
    platform_share = get_package_share_directory("mobile_platform_with_arm")
    urdf_path = os.path.join(
        platform_share, "resource", "mobile_platform_with_arm.urdf"
    )
    return (
        MoveItConfigsBuilder("rv3sdb", package_name="rv3sdb_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path="config/rv3sdb.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )


def generate_launch_description():
    """
    Build the launch description for this file.

    Starts a plain RViz2 instance parameterized with the MoveIt2 config, so the MotionPlanning display can talk to the move_group that's already running 
    from mobile_platform_with_arm_launch.py.

    Returns:
        launch.LaunchDescription:
            Launches RViz2 alone.
    """
    moveit_config = _build_moveit_config()

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([rviz_node])
