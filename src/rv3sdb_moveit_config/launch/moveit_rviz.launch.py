"""
Launches move_group together with RViz2.

This file does not start simulation itself. It only adds the MoveIt2 planning node and an RViz instance for motion planning.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    platform_share = get_package_share_directory("mobile_platform_sim")
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
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )


def generate_launch_description():
    """
    Build the launch description for this file.

    Includes move_group.launch.py from rv3sdb_moveit_config and starts plain RViz2 instance a few seconds later.

    Returns:
        launch.LaunchDescription:
            Launches move_group immediately and RViz after a short delay
    """
    moveit_config = _build_moveit_config()

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rv3sdb_moveit_config"),
                "launch",
                "move_group.launch.py",
            )
        )
    )

    # NOTE: the original ROS1 package shipped a moveit.rviz config, but rviz2's config format isn't compatible with it, so we start plain rviz2 here. 
    # Add the "MotionPlanning" display (fixed frame: base_link, planning group: arm) once, then File > Save Config As to keep it.
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

    # Give move_group a couple seconds' head start before RViz tries to query it.
    delayed_rviz = TimerAction(period=3.0, actions=[rviz_node])

    return LaunchDescription([move_group_launch, delayed_rviz])
