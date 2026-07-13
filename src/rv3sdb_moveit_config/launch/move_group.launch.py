"""
Starts the MoveIt2 move_group node for the rv3sdb arm mounted on the mecanum platform. 
This launch file does not start robot_state_publisher or any controller_manager, as they are already started by mobile_platform_with_arm_launch.py.
Instead, it builds move_group's own robot_description and robot_description_semantic parameters from 
the same URDF/SRDF so that planning and execution stay in sync.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config():
    """
    Build the full MoveIt2 configuration object for the rv3sdb arm.

    This function locates the combined robot URDF and combines it with the rv3sdb SRDF, kinematics, joint limits and controller mapping to construct a
    MoveItConfigs object.

    Returns:
        moveit_configs_utils.MoveItConfigs:
            MoveIt configuration object.
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
    ROS 2 automatically calls this function when this file is passed to ros2 launch.

    Returns:
        launch.LaunchDescription
            A launch description containing the move_group Node configured with the MoveIt parameters built by build_moveit_config().
    """
    moveit_config = build_moveit_config()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([move_group_node])
