"""
Shared MoveIt2 configuration builders for moveit_py-based scripts in demo_app.
"""

import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config():
    """
    Build the MoveIt2 configuration shared by all moveit_py scripts in demo_app.

    Using the same configuration as move_group.launch.py ensures every script plans
    against the same robot description, SRDF, kinematics and planning pipeline.

    Returns:
        moveit_configs_utils.MoveItConfigs:
            The assembled configuration object
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
        .to_moveit_configs()
    )


def build_moveit_cpp_params(
    moveit_config,
    planning_attempts=5,
    planning_time=5.0,
    max_velocity_scaling_factor=0.5,
    max_acceleration_scaling_factor=0.5,
    use_sim_time=False,
):
    """
    Parameter dictionary used to initialize MoveItPy's MoveItCpp backend.

    Args:
        moveit_config: moveit_configs_utils.MoveItConfigs
            The base configuration from build_moveit_config().
        planning_attempts: int
            Number of planning attempts before giving up.
        planning_time: float
            Maximum planning time (in seconds) per attempt.
        max_velocity_scaling_factor: float
            Scaling factor applied to joint velocity limits.
        max_acceleration_scaling_factor: float
            Scaling factor applied to joint acceleration limits.
        use_sim_time: bool
            If True MoveItPy uses sim-clock instead of wall-clock.

    Returns:
        dict:
            Merged parameter dictionary suitable for MoveItPy's config_dict argument.
    """
    params = moveit_config.to_dict()

    params["use_sim_time"] = use_sim_time

    params["planning_pipelines"] = {"pipeline_names": ["ompl"]}

    params["plan_request_params"] = {
        "planning_pipeline": "ompl",
        "planning_attempts": planning_attempts,
        "planning_time": planning_time,
        "max_velocity_scaling_factor": max_velocity_scaling_factor,
        "max_acceleration_scaling_factor": max_acceleration_scaling_factor,
    }

    # Without this, MoveItPy falls back to generic default topic names and may miss
    # the robot's current joint state or planning scene updates in time.
    params["planning_scene_monitor_options"] = {
        "name": "planning_scene_monitor",
        "robot_description": "robot_description",
        "joint_state_topic": "/joint_states",
        "attached_collision_object_topic": "/moveit_cpp/attached_collision_object",
        "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
        "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
        "wait_for_initial_state_timeout": 10.0,
    }

    return params
