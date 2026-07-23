"""
Utility for planning and executing motions to named SRDF group states using MoveItPy.
"""

import rclpy
import time

from moveit.planning import MoveItPy
from rclpy.node import Node
from demo_app.moveit_config_utils import build_moveit_config, build_moveit_cpp_params


def plan_and_execute_named_pose(moveit, target_pose):
    """
    Plan and execute the arm planning component to a named SRDF group_state.

    Args:
        moveit: MoveItPy
            An already-initialized MoveItPy instance.
        target_pose: str
           Name of the SRDF group_state to move to.

    Returns:
        bool:
            True if planning and execution succeeded, False if planning failed.
    """
    arm = moveit.get_planning_component("arm")

    print("Setting start state to current robot state...")
    arm.set_start_state_to_current_state()

    print(f"Setting goal to named pose: '{target_pose}'")
    arm.set_goal_state(configuration_name=target_pose)

    print("Planning...")
    plan_result = arm.plan()

    if not plan_result:
        print(f"PLANNING FAILED for target pose '{target_pose}'.")
        print(
            "Possible causes: pose is out of reach, in self-collision or the group_state name doesn't match what's in rv3sdb.srdf exactly."
        )
        return False

    print("Planning succeeded. Executing...")
    moveit.execute(plan_result.trajectory, controllers=[])
    print(f"Execution complete -- arm should now be at '{target_pose}'.")
    return True


def run_named_pose_task(target_pose = "start_pose", use_sim_time = False):
    """
    Build the MoveIt configuration, create a MoveItPy instance
    and plan and execute to the requested named pose.

    Args:
        target_pose: str
            Name of the group_state to move to (e.g. "ready_to_push", "capture_pose1").
        use_sim_time: bool
            If True MoveItPy uses sim-clock instead of wall-clock.

    Returns:
        bool:
            True if planning and execution succeeded, False if planning failed.
    """
    moveit_config = build_moveit_config()
    moveit_cpp_params = build_moveit_cpp_params(moveit_config, use_sim_time=use_sim_time)

    moveit = MoveItPy(node_name="named_pose_moveit_py", config_dict=moveit_cpp_params)

    return plan_and_execute_named_pose(moveit, target_pose)


def run_from_cli(args=None):
    """
    Initializes rclpy, reads the "target_pose" and "use_sim_time" ROS parameters,
    runs the named pose task, then shuts rclpy down.

    Use this only when running plan_named_pose as its own process (ros2 run).

    Returns:
        bool:
            True if planning and execution succeeded, False if planning failed.
    """
    rclpy.init(args=args)

    param_node = Node("plan_named_pose_params")
    param_node.declare_parameter("target_pose", "ready_to_push")

    target_pose = param_node.get_parameter("target_pose").value
    use_sim_time = param_node.get_parameter("use_sim_time").value

    moveit_config = build_moveit_config()
    moveit_cpp_params = build_moveit_cpp_params(moveit_config, use_sim_time=use_sim_time)
    moveit = MoveItPy(node_name="named_pose_moveit_py", config_dict=moveit_cpp_params)

    success = plan_and_execute_named_pose(moveit, target_pose)

    time.sleep(10)

    success = plan_and_execute_named_pose(moveit, "start_pose")

    param_node.destroy_node()
    rclpy.shutdown()
    if success:
        return 0
    else:
        return 1


def main(args=None):
    run_from_cli(args)


if __name__ == "__main__":
    main()