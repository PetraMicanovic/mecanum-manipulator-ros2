"""
Combined demo task: drive the mecanum platform up to the table, move the arm
to the 'ready_to_push' named pose, then nudge the platform forward so the arm
pushes the cube on the table.

Usage:
    ros2 run mecanum_manipulator_control demo
"""

import rclpy
import threading
import time
import os

from rclpy.node import Node
from mecanum_manipulator_control.mecanum_platform_controller import (
    MecanumPlatformController,
    LINEAR_VELOCITY,
    ANGULAR_VELOCITY,
)
from mecanum_manipulator_control.plan_named_pose import plan_and_execute_named_pose
from mecanum_manipulator_control.moveit_config_utils import build_moveit_config, build_moveit_cpp_params
from moveit.planning import MoveItPy

# Platform sequence: approach the table from wherever the robot starts.
# Steps: (cmd_type, vx, vy, wz, value[m or deg])
TABLE_APPROACH_SEQUENCE = [
    ("linear", -LINEAR_VELOCITY, 0.0, 0.0, 2.0),  # back 2 m
    ("linear", 0.0, LINEAR_VELOCITY, 0.0, 2.0),  # left 2 m
    ("angular", 0.0, 0.0, -ANGULAR_VELOCITY, 195.0),  # rotate_r 195 deg
    ("linear", LINEAR_VELOCITY, 0.0, 0.0, 0.1),  # forward 0.1 m
]

# Small nudge forward once the arm is in the 'ready_to_push' pose,
# so the end-effector actually contacts and pushes the cube.
PUSH_SEQUENCE = [
    ("linear", LINEAR_VELOCITY, 0.0, 0.0, 0.15),  # forward 0.15 m
]

# Retreat: back away from the table after the push, before returning the arm home.
RETREAT_SEQUENCE = [
    ("linear", -LINEAR_VELOCITY, 0.0, 0.0, 0.3),  # back 0.3 m 
]


def run_task(args=None):
    """
    Run the full table-push demo: approach table (platform) -> ready_to_push (arm)
    -> push nudge (platform).
    """
    rclpy.init(args=args)

    # Platform node setup
    platform_node = MecanumPlatformController()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(platform_node,), daemon=True
    )
    spin_thread.start()

    success = True

    try:
        # 1. Platform approaches the table
        platform_node.get_logger().info("Approaching table...")
        platform_node.run_sequence(TABLE_APPROACH_SEQUENCE)

        # 2. Arm moves to the 'ready_to_push' named pose
        platform_node.get_logger().info("Moving arm to 'ready_to_push'...")
        param_node = Node("table_push_task_params")
        use_sim_time = param_node.get_parameter("use_sim_time").value

        moveit_config = build_moveit_config()
        moveit_cpp_params = build_moveit_cpp_params(
            moveit_config, use_sim_time=use_sim_time
        )
        moveit = MoveItPy(
            node_name="table_push_moveit_py", config_dict=moveit_cpp_params
        )

        success = plan_and_execute_named_pose(moveit, "ready_to_push")

        if not success:
            platform_node.get_logger().error(
                "Arm failed to reach 'ready_to_push' -- skipping push."
            )
        else:
            time.sleep(1.0)

            # 3. Platform nudges forward to push the cube
            platform_node.get_logger().info("Pushing cube...")
            platform_node.run_sequence(PUSH_SEQUENCE)

            # 4. Platform backs away from the table, clearing space for the arm
            time.sleep(1.0)
            platform_node.get_logger().info("Retreating from table...")
            platform_node.run_sequence(RETREAT_SEQUENCE)

            # 5. Arm returns to its start pose
            time.sleep(1.0)
            platform_node.get_logger().info("Returning arm to 'start_pose'...")
            return_success = plan_and_execute_named_pose(moveit, "start_pose")
            if not return_success:
                platform_node.get_logger().error(
                    "Arm failed to return to 'start_pose'."
                )
            success = success and return_success

        param_node.destroy_node()

    finally:
        platform_node.stop()
        rclpy.shutdown()
        spin_thread.join()
        platform_node.destroy_node()

    if success:
        exit_value = 0
    else:
        exit_value = 1
    os._exit(exit_value)


def main(args=None):
    run_task(args)


if __name__ == "__main__":
    main()
