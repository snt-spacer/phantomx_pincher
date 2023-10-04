#!/usr/bin/env python3
"""
Demo of repeatedly moving the robot to pre-defined joint configurations.
- ros2 run phantomx_pincher_demos demo_observe_scene.py

This demo is used as a part of Gazebo demo `demo_moon_camera_sensors.launch.py`.
- ros2 launch phantomx_pincher_demos demo_moon_camera_sensors.launch.py
"""


from math import pi
from threading import Thread
from typing import List

import rclpy
from pymoveit2 import MoveIt2
from pymoveit2.robots import phantomx_pincher
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# List of joint positions that the robot should repeatadly reach
JOINT_POSITION_GOALS: List[List[float]] = [
    [-pi / 4, pi / 4, pi / 2, -pi / 5],
    [pi / 4, pi / 4, pi / 2, -pi / 5],
]


def main():
    rclpy.init()

    # Create node for this demo
    node = Node("demo_observe_scene")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=phantomx_pincher.joint_names(),
        base_link_name=phantomx_pincher.base_link_name(),
        end_effector_name=phantomx_pincher.end_effector_name(),
        group_name=phantomx_pincher.MOVE_GROUP_ARM,
        callback_group=callback_group,
        execute_via_moveit=True,
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    # Wait until the simulated robot is ready to be controlled
    rate = node.create_rate(1.0)
    while rclpy.ok() and moveit2.joint_state is None:
        rate.sleep()
    rate.destroy()

    # Repeatedly move the robot to pre-defined joint configurations
    while rclpy.ok():
        for joint_position_goal in JOINT_POSITION_GOALS:
            node.get_logger().info(
                f"Moving to {{joint_positions: {list(joint_position_goal)}}}"
            )
            moveit2.move_to_configuration(joint_position_goal)
            moveit2.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
