#!/usr/bin/env python3
"""
Template for controlling PhantomX Pincher with your Python scripts.
- ros2 run phantomx_pincher_demos template.py
"""

from threading import Thread

import rclpy
from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import phantomx_pincher
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


def main():
    rclpy.init()

    # Create node for this example
    node = Node("phantomx_pincher_template")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interfaces
    moveit2 = MoveIt2(
        node=node,
        joint_names=phantomx_pincher.joint_names(),
        base_link_name=phantomx_pincher.base_link_name(),
        end_effector_name=phantomx_pincher.end_effector_name(),
        group_name=phantomx_pincher.MOVE_GROUP_ARM,
        callback_group=callback_group,
        execute_via_moveit=True,
    )
    gripper = GripperInterface(
        node=node,
        gripper_joint_names=phantomx_pincher.gripper_joint_names(),
        open_gripper_joint_positions=phantomx_pincher.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=phantomx_pincher.CLOSED_GRIPPER_JOINT_POSITIONS,
        gripper_group_name=phantomx_pincher.MOVE_GROUP_GRIPPER,
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
    moveit2.max_velocity = 0.25
    moveit2.max_acceleration = 0.25

    ##
    # TODO: Make your changes here
    ##

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
