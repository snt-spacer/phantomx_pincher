#!/usr/bin/env python3
"""
Example of interacting with the gripper.
`ros2 run phantomx_pincher_demos ex_gripper.py --ros-args -p action:="toggle"`
`ros2 run phantomx_pincher_demos ex_gripper.py --ros-args -p action:="open"`
`ros2 run phantomx_pincher_demos ex_gripper.py --ros-args -p action:="close"`
"""

from threading import Thread

import rclpy
from pymoveit2 import MoveIt2Gripper
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from robots import phantomx_pincher


def main():

    rclpy.init()

    # Create node for this example
    node = Node("ex_gripper")

    # Declare parameter for joint positions
    node.declare_parameter(
        "action",
        "toggle",
    )

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 gripper interface
    moveit2_gripper = MoveIt2Gripper(
        node=node,
        gripper_joint_names=phantomx_pincher.gripper_joint_names(),
        open_gripper_joint_positions=phantomx_pincher.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=phantomx_pincher.CLOSED_GRIPPER_JOINT_POSITIONS,
        gripper_group_name=phantomx_pincher.MOVE_GROUP_GRIPPER,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Sleep a while in order to get the first joint state
    node.create_rate(10.0).sleep()

    # Get parameter
    action = node.get_parameter("action").get_parameter_value().string_value

    # Perform gripper action
    node.get_logger().info(f'Performing gripper action "{action}"')
    if "open" == action:
        moveit2_gripper.open()
        moveit2_gripper.wait_until_executed()
    elif "close" == action:
        moveit2_gripper.close()
        moveit2_gripper.wait_until_executed()
    else:
        period_s = 1.0
        rate = node.create_rate(1 / period_s)
        while rclpy.ok():
            moveit2_gripper()
            moveit2_gripper.wait_until_executed()
            rate.sleep()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
