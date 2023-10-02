#!/usr/bin/env python3
"""
Example of moving to a pose goal.
- ros2 run phantomx_pincher_demos ex_pose_goal.py --ros-args -p position:="[0.075, 0.0, 0.05]" -p quat_xyzw:="[1.0, 0.0, 0.0, 0.0]" -p cartesian:=False
"""

from threading import Thread

import rclpy
from pymoveit2 import MoveIt2
from pymoveit2.robots import phantomx_pincher
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.075, 0.0, 0.05])
    node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("cartesian", False)

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
    moveit2.max_velocity = 0.25
    moveit2.max_acceleration = 0.25

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    num_attempts = 0
    while True:
        moveit2.move_to_pose(
            position=position, quat_xyzw=quat_xyzw, cartesian=cartesian
        )
        if moveit2.wait_until_executed() or num_attempts >= 3:
            break
        num_attempts += 1

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
