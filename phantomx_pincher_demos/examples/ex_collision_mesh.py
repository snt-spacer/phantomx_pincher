#!/usr/bin/env python3
"""
Example of adding and removing a collision object with a mesh geometry.
Note: Python module `trimesh` is required for this example (`pip install trimesh`).
- ros2 run phantomx_pincher_demos ex_collision_mesh.py --ros-args -p position:="[0.25, 0.0, 0.1]" -p quat_xyzw:="[0.0, 0.0, -0.7071, 0.7071]"
- ros2 run phantomx_pincher_demos ex_collision_mesh.py --ros-args -p filepath:="./my_favourity_mesh.stl"
- ros2 run phantomx_pincher_demos ex_collision_mesh.py --ros-args -p action:="remove"
"""

from os import path
from threading import Thread

import rclpy
from pymoveit2 import MoveIt2
from pymoveit2.robots import phantomx_pincher
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

DEFAULT_EXAMPLE_MESH = path.join(
    path.dirname(path.realpath(__file__)), "assets", "suzanne.stl"
)


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_collision_mesh")

    # Declare parameter for joint positions
    node.declare_parameter(
        "filepath",
        "",
    )
    node.declare_parameter(
        "action",
        "add",
    )
    node.declare_parameter("position", [0.25, 0.0, 0.1])
    node.declare_parameter("quat_xyzw", [0.0, 0.0, -0.7071, 0.7071])

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

    # Get parameters
    filepath = node.get_parameter("filepath").get_parameter_value().string_value
    action = node.get_parameter("action").get_parameter_value().string_value
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value

    # Use the default example mesh if invalid
    if not filepath:
        node.get_logger().info(f"Using the default example mesh file")
        filepath = DEFAULT_EXAMPLE_MESH

    # Make sure the mesh file exists
    if not path.exists(filepath):
        node.get_logger().error(f"File '{filepath}' does not exist")
        rclpy.shutdown()
        exit(1)

    # Determine ID of the collision mesh
    object_id = path.basename(filepath).split(".")[0]

    if "add" == action:
        # Add collision mesh
        node.get_logger().info(
            f"Adding collision mesh '{filepath}' {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        moveit2.add_collision_mesh(
            filepath=filepath,
            id=object_id,
            position=position,
            quat_xyzw=quat_xyzw,
        )
    else:
        # Remove collision mesh
        node.get_logger().info(f"Removing collision mesh with ID '{object_id}'")
        moveit2.remove_collision_object(id=object_id)

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
