#!/usr/bin/env python3
"""
Note: !! This file does not contain the full solution. Your task is to implement the missing parts. !!

Demo script for peg-in-hole task sequence. Compatible with fake, simulated and real robot.
- ros2 run phantomx_pincher_demos demo_peg_in_hole.py

This demo is used as a part of Gazebo demo `demo_rover_peg_in_hole.launch.py`.
- ros2 launch phantomx_pincher_demos demo_rover_peg_in_hole.launch.py
"""

from copy import deepcopy
from math import atan2, cos, pi, sin
from os import path
from threading import Thread
from typing import Dict

import geometry_msgs.msg
import rclpy
import std_msgs.msg
import tf2_geometry_msgs
import tf2_ros
from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import phantomx_pincher
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


def rpy2quat(
    roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
) -> Dict[str, float]:
    """
    Helper function for converting rotation represented as roll-pitch-yaw to quaternions (xyzw order).
    """

    r_2 = roll / 2
    p_2 = pitch / 2
    y_2 = yaw / 2

    return dict(
        x=sin(r_2) * cos(p_2) * cos(y_2) - cos(r_2) * sin(p_2) * sin(y_2),
        y=cos(r_2) * sin(p_2) * cos(y_2) + sin(r_2) * cos(p_2) * sin(y_2),
        z=cos(r_2) * cos(p_2) * sin(y_2) - sin(r_2) * sin(p_2) * cos(y_2),
        w=cos(r_2) * cos(p_2) * cos(y_2) + sin(r_2) * sin(p_2) * sin(y_2),
    )


# Scaling factor for the dynamic scene objects - lower value makes it easier to plan when using an inaccurate (cheap) robot
# The default value of 0.75 was tested with the real robot
OBJECT_SCALE = 0.75
# List of dynamic scene objects
# Note: You are allowed to change these
DYNAMIC_SCENE_OBJECTS = [
    {
        "name": "peg1",
        "geometry_type": "cylinder",
        "initial_pose": geometry_msgs.msg.PoseStamped(
            header=std_msgs.msg.Header(
                frame_id="phantomx_pincher_arm_base_link",
            ),
            pose=geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(x=0.12, y=0.05, z=-0.044),
                orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        "add_object_kwargs": {
            "radius": OBJECT_SCALE * 0.0125,
            "height": OBJECT_SCALE * 0.04,
        },
        "insert_pose": geometry_msgs.msg.PoseStamped(
            header=std_msgs.msg.Header(
                frame_id="phantomx_pincher_base_link",
            ),
            pose=geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(x=0.05, y=0.0, z=0.053),
                orientation=geometry_msgs.msg.Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
            ),
        ),
    },
    {
        "name": "peg2",
        "geometry_type": "box",
        "initial_pose": geometry_msgs.msg.PoseStamped(
            header=std_msgs.msg.Header(
                frame_id="phantomx_pincher_arm_base_link",
            ),
            pose=geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(x=0.145, y=-0.05, z=-0.038),
                orientation=geometry_msgs.msg.Quaternion(
                    **rpy2quat(yaw=atan2(-0.05, 0.145))
                ),
            ),
        ),
        "add_object_kwargs": {
            "size": (
                OBJECT_SCALE * 0.025,
                OBJECT_SCALE * 0.025,
                OBJECT_SCALE * 0.05,
            )
        },
        "insert_pose": geometry_msgs.msg.PoseStamped(
            header=std_msgs.msg.Header(
                frame_id="phantomx_pincher_base_link",
            ),
            pose=geometry_msgs.msg.Pose(
                position=geometry_msgs.msg.Point(x=0.1, y=0.0, z=0.057),
                orientation=geometry_msgs.msg.Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
            ),
        ),
    },
]
# List of static scene objects
STATIC_SCENE_OBJECTS = [
    {
        "name": "module_with_holes",
        "geometry_type": "mesh",
        "pose": geometry_msgs.msg.PoseStamped(
            header=std_msgs.msg.Header(
                frame_id="phantomx_pincher_task_board_module_link",
            ),
        ),
        "add_object_kwargs": {
            "filepath": path.join(
                path.dirname(path.realpath(__file__)), "assets", "module_holes.stl"
            )
        },
    },
]


def main():
    rclpy.init()

    # Create node for this demo
    node = Node("demo_peg_in_hole")

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

    # Instantiate tf2 buffer and listener for looking up transforms
    tf2_buffer = tf2_ros.Buffer(node=node)
    tf2_listener = tf2_ros.TransformListener(buffer=tf2_buffer, node=node)

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

    ### Setup the Planning Scene ###
    # Clean the previous scene
    reset_planning_scene(moveit2=moveit2)
    # Move to initial joint configuration
    moveit2.move_to_configuration(
        joint_positions=[
            0.0,
            0.0,
            pi / 2,
            pi / 2,
        ],
    )
    moveit2.wait_until_executed()
    # Setup the new scene
    setup_planning_scene(
        moveit2=moveit2,
        # Note: Planning can be simplified by disabling the dynamic scene objects
        add_dynamic_scene_objects=True,
    )

    ##
    # TODO: Make your changes here (pick and insert the pegs)
    ##

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


def setup_planning_scene(
    moveit2: MoveIt2,
    add_dynamic_scene_objects: bool = True,
):
    """
    Setup the planning scene of MoveIt for collision checking.
    With `add_dynamic_scene_objects` disabled, only static scene objects are added.
    """

    ## Add static objects to the planning scene
    for scene_object in STATIC_SCENE_OBJECTS:
        getattr(moveit2, f'add_collision_{scene_object["geometry_type"]}')(
            id=scene_object["name"],
            pose=scene_object["pose"],
            **scene_object["add_object_kwargs"],
        )

    ## Add dynamic objects to the planning scene (if desired)
    if add_dynamic_scene_objects:
        for scene_object in DYNAMIC_SCENE_OBJECTS:
            getattr(moveit2, f'add_collision_{scene_object["geometry_type"]}')(
                id=scene_object["name"],
                pose=scene_object["initial_pose"],
                **scene_object["add_object_kwargs"],
            )


def reset_planning_scene(
    moveit2: MoveIt2,
):
    """
    Remove all scene objects from previous run.
    """

    moveit2.detach_all_collision_objects()
    for scene_object in STATIC_SCENE_OBJECTS:
        moveit2.remove_collision_object(scene_object["name"])
    for scene_object in DYNAMIC_SCENE_OBJECTS:
        moveit2.remove_collision_object(scene_object["name"])


if __name__ == "__main__":
    main()
