#!/usr/bin/env python3
"""
Demo script for peg-in-hole task sequence. Compatible with fake, simulated and real robot.

Note: This version is a template that you need to finish as a part of your assignment!
"""

import sys
from math import atan2, cos, sin
from os import path
from typing import Tuple

import geometry_msgs.msg
import moveit_commander
import rospy
import std_msgs.msg


def _rpy2quat(
    roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0
) -> Tuple[float, float, float, float]:
    """
    Helper function for converting rotation represented as roll-pitch-yaw to quaternions (xyzw order).
    """

    r_2 = roll / 2
    p_2 = pitch / 2
    y_2 = yaw / 2

    x = sin(r_2) * cos(p_2) * cos(y_2) - cos(r_2) * sin(p_2) * sin(y_2)
    y = cos(r_2) * sin(p_2) * cos(y_2) + sin(r_2) * cos(p_2) * sin(y_2)
    z = cos(r_2) * cos(p_2) * sin(y_2) - sin(r_2) * sin(p_2) * cos(y_2)
    w = cos(r_2) * cos(p_2) * cos(y_2) + sin(r_2) * sin(p_2) * sin(y_2)
    return (x, y, z, w)


# Scaling factor for the dynamic scene objects - lower value makes it easier to plan when using an inaccurate (cheap) robot
# The default value of 0.8 was tested with the real robot
OBJECT_SCALE = 0.8
# List of dynamic scene objects
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
        "target_pose": geometry_msgs.msg.PoseStamped(
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
                    *_rpy2quat(yaw=atan2(-0.05, 0.145))
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
        "target_pose": geometry_msgs.msg.PoseStamped(
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
            "filename": path.join(
                path.dirname(path.realpath(__file__)), "assets", "module_holes.stl"
            )
        },
    },
]


def setup_planning_scene(
    scene: moveit_commander.planning_scene_interface.PlanningSceneInterface,
    add_dynamic_scene_objects: bool = True,
):
    """
    Setup the planning scene of MoveIt for collision checking.
    With `add_dynamic_scene_objects` disabled, only static scene objects are added.
    """

    ## Remove all scene objects from previous run
    scene.remove_attached_object()
    for scene_object_name in scene.get_known_object_names():
        scene.remove_world_object(scene_object_name)

    ## Add static objects to the planning scene
    for scene_object in STATIC_SCENE_OBJECTS:
        scene_object["pose"].header.stamp = rospy.Time.from_sec(rospy.get_time())
        getattr(scene, f'add_{scene_object["geometry_type"]}')(
            name=scene_object["name"],
            pose=scene_object["pose"],
            **scene_object["add_object_kwargs"],
        )

    ## Add dynamic objects to the planning scene (if desired)
    if add_dynamic_scene_objects:
        for scene_object in DYNAMIC_SCENE_OBJECTS:
            scene_object["initial_pose"].header.stamp = rospy.Time.from_sec(
                rospy.get_time()
            )
            getattr(scene, f'add_{scene_object["geometry_type"]}')(
                name=scene_object["name"],
                pose=scene_object["initial_pose"],
                **scene_object["add_object_kwargs"],
            )


def main():

    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create node for this example
    rospy.init_node("phantomx_pincher_template", anonymous=True)

    # Instantiate RobotCommander (outer-level interface to the robot)
    robot = moveit_commander.robot.RobotCommander()

    # Instantiate MoveGroupCommander (interface to one group of joints) for arm and gripper
    arm = moveit_commander.move_group.MoveGroupCommander("arm", wait_for_servers=300.0)
    gripper = moveit_commander.move_group.MoveGroupCommander("gripper")

    # Instantiate moveit_commander.planning_scene_interface.PlanningSceneInterface (interface to the world surrounding the robot)
    scene = moveit_commander.planning_scene_interface.PlanningSceneInterface(
        synchronous=True
    )

    ### Setup the Planning Scene ###
    setup_planning_scene(scene=scene)

    ###
    # TODO: Perform the peg-in-hole task here
    ###

    # Ensure that there is no residual movement
    if not rospy.is_shutdown():
        arm.stop()

    # Shutdown MoveIt Commander and exit
    moveit_commander.roscpp_shutdown()
    exit(0)


if __name__ == "__main__":
    main()
