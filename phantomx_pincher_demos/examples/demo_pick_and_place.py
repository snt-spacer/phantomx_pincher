#!/usr/bin/env python3
"""
Demo script for pick&place task sequence. Compatible with fake, simulated and real robot.
"""


import random
import sys
from copy import deepcopy
from math import pi
from typing import Tuple

import geometry_msgs.msg
import moveit_commander
import rospy
import std_msgs.msg


def setup_planning_scene(
    scene: moveit_commander.planning_scene_interface.PlanningSceneInterface,
    object_pose: geometry_msgs.msg.PoseStamped,
    object_size: Tuple[float, float, float],
):
    """
    Setup the planning scene of MoveIt for collision checking.
    """

    ## 0 (cleanup): Remove all scene objects from previous run
    scene.remove_attached_object()
    for scene_object_name in scene.get_known_object_names():
        scene.remove_world_object(scene_object_name)

    ## 1: Add `module_cover` to the planning scene
    MODULE_COVER_FRAME_ID = "phantomx_pincher_task_board_module_link"
    MODULE_COVER_BOX_SIZE = (0.148, 0.148, 0.003)
    MODULE_COVER_POSITION = geometry_msgs.msg.Point(z=MODULE_COVER_BOX_SIZE[2] / 2.0)
    module_cover_pose = geometry_msgs.msg.PoseStamped(
        header=std_msgs.msg.Header(
            frame_id=MODULE_COVER_FRAME_ID, stamp=rospy.Time.from_sec(rospy.get_time())
        ),
        pose=geometry_msgs.msg.Pose(position=MODULE_COVER_POSITION),
    )
    scene.add_box("module_cover", module_cover_pose, MODULE_COVER_BOX_SIZE)

    ## Optional: Add `object` to the planning scene
    scene.add_box("object", object_pose, object_size)


def pick(
    arm: moveit_commander.move_group.MoveGroupCommander,
    gripper: moveit_commander.move_group.MoveGroupCommander,
    scene: moveit_commander.planning_scene_interface.PlanningSceneInterface,
    object_pose: geometry_msgs.msg.PoseStamped,
):
    """
    Execute a subroutine for picking an object.
    """

    ## 1: Increase velocity & accel. scaling (to 75%)
    arm.set_max_acceleration_scaling_factor(0.75)
    arm.set_max_velocity_scaling_factor(0.75)

    ## 2: Move arm to a pose goal above object (5 cm above the object centre)
    pre_grasp_pose = deepcopy(object_pose.pose)
    pre_grasp_pose.position.z += 0.05
    arm.set_pose_target(pre_grasp_pose)
    arm.go(wait=True)

    ## 3: Open the gripper
    gripper.go(gripper.get_named_target_values("open"), wait=True)
    rospy.sleep(rospy.Duration(secs=2))

    ## 4: Reduce Cartesian speed (2 cm/s)
    arm.limit_max_cartesian_link_speed(0.02)

    ## 5: Descend arm via a Cartesian path (with TCP 2 cm above the object centre)
    grasp_pose = deepcopy(object_pose.pose)
    grasp_pose.position.z += 0.02
    (plan, fraction) = arm.compute_cartesian_path([grasp_pose], 0.005, 0.0)
    if fraction > 0.8:
        arm.execute(plan, wait=True)
    else:
        print(
            f"Error: grasp - Unable to plan the full Cartesian path! Only {100*fraction}% of the path is feasible.",
            file=sys.stderr,
        )
        exit(1)

    ## Optional: Attach object
    ## Note: Attaching the object enables to specify touch links, i.e. links that won't be checked for collisions. For this reason, it is beneficial to attach the object before closing the gripper
    scene.attach_box(
        link="phantomx_pincher_end_effector",
        name="object",
        touch_links=[
            "phantomx_pincher_gripper_finger1_link",
            "phantomx_pincher_gripper_finger2_link",
        ],
    )

    ## 6: Close the gripper (almost closed)
    gripper.go(gripper.get_named_target_values("closed"), wait=True)

    ## 7: Ascend arm via a Cartesian path (re-use the pre-grasp pose)
    post_grasp_pose = pre_grasp_pose
    (plan, fraction) = arm.compute_cartesian_path([post_grasp_pose], 0.005, 0.0)
    if fraction > 0.8:
        arm.execute(plan, wait=True)
    else:
        print(
            f"Error: post_grasp - Unable to plan the full Cartesian path! Only {100*fraction}% of the path is feasible.",
            file=sys.stderr,
        )
        exit(1)


def place(
    arm: moveit_commander.move_group.MoveGroupCommander,
    gripper: moveit_commander.move_group.MoveGroupCommander,
    scene: moveit_commander.planning_scene_interface.PlanningSceneInterface,
    object_pose: geometry_msgs.msg.PoseStamped,
):
    """
    Execute a subroutine for placing an object.
    """

    ## 1: Decrease velocity & accel. scaling (to 50%)
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    ## 2: Move arm to a joint config. for placing (here we sample a random joint configuration)
    elbow_angle = random.uniform(pi / 32, pi / 6)
    pre_place_joint_configuration = [
        random.uniform(-pi / 6, pi / 6),
        elbow_angle,
        pi / 2 - elbow_angle,
        pi / 2,
    ]
    arm.go(pre_place_joint_configuration, wait=True)

    ## 3: Descend arm via a Cartesian path (use the current pose and the same Z coordinate that was used during the grasp)
    place_pose = arm.get_current_pose().pose
    place_pose.position.z = object_pose.pose.position.z + 0.02
    (plan, fraction) = arm.compute_cartesian_path([place_pose], 0.005, 0.0)
    if fraction > 0.8:
        arm.execute(plan, wait=True)
    else:
        print(
            f"Error: place - Unable to plan the full Cartesian path! Only {100*fraction}% of the path is feasible.",
            file=sys.stderr,
        )
        exit(1)

    ## 4: Open the gripper
    gripper.go(gripper.get_named_target_values("open"), wait=True)

    ## Optional: Detach object (planning scene)
    scene.remove_attached_object()

    ## 5: Increase Cartesian speed (4 cm/s)
    arm.limit_max_cartesian_link_speed(0.04)

    ## 6: Ascend arm via a Cartesian path (10 cm along +Z axis)
    post_place_pose = arm.get_current_pose().pose
    post_place_pose.position.z += 0.1
    # Replan the motion multiple times if it fails (for simulation, where the gripper can take longer to open)
    (plan, fraction) = arm.compute_cartesian_path([post_place_pose], 0.005, 0.0)
    if fraction > 0.8:
        arm.execute(plan, wait=True)
    else:
        print(
            f"Error: post_place - Unable to plan the full Cartesian path! Only {100*fraction}% of the path is feasible.",
            file=sys.stderr,
        )
        exit(1)

    ## 7: Close the gripper (partially)
    gripper.go([0.01] * 2, wait=True)


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

    # Instantiate PlanningSceneInterface (interface to the world surrounding the robot)
    scene = moveit_commander.planning_scene_interface.PlanningSceneInterface(
        synchronous=True
    )

    ### Setup the Planning Scene ###
    object_relative_to_frame_id = arm.get_planning_frame()
    object_size = (0.015, 0.015, 0.015)
    # Offset of the task board surface from the ground plane (2cm profile + 4mm task board plate + 3mm module cover)
    TASK_BOARD_SURFACE_OFFSET = 0.02 + 0.004 + 0.003
    object_pose = geometry_msgs.msg.PoseStamped(
        header=std_msgs.msg.Header(
            frame_id=object_relative_to_frame_id,
            stamp=rospy.Time.from_sec(rospy.get_time()),
        ),
        pose=geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(
                x=0.1,
                z=TASK_BOARD_SURFACE_OFFSET
                + object_size[2] / 2.0
                + 0.0025,  # +2.5mm tolerance to account for robot shakiness
            ),
            orientation=geometry_msgs.msg.Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
        ),
    )
    setup_planning_scene(scene=scene, object_pose=object_pose, object_size=object_size)

    # Increase gripper acceleration and velocity
    gripper.set_max_acceleration_scaling_factor(1.0)
    gripper.set_max_velocity_scaling_factor(1.0)

    while not rospy.is_shutdown():
        ### Pick ###
        pick(arm=arm, gripper=gripper, scene=scene, object_pose=object_pose)

        ### Place ###
        place(arm=arm, gripper=gripper, scene=scene, object_pose=object_pose)

        # Update the object pose (for repeated pick&place)
        __tmp_object_pos_z = object_pose.pose.position.z
        object_pose.pose = arm.get_current_pose().pose
        object_pose.pose.position.z = __tmp_object_pos_z

    # Ensure that there is no residual movement
    if not rospy.is_shutdown():
        arm.stop()

    # Shutdown MoveIt Commander and exit
    moveit_commander.roscpp_shutdown()
    exit(0)


if __name__ == "__main__":
    main()
