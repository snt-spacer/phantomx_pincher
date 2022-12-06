#!/usr/bin/env python3
"""
Template for controlling PhantomX Pincher with your Python scripts.
"""


import sys

import geometry_msgs.msg
import moveit_commander
import rospy

# Constants for adding collision geometry of the blank module cover
MODULE_COVER_FRAME_ID = "phantomx_pincher_task_board_module_link"
MODULE_COVER_BOX_SIZE = (0.148, 0.148, 0.003)
MODULE_COVER_POSITION = geometry_msgs.msg.Point(z=MODULE_COVER_BOX_SIZE[2] / 2.0)


def main():

    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create node for this example
    rospy.init_node("phantomx_pincher_template", anonymous=True)

    # Instantiate RobotCommander (outer-level interface to the robot)
    robot = moveit_commander.robot.RobotCommander()

    # Instantiate MoveGroupCommander (interface to one group of joints) for arm and gripper
    arm = moveit_commander.move_group.MoveGroupCommander("arm")
    gripper = moveit_commander.move_group.MoveGroupCommander("gripper")

    # Instantiate PlanningSceneInterface (interface to the world surrounding the robot)
    scene = moveit_commander.planning_scene_interface.PlanningSceneInterface(
        synchronous=True
    )

    ##
    # TODO: Make your changes here (setup the scene, then plan and execute motions)
    ##

    # Ensure that there is no residual movement
    if not rospy.is_shutdown():
        arm.stop()

    # Shutdown MoveIt Commander and exit
    moveit_commander.roscpp_shutdown()
    exit(0)


if __name__ == "__main__":
    main()
