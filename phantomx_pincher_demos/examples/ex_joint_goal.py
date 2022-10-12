#!/usr/bin/env python3
"""
Example of moving to a joint configuration.
`rosrun phantomx_pincher_demos ex_joint_goal.py _joint_positions:="[0.0, 0.0, 1.57, 1.57]"`
"""


import sys
from math import pi

import moveit_commander
import rospy


def main():

    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create node for this example
    rospy.init_node("ex_joint_goal", anonymous=True)

    # Instantiate MoveGroupCommander (interface to one group of joints)
    group = moveit_commander.move_group.MoveGroupCommander("arm")

    # Get parameter for joint positions
    joint_position_goal = rospy.get_param(
        "~joint_positions", default=[0.0, 0.0, pi / 2, pi / 2]
    )

    # Log information about the motion
    rospy.loginfo(
        f"Moving from {{joint_positions: {group.get_current_joint_values()}}} to {{joint_positions: {joint_position_goal}}}"
    )

    # Plan and execute the motion (wait until completion)
    group.go(joint_position_goal, wait=True)

    # Ensure that there is no residual movement
    group.stop()

    # Shutdown MoveIt Commander and exit
    moveit_commander.roscpp_shutdown()
    exit(0)


if __name__ == "__main__":
    main()
