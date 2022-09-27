#!/usr/bin/env python3
"""
Demo of repeatedly moving the robot to pre-defined joint configurations.
`rosrun phantomx_pincher_demos demo_observe_scene.py`
"""


import sys
from math import pi
from typing import List

import moveit_commander
import rospy

# List of joint positions that the robot should repeatadly reach
JOINT_POSITION_GOALS: List[List[float]] = [
    [-pi / 4, pi / 4, pi / 2, -pi / 5],
    [pi / 4, pi / 4, pi / 2, -pi / 5],
]


def main():

    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create node for this example
    rospy.init_node("demo_observe_scene", anonymous=True)

    # Instantiate MoveGroupCommander (interface to one group of joints)
    group = moveit_commander.MoveGroupCommander("arm")

    # Increase max velocity
    group.set_max_velocity_scaling_factor(0.42)

    # Move repeatedly to pre-defined joint configurations
    while not rospy.is_shutdown():
        for joint_position_goal in JOINT_POSITION_GOALS:
            group.go(joint_position_goal, wait=True)

    # Ensure that there is no residual movement
    group.stop()

    # Shutdown MoveIt Commander and exit
    moveit_commander.roscpp_shutdown()
    exit(0)


if __name__ == "__main__":
    main()
