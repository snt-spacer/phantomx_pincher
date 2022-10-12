#!/usr/bin/env python3
"""
Example of interacting with the gripper.
`rosrun phantomx_pincher_demos ex_gripper.py _action:="toggle"`
`rosrun phantomx_pincher_demos ex_gripper.py _action:="open"`
`rosrun phantomx_pincher_demos ex_gripper.py _action:="close"`
"""


import sys

import moveit_commander
import rospy


def main():

    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create node for this example
    rospy.init_node("ex_gripper", anonymous=True)

    # Instantiate MoveGroupCommander (interface to one group of joints)
    group = moveit_commander.move_group.MoveGroupCommander("gripper")

    # Get parameter for gripper action
    action = rospy.get_param("~action", default="toggle")

    # Log information about the motion
    rospy.loginfo(f'Performing gripper action "{action}"')

    # Plan and execute the motion depending on the action (wait until completion)
    if "open" == action:
        group.go(group.get_named_target_values("open"), wait=True)
    elif "close" == action:
        group.go(group.get_named_target_values("closed"), wait=True)
    else:
        while not rospy.is_shutdown():
            group.go(group.get_named_target_values("open"), wait=True)
            group.go(group.get_named_target_values("closed"), wait=True)

    # Ensure that there is no residual movement
    group.stop()

    # Shutdown MoveIt Commander and exit
    moveit_commander.roscpp_shutdown()
    exit(0)


if __name__ == "__main__":
    main()
