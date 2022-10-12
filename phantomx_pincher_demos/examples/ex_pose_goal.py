#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`rosrun phantomx_pincher_demos ex_pose_goal.py _position:="[0.075, 0.0, 0.05]" _quat_xyzw:="[1.0, 0.0, 0.0, 0.0]"`
"""


import sys

import geometry_msgs.msg
import moveit_commander
import rospy


def main():

    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create node for this example
    rospy.init_node("ex_pose_goal", anonymous=True)

    # Instantiate MoveGroupCommander (interface to one group of joints)
    group = moveit_commander.move_group.MoveGroupCommander("arm")

    # Get parameters for position and orientation
    position_goal = rospy.get_param("~position", default=[0.075, 0.0, 0.05])
    orientation_goal = rospy.get_param("~quat_xyzw", default=[1.0, 0.0, 0.0, 0.0])
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = position_goal[0]
    pose_goal.position.y = position_goal[1]
    pose_goal.position.z = position_goal[2]
    pose_goal.orientation.x = orientation_goal[0]
    pose_goal.orientation.y = orientation_goal[1]
    pose_goal.orientation.z = orientation_goal[2]
    pose_goal.orientation.w = orientation_goal[3]

    # Log information about the motion
    rospy.loginfo(
        f"Moving from {{\n{group.get_current_pose().pose}\n}} to {{\n{pose_goal}\n}}"
    )

    # Plan and execute the motion (wait until completion)
    group.set_pose_target(pose_goal)
    group.go(wait=True)

    # Ensure that there is no residual movement
    group.stop()

    # Clear pose targets
    group.clear_pose_targets()

    # Shutdown MoveIt Commander and exit
    moveit_commander.roscpp_shutdown()
    exit(0)


if __name__ == "__main__":
    main()
