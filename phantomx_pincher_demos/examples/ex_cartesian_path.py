#!/usr/bin/env python3
"""
Example of moving to a pose goal via Cartesian path.
`rosrun phantomx_pincher_demos ex_cartesian_path.py _position:="[0.075, 0.0, 0.05]" _quat_xyzw:="[1.0, 0.0, 0.0, 0.0]"`
"""


import copy
import sys

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy


def main():

    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create node for this example
    rospy.init_node("ex_cartesian_path", anonymous=True)

    # Instantiate RobotCommander (outer-level interface to the robot)
    robot = moveit_commander.robot.RobotCommander()

    # Instantiate MoveGroupCommander (interface to one group of joints)
    group = moveit_commander.move_group.MoveGroupCommander("arm")

    # Create DisplayTrajectory publisher used to publish trajectories for RViz to visualize
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=1,
    )

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
        f"Moving via Cartesian path from {{\n{group.get_current_pose().pose}\n}} to {{\n{pose_goal}\n}}"
    )

    # Define waypoints through which the robot's end-effector should move
    waypoints = []
    waypoints.append(copy.deepcopy(pose_goal))

    # Decrease maximum Cartesian speed
    group.limit_max_cartesian_link_speed(0.025)

    # Generate Cartesian path via interpolation at a resolution of 5 mm
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)

    if fraction < 1.0:
        rospy.logwarn(f"{100*fraction:.1f}% of the Cartesian path is feasible")
        moveit_commander.roscpp_shutdown()
        exit(1)

    # Display the trajectory
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)

    # Execute the motion (wait until completion)
    group.execute(plan, wait=True)

    # Ensure that there is no residual movement
    group.stop()

    # Clear pose targets
    group.clear_pose_targets()

    # Shutdown MoveIt Commander and exit
    moveit_commander.roscpp_shutdown()
    exit(0)


if __name__ == "__main__":
    main()
