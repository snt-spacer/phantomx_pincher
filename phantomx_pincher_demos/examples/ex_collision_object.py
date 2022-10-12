#!/usr/bin/env python3
"""
Example of adding and removing a collision object with a mesh geometry.
`rosrun phantomx_pincher_demos ex_collision_object.py _action:="add" _position:="[0.25, 0.0, 0.1]" _quat_xyzw:="[0.0, 0.0, -0.7071, 0.7071]"`
`rosrun phantomx_pincher_demos ex_collision_object.py _action:="add" _filepath:="./my_favourity_mesh.stl"`
`rosrun phantomx_pincher_demos ex_collision_object.py _action:="remove"`
"""

import sys
from os import path

import geometry_msgs.msg
import moveit_commander
import rospy

DEFAULT_EXAMPLE_MESH = path.join(
    path.dirname(path.realpath(__file__)), "assets", "suzanne.stl"
)


def main():

    # Initialize MoveIt Commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create node for this example
    rospy.init_node("ex_collision_object", anonymous=True)

    # Instantiate MoveGroupCommander (interface to one group of joints)
    group = moveit_commander.move_group.MoveGroupCommander("arm")

    # Instantiate PlanningSceneInterface (interface to the world surrounding the robot)
    # Make sure the PlanningSceneInterface is configured by sleeping for a second
    scene = moveit_commander.planning_scene_interface.PlanningSceneInterface()
    rospy.sleep(1)

    # Get parameters for collision mesh manipulation
    filepath = rospy.get_param("~filepath", default="")
    action = rospy.get_param("~action", default="add")
    position = rospy.get_param("~position", default=[0.25, 0.0, 0.1])
    orientation = rospy.get_param("~quat_xyzw", default=[0.0, 0.0, -0.7071, 0.7071])
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = group.get_planning_frame()
    pose.header.stamp = rospy.Time.from_sec(rospy.get_time())
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    pose.pose.orientation.x = orientation[0]
    pose.pose.orientation.y = orientation[1]
    pose.pose.orientation.z = orientation[2]
    pose.pose.orientation.w = orientation[3]

    # Use the default example mesh if invalid
    if not filepath:
        rospy.loginfo(f"Using the default example mesh file")
        filepath = DEFAULT_EXAMPLE_MESH

    # Make sure the mesh file exists
    if not path.exists(filepath):
        rospy.logerror(f"File '{filepath}' does not exist")
        moveit_commander.roscpp_shutdown()
        exit(1)

    # Determine ID of the collision mesh
    mesh_id = path.basename(filepath).split(".")[0]

    if "add" == action:
        # Add collision mesh
        rospy.loginfo(f"Adding collision mesh '{filepath}' {{\n{pose}\n}}")
        scene.add_mesh(
            mesh_id,
            pose,
            filepath,
        )
    else:
        # Remove collision mesh
        rospy.loginfo(f"Removing collision mesh with ID '{mesh_id}'")
        scene.remove_world_object(mesh_id)

    # Shutdown MoveIt Commander and exit
    moveit_commander.roscpp_shutdown()
    exit(0)


if __name__ == "__main__":
    main()
