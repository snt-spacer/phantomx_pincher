#!/usr/bin/env python3

"""
The Parallel Gripper gap calculation was adapted from https://github.com/vanadiumlabs/arbotix_ros/pull/28
This part of code is licensed under the following license:

  Copyright (c) 2014 Vanadium Labs LLC.  All right reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holder nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THEY BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

#  Parallel Gripper gap calculation:
#
#        o           (S) is servo axis
#       /:\    ||      R is radius of servo horn
#    R / :  \C ||      C is connector to finger
#     /  :x   \||      Offset is for foam and offset from connection back to finger
#    /a  :      \
#  (S). . . . . . o
#      n    y  ||
#              || <-- offset
#              ||finger


from math import acos, cos, sin, sqrt

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def widthToAngle(width):
    """Convert width to servo angle"""
    leg = width + offset  # Add offset
    # Law of Cosines
    return -1 * acos(
        (radius * radius + leg * leg - connector * connector) / (2 * radius * leg)
    )


def angleToWidth(ang):
    """Convert angle to width for this gripper"""
    n = cos(ang) * radius  # CAH
    x = sin(ang) * radius  # SOH
    y = sqrt(connector * connector - x * x)  # Pythagorean
    return n + y - offset  # Remove offset


"""
End of the code adapted from arbotix_ros
"""


def velocityRevoluteToPrismatic(ang, vel):
    """Convert velocity"""
    n = cos(ang) * radius
    x = sin(ang) * radius
    y = sqrt(connector * connector - x * x)
    der = -1 * ((n * x) / y) - x  # Derivative of angleToWidth
    return vel * der


def joint_cb(msg):
    for i, name in enumerate(msg.name):
        if name == source_joint:
            new_msg = JointState()
            new_msg.header.stamp = msg.header.stamp
            new_msg.name = [target_joint, mimic_joint]

            ang = msg.position[i]
            vel = msg.velocity[i]

            new_msg.position = [angleToWidth(ang)] * 2
            new_msg.velocity = [velocityRevoluteToPrismatic(ang, vel)] * 2

            joint_pub.publish(new_msg)


def command_cb(msg):
    width = max(min_width, min(max_width, msg.data))
    angle = widthToAngle(width)
    new_msg = Float64(angle)
    command_pub.publish(new_msg)


rospy.init_node("gripper_transmission")

source_joint = rospy.get_param("~source_joint")
target_joint = rospy.get_param("~target_joint")
mimic_joint = rospy.get_param("~mimic_joint")
radius = rospy.get_param("~radius")
connector = rospy.get_param("~connector")
offset = rospy.get_param("~offset")
min_width = rospy.get_param("~min_width")
max_width = rospy.get_param("~max_width")

joint_pub = rospy.Publisher("joint_states", JointState, queue_size=5)
command_pub = rospy.Publisher(source_joint + "/command", Float64, queue_size=5)

joint_sub = rospy.Subscriber("joint_states", JointState, joint_cb, queue_size=5)
command_sub = rospy.Subscriber(
    target_joint + "/command", Float64, command_cb, queue_size=5
)

rospy.spin()
