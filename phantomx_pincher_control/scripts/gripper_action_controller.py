#!/usr/bin/env python3

"""
  Copyright (c) 2011-2014 Vanadium Labs LLC.
  Copyright (c) 2021 Kell Ideas Ltd.

  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from collections import deque
from statistics import mean

import actionlib
import rospy
from control_msgs.msg import GripperCommandAction
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class GripperController:
    def __init__(self):
        self.tolerance = rospy.get_param("~tolerance", 0.002)
        self.min_opening = rospy.get_param("~min_opening", 0.0)
        self.max_opening = rospy.get_param("~max_opening", 0.0316)
        self.joint = rospy.get_param("~joint", "gripper_finger1_joint")
        self.velocity_history_queue = rospy.get_param("~velocity_history_queue", 5)

        self.command_pub = rospy.Publisher(
            self.joint + "/command", Float64, queue_size=5
        )

        self.position = None
        self.velocity_history = deque(maxlen=self.velocity_history_queue)
        rospy.Subscriber("joint_states", JointState, self.stateCb)

        rospy.loginfo("Waiting for the initial gripper state")
        while self.position is None:
            rospy.sleep(0.1)
        rospy.loginfo("Initial gripper state received")

        self.server = actionlib.SimpleActionServer(
            "~gripper_cmd",
            GripperCommandAction,
            execute_cb=self.actionCb,
            auto_start=False,
        )
        self.server.start()

    def actionCb(self, goal):
        command = goal.command

        rospy.loginfo(
            "Gripper controller: Action goal received:%f m" % command.position
        )

        # Clip the command position to be within limits
        command.position = min(
            max(command.position, self.min_opening), self.max_opening
        )
        if goal.command.position != command.position:
            rospy.logwarn(
                "Gripper controller: Action goal clipped to:%f m" % command.position
            )

        self.command_pub.publish(command.position)
        self.velocity_history.clear()

        while True:
            if abs(self.position - command.position) < self.tolerance:
                rospy.loginfo(
                    "Done. Pos=%.5f Goal=%.5f Tol=%.5f",
                    self.position,
                    command.position,
                    self.tolerance,
                )
                self.server.set_succeeded()
                rospy.loginfo("Gripper Controller: Succeeded.")
                return

            if self.velocity_history_queue == len(self.velocity_history):
                if abs(mean(self.velocity_history)) < self.tolerance:
                    if self.position > command.position:
                        self.command_pub.publish(self.position - self.tolerance)
                    else:
                        self.command_pub.publish(self.position + self.tolerance)

                    self.server.set_succeeded()
                    rospy.logwarn(
                        "Gripper Controller: Object in the gripper detected (treating as succeeded)."
                    )
                    return

            if self.server.is_preempt_requested():
                self.server.set_preempted()
                rospy.loginfo(
                    "Done. Pos=%.5f Goal=%.5f Tol=%.5f",
                    self.position,
                    command.position,
                    self.tolerance,
                )
                rospy.logwarn("Gripper Controller: Preempted.")
                return

            rospy.sleep(0.01)

    def stateCb(self, msg):
        for i, name in enumerate(msg.name):
            if name == self.joint:
                self.position = msg.position[i] * 2
                self.velocity_history.append(msg.velocity[i])


if __name__ == "__main__":
    rospy.init_node("gripper_action_controller")
    server = GripperController()
    rospy.spin()
