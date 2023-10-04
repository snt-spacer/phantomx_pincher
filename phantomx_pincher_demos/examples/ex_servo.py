#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
- ros2 run phantomx_pincher_demos ex_servo.py
"""


from math import cos, sin

import rclpy
from pymoveit2 import MoveIt2Servo
from pymoveit2.robots import phantomx_pincher
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

CIRCLE_RADIUS: float = 0.05
CONTROL_FREQUENCY: float = 0.1


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_servo")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 Servo interface
    moveit2_servo = MoveIt2Servo(
        node=node,
        frame_id=phantomx_pincher.base_link_name(),
        callback_group=callback_group,
    )

    def servo_circular_motion():
        """Move in a circular motion using Servo"""

        now_sec = node.get_clock().now().nanoseconds * 1e-9
        moveit2_servo(
            linear=(CIRCLE_RADIUS * sin(now_sec), 0.0, CIRCLE_RADIUS * cos(now_sec)),
            angular=(0.0, 0.0, 0.0),
        )

    # Create timer for moving in a circular motion
    node.create_timer(CONTROL_FREQUENCY, servo_circular_motion)

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
