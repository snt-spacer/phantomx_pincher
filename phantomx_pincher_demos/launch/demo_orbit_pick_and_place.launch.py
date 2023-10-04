#!/usr/bin/env -S ros2 launch
"""Demo within Gazebo to demonstrate pick&place task in orbit"""

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # List of included launch descriptions
    launch_descriptions = [
        # Launch move_group of MoveIt 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("phantomx_pincher"),
                        "launch",
                        "gz.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("spawn_robot", "false"),
                (
                    "world",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("phantomx_pincher_demos"),
                            "worlds",
                            "orbit_pick_and_place.sdf",
                        ]
                    ),
                ),
                ("rviz_config", rviz_config),
            ],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        Node(
            package="phantomx_pincher_demos",
            executable="demo_pick_and_place.py",
            output="log",
            arguments=[
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("phantomx_pincher_demos"),
                "rviz",
                "demo_orbit_pick_and_place.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="error",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
