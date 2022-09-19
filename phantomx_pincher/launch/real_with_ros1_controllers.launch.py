#!/usr/bin/env -S ros2 launch
"""Example of planning with MoveIt2 and executing motions on the real robot using ROS 1 controllers via ros1_bridge (both must be run externally)"""
# TODO: Integrate with regular *_moveit_convig/launch/move_group.launch.py

from os import path
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    moveit_config_package = "phantomx_pincher_moveit_config"
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    collision = LaunchConfiguration("collision")
    ros2_control = LaunchConfiguration("ros2_control")
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")
    ros2_control_command_interface = LaunchConfiguration(
        "ros2_control_command_interface"
    )
    mimic_finger_joints = LaunchConfiguration("mimic_finger_joints")
    gazebo_preserve_fixed_joint = LaunchConfiguration("gazebo_preserve_fixed_joint")
    manage_controllers = LaunchConfiguration("manage_controllers")
    enable_servo = LaunchConfiguration("enable_servo")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # URDF
    _robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_filepath]
            ),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
            " ",
            "collision:=",
            collision,
            " ",
            "ros2_control:=",
            ros2_control,
            " ",
            "ros2_control_plugin:=",
            ros2_control_plugin,
            " ",
            "ros2_control_command_interface:=",
            ros2_control_command_interface,
            " ",
            "mimic_finger_joints:=",
            mimic_finger_joints,
            " ",
            "gazebo_preserve_fixed_joint:=",
            gazebo_preserve_fixed_joint,
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    # SRDF
    _robot_description_semantic_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(moveit_config_package),
                    "srdf",
                    "phantomx_pincher.srdf.xacro",
                ]
            ),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": _robot_description_semantic_xml
    }

    # Kinematics
    kinematics = load_yaml(
        moveit_config_package, path.join("config", "kinematics.yaml")
    )

    # Joint limits
    joint_limits = {
        "robot_description_planning": load_yaml(
            moveit_config_package, path.join("config", "joint_limits.yaml")
        )
    }

    # Servo
    servo_params = {
        "moveit_servo": load_yaml(
            moveit_config_package, path.join("config", "servo.yaml")
        )
    }
    servo_params["moveit_servo"].update({"use_gazebo": use_sim_time})
    # Update the name of the output topic
    servo_params["moveit_servo"].update(
        {"command_out_topic": "/joint_trajectory_controller/command"}
    )

    # Planning pipeline
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            # TODO: Re-enable `default_planner_request_adapters/AddRuckigTrajectorySmoothing` once its issues are resolved
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }
    _ompl_yaml = load_yaml(
        moveit_config_package, path.join("config", "ompl_planning.yaml")
    )
    planning_pipeline["ompl"].update(_ompl_yaml)

    # Planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # MoveIt controller manager
    moveit_controller_manager_yaml = load_yaml(
        moveit_config_package,
        # path.join("config", "moveit_controller_manager_gripper_trajectory.yaml"),
        path.join("config", "moveit_controller_manager_gripper_command.yaml"),
    )
    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": moveit_controller_manager_yaml,
    }

    # Trajectory execution
    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }

    # Controller parameters
    declared_arguments.append(
        DeclareLaunchArgument(
            "__controller_parameters_basename",
            default_value=["controllers_", ros2_control_command_interface, ".yaml"],
        )
    )
    controller_parameters = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package),
            "config",
            LaunchConfiguration("__controller_parameters_basename"),
        ]
    )

    # List of nodes to be launched
    nodes = [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                {
                    "publish_frequency": 100.0,
                    "frame_prefix": "",
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        # ros2_control_node (only for fake controller)
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                controller_parameters,
                {"use_sim_time": use_sim_time},
            ],
            condition=(
                IfCondition(
                    PythonExpression(
                        [
                            "bool('",
                            ros2_control,
                            "') and ",
                            "'",
                            ros2_control_plugin,
                            "'",
                            " == ",
                            "'fake'",
                        ]
                    )
                )
            ),
        ),
        # move_group
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                joint_limits,
                planning_pipeline,
                trajectory_execution,
                planning_scene_monitor_parameters,
                moveit_controller_manager,
                {"use_sim_time": use_sim_time},
            ],
        ),
        # move_servo
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                joint_limits,
                planning_pipeline,
                trajectory_execution,
                planning_scene_monitor_parameters,
                servo_params,
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(enable_servo),
        ),
        # rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=[
                "--display-config",
                rviz_config,
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                planning_pipeline,
                joint_limits,
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(enable_rviz),
        ),
    ]

    # Add nodes for loading controllers
    for controller in moveit_controller_manager_yaml["controller_names"] + [
        "joint_state_broadcaster"
    ]:
        nodes.append(
            # controller_manager_spawner
            Node(
                package="controller_manager",
                executable="spawner",
                output="log",
                arguments=[controller, "--ros-args", "--log-level", log_level],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(manage_controllers),
            ),
        )

    return LaunchDescription(declared_arguments + nodes)


def load_yaml(package_name: str, file_path: str):
    """
    Load yaml configuration based on package name and file path relative to its share.
    """

    package_path = get_package_share_directory(package_name)
    absolute_file_path = path.join(package_path, file_path)
    return parse_yaml(absolute_file_path)


def parse_yaml(absolute_file_path: str):
    """
    Parse yaml from file, given its absolute file path.
    """

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Locations of robot resources
        DeclareLaunchArgument(
            "description_package",
            default_value="phantomx_pincher_description",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=path.join("urdf", "phantomx_pincher.urdf.xacro"),
            description="Path to xacro or URDF description of the robot, relative to share of `description_package`.",
        ),
        # Naming of the robot
        DeclareLaunchArgument(
            "name",
            default_value="phantomx_pincher",
            description="Name of the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="phantomx_pincher_",
            description="Prefix for all robot entities. If modified, then joint names in the configuration of controllers must also be updated.",
        ),
        # Collision geometry
        DeclareLaunchArgument(
            "collision",
            default_value="true",
            description="Flag to enable collision geometry.",
        ),
        # ROS 2 control
        DeclareLaunchArgument(
            "ros2_control",
            default_value="false",
            description="Flag to enable ros2 controllers for manipulator.",
        ),
        DeclareLaunchArgument(
            "ros2_control_plugin",
            default_value="real",
            description="The ros2_control plugin that should be loaded for the manipulator ('fake', 'ign', 'real' or custom).",
        ),
        DeclareLaunchArgument(
            "ros2_control_command_interface",
            default_value="position",
            description="The output control command interface provided by ros2_control ('position', 'velocity', 'effort' or certain combinations 'position,velocity').",
        ),
        # Gripper
        DeclareLaunchArgument(
            "mimic_finger_joints",
            default_value="false",
            description="Flag to enable mimicking of the finger joints.",
        ),
        # Gazebo
        DeclareLaunchArgument(
            "gazebo_preserve_fixed_joint",
            default_value="false",
            description="Flag to preserve fixed joints and prevent lumping when generating SDF for Gazebo.",
        ),
        # Controller management
        DeclareLaunchArgument(
            "manage_controllers",
            default_value="false",
            description="Flag to enable spawning of controllers.",
        ),
        # Servo
        DeclareLaunchArgument(
            "enable_servo",
            default_value="true",
            description="Flag to enable MoveIt2 Servo for manipulator.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "enable_rviz", default_value="true", description="Flag to enable RViz2."
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("phantomx_pincher_moveit_config"),
                "rviz",
                "moveit.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
