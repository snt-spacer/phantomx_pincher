# phantomx_pincher_moveit_config

MoveIt configuration for PhantomX Pincher Robot Arm.

## Instructions

### SRDF

For SRDF, [phantomx_pincher.srdf.xacro](./srdf/phantomx_pincher.srdf.xacro) is the primary descriptor that includes all other xacros and creates a configuration based on the passed arguments. To generate SRDF out of xacro, you can use the included [xacro2srdf.bash](./scripts/xacro2srdf.bash) script and modify its arguments as needed. Once executed, [phantomx_pincher.srdf](./srdf/phantomx_pincher.srdf) will automatically be replaced. Alternatively, `xacro phantomx_pincher.srdf.xacro name:="phantomx_pincher" ...` can be executed directly, e.g. this is preferred within any launch script.

### move_group

In order to configure and setup `move_group` of MoveIt 2 to plan motions inside a simulation, [move_group.launch.py](./launch/move_group.launch.py) script can be launched or included in another launch script.

```bash
ros2 launch phantomx_pincher_moveit_config move_group.launch.py
```

To see all arguments, please use `ros2 launch --show-args phantomx_pincher_moveit_config move_group.launch.py`.

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── config/                                   # [dir] Configuration files for MoveIt 2
    ├── controllers_*.yaml                    # Configuration of ROS 2 controllers for different command interfaces
    ├── joint_limits.yaml                     # List of velocity and acceleration joint limits
    ├── kinematics.yaml                       # Configuration for the kinematic solver
    ├── moveit_controller_manager_*.yaml      # List of controllers with their type and action namespace for use with MoveIt 2
    ├── ompl_planning.yaml                    # Configuration of OMPL planning and specific planners
    └── servo.yaml                            # Configuration for moveit_servo
├── launch/                                   # [dir] ROS 2 launch scripts
    ├── move_group.launch.py                  # Launch script for configuring and setting up move_group of MoveIt 2
    └── move_group_external_control.launch.py # Launch script for configuring and setting up move_group of MoveIt 2 but with external control
├── rviz/moveit.rviz                          # RViz2 config for motion planning with MoveIt 2
├── scripts/                                  # [dir] Additional useful scripts
├── srdf/                                     # [dir] SRDF description (xacros)
├── CMakeLists.txt                            # Colcon-enabled CMake recipe
└── package.xml                               # ROS 2 package metadata
```
