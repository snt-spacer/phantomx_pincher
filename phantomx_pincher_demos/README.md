# phantomx_pincher_demos

Demos for PhantomX Pincher Robot Arm.

## Instructions

### Examples

Examples are simple scripts that perform a simple isolated functionality.

```bash
ros2 run phantomx_pincher_demos ex_*
```

All of these require MoveIt to be already launched in the background with either of these scripts.

```bash
# Fake control inside RViz
ros2 launch phantomx_pincher fake.launch.py
# Simulated control inside Gazebo
ros2 launch phantomx_pincher gz.launch.py
# Real control
ros2 launch phantomx_pincher real.launch.py
```

### Demos

Demos are more elaborate examples that include a launch script and a node that implements a specific functionality. These demos are standalone and do not require MoveIt to be launched in the background.

```bash
ros2 launch phantomx_pincher_demos demo_*
```

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── examples/      # Programmatic examples demonstrating the use of `phantomx_pincher`
├── launch/        # [dir] ROS 2 launch scripts
├── worlds/        # [dir] Gazebo worlds that are used in demos
├── CMakeLists.txt # Colcon-enabled CMake recipe
└── package.xml    # ROS 2 package metadata
```
