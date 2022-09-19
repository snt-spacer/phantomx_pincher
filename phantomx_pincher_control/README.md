# phantomx_pincher_control

Driver for PhantomX Pincher Robot Arm.

## Instructions

### Driver

To run the driver, launch the [driver.launch](./launch/driver.launch) script.

```bash
roslaunch phantomx_pincher_control driver.launch
```

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── config/        # [dir] Configuration files for the driver
├── launch/        # [dir] ROS launch scripts
├── scripts/       # [dir] Scripts that enable usage of the gripper
├── CMakeLists.txt # Colcon-enabled CMake recipe
└── package.xml    # ROS package metadata
```
