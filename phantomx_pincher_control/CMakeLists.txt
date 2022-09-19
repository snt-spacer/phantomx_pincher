cmake_minimum_required(VERSION 3.5)
project(phantomx_pincher_control)

# Find dependencies
find_package(catkin REQUIRED)

# Setup the project
catkin_package()

# Install directories
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/gripper_action_controller.py
  scripts/gripper_transmission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
