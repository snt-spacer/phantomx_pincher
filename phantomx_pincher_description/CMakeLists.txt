cmake_minimum_required(VERSION 3.5)
project(phantomx_pincher_description)

# Find dependencies
find_package(catkin REQUIRED)

# Setup the project
catkin_package()

set(SCRIPTS_DIR scripts)
install(PROGRAMS
    ${SCRIPTS_DIR}/xacro2sdf_direct.bash
    ${SCRIPTS_DIR}/xacro2sdf.bash
    ${SCRIPTS_DIR}/xacro2urdf.bash
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install directories
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
