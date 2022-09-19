cmake_minimum_required(VERSION 3.5)
project(phantomx_pincher_moveit_config)

# Find dependencies
find_package(catkin REQUIRED)
find_package(phantomx_pincher_description REQUIRED)

# Setup the project
catkin_package()

# Install directories
install(DIRECTORY config launch rviz srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
