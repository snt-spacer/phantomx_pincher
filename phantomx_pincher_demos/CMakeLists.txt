cmake_minimum_required(VERSION 3.5)
project(phantomx_pincher_demos)

# Find dependencies
find_package(catkin REQUIRED)
find_package(phantomx_pincher REQUIRED)

# Setup the project
catkin_package()

# Install directories
install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install Python scripts
set(EXAMPLES_DIR examples)
catkin_install_python(PROGRAMS
  ${EXAMPLES_DIR}/demo_observe_scene.py
  ${EXAMPLES_DIR}/ex_cartesian_path.py
  ${EXAMPLES_DIR}/ex_collision_object.py
  ${EXAMPLES_DIR}/ex_gripper.py
  ${EXAMPLES_DIR}/ex_joint_goal.py
  ${EXAMPLES_DIR}/ex_pose_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
