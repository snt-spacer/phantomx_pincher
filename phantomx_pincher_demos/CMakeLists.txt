cmake_minimum_required(VERSION 3.5)
project(phantomx_pincher_demos)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(phantomx_pincher REQUIRED)

# # Install directories
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install examples
set(EXAMPLES_DIR examples)
install(PROGRAMS
    ${EXAMPLES_DIR}/ex_collision_object.py
    ${EXAMPLES_DIR}/ex_gripper_command.py
    ${EXAMPLES_DIR}/ex_gripper.py
    ${EXAMPLES_DIR}/ex_joint_goal.py
    ${EXAMPLES_DIR}/ex_pose_goal.py
    DESTINATION lib/${PROJECT_NAME}
)

# Setup the project
ament_package()
