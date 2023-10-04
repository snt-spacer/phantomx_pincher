cmake_minimum_required(VERSION 3.5)
project(phantomx_pincher_demos)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(phantomx_pincher REQUIRED)

# Install directories
install(DIRECTORY launch rviz worlds examples/assets DESTINATION share/${PROJECT_NAME})

# Install examples
set(EXAMPLES_DIR examples)
install(PROGRAMS
    ${EXAMPLES_DIR}/demo_observe_scene.py
    ${EXAMPLES_DIR}/demo_peg_in_hole.py
    ${EXAMPLES_DIR}/demo_pick_and_place.py
    ${EXAMPLES_DIR}/ex_collision_mesh.py
    ${EXAMPLES_DIR}/ex_collision_primitive.py
    ${EXAMPLES_DIR}/ex_gripper.py
    ${EXAMPLES_DIR}/ex_joint_goal.py
    ${EXAMPLES_DIR}/ex_pose_goal.py
    ${EXAMPLES_DIR}/ex_servo.py
    ${EXAMPLES_DIR}/template.py
    DESTINATION lib/${PROJECT_NAME}
)

# Setup the project
ament_package()
