#!/usr/bin/env bash
# This script converts xacro (URDF variant) into URDF for `phantomx_pincher_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/phantomx_pincher.urdf.xacro"
URDF_PATH="$(dirname "${SCRIPT_DIR}")/urdf/phantomx_pincher.urdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=phantomx_pincher
    collision:=true
    ros2_control:=true
    ros2_control_plugin:=fake
    ros2_control_command_interface:=position
    mimic_finger_joints:=false
    gazebo_preserve_fixed_joint:=false
)

# Remove old URDF file
rm "${URDF_PATH}" 2>/dev/null

# Process xacro into URDF
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" "${@:1}" -o "${URDF_PATH}" &&
echo "Created new ${URDF_PATH}"
