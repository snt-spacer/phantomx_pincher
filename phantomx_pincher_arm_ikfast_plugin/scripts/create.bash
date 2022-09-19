#!/usr/bin/env bash
# This script updates the ikfast plugin for phantomx_pincher

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPO_DIR="$(dirname "$(dirname "${SCRIPT_DIR}")")"

ROBOT_NAME="phantomx_pincher"
PLANNING_GROUP_NAME="arm"
BASE_LINK_NAME="${ROBOT_NAME}_arm_base_link"
EEF_LINK_NAME="${ROBOT_NAME}_end_effector"
IKTYPE="TranslationZAxisAngle4D"

ros2 run moveit_kinematics auto_create_ikfast_moveit_plugin.sh \
    --iktype ${IKTYPE} \
    "${REPO_DIR}/${ROBOT_NAME}_description/urdf/${ROBOT_NAME}.urdf" \
    "${PLANNING_GROUP_NAME}" \
    "${BASE_LINK_NAME}" \
    "${EEF_LINK_NAME}"
