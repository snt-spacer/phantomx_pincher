#!/usr/bin/env bash
# This script updates the ikfast plugin for phantomx_pincher

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
SRC_DIR="$(dirname "${SCRIPT_DIR}")/src"

ROBOT_NAME="phantomx_pincher"
ROBOT_NAME_IN_SRDF="${ROBOT_NAME}"
MOVEIT_CONFIG_PKG="${ROBOT_NAME}_moveit_config"
SRDF_FILENAME="${ROBOT_NAME}.srdf"
PLANNING_GROUP_NAME="arm"
BASE_LINK_NAME="${ROBOT_NAME}_arm_base_link"
EEF_LINK_NAME="${ROBOT_NAME}_end_effector"
IKFAST_PLUGIN_PKG="${ROBOT_NAME}_ikfast_plugin"
IKFAST_OUTPUT_PATH="${SRC_DIR}/${ROBOT_NAME}_arm_ikfast_solver.cpp"
SEARCH_MODE="OPTIMIZE_MAX_JOINT"

rosrun moveit_kinematics create_ikfast_moveit_plugin.py \
    --search_mode="${SEARCH_MODE}" \
    --srdf_filename="${SRDF_FILENAME}" \
    --robot_name_in_srdf="${ROBOT_NAME_IN_SRDF}" \
    --moveit_config_pkg="${MOVEIT_CONFIG_PKG}" \
    "${ROBOT_NAME}" \
    "${PLANNING_GROUP_NAME}" \
    "${IKFAST_PLUGIN_PKG}" \
    "${BASE_LINK_NAME}" \
    "${EEF_LINK_NAME}" \
    "${IKFAST_OUTPUT_PATH}"
