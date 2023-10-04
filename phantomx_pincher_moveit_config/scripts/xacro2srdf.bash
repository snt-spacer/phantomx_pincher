#!/usr/bin/env bash
# This script converts xacro (SRDF variant) into SRDF for `phantomx_pincher_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/srdf/phantomx_pincher.srdf.xacro"
SRDF_PATH="$(dirname "${SCRIPT_DIR}")/srdf/phantomx_pincher.srdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=phantomx_pincher
    use_real_gripper:=true
)

# Remove old SRDF file
rm "${SRDF_PATH}" 2>/dev/null

# Process xacro into SRDF
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" "${@:1}" -o "${SRDF_PATH}" &&
echo "Created new ${SRDF_PATH}"
