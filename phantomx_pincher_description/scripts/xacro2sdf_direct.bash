#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `phantomx_pincher_description` package

TMP_URDF_PATH=$(mktemp /tmp/phantomx_pincher_XXXXXX.urdf)

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
xacro "${1}" "${@:2}" -o "${TMP_URDF_PATH}" &&
SDF_XML=$(ign sdf -p "${TMP_URDF_PATH}" | sed "s/model:\/\/phantomx_pincher_description\///g")

# Remove temporary URDF file
rm "${TMP_URDF_PATH}" 2>/dev/null

# Return SDF as XML string
echo "${SDF_XML}"
