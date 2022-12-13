ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base

### Use bash by default
SHELL ["/bin/bash", "-c"]

### Define relevant directories
ARG WS_DIR=/root/ws
ENV WS_DIR=${WS_DIR}
ENV WS_SRC_DIR=${WS_DIR}/src
ENV WS_DEVEL_DIR=${WS_DIR}/devel
ENV WS_LOGS_DIR=${WS_DIR}/logs
WORKDIR ${WS_DIR}

### Install necessary tools
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    git \
    python3-catkin-tools \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

### Install Gazebo
ARG IGNITION_VERSION=fortress
ENV IGNITION_VERSION=${IGNITION_VERSION}
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    ignition-${IGNITION_VERSION} && \
    rm -rf /var/lib/apt/lists/*

### Import and install dependencies, then build these dependencies (not phantomx_pincher yet)
COPY ./phantomx_pincher.repos ${WS_SRC_DIR}/phantomx_pincher/phantomx_pincher.repos
RUN vcs import --shallow ${WS_SRC_DIR} < ${WS_SRC_DIR}/phantomx_pincher/phantomx_pincher.repos && \
    rosdep update && \
    apt-get update && \
    rosdep install -y -r -i --rosdistro "${ROS_DISTRO}" --from-paths ${WS_SRC_DIR} && \
    rm -rf /var/lib/apt/lists/* && \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    catkin build --cmake-args "-DCMAKE_BUILD_TYPE=Release" && \
    rm -rf ${WS_LOGS_DIR}

### Copy over the rest of phantomx_pincher, then install dependencies and build
COPY ./ ${WS_SRC_DIR}/phantomx_pincher/
RUN rosdep update && \
    apt-get update && \
    rosdep install -y -r -i --rosdistro "${ROS_DISTRO}" --from-paths ${WS_SRC_DIR} && \
    rm -rf /var/lib/apt/lists/* && \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    catkin build --cmake-args "-DCMAKE_BUILD_TYPE=Release" && \
    rm -rf ${WS_LOGS_DIR}

### Download SDF models
RUN ${WS_SRC_DIR}/phantomx_pincher/.docker/utils/download_sdf_models.bash

### Add workspace to the ROS entrypoint
RUN sed -i '$i source "${WS_DEVEL_DIR}/setup.bash" --' /ros_entrypoint.sh && \
    ### Disable `set -e` from the ROS entrypoint
    sed -i '$i set +e' /ros_entrypoint.sh && \
    ### Source the ROS entrypoint inside `~/.bashrc` to enable autocompletion
    sed -i '$a source /ros_entrypoint.sh' ~/.bashrc
