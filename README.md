# PhantomX Pincher

Software packages that enable manipulation with simulated and real [PhantomX Pincher Robot Arm](https://www.trossenrobotics.com/p/PhantomX-Pincher-Robot-Arm.aspx).

<!-- <p align="left" float="middle">
  <a href="">
    <img width="50.0%" src=""/>
  </a>
  <em>PhantomX Pincher</em>
</p> -->

## Overview

<p align="center">
  <a href="http://wiki.ros.org/noetic">
    <img src="https://img.shields.io/badge/Middleware-ROS%20Noetic-38469E"/>
  </a>
  <a href="https://gazebosim.org">
    <img src="https://img.shields.io/badge/Robotics%20Simulator-Gazebo%20Fortress-F58113"/>
  </a>
  <a href="https://moveit.ros.org">
    <img src="https://img.shields.io/badge/Motion%20Planning-MoveIt-0A58F7"/>
  </a>
</p>

This repository consists of the following packages. For more detailed information about each package, please see their corresponding `README.md`.

- [**phantomx_pincher**](./phantomx_pincher) – Metapackage with primary launch scripts
- [**phantomx_pincher_demos**](./phantomx_pincher_demos) – Examples of using this repository
- [**phantomx_pincher_control**](./phantomx_pincher_control) – Driver for the real robot
- [**phantomx_pincher_description**](./phantomx_pincher_description) – URDF and SDF description of the robot
- [**phantomx_pincher_moveit_config**](./phantomx_pincher_moveit_config) – MoveIt configuration for the robot
- [**phantomx_pincher_arm_ikfast_plugin**](./phantomx_pincher_arm_ikfast_plugin) – IKFast plugin for closed-form kinematics

## Instructions

### Installation

<details><summary><b>Option A – Local Installation</b></summary>

#### **Dependencies**

These are the primary dependencies required to use this project.

- ROS [Noetic](http://wiki.ros.org/noetic/Installation)
- Gazebo [Fortress](https://gazebosim.org/docs/fortress)

All additional dependencies are either pulled via [vcstool](https://wiki.ros.org/vcstool) ([phantomx_pincher.repos](./phantomx_pincher.repos)) or installed via [rosdep](https://wiki.ros.org/rosdep) during the building process below.

#### **Building**

Clone this repository, import dependencies, install dependencies and build with [catkin tools](https://catkin-tools.readthedocs.io).

```bash
# Clone this repository into your favourite ROS workspace
git clone https://github.com/AndrejOrsula/phantomx_pincher.git
# Import dependencies
vcs import < phantomx_pincher/phantomx_pincher.repos
# Install dependencies
IGNITION_VERSION=fortress rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
# Build
catkin build --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

#### **Sourcing**

Before utilising this package, remember to source the ROS workspace.

```bash
source devel/local_setup.bash
```

This enables:

- Execution of binaries, scripts and examples via `rosrun phantomx_pincher* <executable>`
- Launching of setup scripts via `roslaunch phantomx_pincher* <launch_script>`
- Discoverability of shared resources

</details>

<details><summary><b>Option B – Docker</b></summary>

#### **Install Docker**

First, ensure your system has a setup for using Docker. You can follow the [`install_docker.bash`](./.docker/host/install_docker.bash) script.

```bash
# Execute script inside a cloned repository
.docker/host/install_docker.bash
# (Alternative) Execute script from URL
bash -c "$(wget -qO - https://raw.githubusercontent.com/snt-spacer/phantomx_pincher/ros1/.docker/host/install_docker.bash)"
```

#### **(Optional) Build a New Image**

A new Docker image can be built locally using the included [Dockerfile](./Dockerfile). To do this, you can run the [`build.bash`](./.docker/build.bash) script as shown below. This script will always print the corresponding low-level `docker build ...` command for your reference.

```bash
# Execute script inside a cloned repository
.docker/build.bash ${TAG:-ros1} ${BUILD_ARGS}
```

#### **Run a Docker Container**

Docker containers can be run with the included [`run.bash`](./.docker/run.bash) script as shown below. It automatically configures environment variables and volumes to enable GPU usage and GUI application. This script will always print the corresponding low-level `docker run ...` command for your reference.

```bash
# Execute script inside a cloned repository
.docker/run.bash ${TAG:-ros1} ${CMD}
# (Alternative) Execute script from URL
bash -c "$(wget -qO - https://raw.githubusercontent.com/snt-spacer/phantomx_pincher/ros1/.docker/run.bash)" -- ${TAG:-ros1} ${CMD}
```

For development, you can run the Docker container with the [`devel.bash`](./.docker/run.bash) script that also mounts the repository as a volume inside the container. In this way, you can modify all packages locally and directly execute them in their updated state inside the container.

```bash
# Execute script inside a cloned repository
.docker/devel.bash ${TAG:-ros1} ${CMD}
```

</details>

### Getting Started

In order to get started, control a fake, simulated or real robot via the Motion Planning interface in RViz.

```bash
# Fake control inside RViz
roslaunch phantomx_pincher fake.launch
# Simulated control inside Gazebo
roslaunch phantomx_pincher gz.launch
# Real control
roslaunch phantomx_pincher real.launch
```

Hereafter, you can experiment with examples of `phantomx_pincher_demos`.

```bash
rosrun phantomx_pincher_demos ...
```