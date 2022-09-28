# PhantomX Pincher

Software packages that enable manipulation with simulated and real [PhantomX Pincher Robot Arm](https://www.trossenrobotics.com/p/PhantomX-Pincher-Robot-Arm.aspx).

<p align="center" float="middle">
  <img width="75%" src="https://user-images.githubusercontent.com/22929099/192652586-916da2f5-9c05-402c-9b39-5452676fa32d.png"/>
</p>

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

<details open><summary><b>Option A – Local Installation</b></summary>

#### **Dependencies**

These are the primary dependencies required to use this project. Please install them following their respective tutorial.

- ROS [Noetic](http://wiki.ros.org/noetic/Installation) (e.g. [Binary installation on Ubuntu 20.04](http://wiki.ros.org/noetic/Installation/Ubuntu))
- Gazebo [Fortress](https://gazebosim.org/docs/fortress/install) (e.g. [Binary installation on Ubuntu 20.04](https://gazebosim.org/docs/fortress/install_ubuntu))

All additional dependencies are either pulled via [vcstool](https://wiki.ros.org/vcstool) ([phantomx_pincher.repos](./phantomx_pincher.repos)) or installed via [rosdep](https://wiki.ros.org/rosdep) during the building process below.

#### **Building**

Clone this repository, import dependencies, install dependencies and build with [catkin tools](https://catkin-tools.readthedocs.io).

```bash
# Install essentials
sudo apt update && sudo apt install -y git python3-catkin-tools python3-vcstool
# Create a workspace
mkdir -p phantomx_pincher_ws/src && cd phantomx_pincher_ws
# Clone this repository into your favourite ROS workspace
git clone https://github.com/snt-spacer/phantomx_pincher.git -b ros1 src/phantomx_pincher
# Import dependencies
vcs import src < src/phantomx_pincher/phantomx_pincher.repos
# Specify which Gazebo version to use
export IGNITION_VERSION=fortress
# Install dependencies
rosdep install -y -r -i --rosdistro noetic --from-paths src
# Build
catkin build --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

#### Setup permissions for real robot

By default, the USB device connected to the real robot prevents read/write access of non-`sudo` users. In order to configure these permissions automatically when you plug the device into your system, you can setup the included [`50-phantomx-pincher.rules`](phantomx_pincher_control/udev/50-phantomx-pincher.rules) udev rules.

```bash
sudo cp src/phantomx_pincher/phantomx_pincher_control/udev/50-phantomx-pincher.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```

#### **Sourcing**

Before utilising this package, remember to source the ROS workspace. It can be convenient to automatically source this workspace every time a new shell is launched by placing it inside `~/.bashrc`.

```bash
source devel/setup.bash
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

Hereafter, you can experiment with examples and investigate the included demos inside [phantomx_pincher_demos](./phantomx_pincher_demos).

```bash
rosrun phantomx_pincher_demos ex_*
```
