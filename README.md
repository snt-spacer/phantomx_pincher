# PhantomX Pincher

Software packages that enable manipulation with simulated and real [PhantomX Pincher Robot Arm](https://www.trossenrobotics.com/p/PhantomX-Pincher-Robot-Arm.aspx).

<p align="center" float="middle">
  <img width="75%" src="https://user-images.githubusercontent.com/22929099/192652586-916da2f5-9c05-402c-9b39-5452676fa32d.png"/>
</p>

## Overview

<p align="center">
  <a href="https://docs.ros.org/en/galactic">
    <img src="https://img.shields.io/badge/Middleware-ROS%202%20Galactic-38469E"/>
  </a>
  <a href="https://gazebosim.org">
    <img src="https://img.shields.io/badge/Robotics%20Simulator-Gazebo%20Fortress-F58113"/>
  </a>
  <a href="https://moveit.ros.org">
    <img src="https://img.shields.io/badge/Motion%20Planning-MoveIt%202-0A58F7"/>
  </a>
</p>

This repository consists of the following packages. For more detailed information about each package, please see their corresponding `README.md`.

- [**phantomx_pincher**](./phantomx_pincher) – Metapackage with primary launch scripts
- [**phantomx_pincher_demos**](./phantomx_pincher_demos) – Examples of using this repository
- [**phantomx_pincher_description**](./phantomx_pincher_description) – URDF and SDF description of the robot
- [**phantomx_pincher_moveit_config**](./phantomx_pincher_moveit_config) – MoveIt 2 configuration for the robot

<p align="center" float="middle">
  <img width="100%" src="https://github.com/snt-spacer/phantomx_pincher/assets/22929099/2e2b4bda-5271-4396-875b-299454754971"/>
</p>

## Instructions

### Installation

<details><summary><b>Option A – Local Installation</b></summary>

> Note: This option does not support using the real robot. Real robot requires ROS 1 controller and `ros1_bridge` to be run at the same time. Please, see [Option B – Docker Installation](#-option-b--docker-installation) for more information.

#### **Dependencies**

These are the primary dependencies required to use this project.

- ROS 2 [Galactic](https://docs.ros.org/en/galactic/Installation.html)
- Gazebo [Fortress](https://gazebosim.org/docs/fortress)

All additional dependencies are either pulled via [vcstool](https://wiki.ros.org/vcstool) ([phantomx_pincher.repos](./phantomx_pincher.repos)) or installed via [rosdep](https://wiki.ros.org/rosdep) during the building process below.

#### **Building**

Clone this repository, import dependencies, install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository into your favourite ROS 2 workspace
git clone https://github.com/snt-spacer/phantomx_pincher.git
# Import dependencies
vcs import < phantomx_pincher/phantomx_pincher.repos
# Install dependencies
IGNITION_VERSION=fortress rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
# Build
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

#### **Sourcing**

Before utilising this package, remember to source the ROS 2 workspace.

```bash
source install/local_setup.bash
```

This enables:

- Execution of binaries, scripts and examples via `ros2 run phantomx_pincher* <executable>`
- Launching of setup scripts via `ros2 launch phantomx_pincher* <launch_script>`
- Discoverability of shared resources

</details>

<details open><summary><b>Option B – Using Docker</b></summary>

### <a href="#-docker"><img src="https://www.svgrepo.com/show/448221/docker.svg" width="16" height="16"></a> Docker

> To install [Docker](https://docs.docker.com/get-docker) on your system, you can run [`install_docker.bash`](.docker/host/install_docker.bash) to configure Docker with NVIDIA GPU support.
>
> ```bash
> .docker/host/install_docker.bash
> ```

#### Build Image

To build a new Docker image from [Dockerfile](Dockerfile), you can run [`build.bash`](.docker/build.bash) as shown below.

```bash
.docker/build.bash ${TAG:-ros2} ${BUILD_ARGS}
```

#### Run Container

To run the Docker container, you can use [`run.bash`](.docker/run.bash) as shown below.

```bash
.docker/run.bash ${TAG:-ros2} ${CMD}
```

For convenience, additional three run scripts are available for directly using a fake (visual-only), simulated and real robot.

```bash
# Fake control inside RViz2
.docker/run_visual.bash
# Simulated control inside Gazebo
.docker/run_sim.bash
# Real control
.docker/run_real.bash
```

#### Run Dev Container

VS Code users familiar with [Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers) can modify the included [`devcontainer.json`](.devcontainer/devcontainer.json) to their needs. For convenience, [`open.bash`](.devcontainer/open.bash) script is available to open this repository as a Dev Container in VS Code.

```bash
.devcontainer/open.bash
```

#### Join Container

To join a running Docker container from another terminal, you can use [`join.bash`](.docker/join.bash) as shown below.

```bash
.docker/join.bash ${CMD:-bash}
```

</details>

### Getting Started

In order to get started, control a fake or simulated via the Motion Planning interface in RViz2.

```bash
# Fake control inside RViz2
ros2 launch phantomx_pincher fake.launch.py
# Simulated control inside Gazebo
ros2 launch phantomx_pincher gz.launch.py
```

The control of the real robot with ROS 2 is accomplished via [`ros1_bridge`](https://github.com/ros2/ros1_bridge) using the ROS 1 controller from [`ros1` branch](https://github.com/snt-spacer/phantomx_pincher/tree/ros1) of this repository. The `ros1_bridge` must be run separately and bridge at least `/joint_states` topic and \[`/joint_trajectory_controller/follow_joint_trajectory`, `/gripper_action_controller/gripper_cmd`\] actions. For convenience, [Docker Compose](./docker-compose.yml) setup that automatically runs everything needed to control the real robot is available. For simplicity, you can get started with the help of [`run_real.bash`](.docker/run_real.bash) script as already shown above.
