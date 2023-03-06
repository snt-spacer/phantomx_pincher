#!/usr/bin/env bash

if which wget >/dev/null; then
    echo "Installing wget because it has not been detected...."
    sudo apt-get update && sudo apt-get install -y wget
fi

# Install Docker
echo "Installing Docker..."
if [[ ! -x "$(command -v docker)" ]]; then
    wget https://get.docker.com -O - -o /dev/null | sh &&
    sudo systemctl --now enable docker
fi

# Install support for NVIDIA (Container Toolkit or Docker depending on Docker version)
LS_HW_DISPLAY=$(lshw -C display 2> /dev/null | grep vendor)
if [[ ${LS_HW_DISPLAY^^} =~ NVIDIA ]]; then
    echo "Installling Docker runtime for NVIDIA GPUs..."
    wget https://nvidia.github.io/nvidia-docker/gpgkey -O - -o /dev/null | sudo apt-key add - && wget "https://nvidia.github.io/nvidia-docker/$(source /etc/os-release && echo "${ID}${VERSION_ID}")/nvidia-docker.list" -O - -o /dev/null | sed "s#deb https://#deb [arch=$(dpkg --print-architecture)] https://#g" | sudo tee /etc/apt/sources.list.d/nvidia-docker.list >/dev/null
    sudo apt-get update
    if dpkg --compare-versions "$(sudo docker version --format '{{.Server.Version}}')" gt "19.3"; then
        # With Docker 19.03, nvidia-docker2 is deprecated since NVIDIA GPUs are natively supported as devices in the Docker runtime
        echo "Installing NVIDIA Container Toolkit"
        sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
    else
        echo "Installing NVIDIA Docker (Docker 19.03 or older)"
        sudo apt-get update && sudo apt-get install -y nvidia-docker2
    fi
    sudo systemctl restart docker
fi

# (Optional) Add user to docker group
[ -z "${PS1}" ] && read -erp "Do you want to add user ${USER} to the docker group? [Y/n]: " ADD_USER_TO_DOCKER_GROUP
if [[ "${ADD_USER_TO_DOCKER_GROUP,,}" =~ (y|yes) && ! "${ADD_USER_TO_DOCKER_GROUP,,}" =~ (n|no) ]]; then
    sudo groupadd -f docker && sudo usermod -aG docker "${USER}" && echo -e "User ${USER} was added to the docker group.\nPlease relog or execute the following command for changes to take affect.\n\tnewgrp docker"
fi
