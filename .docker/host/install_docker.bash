#!/usr/bin/env bash
set -e

## Install curl if missing
if ! command -v curl >/dev/null 2>&1; then
    if command -v apt-get >/dev/null 2>&1; then
        sudo apt-get install -y curl
    elif command -v dnf >/dev/null 2>&1; then
        sudo dnf install -y curl
    elif command -v yum >/dev/null 2>&1; then
        sudo yum install -y curl
    fi
fi

## Install Docker via the convenience script
curl -fsSL https://get.docker.com | sh
sudo systemctl enable --now docker

## (Optional) Install support for NVIDIA if an NVIDIA GPU is detected and the installation is requested
check_nvidia_gpu() {
    if ! lshw -C display 2>/dev/null | grep -qi "vendor.*nvidia"; then
        return 1 # NVIDIA GPU is not present
    elif ! command -v nvidia-smi >/dev/null 2>&1; then
        return 1 # NVIDIA GPU is present but nvidia-utils not installed
    elif ! nvidia-smi -L &>/dev/null; then
        return 1 # NVIDIA GPU is present but is not working properly
    else
        return 0 # NVIDIA GPU is present and appears to be working
    fi
}
configure_nvidia_apt_repository() {
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey |
    sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg &&
    curl -sL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list |
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' |
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    sudo apt-get update
}
if check_nvidia_gpu; then
    read -erp "Do you want to install support for NVIDIA GPU inside Docker containers? [Y/n]: " INSTALL_NVIDIA_SUPPORT
    if ! command -v nvidia-container-toolkit >/dev/null 2>&1; then
        if [[ "${INSTALL_NVIDIA_SUPPORT,,}" =~ (y|yes) && ! "${INSTALL_NVIDIA_SUPPORT,,}" =~ (n|no) ]]; then
            DOCKER_VERSION="$(sudo docker version --format '{{.Server.Version}}' 2>/dev/null)"
            MIN_VERSION_FOR_TOOLKIT="19.3"
            if [ "$(printf '%s\n' "${MIN_VERSION_FOR_TOOLKIT}" "${DOCKER_VERSION}" | sort -V | head -n1)" = "$MIN_VERSION_FOR_TOOLKIT" ]; then
                if ! command -v nvidia-container-toolkit >/dev/null 2>&1; then
                    if command -v apt-get >/dev/null 2>&1; then
                        configure_nvidia_apt_repository
                        sudo apt-get install -y nvidia-container-toolkit
                    elif command -v yum >/dev/null 2>&1; then
                        curl -s -L https://nvidia.github.io/libnvidia-container/stable/rpm/nvidia-container-toolkit.repo |
                        sudo tee /etc/yum.repos.d/nvidia-container-toolkit.repo
                        sudo yum install -y nvidia-container-toolkit
                    else
                        echo >&2 -e "\033[1;31mERROR: Supported package manager not found. Please install nvidia-container-toolkit manually.\033[0m"
                    fi
                else
                    echo "NVIDIA Container Toolkit is already installed."
                fi
            else
                if ! command -v nvidia-docker >/dev/null 2>&1; then
                    if command -v apt-get >/dev/null 2>&1; then
                        configure_nvidia_apt_repository
                        sudo apt-get install -y nvidia-docker2
                        echo >&2 -e "\033[1;31mERROR: Supported package manager not found. Please install nvidia-docker2 manually.\033[0m"
                    fi
                else
                    echo "NVIDIA Docker is already installed."
                fi
            fi
            sudo systemctl restart --now docker
        fi
    fi
fi

if [[ $(grep /etc/group -e "docker") != *"${USER}"* ]]; then
    [ -z "${PS1}" ]
    ## (Optional) Add user to docker group
    read -erp "Do you want to add the current user ${USER} to the docker group? [Y/n]: " ADD_USER_TO_DOCKER_GROUP
    if [[ "${ADD_USER_TO_DOCKER_GROUP,,}" =~ (y|yes) && ! "${ADD_USER_TO_DOCKER_GROUP,,}" =~ (n|no) ]]; then
        sudo groupadd -f docker
        sudo usermod -aG docker "${USER}"
        echo -e "INFO: The current user ${USER} was added to the docker group. Please log out and log back in to apply the changes. Alternatively, run the following command to apply the changes in each new shell until you log out:\n\n\tnewgrp docker\n"
    else
        ## (Optional) Setup rootless mode if requested
        read -erp "Do you want to run the Docker daemon as a non-root user (rootless mode)? [y/N]: " SETUP_ROOTLESS_MODE
        if [[ "${SETUP_ROOTLESS_MODE,,}" =~ (y|yes) && ! "${SETUP_ROOTLESS_MODE,,}" =~ (n|no) ]]; then
            # Install uidmap dependency if missing
            if ! command -v newuidmap >/dev/null 2>&1; then
                if command -v apt-get >/dev/null 2>&1; then
                    sudo apt-get install -y uidmap
                elif command -v dnf >/dev/null 2>&1; then
                    sudo dnf install -y shadow-utils
                elif command -v yum >/dev/null 2>&1; then
                    sudo yum install -y shadow-utils
                fi
            fi
            sudo systemctl disable --now docker
            dockerd-rootless-setuptool.sh install --force
            sudo systemctl restart --now docker
        fi
    fi
fi
