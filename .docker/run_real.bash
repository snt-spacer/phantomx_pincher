#!/usr/bin/env bash

## Configuration
# Flags for running the container
DOCKER_COMPOSE_UP_OPTS="${DOCKER_COMPOSE_UP_OPTS:-
    --detach
}"

## If the current user is not in the docker group, all docker commands will be run as root
if ! grep -qi /etc/group -e "docker.*${USER}"; then
    echo "INFO: The current user ${USER} is not detected in the docker group. All docker commands will be run as root."
    # shellcheck disable=all
    docker-compose() {
        sudo docker-compose "$@"
    }
fi

## Get path to the compose file
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
REPOSITORY_DIR="$(dirname "${SCRIPT_DIR}")"
COMPOSE_FILE="${REPOSITORY_DIR}/docker-compose.yml"

## Bring up all containers
DOCKER_COMPOSE_UP_CMD=(
    docker-compose
    --file "${COMPOSE_FILE}"
    up
    "${DOCKER_COMPOSE_UP_OPTS}"
)
echo -e "\033[1;30m${DOCKER_COMPOSE_UP_CMD[*]}\033[0m" | xargs
# shellcheck disable=SC2048
${DOCKER_COMPOSE_UP_CMD[*]}
IS_SUCCESS=$?

if [ "${IS_SUCCESS}" -eq 0 ]; then
    ## If susccessful, join the primary container
    "${SCRIPT_DIR}/join.bash"

    ## Once done with the interactive session, bring down all containers to clean up
    DOCKER_COMPOSE_DOWN_CMD=(
        docker-compose
        --file "${COMPOSE_FILE}"
        down
    )
    echo -e "\033[1;30m${DOCKER_COMPOSE_DOWN_CMD[*]}\033[0m" | xargs
    # shellcheck disable=SC2048
    ${DOCKER_COMPOSE_DOWN_CMD[*]}
fi
