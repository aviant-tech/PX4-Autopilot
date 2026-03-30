#!/bin/bash

set -euxo pipefail

DOCKER_USER_FLAG=""
if [[ ! $(docker info | grep "Docker Root Dir") =~ "/var/lib/docker" ]]; then
    # rootless docker, needs user flag
    DOCKER_USER_FLAG="--user=root"
    if [[ $(id -u) != 0 ]]; then
        # TODO make it runnable without network=host
        echo "ERROR: Must be root for --network=host"
        exit 1
    fi
fi

DIR_OF_THIS_SCRIPT="$(dirname "$(realpath "$0")")"

# We need to mount the source dir with the same path as on the host machine
# in order to get correct paths in compile_commands.json
HOST_PX4_DIR="$(dirname "$(dirname "$DIR_OF_THIS_SCRIPT")")"

# verify correct directory
if [[ ! -d "$HOST_PX4_DIR/src" ]]; then
    echo "ERROR: failed to detect px4 directory: $HOST_PX4_DIR has no src"
    exit 1
fi

docker build -t aviantsitl -f "${DIR_OF_THIS_SCRIPT}/sitl.Dockerfile" .

SITL_SUFFIX=""
WIND_ENV="--env=PX4_SITL_WORLD=windy"
for arg in "$@"; do
    case "$arg" in
        debug) SITL_SUFFIX="_gdb" ;;
        nowind)  WIND_ENV="" ;;
    esac
done

SITL_CMD="make px4_sitl gazebo_standard_vtol$SITL_SUFFIX"

#--init ensures SIGINT is passed to all child processes
docker run -it --rm --init \
    --network=host \
    --env=HEADLESS=1 \
    --env=PX4_SIM_SPEED_FACTOR=3 \
    ${WIND_ENV} \
    --volume="${HOST_PX4_DIR}:${HOST_PX4_DIR}" \
    --workdir="${HOST_PX4_DIR}" \
    ${DOCKER_USER_FLAG} \
    aviantsitl "$SITL_CMD"
