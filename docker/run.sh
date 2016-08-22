#!/bin/sh

USER_UID=$(id -u)
USER_GID=$(id -g)
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

OMNIMAPER_ROOT="$(dirname "$(pwd)")"
HOST_DESKTOP="/home/${USER}/Desktop"

docker run \
        -it --rm \
        --volume=$XSOCK:$XSOCK:rw \
        --volume=$XAUTH:$XAUTH:rw \
        --device /dev/nvidia0:/dev/nvidia0 \
        --device /dev/nvidiactl:/dev/nvidiactl \
        --env="XAUTHORITY=${XAUTH}" \
        --env="USER_UID=${USER_UID}" \
        --env="USER_GID=${USER_GID}" \
        --env="DISPLAY=${DISPLAY}" \
        -u autoware \
        autoware-image
