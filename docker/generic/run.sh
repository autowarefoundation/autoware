#!/bin/sh

XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
SHARED_DIR=/home/$USER/shared_dir
HOST_DIR=/home/$USER/shared_dir

if [ "$1" = "kinetic" ] || [ "$1" = "indigo" ]
then
    echo "Use $1"
else
    echo "Select distribution, kinetic|indigo"
    exit
fi

if [ "$2" = "" ]
then
    # Create Shared Folder
    mkdir -p $SHARED_DIR
else
    HOST_DIR=$2
fi
echo "Shared directory: ${HOST_DIR}"

if nvidia-smi > /dev/null 2>&1; then # Detect if nvidia is used
  if dpkg -s nvidia-docker2 > /dev/null 2>&1; then # Detect if nvidia-docker v2 is used
    echo "Will run with nvidia acceleration (nvidia-docker v2)"
    DOCKER_CMD="docker"
    RUN_ARG="--runtime=nvidia"
    TAG_SUFFIX="-nvidia"
  else
    echo "Will run without nvidia acceleration (nvidia-docker v1 - deprecated)"
    DOCKER_CMD="nvidia-docker"
    RUN_ARG=""
    TAG_SUFFIX=""
  fi
else
  echo "Will run without nvidia acceleration"
  DOCKER_CMD="docker"
  RUN_ARG=""
  TAG_SUFFIX=""
fi

$DOCKER_CMD run $RUN_ARG \
    -it --rm \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    -u autoware \
    --privileged -v /dev/bus/usb:/dev/bus/usb \
    --net=host \
    autoware-${1}${TAG_SUFFIX}
