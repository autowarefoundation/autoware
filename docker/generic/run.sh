#!/bin/bash

usage() { echo "Usage $0 [-t <tag>] [-r <repo>] [-s <Shared directory>]" 1>&2; exit 1; }

# Defaults
XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
SHARED_DIR=/home/autoware/shared_dir
HOST_DIR=/home/$USER/shared_dir
DOCKER_HUB_REPO="autoware/autoware"
TAG="latest-kinetic"

while getopts ":t:r:s:" opt; do
  case $opt in
    t)
      TAG=$OPTARG
      echo "Using $TAG tag"
      ;;
    r )
      DOCKER_HUB_REPO=$OPTARG
      echo "Using $DOCKER_HUB_REPO repo"
      ;;
    s)
      HOST_DIR=$OPTARG
      echo "Shared directory: ${HOST_DIR}"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

nvidia-docker run \
    -it --rm \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    -u autoware \
    --privileged -v /dev/bus/usb:/dev/bus/usb \
    --net=host \
    $DOCKER_HUB_REPO:$TAG
