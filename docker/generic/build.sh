#!/bin/bash

# Default settings
CUDA="on"
IMAGE_NAME="autoware/autoware"
TAG_PREFIX="local"
ROS_DISTRO="kinetic"
BASE_ONLY="false"

function usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "    -b,--base-only        Build the base image(s) only."
    echo "                          Default:$BASE_ONLY"
    echo "    -c,--cuda <on|off>    Enable Cuda support in the Docker."
    echo "                          Default:$CUDA"
    echo "    -h,--help             Display the usage and exit."
    echo "    -i,--image <name>     Set docker images name."
    echo "                          Default:$IMAGE_NAME"
    echo "    -t,--tag-prefix <tag> Tag prefix use for the docker images."
    echo "                          Default:$TAG_PREFIX"
}

OPTS=`getopt --options bc:hi:t: \
         --long base-only,cuda:,help,image-name:,tag-prefix: \
         --name "$0" -- "$@"`
eval set -- "$OPTS"

while true; do
  case $1 in
    -b|--base-only)
      BASE_ONLY="true"
      shift 1
      ;;
    -c|--cuda)
      param=$(echo $2 | tr '[:upper:]' '[:lower:]')
      case "${param}" in
        "on"|"off") CUDA="${param}" ;;
        *) echo "Invalid cuda option: $2"; exit 1 ;;
      esac
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -i|--image-name)
      IMAGE_NAME="$2"
      shift 2
      ;;
    -t|--tag-prefix)
      TAG_PREFIX="$2"
      shift 2
      ;;
    --)
      if [ ! -z $2 ];
      then
        echo "Invalid parameter: $2"
        exit 1
      fi
      break
      ;;
    *)
      echo "Invalid option"
      exit 1
      ;;
  esac
done

echo "Using options:"
echo -e "\tROS distro: $ROS_DISTRO"
echo -e "\tImage name: $IMAGE_NAME"
echo -e "\tTag prefix: $TAG_PREFIX"
echo -e "\tCuda support: $CUDA"
echo -e "\tBase only: $BASE_ONLY"

BASE=$IMAGE_NAME:$TAG_PREFIX-$ROS_DISTRO-base

docker build \
    --rm \
    --tag $BASE \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    --file Dockerfile.base ./../..

CUDA_SUFFIX=""
if [ $CUDA == "on" ]; then
    CUDA_SUFFIX="-cuda"
    docker build \
        --rm \
        --tag $BASE$CUDA_SUFFIX \
        --build-arg FROM_ARG=$BASE \
        --file Dockerfile.cuda .
fi

if [ "$BASE_ONLY" == "true" ]; then
    echo "Finished building the base image(s) only."
    exit 0
fi

docker build \
    --rm \
    --tag $IMAGE_NAME:$TAG_PREFIX-$ROS_DISTRO$CUDA_SUFFIX \
    --build-arg FROM_ARG=$BASE$CUDA_SUFFIX \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    --file Dockerfile ./../..
