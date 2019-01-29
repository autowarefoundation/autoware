#!/bin/bash

# Build Docker Image
if [[ ("$1" = "kinetic" || "$1" = "melodic" ) && ("$2" = "synquacer" || "$2" = "driveworks" || "$2" = "generic-aarch64") ]]
then
    # Once we support for targets, change this to the appropriate Docker image
    AUTOWARE_DOCKER_ARCH=arm64v8
    AUTOWARE_DOCKER_DATE=20190116
    AUTOWARE_TARGET_ARCH=aarch64
    AUTOWARE_TARGET_ROS_DISTRO=$1
    AUTOWARE_TARGET_PLATFORM=$2

    echo "Using ${AUTOWARE_TARGET_PLATFORM} as the target architecture"
    # Register QEMU as a handler for non-x86 targets
    docker container run --rm --privileged multiarch/qemu-user-static:register

    # Build Docker Image
    docker image build \
        --build-arg AUTOWARE_DOCKER_ARCH=${AUTOWARE_DOCKER_ARCH} \
        --build-arg AUTOWARE_TARGET_ARCH=${AUTOWARE_TARGET_ARCH} \
        --build-arg AUTOWARE_TARGET_PLATFORM=${AUTOWARE_TARGET_PLATFORM} \
        -t autoware/build:${AUTOWARE_TARGET_PLATFORM}-${AUTOWARE_TARGET_ROS_DISTRO}-${AUTOWARE_DOCKER_DATE} \
        -f Dockerfile.${AUTOWARE_TARGET_ROS_DISTRO}-crossbuild .
    if [ "$AUTOWARE_TARGET_PLATFORM" = "driveworks" ]
    then
        docker image build \
        --build-arg AUTOWARE_DOCKER_ARCH=${AUTOWARE_DOCKER_ARCH} \
        --build-arg AUTOWARE_TARGET_ARCH=${AUTOWARE_TARGET_ARCH} \
        --build-arg AUTOWARE_TARGET_PLATFORM=${AUTOWARE_TARGET_PLATFORM} \
        -t autoware/build:${AUTOWARE_TARGET_PLATFORM}-${AUTOWARE_TARGET_ROS_DISTRO}-${AUTOWARE_DOCKER_DATE} \
        -f Dockerfile.${AUTOWARE_TARGET_ROS_DISTRO}-crossbuild-driveworks .
    fi

    # Deregister QEMU as a handler for non-x86 targets
    docker container run --rm --privileged multiarch/qemu-user-static:register --reset
else
    echo "Select target platform: synquacer, driveworks, generic-aarch64"
fi
