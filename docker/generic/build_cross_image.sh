#!/bin/bash

# Build Docker Image
if [ "$1" = "aarch64" ]
then
    # Once we support for targets, change this to the appropriate Docker image
    DOCKER_ARCH=arm64v8

    echo "Using $1 as the target architecture"
    # Register QEMU as a handler for non-x86 targets
    docker run --rm --privileged multiarch/qemu-user-static:register

    # Build Docker Image
    docker build --build-arg DOCKER_ARCH=${DOCKER_ARCH} --build-arg SYSROOT_ARCH=$1 -t autoware-kinetic:crossbuild-$1 -f Dockerfile.kinetic-crossbuild .

    # Deregister QEMU as a handler for non-x86 targets
    docker run --rm --privileged multiarch/qemu-user-static:register --reset
else
    echo "Select target, aarch64"
fi
