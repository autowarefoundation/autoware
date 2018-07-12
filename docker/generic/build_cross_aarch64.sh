#!/bin/bash

# Register QEMU as a handler for aarch64
docker run --rm --privileged multiarch/qemu-user-static:register

# Build Docker Image
docker build -t autoware-kinetic:crossbuild-aarch64 -f Dockerfile.kinetic-crossbuild-aarch64 .

# Deregister QEMU as a handler for aarch64
docker run --rm --privileged multiarch/qemu-user-static:register --reset
