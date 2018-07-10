#!/bin/bash

set -eufo pipefail

# Build Docker Image
cat Dockerfile.kinetic-sysroot-aarch64 | docker build -t autoware-kinetic:sysroot-aarch64 -

# Fix up symlinks so that they are relative in the sysroot
docker run --name autoware-kinetic-sysroot-aarch64 autoware-kinetic:sysroot-aarch64 sh -c "\
symlinks -cr / && \
find /opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e 's#/opt/ros/kinetic/include#\${CMAKE_SYSROOT}/opt/ros/kinetic/include#g' {} \;  && \
find /opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e 's#/opt/ros/kinetic/lib#\${CMAKE_SYSROOT}/opt/ros/kinetic/lib#g' {} \; && \
find /opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e 's#/usr/include#\${CMAKE_SYSROOT}/usr/include#g' {} \; && \
find /opt/ros/kinetic/share/ -name "*.cmake" -type f -exec sed -i -e 's#/usr/lib#\${CMAKE_SYSROOT}/usr/lib#g' {} \; && \
find /usr/lib/cmake/ -name "*.cmake" -type f -exec sed -i -e 's#/usr/lib/aarch64-linux-gnu#\${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu#g' {} \; && \
find /usr/lib/cmake/ -name "*.cmake" -type f -exec sed -i -e 's#/usr/lib/openmpi#\${CMAKE_SYSROOT}/usr/lib/openmpi#g' {} \; && \
find /usr/lib/cmake/ -name "*.cmake" -type f -exec sed -i -e 's#/usr/include#\${CMAKE_SYSROOT}/usr/include#g' {} \; && \
sed -i -e 's#/usr#\${CMAKE_SYSROOT}/usr#g' /usr/lib/aarch64-linux-gnu/cmake/pcl/PCLConfig.cmake \
"

docker export -o autoware-kinetic-sysroot-aarch64.tar autoware-kinetic-sysroot-aarch64
docker container rm autoware-kinetic-sysroot-aarch64
