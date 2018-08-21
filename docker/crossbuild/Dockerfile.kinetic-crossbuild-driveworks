ARG AUTOWARE_DOCKER_ARCH
ARG AUTOWARE_TARGET_ARCH
ARG AUTOWARE_TARGET_PLATFORM
FROM autoware/build:${AUTOWARE_TARGET_PLATFORM}-kinetic-20180809
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential
COPY files/FindCUDA.cmake /usr/share/cmake-3.5/Modules/FindCUDA.cmake
CMD . /opt/ros/kinetic/setup.sh && /bin/bash
