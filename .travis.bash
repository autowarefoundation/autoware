#!/bin/bash

set -e

if [ "${CROSS_COMPILE}" == "1" ]; then
  AUTOWARE_HOME=${TRAVIS_BUILD_DIR}
  AUTOWARE_TARGET_ARCH=aarch64
  AUTOWARE_TARGET_PLATFORM=generic-aarch64
  AUTOWARE_BUILD_PATH=${AUTOWARE_HOME}/ros/build-${AUTOWARE_TARGET_PLATFORM}
  AUTOWARE_DEVEL_PATH=${AUTOWARE_HOME}/ros/devel-${AUTOWARE_TARGET_PLATFORM}
  AUTOWARE_TOOLCHAIN_FILE_PATH=${AUTOWARE_HOME}/ros/cross_toolchain.cmake
  AUTOWARE_SYSROOT=/sysroot/${AUTOWARE_TARGET_PLATFORM}

  docker run \
    -e AUTOWARE_SYSROOT=${AUTOWARE_SYSROOT} \
    --rm \
    -v ${AUTOWARE_HOME}:${AUTOWARE_HOME} \
    -w ${AUTOWARE_HOME}/ros \
    autoware/build:${AUTOWARE_TARGET_PLATFORM}-kinetic-20180809 \
    bash \
      -c "source /opt/ros/kinetic/setup.bash && \
          catkin_make \
            -DCMAKE_TOOLCHAIN_FILE=${AUTOWARE_TOOLCHAIN_FILE_PATH} \
            -DCATKIN_DEVEL_PREFIX=${AUTOWARE_DEVEL_PATH} \
            -DCMAKE_SYSTEM_PROCESSOR=${AUTOWARE_TARGET_ARCH} \
            --build ${AUTOWARE_BUILD_PATH} \
            clean && \
          source devel-${AUTOWARE_TARGET_PLATFORM}/setup.bash && \
          catkin_make \
            -DCMAKE_TOOLCHAIN_FILE=${AUTOWARE_TOOLCHAIN_FILE_PATH} \
            -DCATKIN_DEVEL_PREFIX=${AUTOWARE_DEVEL_PATH} \
            -DCMAKE_SYSTEM_PROCESSOR=${AUTOWARE_TARGET_ARCH} \
            --build ${AUTOWARE_BUILD_PATH} \
            -j4"
else
  catkin_make clean
  source devel/setup.bash
  catkin_make -j4
  catkin_make -j4 run_tests
  catkin_test_results
fi
