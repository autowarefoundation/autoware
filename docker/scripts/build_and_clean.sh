#!/bin/bash

function build_and_clean() {
    local ccache_dir=$1
    local install_base=$2
    local cuda_image=$3

    local cmake_args=" -Wno-dev --no-warn-unused-cli"
    if [ "$cuda_image" = true ]; then
        cmake_args="$cmake_args -DFORCE_CUDA=1"
    fi

    du -sh "$ccache_dir" && ccache -s &&
        colcon build --cmake-args "$cmake_args" \
            --merge-install \
            --install-base "$install_base" \
            --mixin release compile-commands ccache &&
        du -sh "$ccache_dir" && ccache -s &&
        rm -rf /autoware/build /autoware/log
}

build_and_clean "$@"
