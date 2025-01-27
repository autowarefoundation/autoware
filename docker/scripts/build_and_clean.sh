#!/bin/bash
set -eo pipefail

function build_and_clean() {
    local ccache_dir=$1
    local install_base=$2
    local colcon_build_args=$3

    # shellcheck disable=SC2086
    du -sh "$ccache_dir" && ccache -s &&
        colcon build --cmake-args \
            " -Wno-dev" \
            " --no-warn-unused-cli" \
            --merge-install \
            --install-base "$install_base" \
            --mixin release compile-commands ccache \
            $colcon_build_args &&
        du -sh "$ccache_dir" && ccache -s &&
        rm -rf /autoware/build /autoware/log
}

build_and_clean "$@"
