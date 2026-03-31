#!/bin/bash
set -eo pipefail

function build_and_clean() {
    local ccache_dir=$1
    local install_base=$2
    local colcon_build_args=$3

    # Limit CPU cores to (nproc - 1) to prevent OOM on resource-constrained runners
    local taskset_cmd=""
    local num_cores
    num_cores=$(nproc)
    if [ "$num_cores" -gt 1 ]; then
        taskset_cmd="taskset --cpu-list 0-$((num_cores - 2))"
    fi

    # shellcheck disable=SC2086
    du -sh "$ccache_dir" && ccache -s &&
        $taskset_cmd colcon build --cmake-args \
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
