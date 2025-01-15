#!/bin/bash

function resolve_rosdep_keys() {
    local src_path=$1
    local ros_distro=$2
    local dependency_types=$3
    if [ -z "$dependency_types" ]; then
        dependency_types="build build_export buildtool buildtool_export exec test"
    fi

    rosdep keys --dependency-types="$dependency_types" --ignore-src --from-paths "$src_path" |
        xargs rosdep resolve --rosdistro "$ros_distro" |
        grep -v '^#' |
        sed 's/ \+/\n/g' |
        sort
}

resolve_rosdep_keys "$@"
