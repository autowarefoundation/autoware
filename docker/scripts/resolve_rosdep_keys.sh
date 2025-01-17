#!/bin/bash
set -e

function resolve_rosdep_keys() {
    local src_path=$1
    local ros_distro=$2

    rosdep keys --ignore-src --from-paths "$src_path" |
        xargs rosdep resolve --rosdistro "$ros_distro" |
        grep -v '^#' |
        sed 's/ \+/\n/g' |
        sort
}

resolve_rosdep_keys "$@"
