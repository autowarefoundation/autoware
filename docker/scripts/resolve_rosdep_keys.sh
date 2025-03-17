#!/bin/bash
set -eo pipefail

function resolve_rosdep_keys() {
    local src_path=$1
    local ros_distro=$2
    local rosdep_keys_args=$3

    # shellcheck disable=SC2086
    rosdep keys $rosdep_keys_args --ignore-src --from-paths "$src_path" |
        xargs rosdep resolve --rosdistro "$ros_distro" |
        grep -v '^#' |
        sed 's/ \+/\n/g' |
        sort
}

resolve_rosdep_keys "$@"
