#!/bin/bash
set -eo pipefail

function resolve_rosdep_keys() {
    local src_path=$1
    local ros_distro=$2
    local rosdep_keys_args=$3

    # shellcheck disable=SC2086
    local keys
    keys=$(rosdep keys $rosdep_keys_args --ignore-src --from-paths "$src_path" 2>/dev/null || true)

    if [ -z "$keys" ]; then
        return 0
    fi

    # Resolve with RHEL OS override, filter out ros-jazzy-* (built from source)
    echo "$keys" |
        xargs -r rosdep resolve --rosdistro "$ros_distro" --os=rhel:9 2>/dev/null |
        grep -v '^#' |
        sed 's/ \+/\n/g' |
        grep -v '^ros-' |
        grep -v '^$' |
        sort -u || true
}

resolve_rosdep_keys "$@"
