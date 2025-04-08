#!/bin/bash
set -eo pipefail

function cleanup_system() {
    local lib_dir=$1
    local ros_distro=$2

    find /usr/lib/"$lib_dir"-linux-gnu -name "*.a" -type f -delete &&
        find / -name "*.o" -type f -delete &&
        find / -name "*.h" -type f -delete &&
        find / -name "*.hpp" -type f -delete &&
        rm -rf /autoware/ansible /autoware/ansible-galaxy-requirements.yaml /autoware/setup-dev-env.sh /autoware/*.env \
            /root/.local/pipx /opt/ros/"$ros_distro"/include /opt/autoware/include /etc/apt/sources.list.d/cuda*.list \
            /etc/apt/sources.list.d/docker.list /etc/apt/sources.list.d/nvidia-docker.list \
            /usr/include /usr/share/doc /usr/lib/gcc /usr/lib/jvm /usr/lib/llvm*
}

./cleanup_apt.sh
cleanup_system "$@"
