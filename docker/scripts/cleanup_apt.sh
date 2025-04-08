#!/bin/bash
set -eo pipefail

function cleanup_apt() {
    local apt_clean=$1
    apt-get autoremove -y && rm -rf /var/lib/apt/lists/* "$HOME"/.cache
    if [[ $apt_clean == true ]]; then
        apt-get clean
    fi
}

cleanup_apt "$@"
