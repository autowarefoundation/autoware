#!/bin/bash

function cleanup_apt() {
    local remove_var_lib_apt_lists=false
    apt-get autoremove -y && rm -rf "$HOME"/.cache
    if "$remove_var_lib_apt_lists"; then
        echo "----------------------------------------"
        echo "Removing /var/lib/apt/lists/*"
        echo "----------------------------------------"
        rm -rf /var/lib/apt/lists/*
    fi
}

cleanup_apt "$@"
