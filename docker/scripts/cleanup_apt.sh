#!/bin/bash

function cleanup() {
    local remove_var_lib_apt_lists=false
    apt-get autoremove -y && rm -rf "$HOME"/.cache
    if "$remove_var_lib_apt_lists"; then
        rm -rf /var/lib/apt/lists/*
    fi
}

cleanup "$@"
