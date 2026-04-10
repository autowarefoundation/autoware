#!/bin/bash
set -eo pipefail

function cleanup_dnf() {
    dnf clean all 2>/dev/null || true
    rm -rf "$HOME"/.cache 2>/dev/null || true
}

cleanup_dnf "$@"
