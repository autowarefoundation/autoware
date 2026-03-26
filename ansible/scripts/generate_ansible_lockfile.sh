#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ANSIBLE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
if [[ -z ${ROS_DISTRO:-} ]]; then
    echo "Error: ROS_DISTRO is not set." >&2
    exit 1
fi
ARCH=$(dpkg --print-architecture)

OUTPUT_FILE="${ANSIBLE_DIR}/vars/locked-versions-${ROS_DISTRO}-${ARCH}.yaml"

main() {
    echo "Generating Ansible lockfile: $OUTPUT_FILE"

    if [[ ! -f $OUTPUT_FILE ]]; then
        echo "Error: Lockfile not found: $OUTPUT_FILE" >&2
        echo "Create the lockfile with package names first, then run this script to fill in versions." >&2
        exit 1
    fi

    # Read package names from existing lockfile
    local packages
    # cspell:ignore isinstance
    packages=$(python3 -c "
import yaml, sys
with open('$OUTPUT_FILE') as f:
    data = yaml.safe_load(f)
if not isinstance(data, dict):
    sys.exit(0)
for pkg in sorted(data):
    print(pkg)
")

    # Resolve all versions first, fail if any package is missing
    declare -A versions
    local has_error=false

    # APT packages
    for pkg in $packages; do
        local ver
        ver=$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null) || true
        if [[ -z $ver ]]; then
            echo "Error: APT package '$pkg' is not installed." >&2
            has_error=true
        else
            versions[$pkg]=$ver
        fi
    done

    # pip packages (included in the same lockfile)
    local pip_pkgs=("gdown")
    for pkg in "${pip_pkgs[@]}"; do
        local ver
        ver=$(pip3 show "$pkg" 2>/dev/null | grep '^Version:' | awk '{print $2}' || true)
        if [[ -z $ver ]]; then
            echo "Error: pip package '$pkg' is not installed." >&2
            has_error=true
        else
            versions[$pkg]=$ver
        fi
    done

    if [[ $has_error == "true" ]]; then
        exit 1
    fi

    # Write lockfile only after all versions are resolved
    {
        cat <<HEADER
# Dependency Version Lockfile
# cspell:ignore Iseconds
# Generated: $(date -Iseconds)
# ROS Distro: ${ROS_DISTRO}
# Platform: ${ARCH}
# Ubuntu: $(lsb_release -rs) ($(lsb_release -cs))

HEADER

        for pkg in $packages; do
            echo "${pkg}: ${versions[$pkg]}"
        done
    } >"$OUTPUT_FILE"

    echo "Generated: $OUTPUT_FILE"
}

main "$@"
