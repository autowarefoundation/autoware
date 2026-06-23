#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ANSIBLE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ -z ${ROS_DISTRO:-} ]]; then
    echo "Error: ROS_DISTRO is not set." >&2
    exit 1
fi
if [[ -z ${ROS_SNAPSHOT_DATE:-} ]]; then
    echo "Error: ROS_SNAPSHOT_DATE is not set (e.g. 2026-04-13)." >&2
    exit 1
fi
ARCH=$(dpkg --print-architecture)
OUTPUT_FILE="${ANSIBLE_DIR}/vars/locked-versions-${ROS_DISTRO}-${ARCH}.yaml"

main() {
    echo "Generating Ansible lockfile: $OUTPUT_FILE"
    if [[ ! -f $OUTPUT_FILE ]]; then
        echo "Error: Lockfile not found: $OUTPUT_FILE" >&2
        echo "Create it with apt_pins package names first, then run this script." >&2
        exit 1
    fi

    # apt_pins package names come from the existing lockfile (no hardcoded list).
    # cspell:ignore isinstance safeload
    local packages
    packages=$(python3 -c "
import yaml, sys
data = yaml.safe_load(open('$OUTPUT_FILE')) or {}
pins = data.get('apt_pins') or {}
if not isinstance(pins, dict):
    sys.exit('Error: apt_pins must be a mapping')
for pkg in sorted(pins):
    print(pkg)
")

    # pip/pipx-managed packages (resolved separately below)
    local pip_pkgs=("gdown")

    declare -A versions
    local has_error=false

    for pkg in $packages; do
        local is_pip=false
        for pip_pkg in "${pip_pkgs[@]}"; do
            [[ $pkg == "$pip_pkg" ]] && is_pip=true && break
        done
        $is_pip && continue

        local ver
        ver=$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null) || true
        if [[ -z $ver ]]; then
            echo "Error: APT package '$pkg' is not installed." >&2
            has_error=true
            continue
        fi
        versions[$pkg]=$ver

        # Origin check: an apt_pins package served by ros.org should be snapshot-covered, not pinned.
        if apt-cache policy "$pkg" 2>/dev/null | grep -q 'ros.org'; then
            echo "Warning: '$pkg' resolves from a ros.org source; consider removing it from apt_pins (snapshot covers it)." >&2
        fi
    done

    for pkg in "${pip_pkgs[@]}"; do
        local ver
        ver=$(pip3 show "$pkg" 2>/dev/null | awk '/^Version:/{print $2}' || true)
        [[ -z $ver ]] && ver=$(pipx runpip "$pkg" show "$pkg" 2>/dev/null | awk '/^Version:/{print $2}' || true)
        if [[ -z $ver ]]; then
            echo "Error: pip package '$pkg' is not installed (checked pip3 and pipx)." >&2
            has_error=true
        else
            versions[$pkg]=$ver
        fi
    done

    $has_error && exit 1

    # Preserve any existing ros_overrides block verbatim.
    local overrides
    overrides=$(python3 -c "
import yaml
data = yaml.safe_load(open('$OUTPUT_FILE')) or {}
ov = data.get('ros_overrides') or {}
if ov:
    print(yaml.safe_dump({'ros_overrides': ov}, default_flow_style=False).rstrip())
else:
    print('ros_overrides: {}')
")

    {
        cat <<HEADER
# Dependency Version Lockfile
# ROS Distro: ${ROS_DISTRO}
# Platform: ${ARCH}
# Ubuntu: $(lsb_release -rs) ($(lsb_release -cs))
#
# ros_snapshot_date freezes the entire ROS closure via snapshots.ros.org.
ros_snapshot_date: "${ROS_SNAPSHOT_DATE}"
apt_pins:
HEADER
        for pkg in $packages; do
            echo "  ${pkg}: ${versions[$pkg]}"
        done
        echo "$overrides"
    } >"$OUTPUT_FILE"

    echo "Generated: $OUTPUT_FILE"
    "${SCRIPT_DIR}/validate_lockfiles.sh" "$OUTPUT_FILE"
}

main "$@"
