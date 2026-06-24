#!/bin/bash
set -e

# Remap aw user to match host UID/GID (avoids permission issues with mounted volumes)
if [ -n "${HOST_UID}" ] && [ -n "${HOST_GID}" ]; then
    usermod -u "${HOST_UID}" "${USERNAME}" >/dev/null 2>&1 || true
    groupmod -g "${HOST_GID}" "${USERNAME}" >/dev/null 2>&1 || true
fi

try_set() {
    "$@" >/dev/null 2>&1 ||
        echo "[entrypoint] WARN: failed: $* (need --privileged or --cap-add=NET_ADMIN)" >&2
}

# Enable multicast on loopback so DDS discovery works when pinned to lo
try_set ip link set lo multicast on

# Apply system-wide network tuning for DDS (needs --privileged or --cap-add=NET_ADMIN)
# https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/#tune-system-wide-network-settings
try_set sysctl -w net.core.rmem_max=2147483647
try_set sysctl -w net.ipv4.ipfrag_time=3
try_set sysctl -w net.ipv4.ipfrag_high_thresh=134217728

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ "${AUTOWARE_RUNTIME}" = "1" ] && [ -f /opt/autoware/setup.bash ]; then
    # shellcheck source=/dev/null
    source /opt/autoware/setup.bash
fi

exec gosu "${USERNAME}" "$@"
