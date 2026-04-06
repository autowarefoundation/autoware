#!/bin/bash
set -e

# Remap aw user to match host UID/GID (avoids permission issues with mounted volumes)
if [ -n "${HOST_UID}" ] && [ -n "${HOST_GID}" ]; then
    usermod -u "${HOST_UID}" "${USERNAME}" >/dev/null 2>&1 || true
    groupmod -g "${HOST_GID}" "${USERNAME}" >/dev/null 2>&1 || true
fi

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ "${AUTOWARE_RUNTIME}" = "1" ] && [ -f /opt/autoware/setup.bash ]; then
    # shellcheck source=/dev/null
    source /opt/autoware/setup.bash
fi

exec gosu "${USERNAME}" "$@"
