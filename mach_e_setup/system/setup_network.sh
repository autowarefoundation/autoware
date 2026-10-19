#!/usr/bin/env bash
# Create the two NetworkManager profiles the Mach-E sensors need (same as the lab PC, 2026-10-19).
#
#   setup_network.sh <SENSOR_IF> <CAMERA_IF>      e.g. setup_network.sh eno1 enp4s0
#
# SENSOR_IF: to the switch with the Ouster (192.168.88.37) and ADMA (192.168.88.30, sends to .100:4444)
# CAMERA_IF: cabled straight to the LUCID camera (192.168.88.107). A shared 1 Gb/s link drops camera packets.
# Both are on 192.168.88.0/24, so the camera gets /32 host routes on CAMERA_IF; without them its traffic
# leaves through SENSOR_IF and the camera never answers. Existing profiles with these names are left alone.
set -eo pipefail
SENSOR_IF=${1:?usage: setup_network.sh <SENSOR_IF> <CAMERA_IF>}
CAMERA_IF=${2:?usage: setup_network.sh <SENSOR_IF> <CAMERA_IF>}

if nmcli -t -f NAME con show | grep -qx mach-e-sensors; then echo "mach-e-sensors exists, skipped"; else
  sudo nmcli con add type ethernet con-name mach-e-sensors ifname "$SENSOR_IF" \
    ipv4.method manual ipv4.addresses "192.168.88.100/24, 169.254.6.1/16" ipv4.never-default yes \
    ipv6.method disabled 802-3-ethernet.mtu 9000
fi
if nmcli -t -f NAME con show | grep -qx mach-e-camera; then echo "mach-e-camera exists, skipped"; else
  sudo nmcli con add type ethernet con-name mach-e-camera ifname "$CAMERA_IF" \
    ipv4.method manual ipv4.addresses 192.168.88.101/24 ipv4.never-default yes ipv4.route-metric 200 \
    ipv4.routes "192.168.88.107/32 src=192.168.88.101, 192.168.88.126/32 src=192.168.88.101" \
    ipv6.method disabled 802-3-ethernet.mtu 9000 ethtool.ring-rx 4096
fi
sudo nmcli con up mach-e-sensors || true
sudo nmcli con up mach-e-camera || true
ip -br addr show "$SENSOR_IF"; ip -br addr show "$CAMERA_IF"
