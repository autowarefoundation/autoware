#!/usr/bin/env bash
# Install the Mach-E PC configuration that lives outside the colcon workspace.
# Run after cloning ~/autoware.mach_e (see NEW_PC_SETUP.md). Safe to re-run.
#
#   mach_e_setup/install.sh                      # user-level parts only (no sudo)
#   mach_e_setup/install.sh --system             # + sysctl, lo multicast, PTP for the Ouster (sudo)
#   SENSOR_IF=enp3s0 mach_e_setup/install.sh --system    # NIC wired to the lidar/ADMA switch (default eno1)
#
# Network profiles (fixed IPs, MTU 9000) are a separate step: system/setup_network.sh
set -eo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SENSOR_IF=${SENSOR_IF:-eno1}

link() {  # link <target in repo> <path in $HOME>
  if [ -L "$2" ]; then ln -sfn "$1" "$2"
  elif [ -e "$2" ]; then echo "!! $2 exists and is not a symlink; move it away and re-run"; exit 1
  else ln -s "$1" "$2"; fi
  echo "ok  $2 -> $1"
}

# 1. Scripts and environment: ~/utils paths are what run_mach_e.sh, aw.sh and the mapping scripts use.
mkdir -p ~/utils ~/autoware_map ~/autoware_logs/runs
link "$HERE/bringup" ~/utils/mach_e_bringup
link "$HERE/env" ~/utils/autoware_env_setup
grep -q "alias aw=" ~/.bashrc || { echo "alias aw='source ~/utils/autoware_env_setup/aw.sh'" >> ~/.bashrc; echo "ok  added 'aw' alias to ~/.bashrc"; }

# 2. RViz GPU leak fix (2026-10-15): turn off NVIDIA GL threaded optimizations for rviz2.
NV_SRC=$HERE/bringup/display/nvidia-application-profiles-rc
mkdir -p ~/.nv
if [ ! -f ~/.nv/nvidia-application-profiles-rc ]; then cp "$NV_SRC" ~/.nv/; echo "ok  ~/.nv/nvidia-application-profiles-rc"
elif cmp -s "$NV_SRC" ~/.nv/nvidia-application-profiles-rc; then echo "ok  ~/.nv/nvidia-application-profiles-rc (already there)"
else echo "!! ~/.nv/nvidia-application-profiles-rc differs from $NV_SRC; merge the rviz2 rule by hand"; fi

[ "${1:-}" = "--system" ] || { echo "user-level setup done (re-run with --system for sysctl / PTP)"; exit 0; }

# 3. CycloneDDS kernel buffers + loopback multicast (cyclonedds.xml binds DDS to lo).
sudo install -m 644 "$HERE/system/10-cyclone-max.conf" /etc/sysctl.d/10-cyclone-max.conf
if grep -q '^net.core.rmem_max=33554432$' /etc/sysctl.conf 2>/dev/null; then
  sudo sed -i '/^net.core.rmem_max=33554432$/d' /etc/sysctl.conf   # ArenaSDK installer line, overrides the 2 GB value
  echo "ok  removed ArenaSDK rmem_max override from /etc/sysctl.conf"
fi
sudo sysctl --system > /dev/null
sudo install -m 644 "$HERE/system/lo-multicast.service" /etc/systemd/system/lo-multicast.service

# 4. PTP grandmaster for the Ouster (software timestamping, see system/ptp4l-ouster.conf).
command -v ptp4l > /dev/null || sudo apt-get install -y linuxptp
sudo mkdir -p /etc/linuxptp
sed "s/^\[eno1\]$/[$SENSOR_IF]/; s/on eno1 /on $SENSOR_IF /" "$HERE/system/ptp4l-ouster.conf" | sudo tee /etc/linuxptp/ptp4l-ouster.conf > /dev/null
sed "s/on eno1 /on $SENSOR_IF /" "$HERE/system/ptp4l-ouster.service" | sudo tee /etc/systemd/system/ptp4l-ouster.service > /dev/null

sudo systemctl daemon-reload
sudo systemctl enable --now lo-multicast.service ptp4l-ouster.service
echo "ok  system setup done: rmem_max=$(sysctl -n net.core.rmem_max), PTP on $SENSOR_IF"
