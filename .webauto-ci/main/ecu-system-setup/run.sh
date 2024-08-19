#!/bin/bash -e

: "${WEBAUTO_CI_SOURCE_PATH:?is not set}"
: "${WEBAUTO_CI_GITHUB_TOKEN:?is not set}"

: "${AUTOWARE_PATH:?is not set}"
: "${ECU_SYSTEM_SETUP_SOURCE_PATH:?is not set}"
: "${ECU_SYSTEM_SETUP_ANSIBLE_PLAYBOOK:?is not set}"

cd "$AUTOWARE_PATH"
# Delete files for incremental builds created in the autoware-build phase.
find "$AUTOWARE_PATH" \( -name ".rsync-include" -or -name ".rsync-exclude" \) -print0 | xargs -0 rm

rm -rf "$ECU_SYSTEM_SETUP_SOURCE_PATH"
cp -r "${WEBAUTO_CI_SOURCE_PATH}/${ECU_SYSTEM_SETUP_SOURCE_PATH}" "$ECU_SYSTEM_SETUP_SOURCE_PATH"

sudo -E apt-get -y update
sudo -E apt-get -y install "linux-image-$(uname -r)" "linux-headers-$(uname -r)" "linux-modules-extra-$(uname -r)"
sudo -E apt-get -y install systemd udev kmod
sudo -E apt-get -y install ubuntu-minimal openssh-server fonts-ubuntu systemd-coredump vim grub-efi-amd64
sudo -E apt-get -y install ubuntu-desktop-minimal --no-install-recommends

# Disable auto suspend
sudo sed -i 's/\(.*sleep-inactive-ac-timeout=.*\)/sleep-inactive-ac-timeout=0/g' /etc/gdm3/greeter.dconf-defaults
sudo sed -i 's/\(.*sleep-inactive-battery-timeout=.*\)/sleep-inactive-battery-timeout=0/g' /etc/gdm3/greeter.dconf-defaults

export GITHUB_TOKEN="$WEBAUTO_CI_GITHUB_TOKEN"
git config --global --add url."https://${GITHUB_TOKEN}:x-oauth-basic@github.com/".insteadOf "https://github.com/"
git config --global --add url."https://${GITHUB_TOKEN}:x-oauth-basic@github.com/".insteadOf "git@github.com:"

readonly dummy_vehicle_id=default # Reconfigure during OTA update
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
ansible-playbook "${ECU_SYSTEM_SETUP_ANSIBLE_PLAYBOOK}" \
    -e autoware_install_dir="$(pwd)" \
    -e vehicle_id="${dummy_vehicle_id}" \
    -e reload_systemd=no

git config --global --unset-all url."https://${GITHUB_TOKEN}:x-oauth-basic@github.com/".insteadOf

sudo sed -i '/^autoware\sALL=(ALL)\sNOPASSWD:ALL/d' /etc/sudoers
