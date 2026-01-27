#!/bin/bash -e

: "${WEBAUTO_CI_SOURCE_PATH:?is not set}"

: "${AUTOWARE_PATH:?is not set}"

apt-get update

apt-get -y install sudo curl wget unzip gnupg lsb-release ccache python3-apt python3-pip apt-utils software-properties-common jq
add-apt-repository universe

pip install --no-cache-dir 'ansible==6.*'

user=autoware
useradd -m "$user" -s /bin/bash
echo "$user:$user" | chpasswd
echo "$user ALL=(ALL) NOPASSWD:ALL" >>/etc/sudoers
gpasswd -a "$user" sudo

mkdir -p "$AUTOWARE_PATH"
chmod 755 "$AUTOWARE_PATH"
cp -rfT "$WEBAUTO_CI_SOURCE_PATH" "$AUTOWARE_PATH"
chown -R "$user":"$user" "$AUTOWARE_PATH"
