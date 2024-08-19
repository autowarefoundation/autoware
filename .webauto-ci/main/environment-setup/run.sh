#!/bin/bash -e

apt-get update

apt-get -y install sudo curl wget unzip gnupg lsb-release git ccache python3-apt python3-pip apt-utils software-properties-common jq rsync
add-apt-repository universe

pip install --no-cache-dir 'ansible==6.*'

user=autoware
useradd -m "$user" -s /bin/bash
echo "$user:$user" | chpasswd
echo "$user ALL=(ALL) NOPASSWD:ALL" >>/etc/sudoers
gpasswd -a "$user" sudo
