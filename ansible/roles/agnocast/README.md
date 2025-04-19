# agnocast

This role installs [Agnocast](https://github.com/tier4/agnocast), true zero-copy communication middleware for all ROS 2 message types.

## Inputs

None.

## Manual Installation

```bash
agnocast_version="2.1.0"
agnocast_heaphook_package="agnocast-heaphook-v${agnocast_version}"
agnocast_kmod_package="agnocast-kmod-v${agnocast_version}"

sudo add-apt-repository -y ppa:t4-system-software/agnocast
sudo apt update
sudo apt install -y "${agnocast_heaphook_package}"

if dkms status | grep agnocast | grep -q "{agnocast_version}"; then
    echo "agnocast-kmod-v${agnocast_version} is already registered in dkms. Skipping purge and install."
else
    sudo apt purge -y "${agnocast_kmod_package}"
    sudo apt install -y "${agnocast_kmod_package}"
fi

```
