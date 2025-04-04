# agnocast

This role installs [Agnocast](https://github.com/tier4/agnocast), true zero-copy communication middleware for all ROS 2 message types.

## Inputs

None.

## Manual Installation

```bash
agnocast_version="2.0.1"

sudo add-apt-repository -y ppa:t4-system-software/agnocast
sudo apt update
sudo apt install -y "agnocast-heaphook=${agnocast_version}*"

if dkms status | grep agnocast | grep -q "${agnocast_version}"; then
    echo "agnocast-kmod version ${agnocast_version} is already registered in dkms. Skipping purge and install."
else
    sudo apt purge -y "agnocast-kmod=${agnocast_version}*"
    sudo apt install -y "agnocast-kmod=${agnocast_version}*"
fi

```
