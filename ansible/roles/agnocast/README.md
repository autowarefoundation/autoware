# agnocast

This role installs [Agnocast](https://github.com/autowarefoundation/agnocast), true zero-copy communication middleware for all ROS 2 message types.

## Inputs

| Name                      | Required | Description                                                                                      |
| ------------------------- | -------- | ------------------------------------------------------------------------------------------------ |
| agnocast_version          | false    | The version of Agnocast.                                                                         |
| agnocast_heaphook_package | false    | The apt package of the heaphook library. The default is `agnocast-heaphook-v<agnocast_version>`. |
| agnocast_kmod_package     | false    | The apt package of the kernel module. The default is `agnocast-kmod-v<agnocast_version>`.        |

## Installation with Ansible

Install Ansible first. The steps are in the [ansible installation guide](../../README.md#ansible-installation).

```bash
cd ~/autoware # The root directory of the cloned repository
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
ansible-playbook autoware.dev_env.install_dev_env --tags agnocast --ask-become-pass
```

If you need a different version, add `-e agnocast_version=<version>` to the last command.

## Manual Installation

```bash
agnocast_version="2.3.5"
agnocast_heaphook_package="agnocast-heaphook-v${agnocast_version}"
agnocast_kmod_package="agnocast-kmod-v${agnocast_version}"

sudo add-apt-repository -y ppa:t4-system-software/agnocast
sudo apt update
sudo apt install -y "${agnocast_heaphook_package}"

sudo apt install -y "linux-headers-$(uname -r)"

if dkms status "agnocast/${agnocast_version}" | grep -q installed; then
    echo "agnocast-kmod-v${agnocast_version} is already installed in dkms. Skipping purge and install."
else
    sudo apt purge -y "${agnocast_kmod_package}"
    sudo apt install -y "${agnocast_kmod_package}"
fi

echo agnocast | sudo tee /etc/modules-load.d/agnocast.conf
```
