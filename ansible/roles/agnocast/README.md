# agnocast

This role installs [Agnocast](https://github.com/autowarefoundation/agnocast), true zero-copy communication middleware for all ROS 2 message types.

## Inputs

| Name                      | Required | Description                                                                                      |
| ------------------------- | -------- | ------------------------------------------------------------------------------------------------ |
| agnocast_version          | false    | The version of Agnocast.                                                                         |
| agnocast_heaphook_package | false    | The apt package of the heaphook library. The default is `agnocast-heaphook-v<agnocast_version>`. |
| agnocast_kmod_package     | false    | The apt package of the kernel module. The default is `agnocast-kmod-v<agnocast_version>`.        |

## Installation

Install Ansible first. The steps are in the [ansible installation guide](../../README.md#ansible-installation).

```bash
cd ~/autoware # The root directory of the cloned repository
ansible-galaxy collection install -f -r "ansible-galaxy-requirements.yaml"
ansible-playbook autoware.dev_env.install_dev_env --tags agnocast --ask-become-pass
```

If you need a different version, add `-e agnocast_version=<version>` to the last command.
