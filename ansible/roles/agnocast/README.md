# agnocast

This role installs [Agnocast](https://github.com/tier4/agnocast), true zero-copy communication middleware for unsized ROS 2 message types.

## Inputs

None.

## Manual Installation

```bash
sudo add-apt-repository ppa:t4-system-software/agnocast
sudo apt update
sudo apt install "agnocast-heaphook=1.0.1*" "agnocast-kmod=1.0.1*"
```
