# pacmod

This role sets up the prerequisites to install pacmod3_msgs following [this page](https://github.com/astuff/pacmod3_msgs#installation) and [this page](https://github.com/astuff/docker-builds/blob/e9f563ef04b0578ba0b545711ea3e41b20a51d27/ros2/foxy/ros-core/Dockerfile).

## Inputs

| Name      | Required | Description     |
| --------- | -------- | --------------- |
| rosdistro | true     | The ROS distro. |

## Manual Installation

For Universe, the `rosdistro` variable can also be found in:
[../../playbooks/universe.yaml](../../playbooks/universe.yaml)

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# Taken from https://github.com/astuff/pacmod3#installation
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-${rosdistro}-pacmod3
```
