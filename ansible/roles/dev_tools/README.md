# devel

This role installs optional development tools for Autoware.

## Tools

- Git LFS
- pre-commit
- clang-format
- Go
- PlotJuggler

## Inputs

| Name                            | Required | Description                             |
|---------------------------------|----------|-----------------------------------------|
| pre_commit_clang_format_version | true     | The version of clang-format to install. |
| rosdistro                       | true     | The ROS distribution.                   |

## Manual Installation

```bash
# For the environment variables
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

sudo apt-get update

sudo apt install python3-pip
sudo apt-get install -y golang
sudo apt-get install -y ros-${rosdistro}-plotjuggler-ros
sudo apt-get install -y git-lfs

# Setup Git LFS
git lfs install

pip3 install pre-commit
pip3 install clang-format==${pre_commit_clang_format_version}
```
