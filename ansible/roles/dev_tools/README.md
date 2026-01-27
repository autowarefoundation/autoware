# devel

This role installs optional development tools for Autoware.

## Tools

- pipx
- Go
- PlotJuggler
- Git LFS
- pre-commit
- clang-format

## Inputs

| Name      | Required | Description           |
| --------- | -------- | --------------------- |
| rosdistro | true     | The ROS distribution. |

## Manual Installation

## Set up the environment variables

Choose **one** ROS distribution and run the corresponding command.

### ROS 2 Humble

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && \
source /tmp/amd64.env
```

### ROS 2 Jazzy

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64_jazzy.env && \
source /tmp/amd64.env
```

## Install the tools

```bash
sudo apt-get update

sudo apt install pipx

sudo apt install -y golang
sudo apt install -y ros-${rosdistro}-plotjuggler-ros
sudo apt install -y git-lfs

# Setup Git LFS
git lfs install

pipx install pre-commit
pipx install clang-format
```
