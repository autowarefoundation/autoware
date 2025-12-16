# ros2_dev_tools

This role installs ROS 2 development tools following [this page](https://docs.ros.org/en/humble/Installation/Ubuntu-Development-Setup.html).

## Inputs

None.

## Manual Installation

```bash
# Taken from https://docs.ros.org/en/humble/Installation/Ubuntu-Development-Setup.html
sudo apt update && sudo apt install -y \
  python3-colcon-mixin \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  ros-build-essential \
  python3-bloom \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-rosdep \
  python3-vcs2l \
  wget

# Initialize rosdep
sudo rosdep init
rosdep update
```
