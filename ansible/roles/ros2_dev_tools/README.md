# ros2_dev_tools

This role installs ROS 2 development tools following [this page](https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html).

## Inputs

None.

## Manual Installation

```bash
# Taken from https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

# Install some pip packages needed for testing
python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools

# Initialize rosdep
sudo rosdep init
rosdep update
```
