# Kisak Mesa Fix for Ubuntu 22.04 for Rviz2 (Not mandatory)

If you are using Ubuntu 22.04 and Rviz2 (especially inside a container), you may encounter black-screen error on Rviz2: <https://github.com/ros2/rviz/issues/948>

This role will install the Kisak Mesa fix for Ubuntu 22.04 for Rviz2.

## Inputs

None

## Manual Installation

```bash
#!/bin/bash

# Update the package list and install software-properties-common
sudo apt-get update
sudo apt-get install -y software-properties-common

# Add the Kisak Mesa PPA
sudo add-apt-repository -y ppa:kisak/kisak-mesa

# Update the package list after adding the new repository
sudo apt-get update

# Install Mesa libraries
sudo apt-get install -y \
libegl-mesa0 \
libegl1-mesa-dev \
libgbm-dev \
libgbm1 \
libgl1-mesa-dev \
libgl1-mesa-dri \
libglapi-mesa \
libglx-mesa0
```
