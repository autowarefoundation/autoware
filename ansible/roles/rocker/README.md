# rocker

This role installs [osrf/rocker](https://github.com/osrf/rocker) following the [installation guide](https://github.com/osrf/rocker/#installation).

## Inputs

None.

## Manual Installation

Install rocker:

```bash
# Taken from: https://github.com/osrf/rocker#installation

# Add the ROS 2 apt repository to your system. First authorize our GPG key with apt.
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Then add the repository to your sources list.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt-get install python3-rocker
```
