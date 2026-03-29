# rmw_implementation

This role sets up ROS 2 RMW implementation following [this page](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html).

## Inputs

| Name               | Required | Description         |
| ------------------ | -------- | ------------------- |
| rosdistro          | true     | The ROS distro.     |
| rmw_implementation | true     | RMW implementation. |

## Manual Installation

## Set up the environment variables

```bash
# Choose your ROS distribution
rosdistro=humble  # or jazzy

# RMW implementation (see ./defaults/main.yaml for the current default)
rmw_implementation=rmw_cyclonedds_cpp
```

## Install the RMW implementation

For details: <https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html>

```bash
sudo apt update
sudo apt install ros-${rosdistro}-${rmw_implementation//_/-}

# (Optional) You set the default RMW implementation in the ~/.bashrc file.
echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=${rmw_implementation}" >> ~/.bashrc
```
