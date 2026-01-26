# rmw_implementation

This role sets up ROS 2 RMW implementation following [this page](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html).

## Inputs

| Name               | Required | Description         |
| ------------------ | -------- | ------------------- |
| rosdistro          | true     | The ROS distro.     |
| rmw_implementation | true     | RMW implementation. |

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

## Install the RMW implementation

For details: <https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html>

```bash
sudo apt update
sudo apt install ros-${rosdistro}-${rmw_implementation//_/-}

# (Optional) You set the default RMW implementation in the ~/.bashrc file.
echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=${rmw_implementation}" >> ~/.bashrc
```
