# rmw_implementation

This role sets up ROS 2 RMW implementation following [this page](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html).

## Inputs

| Name               | Required | Description         |
| ------------------ | -------- | ------------------- |
| rosdistro          | true     | The ROS distro.     |
| rmw_implementation | true     | RMW implementation. |

## Manual Installation

For Universe, the `rosdistro` and `rmw_implementation` variable can also be found in:
[../../playbooks/universe.yaml](../../playbooks/universe.yaml)

```bash
wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env

# For details: https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html
sudo apt update
rmw_implementation_dashed=$(eval sed -e "s/_/-/g" <<< "${rmw_implementation}")
sudo apt install ros-${rosdistro}-${rmw_implementation_dashed}

# (Optional) You set the default RMW implementation in the ~/.bashrc file.
echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=${rmw_implementation}" >> ~/.bashrc
```
