# Edge.Auto

## Getting Started

```sh
git clone https://github.com/tier4/edge-auto.git
cd edge-auto

./setup-dev-env.sh

cd autoware
mkdir src
vcs import src < autoware.repos

source /opt/ros/humble/setup.bash
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to edge_auto_launch
```
