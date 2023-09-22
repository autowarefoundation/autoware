#!/bin/bash
rosdep install -y --from-paths `colcon list --packages-up-to edge_auto_launch -p` --ignore-src --skip-keys autoware_launch
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to edge_auto_launch
