# Copyright 2021-2022 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch lidar segmentation node."""

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory("lidar_apollo_segmentation_tvm_nodes"), "param/test.param.yaml"
    )
    with open(param_file, "r") as f:
        lidar_apollo_segmentation_tvm_node_params = yaml.safe_load(f)["/**"]["ros__parameters"]

    arguments = [
        DeclareLaunchArgument("input/pointcloud", default_value="/sensing/lidar/pointcloud"),
        DeclareLaunchArgument("output/objects", default_value="labeled_clusters"),
    ]

    # lidar segmentation node execution definition.
    lidar_apollo_segmentation_tvm_node_runner = Node(
        package="lidar_apollo_segmentation_tvm_nodes",
        executable="lidar_apollo_segmentation_tvm_nodes_exe",
        remappings=[
            ("points_in", LaunchConfiguration("input/pointcloud")),
            ("objects_out", LaunchConfiguration("output/objects")),
        ],
        parameters=[lidar_apollo_segmentation_tvm_node_params],
        output="screen",
    )

    return launch.LaunchDescription(arguments + [lidar_apollo_segmentation_tvm_node_runner])
