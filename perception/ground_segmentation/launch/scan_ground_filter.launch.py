# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    nodes = [
        ComposableNode(
            package="ground_segmentation",
            plugin="ground_segmentation::ScanGroundFilterComponent",
            name="scan_ground_filter",
            remappings=[
                ("input", LaunchConfiguration("input/pointcloud")),
                ("output", LaunchConfiguration("output/pointcloud")),
            ],
            parameters=[
                {
                    "global_slope_max_angle_deg": 10.0,
                    "local_slope_max_angle_deg": 30.0,
                    "split_points_distance_tolerance": 0.2,
                    "split_height_distance": 0.2,
                }
            ],
        ),
    ]

    loader = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals("container", ""),
        composable_node_descriptions=nodes,
        target_container=LaunchConfiguration("container"),
    )

    container = ComposableNodeContainer(
        name="scan_ground_filter_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
        condition=LaunchConfigurationEquals("container", ""),
    )

    return launch.LaunchDescription(
        [
            add_launch_arg("container", ""),
            add_launch_arg("input/pointcloud", "pointcloud"),
            add_launch_arg("output/pointcloud", "no_ground/pointcloud"),
            container,
            loader,
        ]
    )
