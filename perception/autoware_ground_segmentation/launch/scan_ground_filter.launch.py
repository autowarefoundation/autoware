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

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def launch_setup(context, *args, **kwargs):
    vehicle_info_param_path = LaunchConfiguration("vehicle_info_param_file").perform(context)
    with open(vehicle_info_param_path, "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    ground_segmentation_param_path = os.path.join(
        get_package_share_directory("autoware_ground_segmentation"),
        "config",
        "ground_segmentation.param.yaml",
    )

    with open(ground_segmentation_param_path, "r") as f:
        ground_segmentation_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    nodes = [
        ComposableNode(
            package="autoware_ground_segmentation",
            plugin="autoware::ground_segmentation::ScanGroundFilterComponent",
            name="scan_ground_filter",
            remappings=[
                ("input", LaunchConfiguration("input/pointcloud")),
                ("output", LaunchConfiguration("output/pointcloud")),
            ],
            parameters=[
                ground_segmentation_param["common_ground_filter"]["parameters"],
                {"input_frame": "base_link"},
                {"output_frame": "base_link"},
                vehicle_info_param,
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

    group = GroupAction(
        [
            container,
            loader,
        ]
    )

    return [group]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    default_vehicle_info_param = os.path.join(
        get_package_share_directory("autoware_vehicle_info_utils"), "config/vehicle_info.param.yaml"
    )

    vehicle_info_param = DeclareLaunchArgument(
        "vehicle_info_param_file",
        default_value=default_vehicle_info_param,
        description="Path to config file for vehicle information",
    )

    return launch.LaunchDescription(
        [
            vehicle_info_param,
            add_launch_arg("container", ""),
            add_launch_arg("input/pointcloud", "pointcloud"),
            add_launch_arg("output/pointcloud", "no_ground/pointcloud"),
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
