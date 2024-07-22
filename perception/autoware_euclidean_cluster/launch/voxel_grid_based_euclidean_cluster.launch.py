# Copyright 2020 Tier IV, Inc. All rights reserved.
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
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    # https://github.com/ros2/launch_ros/issues/156
    def load_composable_node_param(param_path):
        with open(LaunchConfiguration(param_path).perform(context), "r") as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]

    ns = ""
    pkg = "autoware_euclidean_cluster"

    low_height_cropbox_filter_component = ComposableNode(
        package="autoware_pointcloud_preprocessor",
        namespace=ns,
        plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
        name="low_height_crop_box_filter",
        remappings=[
            ("input", LaunchConfiguration("input_pointcloud")),
            ("output", "low_height/pointcloud"),
        ],
        parameters=[load_composable_node_param("voxel_grid_based_euclidean_param_path")],
    )

    use_low_height_euclidean_component = ComposableNode(
        package=pkg,
        namespace=ns,
        plugin="autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterNode",
        name="euclidean_cluster",
        remappings=[
            ("input", "low_height/pointcloud"),
            ("output", LaunchConfiguration("output_clusters")),
        ],
        parameters=[load_composable_node_param("voxel_grid_based_euclidean_param_path")],
    )

    disuse_low_height_euclidean_component = ComposableNode(
        package=pkg,
        namespace=ns,
        plugin="autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterNode",
        name="euclidean_cluster",
        remappings=[
            ("input", LaunchConfiguration("input_pointcloud")),
            ("output", LaunchConfiguration("output_clusters")),
        ],
        parameters=[load_composable_node_param("voxel_grid_based_euclidean_param_path")],
    )

    container = ComposableNodeContainer(
        name="euclidean_cluster_container",
        package="rclcpp_components",
        namespace=ns,
        executable="component_container",
        composable_node_descriptions=[],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    target_container = (
        LaunchConfiguration("pointcloud_container_name")
        if IfCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else container
    )

    use_low_height_pointcloud_loader = LoadComposableNodes(
        composable_node_descriptions=[
            low_height_cropbox_filter_component,
            use_low_height_euclidean_component,
        ],
        target_container=target_container,
        condition=IfCondition(LaunchConfiguration("use_low_height_cropbox")),
    )

    disuse_low_height_pointcloud_loader = LoadComposableNodes(
        composable_node_descriptions=[disuse_low_height_euclidean_component],
        target_container=target_container,
        condition=UnlessCondition(LaunchConfiguration("use_low_height_cropbox")),
    )
    return [
        container,
        use_low_height_pointcloud_loader,
        disuse_low_height_pointcloud_loader,
    ]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    return launch.LaunchDescription(
        [
            add_launch_arg("input_pointcloud", "/perception/obstacle_segmentation/pointcloud"),
            add_launch_arg("input_map", "/map/pointcloud_map"),
            add_launch_arg("output_clusters", "clusters"),
            add_launch_arg("use_low_height_cropbox", "false"),
            add_launch_arg("use_pointcloud_container", "false"),
            add_launch_arg("pointcloud_container_name", "pointcloud_container"),
            add_launch_arg(
                "voxel_grid_based_euclidean_param_path",
                [
                    FindPackageShare("autoware_launch"),
                    "/config/perception/object_recognition/detection/clustering/voxel_grid_based_euclidean_cluster.param.yaml",
                ],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
