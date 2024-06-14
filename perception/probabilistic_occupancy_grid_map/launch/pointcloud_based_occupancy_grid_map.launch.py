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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def get_downsample_filter_node(setting: dict) -> ComposableNode:
    plugin_str = setting["plugin"]
    voxel_size = setting["voxel_size"]
    node_name = setting["node_name"]
    return ComposableNode(
        package="pointcloud_preprocessor",
        plugin=plugin_str,
        name=node_name,
        remappings=[
            ("input", setting["input_topic"]),
            ("output", setting["output_topic"]),
        ],
        parameters=[
            {
                "voxel_size_x": voxel_size,
                "voxel_size_y": voxel_size,
                "voxel_size_z": voxel_size,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )


def get_downsample_preprocess_nodes(voxel_size: float) -> list:
    raw_settings = {
        "plugin": "pointcloud_preprocessor::PickupBasedVoxelGridDownsampleFilterComponent",
        "node_name": "raw_pc_downsample_filter",
        "input_topic": LaunchConfiguration("input/raw_pointcloud"),
        "output_topic": "raw/downsample/pointcloud",
        "voxel_size": voxel_size,
    }
    obstacle_settings = {
        "plugin": "pointcloud_preprocessor::PickupBasedVoxelGridDownsampleFilterComponent",
        "node_name": "obstacle_pc_downsample_filter",
        "input_topic": LaunchConfiguration("input/obstacle_pointcloud"),
        "output_topic": "obstacle/downsample/pointcloud",
        "voxel_size": voxel_size,
    }
    return [get_downsample_filter_node(raw_settings), get_downsample_filter_node(obstacle_settings)]


def launch_setup(context, *args, **kwargs):
    # load parameter files
    param_file = LaunchConfiguration("param_file").perform(context)
    with open(param_file, "r") as f:
        pointcloud_based_occupancy_grid_map_node_params = yaml.safe_load(f)["/**"][
            "ros__parameters"
        ]

    updater_param_file = LaunchConfiguration("updater_param_file").perform(context)
    with open(updater_param_file, "r") as f:
        occupancy_grid_map_updater_params = yaml.safe_load(f)["/**"]["ros__parameters"]

    # composable nodes
    composable_nodes = []

    # add downsample process
    downsample_input_pointcloud: bool = pointcloud_based_occupancy_grid_map_node_params[
        "downsample_input_pointcloud"
    ]
    if downsample_input_pointcloud:
        voxel_grid_size: float = pointcloud_based_occupancy_grid_map_node_params[
            "downsample_voxel_size"
        ]
        downsample_preprocess_nodes = get_downsample_preprocess_nodes(voxel_grid_size)
        composable_nodes.extend(downsample_preprocess_nodes)

    composable_nodes.append(
        ComposableNode(
            package="probabilistic_occupancy_grid_map",
            plugin="occupancy_grid_map::PointcloudBasedOccupancyGridMapNode",
            name="occupancy_grid_map_node",
            remappings=[
                (
                    "~/input/obstacle_pointcloud",
                    (
                        LaunchConfiguration("input/obstacle_pointcloud")
                        if not downsample_input_pointcloud
                        else "obstacle/downsample/pointcloud"
                    ),
                ),
                (
                    "~/input/raw_pointcloud",
                    (
                        LaunchConfiguration("input/raw_pointcloud")
                        if not downsample_input_pointcloud
                        else "raw/downsample/pointcloud"
                    ),
                ),
                ("~/output/occupancy_grid_map", LaunchConfiguration("output")),
            ],
            parameters=[
                pointcloud_based_occupancy_grid_map_node_params,
                occupancy_grid_map_updater_params,
                {"updater_type": LaunchConfiguration("updater_type")},
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    occupancy_grid_map_container = ComposableNodeContainer(
        name=LaunchConfiguration("individual_container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=composable_nodes,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    load_composable_nodes = LoadComposableNodes(
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration("pointcloud_container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    return [occupancy_grid_map_container, load_composable_nodes]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return LaunchDescription(
        [
            add_launch_arg("use_multithread", "false"),
            add_launch_arg("use_intra_process", "true"),
            add_launch_arg("use_pointcloud_container", "false"),
            add_launch_arg("pointcloud_container_name", "pointcloud_container"),
            add_launch_arg("individual_container_name", "occupancy_grid_map_container"),
            add_launch_arg("input/obstacle_pointcloud", "no_ground/oneshot/pointcloud"),
            add_launch_arg("input/raw_pointcloud", "concatenated/pointcloud"),
            add_launch_arg("output", "occupancy_grid"),
            add_launch_arg(
                "param_file",
                get_package_share_directory("probabilistic_occupancy_grid_map")
                + "/config/pointcloud_based_occupancy_grid_map.param.yaml",
            ),
            add_launch_arg("updater_type", "binary_bayes_filter"),
            add_launch_arg(
                "updater_param_file",
                get_package_share_directory("probabilistic_occupancy_grid_map")
                + "/config/binary_bayes_filter_updater.param.yaml",
            ),
            set_container_executable,
            set_container_mt_executable,
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
