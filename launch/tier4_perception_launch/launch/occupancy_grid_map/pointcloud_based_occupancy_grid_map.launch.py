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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


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

    composable_nodes = [
        ComposableNode(
            package="probabilistic_occupancy_grid_map",
            plugin="occupancy_grid_map::PointcloudBasedOccupancyGridMapNode",
            name="occupancy_grid_map_node",
            remappings=[
                ("~/input/obstacle_pointcloud", LaunchConfiguration("input/obstacle_pointcloud")),
                ("~/input/raw_pointcloud", LaunchConfiguration("input/raw_pointcloud")),
                ("~/output/occupancy_grid_map", LaunchConfiguration("output")),
            ],
            parameters=[
                {
                    "map_resolution": 0.5,
                    "use_height_filter": True,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
    ]

    occupancy_grid_map_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=composable_nodes,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    load_composable_nodes = LoadComposableNodes(
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    return LaunchDescription(
        [
            add_launch_arg("use_multithread", "False"),
            add_launch_arg("use_intra_process", "True"),
            add_launch_arg("use_pointcloud_container", "False"),
            add_launch_arg("container_name", "occupancy_grid_map_container"),
            add_launch_arg("input/obstacle_pointcloud", "no_ground/oneshot/pointcloud"),
            add_launch_arg("input/raw_pointcloud", "concatenated/pointcloud"),
            add_launch_arg("output", "occupancy_grid"),
            set_container_executable,
            set_container_mt_executable,
            occupancy_grid_map_container,
            load_composable_nodes,
        ]
    )
