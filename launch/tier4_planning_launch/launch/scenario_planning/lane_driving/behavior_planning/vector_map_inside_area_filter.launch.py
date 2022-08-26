# Copyright 2022 TIER IV, Inc. All rights reserved.
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
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::VectorMapInsideAreaFilterComponent",
            name="vector_map_inside_area_filter_node",
            remappings=[
                ("input", "/perception/obstacle_segmentation/pointcloud"),
                ("input/vector_map", "/map/vector_map"),
                ("output", "vector_map_inside_area_filtered/pointcloud"),
            ],
            parameters=[
                {
                    "polygon_type": LaunchConfiguration("polygon_type"),
                }
            ],
            # this node has QoS of transient local
            extra_arguments=[{"use_intra_process_comms": False}],
        ),
    ]

    vector_map_area_filter_container = ComposableNodeContainer(
        name="vector_map_area_filter_container",
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
            add_launch_arg("use_multithread", "true"),
            add_launch_arg("use_pointcloud_container", "true"),
            add_launch_arg("container_name", "vector_map_area_filter_container"),
            set_container_executable,
            set_container_mt_executable,
            vector_map_area_filter_container,
            load_composable_nodes,
        ]
    )
