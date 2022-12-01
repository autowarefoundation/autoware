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
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    lanelet2_map_loader_param_path = LaunchConfiguration("lanelet2_map_loader_param_path").perform(
        context
    )
    pointcloud_map_loader_param_path = LaunchConfiguration(
        "pointcloud_map_loader_param_path"
    ).perform(context)

    with open(lanelet2_map_loader_param_path, "r") as f:
        lanelet2_map_loader_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(pointcloud_map_loader_param_path, "r") as f:
        pointcloud_map_loader_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    map_hash_generator = Node(
        package="map_loader",
        executable="map_hash_generator",
        name="map_hash_generator",
        parameters=[
            {
                "lanelet2_map_path": LaunchConfiguration("lanelet2_map_path"),
                "pointcloud_map_path": LaunchConfiguration("pointcloud_map_path"),
            }
        ],
    )

    lanelet2_map_loader = ComposableNode(
        package="map_loader",
        plugin="Lanelet2MapLoaderNode",
        name="lanelet2_map_loader",
        remappings=[("output/lanelet2_map", "vector_map")],
        parameters=[
            {
                "lanelet2_map_path": LaunchConfiguration("lanelet2_map_path"),
            },
            lanelet2_map_loader_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    lanelet2_map_visualization = ComposableNode(
        package="map_loader",
        plugin="Lanelet2MapVisualizationNode",
        name="lanelet2_map_visualization",
        remappings=[
            ("input/lanelet2_map", "vector_map"),
            ("output/lanelet2_map_marker", "vector_map_marker"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    pointcloud_map_loader = ComposableNode(
        package="map_loader",
        plugin="PointCloudMapLoaderNode",
        name="pointcloud_map_loader",
        remappings=[
            ("output/pointcloud_map", "pointcloud_map"),
            ("service/get_partial_pcd_map", "/map/get_partial_pointcloud_map"),
            ("service/get_differential_pcd_map", "/map/get_differential_pointcloud_map"),
        ],
        parameters=[
            {"pcd_paths_or_directory": ["[", LaunchConfiguration("pointcloud_map_path"), "]"]},
            pointcloud_map_loader_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    map_tf_generator = ComposableNode(
        package="map_tf_generator",
        plugin="VectorMapTFGeneratorNode",
        name="vector_map_tf_generator",
        parameters=[
            {
                "map_frame": "map",
                "viewer_frame": "viewer",
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
        name="map_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            lanelet2_map_loader,
            lanelet2_map_visualization,
            pointcloud_map_loader,
            map_tf_generator,
        ],
        output="screen",
    )

    group = GroupAction(
        [
            PushRosNamespace("map"),
            container,
            map_hash_generator,
        ]
    )

    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("map_path", "", "path to map directory"),
    add_launch_arg(
        "lanelet2_map_path",
        [LaunchConfiguration("map_path"), "/lanelet2_map.osm"],
        "path to lanelet2 map file",
    ),
    add_launch_arg(
        "pointcloud_map_path",
        [LaunchConfiguration("map_path"), "/pointcloud_map.pcd"],
        "path to pointcloud map file",
    ),
    add_launch_arg(
        "lanelet2_map_loader_param_path",
        [
            FindPackageShare("map_loader"),
            "/config/lanelet2_map_loader.param.yaml",
        ],
        "path to lanelet2_map_loader param file",
    ),
    add_launch_arg(
        "pointcloud_map_loader_param_path",
        [
            FindPackageShare("tier4_map_launch"),
            "/config/pointcloud_map_loader.param.yaml",
        ],
        "path to pointcloud_map_loader param file",
    ),
    add_launch_arg("use_intra_process", "false", "use ROS2 component container communication"),
    add_launch_arg("use_multithread", "false", "use multithread"),

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

    return launch.LaunchDescription(
        launch_arguments
        + [
            set_container_executable,
            set_container_mt_executable,
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
