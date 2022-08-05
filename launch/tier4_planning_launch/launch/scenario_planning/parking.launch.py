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

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):

    vehicle_info_param_path = LaunchConfiguration("vehicle_info_param_file").perform(context)
    with open(vehicle_info_param_path, "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    freespace_planner_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "parking",
        "freespace_planner",
        "freespace_planner.param.yaml",
    )
    with open(freespace_planner_param_path, "r") as f:
        freespace_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    container = ComposableNodeContainer(
        name="parking_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            ComposableNode(
                package="costmap_generator",
                plugin="CostmapGenerator",
                name="costmap_generator",
                remappings=[
                    ("~/input/objects", "/perception/object_recognition/objects"),
                    (
                        "~/input/points_no_ground",
                        "/perception/obstacle_segmentation/pointcloud",
                    ),
                    ("~/input/vector_map", "/map/vector_map"),
                    ("~/input/scenario", "/planning/scenario_planning/scenario"),
                    ("~/output/grid_map", "costmap_generator/grid_map"),
                    ("~/output/occupancy_grid", "costmap_generator/occupancy_grid"),
                ],
                parameters=[
                    {
                        "costmap_frame": "map",
                        "vehicle_frame": "base_link",
                        "map_frame": "map",
                        "update_rate": 10.0,
                        "use_wayarea": True,
                        "use_parkinglot": True,
                        "use_objects": True,
                        "use_points": True,
                        "grid_min_value": 0.0,
                        "grid_max_value": 1.0,
                        "grid_resolution": 0.2,
                        "grid_length_x": 70.0,
                        "grid_length_y": 70.0,
                        "grid_position_x": 0.0,
                        "grid_position_y": 0.0,
                        "maximum_lidar_height_thres": 0.3,
                        "minimum_lidar_height_thres": -2.2,
                        "expand_polygon_size": 1.0,
                        "size_of_expansion_kernel": 9,
                    },
                    vehicle_info_param,
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
            ComposableNode(
                package="freespace_planner",
                plugin="freespace_planner::FreespacePlannerNode",
                name="freespace_planner",
                remappings=[
                    ("~/input/route", "/planning/mission_planning/route"),
                    ("~/input/occupancy_grid", "costmap_generator/occupancy_grid"),
                    ("~/input/scenario", "/planning/scenario_planning/scenario"),
                    ("~/input/odometry", "/localization/kinematic_state"),
                    ("~/output/trajectory", "/planning/scenario_planning/parking/trajectory"),
                    ("is_completed", "/planning/scenario_planning/parking/is_completed"),
                ],
                parameters=[
                    freespace_planner_param,
                    vehicle_info_param,
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
    )

    group = GroupAction(
        [
            PushRosNamespace("parking"),
            container,
        ]
    )

    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg(
        "vehicle_info_param_file",
        [
            FindPackageShare("vehicle_info_util"),
            "/config/vehicle_info.param.yaml",
        ],
        "path to the parameter file of vehicle information",
    )

    # component
    add_launch_arg("use_intra_process", "false", "use ROS2 component container communication")
    add_launch_arg("use_multithread", "false", "use multithread")

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
