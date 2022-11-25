# Copyright 2022 The Autoware Contributors
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    config_file_path = LaunchConfiguration("config_file").perform(context)
    with open(config_file_path, "r") as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]

    component = ComposableNode(
        package="mrm_emergency_stop_operator",
        plugin="mrm_emergency_stop_operator::MrmEmergencyStopOperator",
        name="mrm_emergency_stop_operator",
        parameters=[
            params,
        ],
        remappings=[
            ("~/input/mrm/emergency_stop/operate", "/system/mrm/emergency_stop/operate"),
            ("~/input/control/control_cmd", "/control/command/control_cmd"),
            ("~/output/mrm/emergency_stop/status", "/system/mrm/emergency_stop/status"),
            ("~/output/mrm/emergency_stop/control_cmd", "/system/emergency/control_cmd"),
        ],
    )

    container = ComposableNodeContainer(
        name="mrm_emergency_stop_operator_container",
        namespace="mrm_emergency_stop_operator",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            component,
        ],
        output="screen",
    )

    return [container]


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=[
                FindPackageShare("mrm_emergency_stop_operator"),
                "/config/mrm_emergency_stop_operator.param.yaml",
            ],
            description="path to the parameter file of mrm_emergency_stop_operator",
        )
    ]

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
