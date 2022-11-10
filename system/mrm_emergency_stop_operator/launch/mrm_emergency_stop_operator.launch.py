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

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():
    param_path = os.path.join(
        get_package_share_directory("mrm_emergency_stop_operator"),
        "config/mrm_emergency_stop_operator.config.yaml",
    )

    with open(param_path, "r") as f:
        param = yaml.safe_load(f)["/**"]["ros__parameters"]

    component = ComposableNode(
        package="mrm_emergency_stop_operator",
        plugin="mrm_emergency_stop_operator::MrmEmergencyStopOperator",
        name="mrm_emergency_stop_operator",
        parameters=[
            param,
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

    return launch.LaunchDescription(
        [
            container,
        ]
    )
