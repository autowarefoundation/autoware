# Copyright 2021 Tier IV, Inc.
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
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
import yaml


def _create_mapping_tuple(name):
    return ("~/" + name, LaunchConfiguration(name))


def launch_setup(context, *args, **kwargs):
    with open(LaunchConfiguration("external_cmd_selector_param_path").perform(context), "r") as f:
        external_cmd_selector_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    component = ComposableNode(
        package="autoware_external_cmd_selector",
        plugin="autoware::external_cmd_selector::ExternalCmdSelector",
        name="external_cmd_selector",
        remappings=[
            _create_mapping_tuple("service/select_external_command"),
            _create_mapping_tuple("input/local/control_cmd"),
            _create_mapping_tuple("input/local/shift_cmd"),
            _create_mapping_tuple("input/local/turn_signal_cmd"),
            _create_mapping_tuple("input/local/heartbeat"),
            _create_mapping_tuple("input/remote/control_cmd"),
            _create_mapping_tuple("input/remote/shift_cmd"),
            _create_mapping_tuple("input/remote/turn_signal_cmd"),
            _create_mapping_tuple("input/remote/heartbeat"),
            _create_mapping_tuple("output/control_cmd"),
            _create_mapping_tuple("output/gear_cmd"),
            _create_mapping_tuple("output/turn_indicators_cmd"),
            _create_mapping_tuple("output/hazard_lights_cmd"),
            _create_mapping_tuple("output/heartbeat"),
            _create_mapping_tuple("output/current_selector_mode"),
        ],
        parameters=[
            external_cmd_selector_param,
        ],
        extra_arguments=[
            {
                "use_intra_process_comms": LaunchConfiguration("use_intra_process"),
            }
        ],
    )

    loader = LoadComposableNodes(
        composable_node_descriptions=[component],
        target_container=LaunchConfiguration("target_container"),
    )

    group = GroupAction(
        [
            PushRosNamespace(""),
            loader,
        ]
    )

    return [group]


def generate_launch_description():
    arguments = [
        # component
        DeclareLaunchArgument("use_intra_process"),
        DeclareLaunchArgument("target_container"),
        # service
        DeclareLaunchArgument(
            "service/select_external_command", default_value="~/select_external_command"
        ),
        # local input
        DeclareLaunchArgument(
            "input/local/control_cmd", default_value="/api/external/set/command/local/control"
        ),
        DeclareLaunchArgument(
            "input/local/shift_cmd", default_value="/api/external/set/command/local/shift"
        ),
        DeclareLaunchArgument(
            "input/local/turn_signal_cmd",
            default_value="/api/external/set/command/local/turn_signal",
        ),
        DeclareLaunchArgument(
            "input/local/heartbeat", default_value="/api/external/set/command/local/heartbeat"
        ),
        # remote input
        DeclareLaunchArgument(
            "input/remote/control_cmd", default_value="/api/external/set/command/remote/control"
        ),
        DeclareLaunchArgument(
            "input/remote/shift_cmd", default_value="/api/external/set/command/remote/shift"
        ),
        DeclareLaunchArgument(
            "input/remote/turn_signal_cmd",
            default_value="/api/external/set/command/remote/turn_signal",
        ),
        DeclareLaunchArgument(
            "input/remote/heartbeat", default_value="/api/external/set/command/remote/heartbeat"
        ),
        # output
        DeclareLaunchArgument(
            "output/control_cmd", default_value="/external/selected/external_control_cmd"
        ),
        DeclareLaunchArgument("output/gear_cmd", default_value="/external/selected/gear_cmd"),
        DeclareLaunchArgument(
            "output/turn_indicators_cmd", default_value="/external/selected/turn_indicators_cmd"
        ),
        DeclareLaunchArgument(
            "output/hazard_lights_cmd", default_value="/external/selected/hazard_lights_cmd"
        ),
        DeclareLaunchArgument("output/heartbeat", default_value="/external/selected/heartbeat"),
        DeclareLaunchArgument(
            "output/current_selector_mode", default_value="~/current_selector_mode"
        ),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
