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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _create_mapping_tuple(name):
    return (name, LaunchConfiguration(name))


def generate_launch_description():
    arguments = [
        # component
        DeclareLaunchArgument("use_intra_process"),
        DeclareLaunchArgument("target_container"),
        # map file
        DeclareLaunchArgument(
            "csv_path_accel_map",
            default_value=[
                FindPackageShare("autoware_raw_vehicle_cmd_converter"),
                "/data/default/accel_map.csv",
            ],  # noqa: E501
            description="csv file path for accel map",
        ),
        DeclareLaunchArgument(
            "csv_path_brake_map",
            default_value=[
                FindPackageShare("autoware_raw_vehicle_cmd_converter"),
                "/data/default/brake_map.csv",
            ],  # noqa: E501
            description="csv file path for brake map",
        ),
        # settings
        DeclareLaunchArgument(
            "ref_vel_gain",
            default_value="3.0",
            description="gain for external command accel",
        ),
        DeclareLaunchArgument(
            "timer_rate",
            default_value="10.0",
            description="timer's update rate",
        ),
        DeclareLaunchArgument(
            "wait_for_first_topic",
            default_value="true",
            description="disable topic disruption detection until subscribing first topics",
        ),
        DeclareLaunchArgument(
            "control_command_timeout",
            default_value="1.0",
            description="external control command timeout",
        ),
        DeclareLaunchArgument(
            "emergency_stop_timeout",
            default_value="3.0",
            description="emergency stop timeout for external heartbeat",
        ),
        # input
        DeclareLaunchArgument(
            "in/external_control_cmd",
            default_value="/external/selected/external_control_cmd",
        ),
        DeclareLaunchArgument(
            "in/shift_cmd",
            default_value="/external/selected/gear_cmd",
        ),
        DeclareLaunchArgument(
            "in/emergency_stop",
            default_value="/external/selected/heartbeat",
        ),
        DeclareLaunchArgument(
            "in/current_gate_mode",
            default_value="/control/current_gate_mode",
        ),
        DeclareLaunchArgument(
            "in/odometry",
            default_value="/localization/kinematic_state",
        ),
        # output
        DeclareLaunchArgument(
            "out/control_cmd",
            default_value="/external/selected/control_cmd",
        ),
        DeclareLaunchArgument(
            "out/latest_external_control_cmd",
            default_value="/api/external/get/command/selected/control",
        ),
    ]

    component = ComposableNode(
        package="autoware_external_cmd_converter",
        plugin="autoware::external_cmd_converter::ExternalCmdConverterNode",
        name="external_cmd_converter",
        remappings=[
            _create_mapping_tuple("in/external_control_cmd"),
            _create_mapping_tuple("in/shift_cmd"),
            _create_mapping_tuple("in/emergency_stop"),
            _create_mapping_tuple("in/current_gate_mode"),
            _create_mapping_tuple("in/odometry"),
            _create_mapping_tuple("out/control_cmd"),
            _create_mapping_tuple("out/latest_external_control_cmd"),
        ],
        parameters=[
            dict(  # noqa: C406 for using helper function
                [
                    _create_mapping_tuple("csv_path_accel_map"),
                    _create_mapping_tuple("csv_path_brake_map"),
                    _create_mapping_tuple("ref_vel_gain"),
                    _create_mapping_tuple("timer_rate"),
                    _create_mapping_tuple("wait_for_first_topic"),
                    _create_mapping_tuple("control_command_timeout"),
                    _create_mapping_tuple("emergency_stop_timeout"),
                ]
            )
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

    return LaunchDescription(arguments + [loader])
