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
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    vehicle_info_param_path = LaunchConfiguration("vehicle_info_param_file").perform(context)
    with open(vehicle_info_param_path, "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    lat_controller_param_path = LaunchConfiguration("lat_controller_param_path").perform(context)
    with open(lat_controller_param_path, "r") as f:
        lat_controller_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    lon_controller_param_path = LaunchConfiguration("lon_controller_param_path").perform(context)
    with open(lon_controller_param_path, "r") as f:
        lon_controller_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    latlon_muxer_param_path = LaunchConfiguration("latlon_muxer_param_path").perform(context)
    with open(latlon_muxer_param_path, "r") as f:
        latlon_muxer_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    vehicle_cmd_gate_param_path = LaunchConfiguration("vehicle_cmd_gate_param_path").perform(
        context
    )
    with open(vehicle_cmd_gate_param_path, "r") as f:
        vehicle_cmd_gate_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    lane_departure_checker_param_path = LaunchConfiguration(
        "lane_departure_checker_param_path"
    ).perform(context)
    with open(lane_departure_checker_param_path, "r") as f:
        lane_departure_checker_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # lateral controller
    lat_controller_component = ComposableNode(
        package="trajectory_follower_nodes",
        plugin="autoware::motion::control::trajectory_follower_nodes::LateralController",
        name="lateral_controller_node_exe",
        namespace="trajectory_follower",
        remappings=[
            ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
            ("~/input/current_odometry", "/localization/kinematic_state"),
            ("~/input/current_steering", "/vehicle/status/steering_status"),
            ("~/output/control_cmd", "lateral/control_cmd"),
            ("~/output/predicted_trajectory", "lateral/predicted_trajectory"),
            ("~/output/diagnostic", "lateral/diagnostic"),
        ],
        parameters=[
            lat_controller_param,
            vehicle_info_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # longitudinal controller
    lon_controller_component = ComposableNode(
        package="trajectory_follower_nodes",
        plugin="autoware::motion::control::trajectory_follower_nodes::LongitudinalController",
        name="longitudinal_controller_node_exe",
        namespace="trajectory_follower",
        remappings=[
            ("~/input/current_trajectory", "/planning/scenario_planning/trajectory"),
            ("~/input/current_odometry", "/localization/kinematic_state"),
            ("~/output/control_cmd", "longitudinal/control_cmd"),
            ("~/output/slope_angle", "longitudinal/slope_angle"),
            ("~/output/diagnostic", "longitudinal/diagnostic"),
        ],
        parameters=[
            lon_controller_param,
            vehicle_info_param,
            {
                "control_rate": LaunchConfiguration("control_rate"),
                "show_debug_info": LaunchConfiguration("show_debug_info"),
                "enable_smooth_stop": LaunchConfiguration("enable_smooth_stop"),
                "enable_pub_debug": LaunchConfiguration("enable_pub_debug"),
            },
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # latlon muxer
    latlon_muxer_component = ComposableNode(
        package="trajectory_follower_nodes",
        plugin="autoware::motion::control::trajectory_follower_nodes::LatLonMuxer",
        name="latlon_muxer_node_exe",
        namespace="trajectory_follower",
        remappings=[
            ("~/input/lateral/control_cmd", "lateral/control_cmd"),
            ("~/input/longitudinal/control_cmd", "longitudinal/control_cmd"),
            ("~/output/control_cmd", "control_cmd"),
        ],
        parameters=[
            latlon_muxer_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # lane departure checker
    lane_departure_component = ComposableNode(
        package="lane_departure_checker",
        plugin="lane_departure_checker::LaneDepartureCheckerNode",
        name="lane_departure_checker_node",
        namespace="trajectory_follower",
        remappings=[
            ("~/input/odometry", "/localization/kinematic_state"),
            ("~/input/lanelet_map_bin", "/map/vector_map"),
            ("~/input/route", "/planning/mission_planning/route"),
            ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
            (
                "~/input/predicted_trajectory",
                "/control/trajectory_follower/lateral/predicted_trajectory",
            ),
        ],
        parameters=[lane_departure_checker_param, vehicle_info_param],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # shift decider
    shift_decider_component = ComposableNode(
        package="shift_decider",
        plugin="ShiftDecider",
        name="shift_decider",
        remappings=[
            ("input/control_cmd", "/control/trajectory_follower/control_cmd"),
            ("output/gear_cmd", "/control/shift_decider/gear_cmd"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # vehicle cmd gate
    vehicle_cmd_gate_component = ComposableNode(
        package="vehicle_cmd_gate",
        plugin="VehicleCmdGate",
        name="vehicle_cmd_gate",
        remappings=[
            ("input/emergency_state", "/system/emergency/emergency_state"),
            ("input/steering", "/vehicle/status/steering_status"),
            ("input/auto/control_cmd", "/control/trajectory_follower/control_cmd"),
            ("input/auto/turn_indicators_cmd", "/planning/turn_indicators_cmd"),
            ("input/auto/hazard_lights_cmd", "/planning/hazard_lights_cmd"),
            ("input/auto/gear_cmd", "/control/shift_decider/gear_cmd"),
            ("input/external/control_cmd", "/external/selected/control_cmd"),
            ("input/external/turn_indicators_cmd", "/external/selected/turn_indicators_cmd"),
            ("input/external/hazard_lights_cmd", "/external/selected/hazard_lights_cmd"),
            ("input/external/gear_cmd", "/external/selected/gear_cmd"),
            ("input/external_emergency_stop_heartbeat", "/external/selected/heartbeat"),
            ("input/gate_mode", "/control/gate_mode_cmd"),
            ("input/emergency/control_cmd", "/system/emergency/control_cmd"),
            ("input/emergency/hazard_lights_cmd", "/system/emergency/hazard_lights_cmd"),
            ("input/emergency/gear_cmd", "/system/emergency/gear_cmd"),
            ("output/vehicle_cmd_emergency", "/control/command/emergency_cmd"),
            ("output/control_cmd", "/control/command/control_cmd"),
            ("output/gear_cmd", "/control/command/gear_cmd"),
            ("output/turn_indicators_cmd", "/control/command/turn_indicators_cmd"),
            ("output/hazard_lights_cmd", "/control/command/hazard_lights_cmd"),
            ("output/gate_mode", "/control/current_gate_mode"),
            ("output/engage", "/api/autoware/get/engage"),
            ("output/external_emergency", "/api/autoware/get/emergency"),
            ("~/service/engage", "/api/autoware/set/engage"),
            ("~/service/external_emergency", "/api/autoware/set/emergency"),
            # TODO(Takagi, Isamu): deprecated
            ("input/engage", "/autoware/engage"),
            ("~/service/external_emergency_stop", "~/external_emergency_stop"),
            ("~/service/clear_external_emergency_stop", "~/clear_external_emergency_stop"),
        ],
        parameters=[
            vehicle_cmd_gate_param,
            vehicle_info_param,
            {
                "use_emergency_handling": LaunchConfiguration("use_emergency_handling"),
                "use_external_emergency_stop": LaunchConfiguration("use_external_emergency_stop"),
                "use_start_request": LaunchConfiguration("use_start_request"),
            },
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # external cmd selector
    external_cmd_selector_loader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("external_cmd_selector"), "/launch/external_cmd_selector.launch.py"]
        ),
        launch_arguments=[
            ("use_intra_process", LaunchConfiguration("use_intra_process")),
            ("target_container", "/control/control_container"),
            ("initial_selector_mode", "remote"),
        ],
    )

    # external cmd converter
    external_cmd_converter_loader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("external_cmd_converter"), "/launch/external_cmd_converter.launch.py"]
        ),
        launch_arguments=[
            ("use_intra_process", LaunchConfiguration("use_intra_process")),
            ("target_container", "/control/control_container"),
        ],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="control_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            lon_controller_component,
            latlon_muxer_component,
            lane_departure_component,
            shift_decider_component,
            vehicle_cmd_gate_component,
        ],
    )

    # lateral controller is separated since it may be another controller (e.g. pure pursuit)
    lat_controller_loader = LoadComposableNodes(
        composable_node_descriptions=[lat_controller_component],
        target_container=container,
        # condition=LaunchConfigurationEquals("lateral_controller_mode", "mpc"),
    )

    group = GroupAction(
        [
            PushRosNamespace("control"),
            container,
            external_cmd_selector_loader,
            external_cmd_converter_loader,
            lat_controller_loader,
        ]
    )

    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # add_launch_arg(
    #     "lateral_controller_mode",
    #     "mpc_follower",
    #     "lateral controller mode: `mpc_follower` or `pure_pursuit`",
    # )

    add_launch_arg(
        "vehicle_info_param_file",
        [
            FindPackageShare("vehicle_info_util"),
            "/config/vehicle_info.param.yaml",
        ],
        "path to the parameter file of vehicle information",
    )

    add_launch_arg(
        "lat_controller_param_path",
        [
            FindPackageShare("control_launch"),
            "/config/trajectory_follower/lateral_controller.param.yaml",
        ],
        "path to the parameter file of lateral controller",
    )
    add_launch_arg(
        "lon_controller_param_path",
        [
            FindPackageShare("control_launch"),
            "/config/trajectory_follower/longitudinal_controller.param.yaml",
        ],
        "path to the parameter file of longitudinal controller",
    )
    add_launch_arg(
        "latlon_muxer_param_path",
        [
            FindPackageShare("control_launch"),
            "/config/trajectory_follower/latlon_muxer.param.yaml",
        ],
        "path to the parameter file of latlon muxer",
    )
    add_launch_arg(
        "vehicle_cmd_gate_param_path",
        [
            FindPackageShare("control_launch"),
            "/config/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml",
        ],
        "path to the parameter file of vehicle_cmd_gate",
    )
    add_launch_arg(
        "lane_departure_checker_param_path",
        [FindPackageShare("lane_departure_checker"), "/config/lane_departure_checker.param.yaml"],
    )

    # velocity controller
    add_launch_arg("control_rate", "30.0", "control rate")
    add_launch_arg("show_debug_info", "false", "show debug information")
    add_launch_arg(
        "enable_smooth_stop", "true", "enable smooth stop (in velocity controller state)"
    )
    add_launch_arg("enable_pub_debug", "true", "enable to publish debug information")

    # vehicle cmd gate
    add_launch_arg("use_emergency_handling", "false", "use emergency handling")
    add_launch_arg("use_external_emergency_stop", "true", "use external emergency stop")
    add_launch_arg("use_start_request", "false", "use start request service")

    # external cmd selector
    add_launch_arg("initial_selector_mode", "remote", "local or remote")

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
