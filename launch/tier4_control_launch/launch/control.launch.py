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

import os

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

    lat_controller_param_path = os.path.join(
        LaunchConfiguration("tier4_control_launch_param_path").perform(context),
        "trajectory_follower",
        "lateral_controller.param.yaml",
    )
    with open(lat_controller_param_path, "r") as f:
        lat_controller_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    nearest_search_param_path = os.path.join(
        LaunchConfiguration("tier4_control_launch_param_path").perform(context),
        "common",
        "nearest_search.param.yaml",
    )
    with open(nearest_search_param_path, "r") as f:
        nearest_search_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    lon_controller_param_path = os.path.join(
        LaunchConfiguration("tier4_control_launch_param_path").perform(context),
        "trajectory_follower",
        "longitudinal_controller.param.yaml",
    )
    with open(lon_controller_param_path, "r") as f:
        lon_controller_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    vehicle_cmd_gate_param_path = os.path.join(
        LaunchConfiguration("tier4_control_launch_param_path").perform(context),
        "vehicle_cmd_gate",
        "vehicle_cmd_gate.param.yaml",
    )
    with open(vehicle_cmd_gate_param_path, "r") as f:
        vehicle_cmd_gate_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    lane_departure_checker_param_path = LaunchConfiguration(
        "lane_departure_checker_param_path"
    ).perform(context)
    with open(lane_departure_checker_param_path, "r") as f:
        lane_departure_checker_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    operation_mode_transition_manager_param_path = os.path.join(
        LaunchConfiguration("tier4_control_launch_param_path").perform(context),
        "operation_mode_transition_manager",
        "operation_mode_transition_manager.param.yaml",
    )
    with open(operation_mode_transition_manager_param_path, "r") as f:
        operation_mode_transition_manager_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    shift_decider_param_path = os.path.join(
        LaunchConfiguration("tier4_control_launch_param_path").perform(context),
        "shift_decider",
        "shift_decider.param.yaml",
    )
    with open(shift_decider_param_path, "r") as f:
        shift_decider_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    obstacle_collision_checker_param_path = os.path.join(
        LaunchConfiguration("tier4_control_launch_param_path").perform(context),
        "obstacle_collision_checker",
        "obstacle_collision_checker.param.yaml",
    )

    with open(obstacle_collision_checker_param_path, "r") as f:
        obstacle_collision_checker_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    controller_component = ComposableNode(
        package="trajectory_follower_nodes",
        plugin="autoware::motion::control::trajectory_follower_nodes::Controller",
        name="controller_node_exe",
        namespace="trajectory_follower",
        remappings=[
            ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
            ("~/input/current_odometry", "/localization/kinematic_state"),
            ("~/input/current_steering", "/vehicle/status/steering_status"),
            ("~/input/current_accel", "/localization/acceleration"),
            ("~/output/predicted_trajectory", "lateral/predicted_trajectory"),
            ("~/output/lateral_diagnostic", "lateral/diagnostic"),
            ("~/output/slope_angle", "longitudinal/slope_angle"),
            ("~/output/longitudinal_diagnostic", "longitudinal/diagnostic"),
            ("~/output/control_cmd", "control_cmd"),
        ],
        parameters=[
            {
                "ctrl_period": 0.03,
                "lateral_controller_mode": LaunchConfiguration("lateral_controller_mode"),
            },
            nearest_search_param,
            lon_controller_param,
            lat_controller_param,
            vehicle_info_param,
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
        parameters=[nearest_search_param, lane_departure_checker_param, vehicle_info_param],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # shift decider
    shift_decider_component = ComposableNode(
        package="shift_decider",
        plugin="ShiftDecider",
        name="shift_decider",
        remappings=[
            ("input/control_cmd", "/control/trajectory_follower/control_cmd"),
            ("input/state", "/autoware/state"),
            ("output/gear_cmd", "/control/shift_decider/gear_cmd"),
        ],
        parameters=[
            shift_decider_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # vehicle cmd gate
    vehicle_cmd_gate_component = ComposableNode(
        package="vehicle_cmd_gate",
        plugin="vehicle_cmd_gate::VehicleCmdGate",
        name="vehicle_cmd_gate",
        remappings=[
            ("input/emergency_state", "/system/emergency/emergency_state"),
            ("input/steering", "/vehicle/status/steering_status"),
            ("input/operation_mode", "/system/operation_mode/state"),
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
            ("input/mrm_state", "/system/fail_safe/mrm_state"),
            ("output/vehicle_cmd_emergency", "/control/command/emergency_cmd"),
            ("output/control_cmd", "/control/command/control_cmd"),
            ("output/gear_cmd", "/control/command/gear_cmd"),
            ("output/turn_indicators_cmd", "/control/command/turn_indicators_cmd"),
            ("output/hazard_lights_cmd", "/control/command/hazard_lights_cmd"),
            ("output/gate_mode", "/control/current_gate_mode"),
            ("output/engage", "/api/autoware/get/engage"),
            ("output/external_emergency", "/api/autoware/get/emergency"),
            ("output/operation_mode", "/control/vehicle_cmd_gate/operation_mode"),
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
                "check_external_emergency_heartbeat": LaunchConfiguration(
                    "check_external_emergency_heartbeat"
                ),
                "use_start_request": LaunchConfiguration("use_start_request"),
            },
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # operation mode transition manager
    operation_mode_transition_manager_component = ComposableNode(
        package="operation_mode_transition_manager",
        plugin="operation_mode_transition_manager::OperationModeTransitionManager",
        name="operation_mode_transition_manager",
        remappings=[
            # input
            ("kinematics", "/localization/kinematic_state"),
            ("steering", "/vehicle/status/steering_status"),
            ("trajectory", "/planning/scenario_planning/trajectory"),
            ("control_cmd", "/control/command/control_cmd"),
            ("control_mode_report", "/vehicle/status/control_mode"),
            ("gate_operation_mode", "/control/vehicle_cmd_gate/operation_mode"),
            # output
            ("is_autonomous_available", "/control/is_autonomous_available"),
            ("control_mode_request", "/control/control_mode_request"),
        ],
        parameters=[
            nearest_search_param_path,
            operation_mode_transition_manager_param,
            vehicle_info_param,
        ],
    )

    # external cmd selector
    external_cmd_selector_loader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("external_cmd_selector"), "/launch/external_cmd_selector.launch.py"]
        ),
        launch_arguments=[
            ("use_intra_process", LaunchConfiguration("use_intra_process")),
            ("target_container", "/control/control_container"),
            ("initial_selector_mode", LaunchConfiguration("initial_selector_mode")),
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

    # obstacle collision checker
    obstacle_collision_checker_component = ComposableNode(
        package="obstacle_collision_checker",
        plugin="obstacle_collision_checker::ObstacleCollisionCheckerNode",
        name="obstacle_collision_checker",
        remappings=[
            ("input/lanelet_map_bin", "/map/vector_map"),
            ("input/obstacle_pointcloud", "/perception/obstacle_segmentation/pointcloud"),
            ("input/reference_trajectory", "/planning/scenario_planning/trajectory"),
            (
                "input/predicted_trajectory",
                "/control/trajectory_follower/lateral/predicted_trajectory",
            ),
            ("input/odometry", "/localization/kinematic_state"),
        ],
        parameters=[
            obstacle_collision_checker_param,
            vehicle_info_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    obstacle_collision_checker_loader = LoadComposableNodes(
        condition=IfCondition(LaunchConfiguration("enable_obstacle_collision_checker")),
        composable_node_descriptions=[obstacle_collision_checker_component],
        target_container="/control/control_container",
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="control_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            controller_component,
            lane_departure_component,
            shift_decider_component,
            vehicle_cmd_gate_component,
            operation_mode_transition_manager_component,
        ],
    )

    group = GroupAction(
        [
            PushRosNamespace("control"),
            container,
            external_cmd_selector_loader,
            external_cmd_converter_loader,
            obstacle_collision_checker_loader,
        ]
    )

    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # parameter
    add_launch_arg(
        "tier4_control_launch_param_path",
        [
            FindPackageShare("tier4_control_launch"),
            "/config",
        ],
        "tier4_control_launch parameter path",
    )

    # lateral controller
    add_launch_arg(
        "lateral_controller_mode",
        "mpc_follower",
        "lateral controller mode: `mpc_follower` or `pure_pursuit`",
    )

    # longitudinal controller mode
    add_launch_arg(
        "longitudinal_controller_mode",
        "pid",
        "longitudinal controller mode: `pid`",
    )

    add_launch_arg(
        "vehicle_info_param_file",
        [
            FindPackageShare("vehicle_info_util"),
            "/config/vehicle_info.param.yaml",
        ],
        "path to the parameter file of vehicle information",
    )

    add_launch_arg(
        "lane_departure_checker_param_path",
        [FindPackageShare("lane_departure_checker"), "/config/lane_departure_checker.param.yaml"],
    )

    # obstacle collision checker
    add_launch_arg("enable_obstacle_collision_checker", "false", "use obstacle collision checker")

    # velocity controller
    add_launch_arg("show_debug_info", "false", "show debug information")
    add_launch_arg("enable_pub_debug", "true", "enable to publish debug information")

    # vehicle cmd gate
    add_launch_arg("use_emergency_handling", "false", "use emergency handling")
    add_launch_arg("check_external_emergency_heartbeat", "true", "use external emergency stop")
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
