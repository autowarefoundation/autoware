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
    with open(LaunchConfiguration("vehicle_param_file").perform(context), "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    with open(LaunchConfiguration("nearest_search_param_path").perform(context), "r") as f:
        nearest_search_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    with open(
        LaunchConfiguration("trajectory_follower_node_param_path").perform(context), "r"
    ) as f:
        trajectory_follower_node_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("lat_controller_param_path").perform(context), "r") as f:
        lat_controller_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("lon_controller_param_path").perform(context), "r") as f:
        lon_controller_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("vehicle_cmd_gate_param_path").perform(context), "r") as f:
        vehicle_cmd_gate_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("lane_departure_checker_param_path").perform(context), "r") as f:
        lane_departure_checker_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("control_validator_param_path").perform(context), "r") as f:
        control_validator_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(
        LaunchConfiguration("operation_mode_transition_manager_param_path").perform(context), "r"
    ) as f:
        operation_mode_transition_manager_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("shift_decider_param_path").perform(context), "r") as f:
        shift_decider_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(
        LaunchConfiguration("obstacle_collision_checker_param_path").perform(context), "r"
    ) as f:
        obstacle_collision_checker_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("aeb_param_path").perform(context), "r") as f:
        aeb_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("predicted_path_checker_param_path").perform(context), "r") as f:
        predicted_path_checker_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    controller_component = ComposableNode(
        package="trajectory_follower_node",
        plugin="autoware::motion::control::trajectory_follower_node::Controller",
        name="controller_node_exe",
        namespace="trajectory_follower",
        remappings=[
            ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
            ("~/input/current_odometry", "/localization/kinematic_state"),
            ("~/input/current_steering", "/vehicle/status/steering_status"),
            ("~/input/current_accel", "/localization/acceleration"),
            ("~/input/current_operation_mode", "/system/operation_mode/state"),
            ("~/output/predicted_trajectory", "lateral/predicted_trajectory"),
            ("~/output/lateral_diagnostic", "lateral/diagnostic"),
            ("~/output/slope_angle", "longitudinal/slope_angle"),
            ("~/output/longitudinal_diagnostic", "longitudinal/diagnostic"),
            ("~/output/control_cmd", "control_cmd"),
        ],
        parameters=[
            {
                "lateral_controller_mode": LaunchConfiguration("lateral_controller_mode"),
                "longitudinal_controller_mode": LaunchConfiguration("longitudinal_controller_mode"),
            },
            nearest_search_param,
            trajectory_follower_node_param,
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
    # control validator checker
    control_validator_component = ComposableNode(
        package="control_validator",
        plugin="control_validator::ControlValidator",
        name="control_validator",
        remappings=[
            ("~/input/kinematics", "/localization/kinematic_state"),
            ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
            (
                "~/input/predicted_trajectory",
                "/control/trajectory_follower/lateral/predicted_trajectory",
            ),
            ("~/output/validation_status", "~/validation_status"),
        ],
        parameters=[control_validator_param],
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
            ("input/current_gear", "/vehicle/status/gear_status"),
            ("output/gear_cmd", "/control/shift_decider/gear_cmd"),
        ],
        parameters=[
            shift_decider_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # autonomous emergency braking
    autonomous_emergency_braking = ComposableNode(
        package="autonomous_emergency_braking",
        plugin="autoware::motion::control::autonomous_emergency_braking::AEB",
        name="autonomous_emergency_braking",
        remappings=[
            ("~/input/pointcloud", "/perception/obstacle_segmentation/pointcloud"),
            ("~/input/velocity", "/vehicle/status/velocity_status"),
            ("~/input/imu", "/sensing/imu/imu_data"),
            ("~/input/odometry", "/localization/kinematic_state"),
            (
                "~/input/predicted_trajectory",
                "/control/trajectory_follower/lateral/predicted_trajectory",
            ),
        ],
        parameters=[
            aeb_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    autonomous_emergency_braking_loader = LoadComposableNodes(
        condition=IfCondition(LaunchConfiguration("enable_autonomous_emergency_braking")),
        composable_node_descriptions=[autonomous_emergency_braking],
        target_container="/control/control_container",
    )

    # autonomous emergency braking
    predicted_path_checker = ComposableNode(
        package="predicted_path_checker",
        plugin="autoware::motion::control::predicted_path_checker::PredictedPathCheckerNode",
        name="predicted_path_checker",
        remappings=[
            ("~/input/objects", "/perception/object_recognition/objects"),
            ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
            ("~/input/current_accel", "/localization/acceleration"),
            ("~/input/odometry", "/localization/kinematic_state"),
            (
                "~/input/predicted_trajectory",
                "/control/trajectory_follower/lateral/predicted_trajectory",
            ),
        ],
        parameters=[
            vehicle_info_param,
            predicted_path_checker_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    predicted_path_checker_loader = LoadComposableNodes(
        condition=IfCondition(LaunchConfiguration("enable_predicted_path_checker")),
        composable_node_descriptions=[predicted_path_checker],
        target_container="/control/control_container",
    )

    # vehicle cmd gate
    vehicle_cmd_gate_component = ComposableNode(
        package="vehicle_cmd_gate",
        plugin="vehicle_cmd_gate::VehicleCmdGate",
        name="vehicle_cmd_gate",
        remappings=[
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
            ("input/kinematics", "/localization/kinematic_state"),
            ("input/acceleration", "/localization/acceleration"),
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
                "check_external_emergency_heartbeat": LaunchConfiguration(
                    "check_external_emergency_heartbeat"
                ),
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
            ("trajectory_follower_control_cmd", "/control/trajectory_follower/control_cmd"),
            ("control_mode_report", "/vehicle/status/control_mode"),
            ("gate_operation_mode", "/control/vehicle_cmd_gate/operation_mode"),
            # output
            ("is_autonomous_available", "/control/is_autonomous_available"),
            ("control_mode_request", "/control/control_mode_request"),
        ],
        parameters=[
            nearest_search_param,
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
            (
                "external_cmd_selector_param_path",
                LaunchConfiguration("external_cmd_selector_param_path"),
            ),
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

    glog_component = ComposableNode(
        package="glog_component",
        plugin="GlogComponent",
        name="glog_component",
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="control_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            controller_component,
            control_validator_component,
            lane_departure_component,
            shift_decider_component,
            vehicle_cmd_gate_component,
            operation_mode_transition_manager_component,
            glog_component,
        ],
    )

    group = GroupAction(
        [
            PushRosNamespace("control"),
            container,
            external_cmd_selector_loader,
            external_cmd_converter_loader,
            obstacle_collision_checker_loader,
            autonomous_emergency_braking_loader,
            predicted_path_checker_loader,
        ]
    )

    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # option
    add_launch_arg("vehicle_param_file")
    add_launch_arg("vehicle_id")
    add_launch_arg("enable_obstacle_collision_checker")
    add_launch_arg("lateral_controller_mode")
    add_launch_arg("longitudinal_controller_mode")
    # common param path
    add_launch_arg("nearest_search_param_path")
    # package param path
    add_launch_arg("trajectory_follower_node_param_path")
    add_launch_arg("lat_controller_param_path")
    add_launch_arg("lon_controller_param_path")
    add_launch_arg("vehicle_cmd_gate_param_path")
    add_launch_arg("lane_departure_checker_param_path")
    add_launch_arg("control_validator_param_path")
    add_launch_arg("operation_mode_transition_manager_param_path")
    add_launch_arg("shift_decider_param_path")
    add_launch_arg("obstacle_collision_checker_param_path")
    add_launch_arg("external_cmd_selector_param_path")
    add_launch_arg("aeb_param_path")
    add_launch_arg("predicted_path_checker_param_path")
    add_launch_arg("enable_predicted_path_checker")
    add_launch_arg("enable_autonomous_emergency_braking")
    add_launch_arg("check_external_emergency_heartbeat")

    # component
    add_launch_arg("use_intra_process", "false", "use ROS 2 component container communication")
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
