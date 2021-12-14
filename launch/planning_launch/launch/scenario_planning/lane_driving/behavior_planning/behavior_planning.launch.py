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

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):

    # vehicle information parameter
    vehicle_info_param_path = LaunchConfiguration("vehicle_info_param_file").perform(context)
    with open(vehicle_info_param_path, "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # behavior path planner
    side_shift_param_path = os.path.join(
        get_package_share_directory("planning_launch"),
        "config",
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_path_planner",
        "side_shift",
        "side_shift.param.yaml",
    )
    with open(side_shift_param_path, "r") as f:
        side_shift_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    avoidance_param_path = os.path.join(
        get_package_share_directory("planning_launch"),
        "config",
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_path_planner",
        "avoidance",
        "avoidance.param.yaml",
    )
    with open(avoidance_param_path, "r") as f:
        avoidance_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    lane_change_param_path = os.path.join(
        get_package_share_directory("planning_launch"),
        "config",
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_path_planner",
        "lane_change",
        "lane_change.param.yaml",
    )
    with open(lane_change_param_path, "r") as f:
        lane_change_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    lane_following_param_path = os.path.join(
        get_package_share_directory("planning_launch"),
        "config",
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_path_planner",
        "lane_following",
        "lane_following.param.yaml",
    )
    with open(lane_following_param_path, "r") as f:
        lane_following_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    pull_over_param_path = os.path.join(
        get_package_share_directory("planning_launch"),
        "config",
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_path_planner",
        "pull_over",
        "pull_over.param.yaml",
    )
    with open(pull_over_param_path, "r") as f:
        pull_over_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    pull_out_param_path = os.path.join(
        get_package_share_directory("planning_launch"),
        "config",
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_path_planner",
        "pull_out",
        "pull_out.param.yaml",
    )
    with open(pull_out_param_path, "r") as f:
        pull_out_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    behavior_path_planner_param_path = os.path.join(
        get_package_share_directory("planning_launch"),
        "config",
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_path_planner",
        "behavior_path_planner.param.yaml",
    )
    with open(behavior_path_planner_param_path, "r") as f:
        behavior_path_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    behavior_path_planner_component = ComposableNode(
        package="behavior_path_planner",
        plugin="behavior_path_planner::BehaviorPathPlannerNode",
        name="behavior_path_planner",
        namespace="",
        remappings=[
            ("~/input/route", LaunchConfiguration("input_route_topic_name")),
            ("~/input/vector_map", LaunchConfiguration("map_topic_name")),
            ("~/input/perception", "/perception/object_recognition/objects"),
            ("~/input/odometry", "/localization/kinematic_state"),
            (
                "~/input/external_approval",
                "/planning/scenario_planning/lane_driving/behavior_planning/"
                "behavior_path_planner/path_change_approval",
            ),
            (
                "~/input/force_approval",
                "/planning/scenario_planning/lane_driving/behavior_planning/"
                "behavior_path_planner/path_change_force",
            ),
            ("~/output/path", "path_with_lane_id"),
            (
                "~/output/ready",
                "/planning/scenario_planning/lane_driving/behavior_planning/"
                "behavior_path_planner/ready_module",
            ),
            (
                "~/output/running",
                "/planning/scenario_planning/lane_driving/behavior_planning/"
                "behavior_path_planner/running_modules",
            ),
            (
                "~/output/force_available",
                "/planning/scenario_planning/lane_driving/behavior_planning/"
                "behavior_path_planner/force_available",
            ),
            ("~/output/turn_indicators_cmd", "/planning/turn_indicators_cmd"),
            ("~/output/hazard_lights_cmd", "/planning/hazard_lights_cmd"),
        ],
        parameters=[
            side_shift_param,
            avoidance_param,
            lane_change_param,
            lane_following_param,
            pull_over_param,
            pull_out_param,
            behavior_path_planner_param,
            vehicle_info_param,
            {
                "bt_tree_config_path": [
                    FindPackageShare("behavior_path_planner"),
                    "/config/behavior_path_planner_tree.xml",
                ],
                "planning_hz": 10.0,
            },
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # behavior velocity planner
    blind_spot_param_path = os.path.join(
        get_package_share_directory("behavior_velocity_planner"),
        "config",
        "blind_spot.param.yaml",
    )
    with open(blind_spot_param_path, "r") as f:
        blind_spot_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    crosswalk_param_path = os.path.join(
        get_package_share_directory("behavior_velocity_planner"),
        "config",
        "crosswalk.param.yaml",
    )
    with open(crosswalk_param_path, "r") as f:
        crosswalk_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    detection_area_param_path = os.path.join(
        get_package_share_directory("behavior_velocity_planner"),
        "config",
        "detection_area.param.yaml",
    )
    with open(detection_area_param_path, "r") as f:
        detection_area_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    intersection_param_path = os.path.join(
        get_package_share_directory("behavior_velocity_planner"),
        "config",
        "intersection.param.yaml",
    )
    with open(intersection_param_path, "r") as f:
        intersection_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    stop_line_param_path = os.path.join(
        get_package_share_directory("behavior_velocity_planner"),
        "config",
        "stop_line.param.yaml",
    )
    with open(stop_line_param_path, "r") as f:
        stop_line_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    traffic_light_param_path = os.path.join(
        get_package_share_directory("behavior_velocity_planner"),
        "config",
        "traffic_light.param.yaml",
    )
    with open(traffic_light_param_path, "r") as f:
        traffic_light_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    virtual_traffic_light_param_path = os.path.join(
        get_package_share_directory("behavior_velocity_planner"),
        "config",
        "virtual_traffic_light.param.yaml",
    )
    with open(virtual_traffic_light_param_path, "r") as f:
        virtual_traffic_light_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    occlusion_spot_param_path = os.path.join(
        get_package_share_directory("behavior_velocity_planner"),
        "config",
        "occlusion_spot.param.yaml",
    )
    with open(occlusion_spot_param_path, "r") as f:
        occlusion_spot_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    no_stopping_area_param_path = os.path.join(
        get_package_share_directory("behavior_velocity_planner"),
        "config",
        "no_stopping_area.param.yaml",
    )
    with open(no_stopping_area_param_path, "r") as f:
        no_stopping_area_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    behavior_velocity_planner_component = ComposableNode(
        package="behavior_velocity_planner",
        plugin="behavior_velocity_planner::BehaviorVelocityPlannerNode",
        name="behavior_velocity_planner",
        namespace="",
        remappings=[
            ("~/input/path_with_lane_id", "path_with_lane_id"),
            ("~/input/vector_map", "/map/vector_map"),
            ("~/input/vehicle_odometry", "/localization/kinematic_state"),
            ("~/input/dynamic_objects", "/perception/object_recognition/objects"),
            (
                "~/input/no_ground_pointcloud",
                "/perception/obstacle_segmentation/pointcloud",
            ),
            (
                "~/input/traffic_signals",
                "/perception/traffic_light_recognition/traffic_signals",
            ),
            (
                "~/input/external_traffic_signals",
                "/external/traffic_light_recognition/traffic_signals",
            ),
            ("~/input/virtual_traffic_light_states", "/awapi/tmp/virtual_traffic_light_states"),
            (
                "~/input/occupancy_grid",
                "/perception/occupancy_grid_map/map",
            ),
            ("~/output/path", "path"),
            ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
            (
                "~/output/infrastructure_commands",
                "/planning/scenario_planning/status/infrastructure_commands",
            ),
            ("~/output/traffic_signal", "debug/traffic_signal"),
        ],
        parameters=[
            {
                "launch_stop_line": True,
                "launch_crosswalk": True,
                "launch_traffic_light": True,
                "launch_intersection": True,
                "launch_blind_spot": True,
                "launch_detection_area": True,
                "launch_virtual_traffic_light": True,
                "launch_occlusion_spot": True,
                "launch_no_stopping_area": True,
                "forward_path_length": 1000.0,
                "backward_path_length": 5.0,
                "max_accel": -2.8,
                "delay_response_time": 1.3,
            },
            blind_spot_param,
            crosswalk_param,
            detection_area_param,
            intersection_param,
            stop_line_param,
            traffic_light_param,
            virtual_traffic_light_param,
            occlusion_spot_param,
            no_stopping_area_param,
            vehicle_info_param,
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
        name="behavior_planning_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            behavior_path_planner_component,
            behavior_velocity_planner_component,
        ],
        output="screen",
    )

    group = GroupAction(
        [
            container,
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "/planning/scenario_planning/lane_driving/behavior_planning/"
                    "behavior_path_planner/path_change_approval",
                    "tier4_planning_msgs/msg/Approval",
                    "{approval: true}",
                    "-r",
                    "10",
                ]
            ),
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

    add_launch_arg(
        "input_route_topic_name", "/planning/mission_planning/route", "input topic of route"
    )
    add_launch_arg("map_topic_name", "/map/vector_map", "input topic of map")

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
