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
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):

    # vehicle information parameter
    vehicle_info_param_path = LaunchConfiguration("vehicle_info_param_file").perform(context)
    with open(vehicle_info_param_path, "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # nearest search parameter
    nearest_search_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "common",
        "nearest_search.param.yaml",
    )
    with open(nearest_search_param_path, "r") as f:
        nearest_search_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # behavior path planner
    side_shift_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
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
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
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
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
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
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
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
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
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
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_path_planner",
        "pull_out",
        "pull_out.param.yaml",
    )
    with open(pull_out_param_path, "r") as f:
        pull_out_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    drivable_area_expansion_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_path_planner",
        "drivable_area_expansion.param.yaml",
    )
    with open(drivable_area_expansion_param_path, "r") as f:
        drivable_area_expansion_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    behavior_path_planner_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
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
            ("~/input/accel", "/localization/acceleration"),
            ("~/input/scenario", "/planning/scenario_planning/scenario"),
            ("~/output/path", "path_with_lane_id"),
            ("~/output/turn_indicators_cmd", "/planning/turn_indicators_cmd"),
            ("~/output/hazard_lights_cmd", "/planning/hazard_lights_cmd"),
        ],
        parameters=[
            nearest_search_param,
            side_shift_param,
            avoidance_param,
            lane_change_param,
            lane_following_param,
            pull_over_param,
            pull_out_param,
            drivable_area_expansion_param,
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

    # smoother param
    common_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "common",
        "common.param.yaml",
    )
    with open(common_param_path, "r") as f:
        common_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    motion_velocity_smoother_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "common",
        "motion_velocity_smoother",
        "motion_velocity_smoother.param.yaml",
    )
    with open(motion_velocity_smoother_param_path, "r") as f:
        motion_velocity_smoother_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    smoother_type_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "common",
        "motion_velocity_smoother",
        "Analytical.param.yaml",
    )
    with open(smoother_type_param_path, "r") as f:
        smoother_type_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # behavior velocity planner
    blind_spot_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_velocity_planner",
        "blind_spot.param.yaml",
    )
    with open(blind_spot_param_path, "r") as f:
        blind_spot_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    crosswalk_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_velocity_planner",
        "crosswalk.param.yaml",
    )
    with open(crosswalk_param_path, "r") as f:
        crosswalk_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    detection_area_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_velocity_planner",
        "detection_area.param.yaml",
    )
    with open(detection_area_param_path, "r") as f:
        detection_area_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    intersection_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_velocity_planner",
        "intersection.param.yaml",
    )
    with open(intersection_param_path, "r") as f:
        intersection_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    stop_line_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_velocity_planner",
        "stop_line.param.yaml",
    )
    with open(stop_line_param_path, "r") as f:
        stop_line_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    traffic_light_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_velocity_planner",
        "traffic_light.param.yaml",
    )
    with open(traffic_light_param_path, "r") as f:
        traffic_light_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    virtual_traffic_light_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_velocity_planner",
        "virtual_traffic_light.param.yaml",
    )
    with open(virtual_traffic_light_param_path, "r") as f:
        virtual_traffic_light_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    occlusion_spot_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_velocity_planner",
        "occlusion_spot.param.yaml",
    )
    with open(occlusion_spot_param_path, "r") as f:
        occlusion_spot_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    no_stopping_area_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_velocity_planner",
        "no_stopping_area.param.yaml",
    )
    with open(no_stopping_area_param_path, "r") as f:
        no_stopping_area_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    run_out_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_velocity_planner",
        "run_out.param.yaml",
    )
    with open(run_out_param_path, "r") as f:
        run_out_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    behavior_velocity_planner_param_path = os.path.join(
        LaunchConfiguration("tier4_planning_launch_param_path").perform(context),
        "scenario_planning",
        "lane_driving",
        "behavior_planning",
        "behavior_velocity_planner",
        "behavior_velocity_planner.param.yaml",
    )
    with open(behavior_velocity_planner_param_path, "r") as f:
        behavior_velocity_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    behavior_velocity_planner_component = ComposableNode(
        package="behavior_velocity_planner",
        plugin="behavior_velocity_planner::BehaviorVelocityPlannerNode",
        name="behavior_velocity_planner",
        namespace="",
        remappings=[
            ("~/input/path_with_lane_id", "path_with_lane_id"),
            ("~/input/vector_map", "/map/vector_map"),
            ("~/input/vehicle_odometry", "/localization/kinematic_state"),
            ("~/input/accel", "/localization/acceleration"),
            ("~/input/dynamic_objects", "/perception/object_recognition/objects"),
            (
                "~/input/no_ground_pointcloud",
                "/perception/obstacle_segmentation/pointcloud",
            ),
            (
                "~/input/compare_map_filtered_pointcloud",
                "compare_map_filtered/pointcloud",
            ),
            (
                "~/input/traffic_signals",
                "/perception/traffic_light_recognition/traffic_signals",
            ),
            (
                "~/input/external_traffic_signals",
                "/external/traffic_light_recognition/traffic_signals",
            ),
            (
                "~/input/external_velocity_limit_mps",
                "/planning/scenario_planning/max_velocity_default",
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
            nearest_search_param,
            behavior_velocity_planner_param,
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
            run_out_param,
            common_param,
            motion_velocity_smoother_param,
            smoother_type_param,
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

    # This condition is true if run_out module is enabled and its detection method is Points
    launch_run_out_with_points_method = PythonExpression(
        [
            LaunchConfiguration(
                "launch_run_out", default=behavior_velocity_planner_param["launch_run_out"]
            ),
            " and ",
            "'",
            run_out_param["run_out"]["detection_method"],
            "' == 'Points'",
        ]
    )

    # load compare map for run_out module
    load_compare_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("tier4_planning_launch"),
                "/launch/scenario_planning/lane_driving/behavior_planning/compare_map.launch.py",
            ],
        ),
        launch_arguments={
            "use_pointcloud_container": LaunchConfiguration("use_pointcloud_container"),
            "container_name": LaunchConfiguration("container_name"),
            "use_multithread": "true",
        }.items(),
        # launch compare map only when run_out module is enabled and detection method is Points
        condition=IfCondition(launch_run_out_with_points_method),
    )

    load_vector_map_inside_area_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("tier4_planning_launch"),
                "/launch/scenario_planning/lane_driving/behavior_planning/vector_map_inside_area_filter.launch.py",
            ]
        ),
        launch_arguments={
            "use_pointcloud_container": LaunchConfiguration("use_pointcloud_container"),
            "container_name": LaunchConfiguration("container_name"),
            "use_multithread": "true",
            "polygon_type": "no_obstacle_segmentation_area_for_run_out",
        }.items(),
        # launch vector map filter only when run_out module is enabled and detection method is Points
        condition=IfCondition(launch_run_out_with_points_method),
    )

    group = GroupAction(
        [
            container,
            load_compare_map,
            load_vector_map_inside_area_filter,
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

    add_launch_arg("tier4_planning_launch_param_path", None, "tier4_planning_launch parameter path")

    # component
    add_launch_arg("use_intra_process", "false", "use ROS2 component container communication")
    add_launch_arg("use_multithread", "false", "use multithread")

    # for points filter of run out module
    add_launch_arg("use_pointcloud_container", "true")
    add_launch_arg("container_name", "pointcloud_container")

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
