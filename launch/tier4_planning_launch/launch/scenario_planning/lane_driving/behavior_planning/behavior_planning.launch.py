# Copyright 2021-2023 TIER IV, Inc. All rights reserved.
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
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    # vehicle information parameter
    vehicle_param_path = LaunchConfiguration("vehicle_param_file").perform(context)
    with open(vehicle_param_path, "r") as f:
        vehicle_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # common parameter
    with open(LaunchConfiguration("common_param_path").perform(context), "r") as f:
        common_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    # nearest search parameter
    with open(LaunchConfiguration("nearest_search_param_path").perform(context), "r") as f:
        nearest_search_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # behavior path planner
    with open(LaunchConfiguration("side_shift_param_path").perform(context), "r") as f:
        side_shift_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("avoidance_param_path").perform(context), "r") as f:
        avoidance_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("avoidance_by_lc_param_path").perform(context), "r") as f:
        avoidance_by_lc_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("dynamic_avoidance_param_path").perform(context), "r") as f:
        dynamic_avoidance_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("lane_change_param_path").perform(context), "r") as f:
        lane_change_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("goal_planner_param_path").perform(context), "r") as f:
        goal_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("start_planner_param_path").perform(context), "r") as f:
        start_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("drivable_area_expansion_param_path").perform(context), "r") as f:
        drivable_area_expansion_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("scene_module_manager_param_path").perform(context), "r") as f:
        scene_module_manager_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(LaunchConfiguration("behavior_path_planner_param_path").perform(context), "r") as f:
        behavior_path_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    glog_component = ComposableNode(
        package="glog_component",
        plugin="GlogComponent",
        name="glog_component",
    )

    behavior_path_planner_component = ComposableNode(
        package="behavior_path_planner",
        plugin="behavior_path_planner::BehaviorPathPlannerNode",
        name="behavior_path_planner",
        namespace="",
        remappings=[
            ("~/input/route", LaunchConfiguration("input_route_topic_name")),
            ("~/input/vector_map", LaunchConfiguration("map_topic_name")),
            ("~/input/perception", "/perception/object_recognition/objects"),
            ("~/input/occupancy_grid_map", "/perception/occupancy_grid_map/map"),
            (
                "~/input/costmap",
                "/planning/scenario_planning/parking/costmap_generator/occupancy_grid",
            ),
            (
                "~/input/traffic_signals",
                "/perception/traffic_light_recognition/traffic_signals",
            ),
            ("~/input/odometry", "/localization/kinematic_state"),
            ("~/input/accel", "/localization/acceleration"),
            ("~/input/scenario", "/planning/scenario_planning/scenario"),
            ("~/output/path", "path_with_lane_id"),
            ("~/output/turn_indicators_cmd", "/planning/turn_indicators_cmd"),
            ("~/output/hazard_lights_cmd", "/planning/hazard_lights_cmd"),
            ("~/output/modified_goal", "/planning/scenario_planning/modified_goal"),
            ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
        ],
        parameters=[
            common_param,
            nearest_search_param,
            side_shift_param,
            avoidance_param,
            avoidance_by_lc_param,
            dynamic_avoidance_param,
            lane_change_param,
            goal_planner_param,
            start_planner_param,
            drivable_area_expansion_param,
            scene_module_manager_param,
            behavior_path_planner_param,
            vehicle_param,
            {
                "lane_change.enable_collision_check_at_prepare_phase": LaunchConfiguration(
                    "use_experimental_lane_change_function"
                ),
                "lane_change.use_predicted_path_outside_lanelet": LaunchConfiguration(
                    "use_experimental_lane_change_function"
                ),
                "lane_change.use_all_predicted_path": LaunchConfiguration(
                    "use_experimental_lane_change_function"
                ),
            },
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # smoother param
    with open(
        LaunchConfiguration("motion_velocity_smoother_param_path").perform(context), "r"
    ) as f:
        motion_velocity_smoother_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    with open(
        LaunchConfiguration("behavior_velocity_smoother_type_param_path").perform(context), "r"
    ) as f:
        behavior_velocity_smoother_type_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # behavior velocity planner
    behavior_velocity_planner_common_param_path = LaunchConfiguration(
        "behavior_velocity_planner_common_param_path"
    ).perform(context)
    behavior_velocity_planner_module_param_paths = LaunchConfiguration(
        "behavior_velocity_planner_module_param_paths"
    ).perform(context)

    behavior_velocity_planner_params_paths = [
        behavior_velocity_planner_common_param_path,
        *yaml.safe_load(behavior_velocity_planner_module_param_paths),
    ]

    behavior_velocity_planner_params = {}
    for path in behavior_velocity_planner_params_paths:
        with open(path) as f:
            behavior_velocity_planner_params.update(yaml.safe_load(f)["/**"]["ros__parameters"])

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
                "~/input/vector_map_inside_area_filtered_pointcloud",
                "vector_map_inside_area_filtered/pointcloud",
            ),
            (
                "~/input/traffic_signals",
                "/perception/traffic_light_recognition/traffic_signals",
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
            behavior_velocity_planner_params,
            vehicle_param,
            common_param,
            motion_velocity_smoother_param,
            behavior_velocity_smoother_type_param,
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
            glog_component,
        ],
        output="screen",
    )

    # This condition is true if run_out module is enabled and its detection method is Points
    run_out_module = "behavior_velocity_planner::RunOutModulePlugin"
    run_out_method = behavior_velocity_planner_params.get("run_out", {}).get("detection_method")
    launch_run_out = run_out_module in behavior_velocity_planner_params["launch_modules"]
    launch_run_out_with_points_method = launch_run_out and run_out_method == "Points"

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
        condition=IfCondition(PythonExpression(str(launch_run_out_with_points_method))),
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
        condition=IfCondition(PythonExpression(str(launch_run_out_with_points_method))),
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

    # vehicle parameter
    add_launch_arg("vehicle_param_file")

    # interface parameter
    add_launch_arg(
        "input_route_topic_name", "/planning/mission_planning/route", "input topic of route"
    )
    add_launch_arg("map_topic_name", "/map/vector_map", "input topic of map")

    # package parameter
    add_launch_arg("use_experimental_lane_change_function")

    # component
    add_launch_arg("use_intra_process", "false", "use ROS 2 component container communication")
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
