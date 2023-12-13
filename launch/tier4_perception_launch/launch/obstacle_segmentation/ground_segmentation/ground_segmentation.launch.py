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
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


class GroundSegmentationPipeline:
    def __init__(self, context):
        self.context = context
        self.vehicle_info = self.get_vehicle_info()
        ground_segmentation_param_path = os.path.join(
            LaunchConfiguration("obstacle_segmentation_ground_segmentation_param_path").perform(
                context
            ),
        )
        with open(ground_segmentation_param_path, "r") as f:
            self.ground_segmentation_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        self.single_frame_obstacle_seg_output = (
            "/perception/obstacle_segmentation/single_frame/pointcloud_raw"
        )
        self.output_topic = "/perception/obstacle_segmentation/pointcloud"
        self.use_single_frame_filter = self.ground_segmentation_param["use_single_frame_filter"]
        self.use_time_series_filter = self.ground_segmentation_param["use_time_series_filter"]

    def get_vehicle_info(self):
        # TODO(TIER IV): Use Parameter Substitution after we drop Galactic support
        # https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/substitutions/parameter.py
        gp = self.context.launch_configurations.get("ros_params", {})
        if not gp:
            gp = dict(self.context.launch_configurations.get("global_params", {}))
        p = {}
        p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
        p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
        p["min_longitudinal_offset"] = -gp["rear_overhang"]
        p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
        p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
        p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
        p["min_height_offset"] = 0.0
        p["max_height_offset"] = gp["vehicle_height"]
        return p

    def get_vehicle_mirror_info(self):
        path = LaunchConfiguration("vehicle_mirror_param_file").perform(self.context)
        with open(path, "r") as f:
            p = yaml.safe_load(f)
        return p

    def create_additional_pipeline(self, lidar_name):
        components = []
        components.append(
            ComposableNode(
                package="pointcloud_preprocessor",
                plugin="pointcloud_preprocessor::CropBoxFilterComponent",
                name=f"{lidar_name}_crop_box_filter",
                remappings=[
                    ("input", f"/sensing/lidar/{lidar_name}/pointcloud"),
                    ("output", f"{lidar_name}/range_cropped/pointcloud"),
                ],
                parameters=[
                    {
                        "input_frame": LaunchConfiguration("base_frame"),
                        "output_frame": LaunchConfiguration("base_frame"),
                    },
                    self.ground_segmentation_param[f"{lidar_name}_crop_box_filter"]["parameters"],
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        components.append(
            ComposableNode(
                package="ground_segmentation",
                plugin=self.ground_segmentation_param[f"{lidar_name}_ground_filter"]["plugin"],
                name=f"{lidar_name}_ground_filter",
                remappings=[
                    ("input", f"{lidar_name}/range_cropped/pointcloud"),
                    ("output", f"{lidar_name}/pointcloud"),
                ],
                parameters=[
                    self.ground_segmentation_param[f"{lidar_name}_ground_filter"]["parameters"]
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        return components

    def create_ransac_pipeline(self):
        components = []
        components.append(
            ComposableNode(
                package="pointcloud_preprocessor",
                plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
                name="concatenate_data",
                namespace="plane_fitting",
                remappings=[
                    ("~/input/odom", "/localization/kinematic_state"),
                    ("output", "concatenated/pointcloud"),
                ],
                parameters=[
                    {
                        "input_topics": self.ground_segmentation_param["ransac_input_topics"],
                        "output_frame": LaunchConfiguration("base_frame"),
                        "timeout_sec": 1.0,
                        "input_twist_topic_type": "odom",
                    }
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        components.append(
            ComposableNode(
                package="pointcloud_preprocessor",
                plugin="pointcloud_preprocessor::CropBoxFilterComponent",
                name="short_height_obstacle_detection_area_filter",
                namespace="plane_fitting",
                remappings=[
                    ("input", "concatenated/pointcloud"),
                    ("output", "detection_area/pointcloud"),
                ],
                parameters=[
                    {
                        "input_frame": LaunchConfiguration("base_frame"),
                        "output_frame": LaunchConfiguration("base_frame"),
                    },
                    self.ground_segmentation_param["short_height_obstacle_detection_area_filter"][
                        "parameters"
                    ],
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        components.append(
            ComposableNode(
                package="pointcloud_preprocessor",
                plugin="pointcloud_preprocessor::Lanelet2MapFilterComponent",
                name="vector_map_filter",
                namespace="plane_fitting",
                remappings=[
                    ("input/pointcloud", "detection_area/pointcloud"),
                    ("input/vector_map", "/map/vector_map"),
                    ("output", "vector_map_filtered/pointcloud"),
                ],
                parameters=[
                    {
                        "voxel_size_x": 0.25,
                        "voxel_size_y": 0.25,
                    }
                ],
                # cannot use intra process because vector map filter uses transient local.
                extra_arguments=[{"use_intra_process_comms": False}],
            )
        )

        components.append(
            ComposableNode(
                package="ground_segmentation",
                plugin="ground_segmentation::RANSACGroundFilterComponent",
                name="ransac_ground_filter",
                namespace="plane_fitting",
                remappings=[
                    ("input", "vector_map_filtered/pointcloud"),
                    ("output", "pointcloud"),
                ],
                parameters=[self.ground_segmentation_param["ransac_ground_filter"]["parameters"]],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        return components

    def create_common_pipeline(self, input_topic, output_topic):
        components = []
        components.append(
            ComposableNode(
                package="pointcloud_preprocessor",
                plugin="pointcloud_preprocessor::CropBoxFilterComponent",
                name="crop_box_filter",
                remappings=[
                    ("input", input_topic),
                    ("output", "range_cropped/pointcloud"),
                ],
                parameters=[
                    {
                        "input_frame": LaunchConfiguration("base_frame"),
                        "output_frame": LaunchConfiguration("base_frame"),
                    },
                    self.ground_segmentation_param["common_crop_box_filter"]["parameters"],
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        components.append(
            ComposableNode(
                package="ground_segmentation",
                plugin=self.ground_segmentation_param["common_ground_filter"]["plugin"],
                name="common_ground_filter",
                remappings=[
                    ("input", "range_cropped/pointcloud"),
                    ("output", output_topic),
                ],
                parameters=[
                    self.ground_segmentation_param["common_ground_filter"]["parameters"],
                    self.vehicle_info,
                    {"input_frame": "base_link"},
                    {"output_frame": "base_link"},
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )
        return components

    def create_single_frame_obstacle_segmentation_components(self, input_topic, output_topic):
        additional_lidars = self.ground_segmentation_param["additional_lidars"]
        use_ransac = bool(self.ground_segmentation_param["ransac_input_topics"])
        use_additional = bool(additional_lidars)
        relay_topic = "all_lidars/pointcloud"
        common_pipeline_output = (
            "single_frame/pointcloud" if use_additional or use_ransac else output_topic
        )

        components = self.create_common_pipeline(
            input_topic=input_topic,
            output_topic=common_pipeline_output,
        )

        if use_additional:
            for lidar_name in additional_lidars:
                components.extend(self.create_additional_pipeline(lidar_name))
            components.append(
                self.get_additional_lidars_concatenated_component(
                    input_topics=[common_pipeline_output]
                    + [f"{x}/pointcloud" for x in additional_lidars],
                    output_topic=relay_topic if use_ransac else output_topic,
                )
            )

        if use_ransac:
            components.extend(self.create_ransac_pipeline())
            components.append(
                self.get_single_frame_obstacle_segmentation_concatenated_component(
                    input_topics=[
                        "plane_fitting/pointcloud",
                        relay_topic if use_additional else common_pipeline_output,
                    ],
                    output_topic=output_topic,
                )
            )

        return components

    @staticmethod
    def create_time_series_outlier_filter_components(input_topic, output_topic):
        components = []
        components.append(
            ComposableNode(
                package="occupancy_grid_map_outlier_filter",
                plugin="occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent",
                name="occupancy_grid_map_outlier_filter",
                remappings=[
                    ("~/input/occupancy_grid_map", "/perception/occupancy_grid_map/map"),
                    ("~/input/pointcloud", input_topic),
                    ("~/output/pointcloud", output_topic),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        return components

    @staticmethod
    def create_single_frame_outlier_filter_components(input_topic, output_topic, context):
        components = []
        components.append(
            ComposableNode(
                package="elevation_map_loader",
                plugin="ElevationMapLoaderNode",
                name="elevation_map_loader",
                namespace="elevation_map",
                remappings=[
                    ("output/elevation_map", "map"),
                    ("input/pointcloud_map", "/map/pointcloud_map"),
                    ("input/vector_map", "/map/vector_map"),
                    ("input/pointcloud_map_metadata", "/map/pointcloud_map_metadata"),
                    ("service/get_selected_pointcloud_map", "/map/get_selected_pointcloud_map"),
                ],
                parameters=[
                    {
                        "use_lane_filter": False,
                        "use_sequential_load": False,
                        "sequential_map_load_num": 1,
                        "use_inpaint": True,
                        "inpaint_radius": 1.0,
                        "lane_margin": 2.0,
                        "param_file_path": PathJoinSubstitution(
                            [
                                LaunchConfiguration(
                                    "obstacle_segmentation_ground_segmentation_elevation_map_param_path"
                                ).perform(context),
                            ]
                        ),
                        "elevation_map_directory": PathJoinSubstitution(
                            [FindPackageShare("elevation_map_loader"), "data", "elevation_maps"]
                        ),
                        "use_elevation_map_cloud_publisher": False,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": False}],
            )
        )

        components.append(
            ComposableNode(
                package="compare_map_segmentation",
                plugin="compare_map_segmentation::CompareElevationMapFilterComponent",
                name="compare_elevation_map_filter",
                namespace="elevation_map",
                remappings=[
                    ("input", input_topic),
                    ("output", "map_filtered/pointcloud"),
                    ("input/elevation_map", "map"),
                ],
                parameters=[
                    {
                        "map_frame": "map",
                        "map_layer_name": "elevation",
                        "height_diff_thresh": 0.15,
                        "input_frame": "map",
                        "output_frame": "base_link",
                    }
                ],
                extra_arguments=[
                    {"use_intra_process_comms": False}
                ],  # can't use this with transient_local
            )
        )

        components.append(
            ComposableNode(
                package="pointcloud_preprocessor",
                plugin="pointcloud_preprocessor::VoxelGridDownsampleFilterComponent",
                name="voxel_grid_filter",
                namespace="elevation_map",
                remappings=[
                    ("input", "map_filtered/pointcloud"),
                    ("output", "voxel_grid_filtered/pointcloud"),
                ],
                parameters=[
                    {
                        "input_frame": LaunchConfiguration("base_frame"),
                        "output_frame": LaunchConfiguration("base_frame"),
                        "voxel_size_x": 0.04,
                        "voxel_size_y": 0.04,
                        "voxel_size_z": 0.08,
                    }
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        components.append(
            ComposableNode(
                package="pointcloud_preprocessor",
                plugin="pointcloud_preprocessor::VoxelGridOutlierFilterComponent",
                name="voxel_grid_outlier_filter",
                namespace="elevation_map",
                remappings=[
                    ("input", "voxel_grid_filtered/pointcloud"),
                    ("output", output_topic),
                ],
                parameters=[
                    {
                        "voxel_size_x": 0.4,
                        "voxel_size_y": 0.4,
                        "voxel_size_z": 100.0,
                        "voxel_points_threshold": 5,
                    }
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )
        )

        return components

    @staticmethod
    def get_additional_lidars_concatenated_component(input_topics, output_topic):
        return ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
            name="concatenate_data",
            remappings=[
                ("~/input/odom", "/localization/kinematic_state"),
                ("output", output_topic),
            ],
            parameters=[
                {
                    "input_topics": input_topics,
                    "output_frame": LaunchConfiguration("base_frame"),
                    "input_twist_topic_type": "odom",
                }
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )

    @staticmethod
    def get_single_frame_obstacle_segmentation_concatenated_component(input_topics, output_topic):
        return ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
            name="concatenate_no_ground_data",
            remappings=[
                ("~/input/odom", "/localization/kinematic_state"),
                ("output", output_topic),
            ],
            parameters=[
                {
                    "input_topics": input_topics,
                    "output_frame": LaunchConfiguration("base_frame"),
                    "input_twist_topic_type": "odom",
                }
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )


def launch_setup(context, *args, **kwargs):
    pipeline = GroundSegmentationPipeline(context)

    glog_component = ComposableNode(
        package="glog_component",
        plugin="GlogComponent",
        name="glog_component",
    )

    components = [glog_component]
    components.extend(
        pipeline.create_single_frame_obstacle_segmentation_components(
            input_topic=LaunchConfiguration("input/pointcloud"),
            output_topic=pipeline.single_frame_obstacle_seg_output
            if pipeline.use_single_frame_filter or pipeline.use_time_series_filter
            else pipeline.output_topic,
        )
    )

    relay_topic = "single_frame/filtered/pointcloud"
    if pipeline.use_single_frame_filter:
        components.extend(
            pipeline.create_single_frame_outlier_filter_components(
                input_topic=pipeline.single_frame_obstacle_seg_output,
                output_topic=relay_topic
                if pipeline.use_time_series_filter
                else pipeline.output_topic,
                context=context,
            )
        )
    if pipeline.use_time_series_filter:
        components.extend(
            pipeline.create_time_series_outlier_filter_components(
                input_topic=relay_topic
                if pipeline.use_single_frame_filter
                else pipeline.single_frame_obstacle_seg_output,
                output_topic=pipeline.output_topic,
            )
        )
    individual_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=components,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )
    pointcloud_container_loader = LoadComposableNodes(
        composable_node_descriptions=components,
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )
    return [individual_container, pointcloud_container_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "True")
    add_launch_arg("use_pointcloud_container", "False")
    add_launch_arg("container_name", "perception_pipeline_container")
    add_launch_arg("input/pointcloud", "/sensing/lidar/concatenated/pointcloud")

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
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
