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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml

# In this file, we use "ogm" as a meaning of occupancy grid map


# overwrite parameter
def overwrite_config(param_dict, launch_config_name, node_params_name, context):
    if LaunchConfiguration(launch_config_name).perform(context) != "":
        param_dict[node_params_name] = LaunchConfiguration(launch_config_name).perform(context)


# load parameter files
def load_config_file(context, configuration_name: str):
    config_file = LaunchConfiguration(configuration_name).perform(context)
    with open(config_file, "r") as f:
        config_param_dict = yaml.safe_load(f)["/**"]["ros__parameters"]
    return config_param_dict


# check if the length of the parameters are the same
def fusion_config_sanity_check(fusion_config: dict):
    listed_param_names = [
        "raw_pointcloud_topics",
        "fusion_input_ogm_topics",
        "input_ogm_reliabilities",
    ]
    param_length_list = []

    for param_name in listed_param_names:
        # parameters is not in the config file dict
        if param_name not in fusion_config:
            raise Exception("Error: " + param_name + " is not in the config file")
        # get len of the parameter
        param_length_list.append(len(fusion_config[param_name]))

    # check if the length of the parameters are the same
    if not all(x == param_length_list[0] for x in param_length_list):
        raise Exception("Error: the length of the parameters are not the same")


def get_fusion_config(total_config: dict) -> dict:
    fusion_config = total_config["fusion_config"]
    shared_config = total_config["shared_config"]
    # merge shared config and fusion config
    fusion_config.update(shared_config)
    # overwrite ogm size settings
    fusion_config["fusion_map_length_x"] = shared_config["map_length_x"]
    fusion_config["fusion_map_length_y"] = shared_config["map_length_y"]
    fusion_config["fusion_map_resolution"] = shared_config["map_resolution"]
    return fusion_config


# extract ogm creation config from fusion config
def get_ogm_creation_config(total_config: dict, list_iter: int) -> dict:
    ogm_creation_config = total_config["ogm_creation_config"]
    shared_config = total_config["shared_config"]
    # fusion_config_ = total_config["fusion_config"]
    # merge shared config and ogm creation config
    ogm_creation_config.update(shared_config)
    # overwrite scan_origin_frame with sensor frame settings
    # ogm_creation_config["scan_origin_frame"] = fusion_config_["each_ogm_sensor_frames"][list_iter]
    # use fusion_map_length to set map_length
    ogm_creation_config["map_length"] = max(
        shared_config["map_length_x"], shared_config["map_length_y"]
    )
    return ogm_creation_config


# generate downsample node
def get_downsample_filter_node(setting: dict) -> ComposableNode:
    plugin_str = setting["plugin"]
    voxel_size = setting["voxel_size"]
    node_name = setting["node_name"]
    return ComposableNode(
        package="autoware_pointcloud_preprocessor",
        plugin=plugin_str,
        name=node_name,
        remappings=[
            ("input", setting["input_topic"]),
            ("output", setting["output_topic"]),
        ],
        parameters=[
            {
                "voxel_size_x": voxel_size,
                "voxel_size_y": voxel_size,
                "voxel_size_z": voxel_size,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )


def get_downsample_preprocess_nodes(voxel_size: float, raw_pointcloud_topics: list) -> list:
    nodes = []
    for i in range(len(raw_pointcloud_topics)):
        raw_pointcloud_topic: str = raw_pointcloud_topics[i]
        frame_name: str = raw_pointcloud_topic.split("/")[
            -2
        ]  # `/sensing/lidar/top/pointcloud` -> `top
        processed_pointcloud_topic: str = frame_name + "/raw/downsample/pointcloud"
        raw_settings = {
            "plugin": "autoware::pointcloud_preprocessor::PickupBasedVoxelGridDownsampleFilterComponent",
            "node_name": "raw_pc_downsample_filter_" + frame_name,
            "input_topic": raw_pointcloud_topic,
            "output_topic": processed_pointcloud_topic,
            "voxel_size": voxel_size,
        }
        nodes.append(get_downsample_filter_node(raw_settings))
    obstacle_settings = {
        "plugin": "autoware::pointcloud_preprocessor::PickupBasedVoxelGridDownsampleFilterComponent",
        "node_name": "obstacle_pc_downsample_filter",
        "input_topic": LaunchConfiguration("input/obstacle_pointcloud"),
        "output_topic": "/perception/occupancy_grid_map/obstacle/downsample/pointcloud",
        "voxel_size": voxel_size,
    }
    nodes.append(get_downsample_filter_node(obstacle_settings))
    return nodes


def launch_setup(context, *args, **kwargs):
    """Launch fusion based occupancy grid map creation nodes.

    1. describe occupancy grid map generation nodes for each sensor input
    2. describe occupancy grid map fusion node
    3. launch setting
    """
    # 1. launch occupancy grid map generation nodes for each sensor input

    # load fusion config parameter
    total_config = load_config_file(context, "multi_lidar_fusion_config_file")
    fusion_config = get_fusion_config(total_config)
    fusion_config_sanity_check(fusion_config)
    updater_config = load_config_file(context, "updater_param_file")

    # pointcloud based occupancy grid map nodes
    gridmap_generation_composable_nodes = []

    number_of_nodes = len(fusion_config["raw_pointcloud_topics"])
    print(
        "launching multi_lidar_pointcloud_based occupancy grid map",
        number_of_nodes,
        "nodes in the container named",
        LaunchConfiguration("pointcloud_container_name").perform(context),
    )

    # Down sample settings
    downsample_input_pointcloud: bool = total_config["downsample_input_pointcloud"]
    downsample_voxel_size: float = total_config["downsample_voxel_size"]

    # get obstacle pointcloud
    obstacle_pointcloud_topic: str = (
        "/perception/occupancy_grid_map/obstacle/downsample/pointcloud"
        if downsample_input_pointcloud
        else LaunchConfiguration("input/obstacle_pointcloud").perform(context)
    )

    for i in range(number_of_nodes):
        # load parameter file
        ogm_creation_config = get_ogm_creation_config(total_config, i)
        ogm_creation_config["updater_type"] = LaunchConfiguration("updater_type").perform(context)
        ogm_creation_config.update(updater_config)

        raw_pointcloud_topic: str = fusion_config["raw_pointcloud_topics"][i]
        frame_name: str = raw_pointcloud_topic.split("/")[
            -2
        ]  # assume `/sensing/lidar/top/pointcloud` -> `top
        if downsample_input_pointcloud:
            raw_pointcloud_topic = "raw/downsample/pointcloud"

        # generate composable node
        node = ComposableNode(
            package="autoware_probabilistic_occupancy_grid_map",
            plugin="autoware::occupancy_grid_map::PointcloudBasedOccupancyGridMapNode",
            name="occupancy_grid_map_node",
            namespace=frame_name,
            remappings=[
                ("~/input/obstacle_pointcloud", obstacle_pointcloud_topic),
                ("~/input/raw_pointcloud", raw_pointcloud_topic),
                ("~/output/occupancy_grid_map", fusion_config["fusion_input_ogm_topics"][i]),
            ],
            parameters=[ogm_creation_config],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
        gridmap_generation_composable_nodes.append(node)

    if downsample_input_pointcloud:
        downsample_nodes = get_downsample_preprocess_nodes(
            downsample_voxel_size, fusion_config["raw_pointcloud_topics"]
        )
        gridmap_generation_composable_nodes.extend(downsample_nodes)

    # 2. launch occupancy grid map fusion node
    gridmap_fusion_node = [
        ComposableNode(
            package="autoware_probabilistic_occupancy_grid_map",
            plugin="autoware::occupancy_grid_map::GridMapFusionNode",
            name="occupancy_grid_map_fusion_node",
            remappings=[
                ("~/output/occupancy_grid_map", LaunchConfiguration("output")),
            ],
            parameters=[fusion_config],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
    ]

    # 3. launch setting
    occupancy_grid_map_container = ComposableNodeContainer(
        name=LaunchConfiguration("pointcloud_container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=gridmap_generation_composable_nodes + gridmap_fusion_node,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    load_composable_nodes = LoadComposableNodes(
        composable_node_descriptions=gridmap_generation_composable_nodes + gridmap_fusion_node,
        target_container=LaunchConfiguration("pointcloud_container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    return [occupancy_grid_map_container, load_composable_nodes]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)

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

    return LaunchDescription(
        [
            add_launch_arg("use_multithread", "false"),
            add_launch_arg("use_intra_process", "true"),
            add_launch_arg("use_pointcloud_container", "false"),
            add_launch_arg("pointcloud_container_name", "occupancy_grid_map_container"),
            add_launch_arg("input/obstacle_pointcloud", "no_ground/oneshot/pointcloud"),
            add_launch_arg("output", "occupancy_grid"),
            add_launch_arg(
                "multi_lidar_fusion_config_file",
                get_package_share_directory("autoware_probabilistic_occupancy_grid_map")
                + "/config/multi_lidar_pointcloud_based_occupancy_grid_map.param.yaml",
            ),
            add_launch_arg("updater_type", "binary_bayes_filter"),
            add_launch_arg(
                "updater_param_file",
                get_package_share_directory("autoware_probabilistic_occupancy_grid_map")
                + "/config/binary_bayes_filter_updater.param.yaml",
            ),
            set_container_executable,
            set_container_mt_executable,
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
