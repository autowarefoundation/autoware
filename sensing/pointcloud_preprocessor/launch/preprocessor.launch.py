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
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ns = "pointcloud_preprocessor"
    pkg = "pointcloud_preprocessor"

    # declare launch arguments
    input_points_raw_list_param = DeclareLaunchArgument(
        "input_points_raw_list",
        default_value="['/points_raw']",
        description="Input pointcloud topic_name list as a string_array. "
        "To subscribe multiple topics, write as: \"['/points_raw0', '/points_raw1', ...]\"",
    )

    output_points_raw_param = DeclareLaunchArgument(
        "output_points_raw", default_value="/points_raw/cropbox/filtered"
    )

    tf_output_frame_param = DeclareLaunchArgument("tf_output_frame", default_value="base_link")

    # set concat filter as a component
    concat_component = ComposableNode(
        package=pkg,
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_filter",
        remappings=[("output", "points_raw/concatenated")],
        parameters=[
            {
                "input_topics": LaunchConfiguration("input_points_raw_list"),
                "output_frame": LaunchConfiguration("tf_output_frame"),
                "approximate_sync": True,
            }
        ],
    )

    # set crop box filter as a component
    cropbox_component = ComposableNode(
        package=pkg,
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter",
        remappings=[
            (
                "input",
                PythonExpression(
                    [
                        "'points_raw/concatenated' if len(",
                        LaunchConfiguration("input_points_raw_list"),
                        ") > 1 else 'input_points_raw0'",
                    ]
                ),
            ),
            ("output", LaunchConfiguration("output_points_raw")),
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("tf_output_frame"),
                "output_frame": LaunchConfiguration("tf_output_frame"),
                "min_x": -200.0,
                "max_x": 1000.0,
                "min_y": -50.0,
                "max_y": 50.0,
                "min_z": -2.0,
                "max_z": 3.0,
                "negative": False,
            }
        ],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="pointcloud_preprocessor_container",
        namespace=ns,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[concat_component, cropbox_component],
        output="screen",
    )

    # check the size of input_points_raw_list
    log_info = LogInfo(
        msg=PythonExpression(
            [
                "'input_points_raw_list size = ' + str(len(",
                LaunchConfiguration("input_points_raw_list"),
                "))",
            ]
        )
    )

    return launch.LaunchDescription(
        [
            input_points_raw_list_param,
            output_points_raw_param,
            tf_output_frame_param,
            container,
            log_info,
        ]
    )
