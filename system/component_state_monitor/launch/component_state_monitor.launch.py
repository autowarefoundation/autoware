# Copyright 2022 TIER IV, Inc.
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


from collections import defaultdict
from pathlib import Path

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def create_diagnostic_name(row):
    return "{}_topic_status".format(row["module"])


def create_topic_monitor_name(row):
    diag_name = create_diagnostic_name(row)
    return "topic_state_monitor_{}: {}".format(row["args"]["node_name_suffix"], diag_name)


def create_topic_monitor_node(row):
    tf_mode = "" if "topic_type" in row["args"] else "_tf"
    package = FindPackageShare("topic_state_monitor")
    include = PathJoinSubstitution([package, f"launch/topic_state_monitor{tf_mode}.launch.xml"])
    diag_name = create_diagnostic_name(row)
    arguments = [("diag_name", diag_name)] + [(k, str(v)) for k, v in row["args"].items()]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def launch_setup(context, *args, **kwargs):
    # create topic monitors
    mode = LaunchConfiguration("mode").perform(context)
    rows = yaml.safe_load(Path(LaunchConfiguration("file").perform(context)).read_text())
    rows = [row for row in rows if mode in row["mode"]]
    topic_monitor_nodes = [create_topic_monitor_node(row) for row in rows]
    topic_monitor_names = [create_topic_monitor_name(row) for row in rows]
    topic_monitor_param = defaultdict(lambda: defaultdict(list))
    for row in rows:
        topic_monitor_param[row["type"]][row["module"]].append(create_topic_monitor_name(row))
    topic_monitor_param = {name: dict(module) for name, module in topic_monitor_param.items()}

    # create component
    component = ComposableNode(
        namespace="component_state_monitor",
        name="component",
        package="component_state_monitor",
        plugin="component_state_monitor::StateMonitor",
        parameters=[{"topic_monitor_names": topic_monitor_names}, topic_monitor_param],
    )
    container = ComposableNodeContainer(
        namespace="component_state_monitor",
        name="container",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[component],
    )
    return [container, *topic_monitor_nodes]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("file"),
            DeclareLaunchArgument("mode"),
            OpaqueFunction(function=launch_setup),
        ]
    )
