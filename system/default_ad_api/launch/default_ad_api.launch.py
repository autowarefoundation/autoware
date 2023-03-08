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

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def create_api_node(node_name, class_name, **kwargs):
    return ComposableNode(
        namespace="default_ad_api/node",
        name=node_name,
        package="default_ad_api",
        plugin="default_ad_api::" + class_name,
        parameters=[ParameterFile(LaunchConfiguration("config"))],
    )


def get_default_config():
    path = FindPackageShare("default_ad_api")
    path = PathJoinSubstitution([path, "config/default_ad_api.param.yaml"])
    return path


def generate_launch_description():
    components = [
        create_api_node("autoware_state", "AutowareStateNode"),
        create_api_node("fail_safe", "FailSafeNode"),
        create_api_node("interface", "InterfaceNode"),
        create_api_node("localization", "LocalizationNode"),
        create_api_node("motion", "MotionNode"),
        create_api_node("operation_mode", "OperationModeNode"),
        create_api_node("planning", "PlanningNode"),
        create_api_node("routing", "RoutingNode"),
    ]
    container = ComposableNodeContainer(
        namespace="default_ad_api",
        name="container",
        package="rclcpp_components",
        executable="component_container_mt",
        ros_arguments=["--log-level", "default_ad_api.container:=WARN"],
        composable_node_descriptions=components,
    )
    web_server = Node(
        namespace="default_ad_api",
        package="default_ad_api",
        name="web_server",
        executable="web_server.py",
    )
    argument = DeclareLaunchArgument("config", default_value=get_default_config())
    return launch.LaunchDescription([argument, container, web_server])
