# Copyright 2021 Tier IV, Inc.
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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _create_api_node(node_name, class_name, **kwargs):
    return ComposableNode(
        namespace="internal",
        name=node_name,
        package="autoware_iv_internal_api_adaptor",
        plugin="internal_api::" + class_name,
        **kwargs
    )


def generate_launch_description():
    components = [
        _create_api_node("iv_msgs", "IVMsgs"),
        _create_api_node("operator", "Operator"),
        _create_api_node("velocity", "Velocity"),
    ]
    container = ComposableNodeContainer(
        namespace="internal",
        name="autoware_iv_adaptor",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=components,
        output="screen",
    )
    return launch.LaunchDescription([container])
