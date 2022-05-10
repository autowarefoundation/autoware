# Copyright 2021 the Autoware Foundation
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
#
# Developed by Robotec.ai.

import os
import shlex
import time
import unittest

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy


def resolve_node(context, *args, **kwargs):
    parameters = [
        os.path.join(
            get_package_share_directory(LaunchConfiguration("arg_package").perform(context)),
            "param",
            file_name,
        )
        for file_name in shlex.split(LaunchConfiguration("arg_param_filenames").perform(context))
    ]

    smoke_test_node = Node(
        package=LaunchConfiguration("arg_package"),
        executable=LaunchConfiguration("arg_package_exe"),
        namespace="test",
        parameters=parameters,
        arguments=shlex.split(LaunchConfiguration("arg_executable_arguments").perform(context)),
    )
    return [smoke_test_node]


@pytest.mark.launch_test
def generate_test_description():

    arg_package = DeclareLaunchArgument(
        "arg_package", default_value=["default"], description="Package containing tested executable"
    )
    arg_package_exe = DeclareLaunchArgument(
        "arg_package_exe", default_value=["default"], description="Tested executable"
    )
    arg_param_filenames = DeclareLaunchArgument(
        "arg_param_filenames", default_value=["test.param.yaml"], description="Test param file"
    )
    arg_executable_arguments = DeclareLaunchArgument(
        "arg_executable_arguments", default_value=[""], description="Tested executable arguments"
    )

    return LaunchDescription(
        [
            arg_package,
            arg_package_exe,
            arg_param_filenames,
            arg_executable_arguments,
            OpaqueFunction(function=resolve_node),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class DummyTest(unittest.TestCase):
    def test_wait_for_node_ready(self):
        """Waiting for the node is ready."""
        rclpy.init()
        test_node = rclpy.create_node("test_node")
        while len(test_node.get_node_names()) == 0:
            time.sleep(0.1)
            print("waiting for Node to be ready")
        rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_output, proc_info):
        # Check that process exits with code 0
        launch_testing.asserts.assertExitCodes(proc_info)
