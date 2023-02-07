# Copyright 2021-2022 Arm Ltd.
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
import unittest

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest


@pytest.mark.launch_test
def generate_test_description():
    lidar_apollo_segmentation_tvm = Node(
        package="lidar_apollo_segmentation_tvm_nodes",
        executable="lidar_apollo_segmentation_tvm_nodes_exe",
        name="lidar_apollo_segmentation_tvm_nodes",
        namespace="benchmark",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("lidar_apollo_segmentation_tvm_nodes"),
                "param/test.param.yaml",
            )
        ],
    )

    context = {"lidar_apollo_segmentation_tvm": lidar_apollo_segmentation_tvm}

    launch_description = LaunchDescription(
        [
            lidar_apollo_segmentation_tvm,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )

    return launch_description, context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_output, proc_info, lidar_apollo_segmentation_tvm):
        # Check that process exits with code -2 or -15
        launch_testing.asserts.assertExitCodes(
            proc_info, [-2, -15], process=lidar_apollo_segmentation_tvm
        )
