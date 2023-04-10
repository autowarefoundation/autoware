#!/usr/bin/env python3

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

import os
import unittest

from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest


@pytest.mark.launch_test
def generate_test_description():
    lanelet2_map_path = os.path.join(
        get_package_share_directory("static_centerline_optimizer"), "test/data/lanelet2_map.osm"
    )

    static_centerline_optimizer_node = Node(
        package="static_centerline_optimizer",
        executable="main",
        output="screen",
        parameters=[
            {"lanelet2_map_path": lanelet2_map_path},
            {"run_background": False},
            {"rviz": False},
            {"lanelet2_input_file_path": lanelet2_map_path},
            {"lanelet2_output_file_path": "/tmp/lanelet2_map.osm"},
            {"start_lanelet_id": 215},
            {"end_lanelet_id": 216},
            os.path.join(
                get_package_share_directory("mission_planner"),
                "config",
                "mission_planner.param.yaml",
            ),
            os.path.join(
                get_package_share_directory("static_centerline_optimizer"),
                "config/static_centerline_optimizer.param.yaml",
            ),
            os.path.join(
                get_package_share_directory("obstacle_avoidance_planner"),
                "config/obstacle_avoidance_planner.param.yaml",
            ),
            os.path.join(
                get_package_share_directory("map_loader"),
                "config/lanelet2_map_loader.param.yaml",
            ),
            os.path.join(
                get_package_share_directory("static_centerline_optimizer"),
                "config/common.param.yaml",
            ),
            os.path.join(
                get_package_share_directory("static_centerline_optimizer"),
                "config/nearest_search.param.yaml",
            ),
            os.path.join(
                get_package_share_directory("static_centerline_optimizer"),
                "config/vehicle_info.param.yaml",
            ),
        ],
    )

    context = {}

    return (
        LaunchDescription(
            [
                static_centerline_optimizer_node,
                # Start test after 1s - gives time for the static_centerline_optimizer to finish initialization
                launch.actions.TimerAction(
                    period=1.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        context,
    )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
