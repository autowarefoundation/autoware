#!/usr/bin/env python3

# Copyright 2023 TIER IV, Inc.
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.logging import get_logger
import launch_testing
import pytest
import rclpy
from std_srvs.srv import SetBool

logger = get_logger(__name__)


@pytest.mark.launch_test
def generate_test_description():
    test_ndt_scan_matcher_launch_file = os.path.join(
        get_package_share_directory("ndt_scan_matcher"),
        "launch",
        "ndt_scan_matcher.launch.xml",
    )
    ndt_scan_matcher = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(test_ndt_scan_matcher_launch_file),
    )

    return launch.LaunchDescription(
        [
            ndt_scan_matcher,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestNDTScanMatcher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.test_node = rclpy.create_node("test_node")

    def tearDown(self):
        self.test_node.destroy_node()

    @staticmethod
    def print_message(stat):
        logger.debug("===========================")
        logger.debug(stat)

    def test_launch(self):
        # Trigger ndt_scan_matcher to activate the node
        cli_trigger = self.test_node.create_client(SetBool, "/trigger_node")
        while not cli_trigger.wait_for_service(timeout_sec=1.0):
            continue

        request = SetBool.Request()
        request.data = True
        future = cli_trigger.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future)

        if future.result() is not None:
            self.test_node.get_logger().info("Result of bool service: %s" % future.result().message)
            self.assertTrue(future.result().success, "ndt_scan_matcher is not activated")
        else:
            self.test_node.get_logger().error(
                "Exception while calling service: %r" % future.exception()
            )
            raise self.failureException("service trigger failed")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
