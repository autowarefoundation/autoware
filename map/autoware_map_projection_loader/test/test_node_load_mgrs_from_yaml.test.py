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
from launch.logging import get_logger
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from tier4_map_msgs.msg import MapProjectorInfo
import yaml

logger = get_logger(__name__)

YAML_FILE_PATH = "test/data/map_projector_info_mgrs.yaml"


@pytest.mark.launch_test
def generate_test_description():
    map_projector_info_path = os.path.join(
        get_package_share_directory("autoware_map_projection_loader"), YAML_FILE_PATH
    )

    map_projection_loader_node = Node(
        package="autoware_map_projection_loader",
        executable="autoware_map_projection_loader_node",
        output="screen",
        parameters=[
            {
                "map_projector_info_path": map_projector_info_path,
                "lanelet2_map_path": "",
                "use_local_projector": False,
            },
        ],
    )

    context = {}

    return (
        LaunchDescription(
            [
                map_projection_loader_node,
                # Start test after 1s - gives time for the static_centerline_generator to finish initialization
                launch.actions.TimerAction(
                    period=1.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ]
        ),
        context,
    )


class TestLoadMGRSFromYaml(unittest.TestCase):
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
        self.evaluation_time = 0.2  # 200ms
        self.received_message = None

    def tearDown(self):
        self.test_node.destroy_node()

    def callback(self, msg):
        self.received_message = msg

    @staticmethod
    def print_message(stat):
        logger.debug("===========================")
        logger.debug(stat)

    def test_node_link(self):
        # Create custom QoS profile for subscription
        custom_qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Create subscription to map_projector_info topic
        subscription = self.test_node.create_subscription(
            MapProjectorInfo,
            "/map/map_projector_info",
            self.callback,
            custom_qos_profile,
        )

        # Give time for the message to be received and processed
        rclpy.spin_until_future_complete(
            self.test_node, rclpy.task.Future(), timeout_sec=self.evaluation_time
        )

        # Load the yaml file directly
        map_projector_info_path = os.path.join(
            get_package_share_directory("autoware_map_projection_loader"), YAML_FILE_PATH
        )
        with open(map_projector_info_path) as f:
            yaml_data = yaml.load(f, Loader=yaml.FullLoader)

        # Test if message received
        self.assertIsNotNone(
            self.received_message, "No message received on map_projector_info topic"
        )
        self.assertEqual(self.received_message.projector_type, yaml_data["projector_type"])
        self.assertEqual(self.received_message.vertical_datum, yaml_data["vertical_datum"])
        self.assertEqual(self.received_message.mgrs_grid, yaml_data["mgrs_grid"])

        self.test_node.destroy_subscription(subscription)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
