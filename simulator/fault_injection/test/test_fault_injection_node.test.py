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

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.logging import get_logger
import launch_testing
import pytest
import rclpy
from tier4_simulation_msgs.msg import FaultInjectionEvent
from tier4_simulation_msgs.msg import SimulationEvents

logger = get_logger(__name__)


@pytest.mark.launch_test
def generate_test_description():
    test_fault_injection_launch_file = os.path.join(
        get_package_share_directory("fault_injection"),
        "launch",
        "test_fault_injection.launch.xml",
    )
    fault_injection = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(test_fault_injection_launch_file),
    )

    return launch.LaunchDescription(
        [
            fault_injection,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestFaultInjectionLink(unittest.TestCase):
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
        self.event_name = "cpu_temperature"
        self.evaluation_time = 0.5  # 500ms

    def tearDown(self):
        self.test_node.destroy_node()

    @staticmethod
    def print_message(stat):
        logger.debug("===========================")
        logger.debug(stat)

    @staticmethod
    def get_num_valid_data(msg_buffer, level: bytes) -> int:
        received_diagnostics = [stat for diag_array in msg_buffer for stat in diag_array.status]
        filtered_diagnostics = [
            stat
            for stat in received_diagnostics
            if ("Node starting up" not in stat.message) and (stat.level == level)
        ]

        for stat in filtered_diagnostics:
            TestFaultInjectionLink.print_message(stat)

        return len(filtered_diagnostics)

    def test_node_link(self):
        """
        Test node linkage.

        Expect fault_injection_node publish /diagnostics.status
        when the talker to publish strings
        """
        pub_events = self.test_node.create_publisher(SimulationEvents, "/simulation/events", 10)

        msg_buffer = []
        self.test_node.create_subscription(
            DiagnosticArray, "/diagnostics", lambda msg: msg_buffer.append(msg), 10
        )

        # Test init state.
        # Expect fault_injection_node does not publish /diagnostics.status
        # while the talker does not to publish
        # Wait until the talker transmits two messages over the ROS topic
        end_time = time.time() + self.evaluation_time
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Return False if no valid data is received
        self.assertEqual(self.get_num_valid_data(msg_buffer, DiagnosticStatus.ERROR), 0)

        # Test node linkage.
        # Wait until the talker transmits messages over the ROS topic
        item = FaultInjectionEvent(name=self.event_name, level=FaultInjectionEvent.ERROR)
        msg = SimulationEvents(fault_injection_events=[item])
        pub_events.publish(msg)
        end_time = time.time() + self.evaluation_time
        while time.time() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        self.assertGreaterEqual(self.get_num_valid_data(msg_buffer, DiagnosticStatus.ERROR), 1)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        """
        Test process exit code.

        Check that all processes in the launch (in this case, there's just one) exit
        with code 0
        """
        launch_testing.asserts.assertExitCodes(proc_info)
