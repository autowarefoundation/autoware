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

import time
import unittest

from geometry_msgs.msg import TransformStamped
import launch
import launch.actions
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

PACKAGE_NAME = "autoware_probabilistic_occupancy_grid_map"
INPUT_TOPICS = ["/topic1", "/topic2"]
DEBUG_OUTPUT_TOPIC = "/synchronized_grid_map_fusion_node/debug/single_frame_map"
OUTPUT_TOPIC = "/synchronized_grid_map_fusion_node/output/occupancy_grid_map"

FREE_VALUE = 1
UNKNOWN_VALUE = 50
OCCUPIED_VALUE = 99


# test launcher to launch grid map fusion node
@pytest.mark.launch_test
def generate_test_description():
    """Launch file test description.

    Returns:
        _type_: launch.LaunchDescription
    """
    # get launch file path
    launch_file_path = (
        FindPackageShare(PACKAGE_NAME).find(PACKAGE_NAME)
        + "/launch/synchronized_occupancy_grid_map_fusion.launch.xml"
    )
    # use default launch arguments and params
    launch_args = []
    # action to include launch file
    test_launch_file = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.AnyLaunchDescriptionSource(launch_file_path),
        launch_arguments=launch_args,
    )

    return launch.LaunchDescription(
        [
            test_launch_file,
            launch_testing.actions.ReadyToTest(),
        ]
    )


# util functions
def create_nav_msgs_occupancy_grid_msg(fill_value: int = 0, stamp: rclpy.time.Time = None):
    """Create nav_msgs occupancy grid message.

    Args:
        fill_value (int, optional): fill value. Defaults to 0.

    Returns:
        OccupancyGrid: nav_msgs occupancy grid message
    """
    msg = OccupancyGrid()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg() if stamp is None else stamp.to_msg()
    msg.header.frame_id = "map"
    msg.info.resolution = 0.5  # 0.5m x 0.5m
    msg.info.width = 200  # 100m x 100m
    msg.info.height = 200
    msg.info.origin.position.x = -msg.info.width * msg.info.resolution / 2.0
    msg.info.origin.position.y = -msg.info.height * msg.info.resolution / 2.0
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.x = 0.0
    msg.info.origin.orientation.y = 0.0
    msg.info.origin.orientation.z = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [fill_value] * msg.info.width * msg.info.height
    return msg


def parse_ogm_msg(msg: OccupancyGrid):
    """Parse nav_msgs occupancy grid message.

    Args:
        msg (OccupancyGrid): nav_msgs occupancy grid message

    Returns:
        np.ndarray: occupancy grid map
    """
    ogm = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    return ogm


# dummy tf broadcaster
def generate_static_transform_msg(stamp: rclpy.time.Time = None):
    """Generate static transform message from base_link to map.

    Returns:
        TransformStamped: static transform message
    """
    msg = TransformStamped()
    if stamp is None:
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    else:
        msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = "map"
    msg.child_frame_id = "base_link"
    msg.transform.translation.x = 0.0
    msg.transform.translation.y = 0.0
    msg.transform.translation.z = 0.0
    msg.transform.rotation.x = 0.0
    msg.transform.rotation.y = 0.0
    msg.transform.rotation.z = 0.0
    msg.transform.rotation.w = 1.0
    return msg


# --- TestSynchronizedOGMFusion ---
# 1. test free ogm and free ogm input fusion
# 2. test occupied ogm and occupied ogm input fusion
# 3. test unknown ogm and free ogm input fusion
# 4. test unknown ogm and occupied ogm input fusion
# 5. test free ogm and occupied ogm input fusion
class TestSynchronizedOGMFusion(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # init ROS at once
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # shutdown ROS at once
        rclpy.shutdown()

    def setUp(self):
        # called when each test started
        self.node = rclpy.create_node("grid_map_fusion_node_test_node")
        # send static transform from map to base_link
        tf_msg = generate_static_transform_msg()
        self.tf_broadcaster = StaticTransformBroadcaster(self.node)
        self.tf_broadcaster.sendTransform(tf_msg)

    def tearDown(self):
        # called when each test finished
        self.node.destroy_node()

    def callback(self, msg: OccupancyGrid):
        self.msg_buffer.append(msg)
        print("callback", len(self.msg_buffer))

    def get_newest_ogm_value(self):
        return np.mean(self.msg_buffer[-1].data)

    # util functions test 1~3
    def create_pub_sub(self):
        # create publisher
        pub1 = self.node.create_publisher(OccupancyGrid, INPUT_TOPICS[0], 10)
        pub2 = self.node.create_publisher(OccupancyGrid, INPUT_TOPICS[1], 10)

        # create subscriber for debug output
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.msg_buffer = []
        # subscribe to occupancy grid with buffer
        sub = self.node.create_subscription(
            OccupancyGrid, DEBUG_OUTPUT_TOPIC, self.callback, qos_profile=sensor_qos
        )
        return [pub1, pub2], sub

    # test functions
    def test_same_ogm_fusion(self):
        """Test 1~3.

        Expected output: same ogm.
        """
        # wait for the node to be ready
        time.sleep(3)
        pubs, sub = self.create_pub_sub()  # pubs have two publishers

        fill_values = [FREE_VALUE, OCCUPIED_VALUE, UNKNOWN_VALUE]

        for fill_value in fill_values:
            # create free/occupied/unknown ogm
            ogm = create_nav_msgs_occupancy_grid_msg(fill_value=fill_value)
            # publish free/occupied/unknown ogm
            pubs[0].publish(ogm)
            pubs[1].publish(ogm)

            # try to subscribe output pointcloud once
            rclpy.spin_once(self.node, timeout_sec=2.0)
            fused_mean = self.get_newest_ogm_value()
            print("same ogm test: ", fill_value, fused_mean)
            # assert almost equal
            self.assertAlmostEqual(fused_mean, fill_value, delta=3.0)

    def test_unknown_fusion(self):
        """Test unknown ogm and free ogm input fusion.

        Expected output: free, occupied, unknown.
        """
        # wait for the node to be ready
        time.sleep(3)
        pubs, sub = self.create_pub_sub()

        fill_values = [FREE_VALUE, OCCUPIED_VALUE, UNKNOWN_VALUE]
        now = rclpy.clock.Clock().now()
        unknown_ogm = create_nav_msgs_occupancy_grid_msg(fill_value=UNKNOWN_VALUE, stamp=now)

        for fill_value in fill_values:
            # publish unknown ogm
            pubs[0].publish(unknown_ogm)
            # create free/occupied/unknown ogm
            ogm = create_nav_msgs_occupancy_grid_msg(fill_value=fill_value, stamp=now)
            # publish ogm
            pubs[1].publish(ogm)

            # try to subscribe output pointcloud once
            rclpy.spin_once(self.node, timeout_sec=2.0)
            fused_mean = self.get_newest_ogm_value()
            print("unknown ogm test: ", fill_value, fused_mean)
            # assert almost equal
            self.assertAlmostEqual(fused_mean, fill_value, delta=3.0)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # check exit code
        launch_testing.asserts.assertExitCodes(proc_info)
