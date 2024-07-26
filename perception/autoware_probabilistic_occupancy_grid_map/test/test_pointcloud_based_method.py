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

import struct
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
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

PACKAGE_NAME = "autoware_probabilistic_occupancy_grid_map"
INPUT_TOPIC_RAW = "/raw"
INPUT_TOPIC_OBSTACLE = "/obstacle"


# test launcher
@pytest.mark.launch_test
def generate_test_description():
    """Launch file test description.

    Returns:
        _type_: launch.LaunchDescription
    """
    # get launch file path
    launch_file_path = (
        FindPackageShare(PACKAGE_NAME).find(PACKAGE_NAME)
        + "/launch/pointcloud_based_occupancy_grid_map.launch.py"
    )
    launch_args = [
        ("input/obstacle_pointcloud", INPUT_TOPIC_OBSTACLE),
        ("input/raw_pointcloud", INPUT_TOPIC_RAW),
    ]
    # action to include launch file
    test_launch_file = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments=launch_args,
    )

    return launch.LaunchDescription(
        [
            test_launch_file,
            launch_testing.actions.ReadyToTest(),
        ]
    )


# util functions
def get_pointcloud_msg(pts: list):
    """Create ros2 point cloud message from list of points.

    Args:
        pts (list): list of points [[x, y, z], ...]

    Returns:
        PointCloud2: ros2 point cloud message
    """
    msg = PointCloud2()
    np_pts = np.array(pts, dtype=np.float32).reshape(-1, 3)
    binary_pts = np_pts.tobytes()

    # set current time
    now = rclpy.clock.Clock().now()
    msg.header.stamp = now.to_msg()
    msg.header.frame_id = "base_link"
    msg.height = 1
    msg.width = np_pts.shape[0]
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    # msg.data = b""
    # msg_data_list = []
    # for pt in pts:
    #     msg_data_list.append(struct.pack('fff', pt[0], pt[1], pt[2]))
    # msg.data = b"".join(msg_data_list)
    msg.data = binary_pts
    return msg


def parse_pointcloud_msg(msg: PointCloud2):
    """Parse ros2 point cloud message to list of points.

    Args:
        msg (PointCloud2): ros2 point cloud message

    Returns:
        list: list of points [[x, y, z], ...]
    """
    pts = []
    for i in range(msg.width):
        offset = msg.point_step * i
        x, y, z = struct.unpack("fff", msg.data[offset : offset + 12])
        pts.append([x, y, z])
    return pts


def generate_static_transform_msg():
    """Generate static transform message from base_link to map.

    Returns:
        TransformStamped: static transform message
    """
    msg = TransformStamped()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
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


# Test Node IO
class TestNodeIO(unittest.TestCase):
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
        self.node = rclpy.create_node("test_node_io")
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

    # util functions
    def create_pub_sub(self):
        # create publisher
        pub_raw = self.node.create_publisher(PointCloud2, INPUT_TOPIC_RAW, 10)
        pub_obstacle = self.node.create_publisher(PointCloud2, INPUT_TOPIC_OBSTACLE, 10)

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # create subscriber
        self.msg_buffer = []
        # subscribe to occupancy grid with buffer
        sub = self.node.create_subscription(
            OccupancyGrid, "/occupancy_grid", self.callback, qos_profile=sensor_qos
        )
        return pub_raw, pub_obstacle, sub

    # test functions
    def test_normal_input(self):
        """Test normal input.

        input: normal pointcloud
        output: normal ogm
        """
        # wait for the node to be ready
        time.sleep(3)
        # min_height, max_height = -1.0, 2.0
        input_points = [[1.0, 1.0, 1.0], [2.0, 2.0, 2.0]]
        pub_raw, pub_obstacle, sub = self.create_pub_sub()
        # publish input pointcloud
        pt_msg = get_pointcloud_msg(input_points)
        pub_raw.publish(pt_msg)
        pub_obstacle.publish(pt_msg)
        # try to subscribe output pointcloud once
        rclpy.spin_once(self.node, timeout_sec=3.0)
        self.assertEqual(len(self.msg_buffer), 1)

    def test_null_input(self, proc_info):
        """Test null input.

        input: null pointcloud
        output: null ogm
        """
        # wait for the node to be ready
        time.sleep(3)
        input_points = []
        pub_raw, pub_obstacle, sub = self.create_pub_sub()
        # publish input pointcloud
        pt_msg = get_pointcloud_msg(input_points)
        pub_raw.publish(pt_msg)
        pub_obstacle.publish(pt_msg)
        # try to subscribe output pointcloud once
        rclpy.spin_once(self.node, timeout_sec=3.0)

        # check if process is successfully terminated
        nodes = self.node.get_node_names()
        self.assertIn("occupancy_grid_map_node", nodes)
        self.assertEqual(len(self.msg_buffer), 1)

    def test_null_input2(self, proc_info):
        """Test null input.

        input: null pointcloud without even frame_id
        output: null ogm
        """
        # wait for the node to be ready
        time.sleep(3)
        input_points = []
        pub_raw, pub_obstacle, sub = self.create_pub_sub()
        # publish input pointcloud
        pt_msg = get_pointcloud_msg(input_points)
        pt_msg.header.frame_id = ""
        pub_raw.publish(pt_msg)
        pub_obstacle.publish(pt_msg)
        # try to subscribe output pointcloud once
        rclpy.spin_once(self.node, timeout_sec=3.0)

        # check if process is successfully terminated
        nodes = self.node.get_node_names()
        self.assertIn("occupancy_grid_map_node", nodes)
        self.assertEqual(len(self.msg_buffer), 0)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # check exit code
        launch_testing.asserts.assertExitCodes(proc_info)
