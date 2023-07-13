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

import unittest

from geometry_msgs.msg import TwistWithCovarianceStamped
import launch
from launch.logging import get_logger
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

logger = get_logger(__name__)


@pytest.mark.launch_test
def generate_test_description():
    nodes = []

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            parameters=[
                {"use_imu": True},
            ],
            remappings=[
                ("~/input/twist", "/test/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/test/sensing/imu/imu_data"),
                ("~/input/pointcloud", "/test/sensing/lidar/top/mirror_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "/test/sensing/lidar/top/rectified/pointcloud_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    container = ComposableNodeContainer(
        name="test_distortion_corrector_container",
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
    )

    return launch.LaunchDescription(
        [
            container,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestDistortionCorrector(unittest.TestCase):
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
        self.evaluation_time = 2  # 200ms

    def tearDown(self):
        self.test_node.destroy_node()

    @staticmethod
    def print_message(stat):
        logger.debug("===========================")
        logger.debug(stat)

    @staticmethod
    def pointcloud2_to_xyz_array(cloud_msg):
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=np.float32)
        cloud_arr = np.reshape(cloud_arr, (cloud_msg.width, int(cloud_msg.point_step / 4)))
        return cloud_arr[:, :3]

    @staticmethod
    def generate_pointcloud(num_points, header, timestamp_offset_per_point=0.01):
        points_xyz = np.random.rand(num_points, 3).astype(np.float32)
        timestamps = np.array(
            [
                header.stamp.sec + header.stamp.nanosec * 1e-9 + timestamp_offset_per_point * i
                for i in range(num_points)
            ]
        ).astype(np.float64)
        xyz_data = points_xyz.tobytes()
        timestamp_data = timestamps.tobytes()
        pointcloud_data = b"".join(
            xyz_data[i * 12 : i * 12 + 12] + timestamp_data[i * 8 : i * 8 + 8]
            for i in range(len(xyz_data))
        )
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="time_stamp", offset=12, datatype=PointField.FLOAT64, count=1),
        ]
        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=10,
            is_dense=True,
            is_bigendian=False,
            point_step=20,  # 4 float32 fields * 4 bytes/field
            row_step=20 * num_points,  # point_step * width
            fields=fields,
            data=pointcloud_data,
        )
        return pointcloud_msg

    def test_node_link(self):
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Publishers
        imu_pub = self.test_node.create_publisher(Imu, "/test/sensing/imu/imu_data", 10)
        velocity_pub = self.test_node.create_publisher(
            TwistWithCovarianceStamped,
            "/test/sensing/vehicle_velocity_converter/twist_with_covariance",
            10,
        )
        pointcloud_pub = self.test_node.create_publisher(
            PointCloud2,
            "/test/sensing/lidar/top/mirror_cropped/pointcloud_ex",
            qos_profile=sensor_qos,
        )

        # Subscribers
        received_data = {"msg": None}

        def pointcloud_callback(msg):
            received_data["msg"] = msg

        self.test_node.create_subscription(
            PointCloud2,
            "/test/sensing/lidar/top/rectified/pointcloud_ex",
            pointcloud_callback,
            qos_profile=sensor_qos,
        )

        # Wait for composable node launches
        rclpy.spin_once(self.test_node, timeout_sec=1.0)

        # Prepare header
        header = Header()
        header.stamp = self.test_node.get_clock().now().to_msg()
        header.frame_id = "base_link"

        # Publish IMU and velocity data
        for i in range(50):
            header_tmp = Header()
            header_tmp.stamp = self.test_node.get_clock().now().to_msg()
            header_tmp.frame_id = "base_link"

            imu_msg = Imu()
            imu_msg.header = header_tmp
            velocity_msg = TwistWithCovarianceStamped()
            velocity_msg.header = header_tmp
            velocity_msg.twist.twist.linear.x = 1.0

            imu_pub.publish(imu_msg)
            velocity_pub.publish(velocity_msg)
            rclpy.spin_once(self.test_node, timeout_sec=0.01)

        # Publish pointcloud data
        num_points = 10
        pointcloud_msg = self.generate_pointcloud(num_points, header)
        pointcloud_pub.publish(pointcloud_msg)

        # Wait for output with a timeout
        start_time = self.test_node.get_clock().now()
        while self.test_node.get_clock().now() - start_time < rclpy.duration.Duration(
            seconds=self.evaluation_time
        ):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if received_data["msg"] is not None:
                break

        # Check if the output was received
        self.assertIsNotNone(received_data["msg"], "Did not receive output pointcloud data")

        # Check that the received pointcloud data has same length as num_points
        received_pointcloud = received_data["msg"]
        self.assertEqual(
            received_pointcloud.width,
            num_points,
            "The received pointcloud data has a different length than expected",
        )

        # Check that the received pointcloud data is different from the original one
        original_pointcloud_arr = self.pointcloud2_to_xyz_array(pointcloud_msg)
        received_pointcloud_arr = self.pointcloud2_to_xyz_array(received_pointcloud)
        self.assertFalse(
            np.allclose(original_pointcloud_arr, received_pointcloud_arr, atol=1e-6),
            "The received pointcloud data is not different from the original one",
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
