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

import math
import time

from geometry_msgs.msg import Quaternion
import numpy as np
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler


def get_rosbag_options(path, serialization_format="cdr"):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    return storage_options, converter_options


def open_reader(path: str):
    storage_options, converter_options = get_rosbag_options(path)
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def calc_squared_distance(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def create_empty_pointcloud(timestamp):
    pointcloud_msg = PointCloud2()
    pointcloud_msg.header.stamp = timestamp
    pointcloud_msg.header.frame_id = "map"
    pointcloud_msg.height = 1
    pointcloud_msg.is_dense = True
    pointcloud_msg.point_step = 16
    field_name_vec = ["x", "y", "z"]
    offset_vec = [0, 4, 8]
    for field_name, offset in zip(field_name_vec, offset_vec):
        field = PointField()
        field.name = field_name
        field.offset = offset
        field.datatype = 7
        field.count = 1
        pointcloud_msg.fields.append(field)
    return pointcloud_msg


def get_yaw_from_quaternion(orientation):
    orientation_list = [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ]
    return euler_from_quaternion(orientation_list)[2]


def get_quaternion_from_yaw(yaw):
    q = quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion()
    orientation.x = q[0]
    orientation.y = q[1]
    orientation.z = q[2]
    orientation.w = q[3]
    return orientation


def translate_objects_coordinate(ego_pose, log_ego_pose, objects_msg):
    log_ego_yaw = get_yaw_from_quaternion(log_ego_pose.orientation)
    log_ego_pose_trans_mat = np.array(
        [
            [
                math.cos(log_ego_yaw),
                -math.sin(log_ego_yaw),
                log_ego_pose.position.x,
            ],
            [math.sin(log_ego_yaw), math.cos(log_ego_yaw), log_ego_pose.position.y],
            [0.0, 0.0, 1.0],
        ]
    )

    ego_yaw = get_yaw_from_quaternion(ego_pose.orientation)
    ego_pose_trans_mat = np.array(
        [
            [math.cos(ego_yaw), -math.sin(ego_yaw), ego_pose.position.x],
            [math.sin(ego_yaw), math.cos(ego_yaw), ego_pose.position.y],
            [0.0, 0.0, 1.0],
        ]
    )

    for o in objects_msg.objects:
        log_object_pose = o.kinematics.pose_with_covariance.pose
        log_object_yaw = get_yaw_from_quaternion(log_object_pose.orientation)
        log_object_pos_vec = np.array([log_object_pose.position.x, log_object_pose.position.y, 1.0])

        # translate object pose from ego pose in log to ego pose in simulation
        object_pos_vec = np.linalg.inv(ego_pose_trans_mat).dot(
            log_ego_pose_trans_mat.dot(log_object_pos_vec.T)
        )

        object_pose = o.kinematics.pose_with_covariance.pose
        object_pose.position.x = object_pos_vec[0]
        object_pose.position.y = object_pos_vec[1]
        object_pose.orientation = get_quaternion_from_yaw(log_object_yaw + log_ego_yaw - ego_yaw)


class StopWatch:
    def __init__(self, verbose):
        # A dictionary to store the starting times
        self.start_times = {}
        self.verbose = verbose

    def tic(self, name):
        """Store the current time with the given name."""
        self.start_times[name] = time.perf_counter()

    def toc(self, name):
        """Print the elapsed time since the last call to tic() with the same name."""
        if name not in self.start_times:
            print(f"No start time found for {name}!")
            return

        elapsed_time = (
            time.perf_counter() - self.start_times[name]
        ) * 1000  # Convert to milliseconds
        if self.verbose:
            print(f"Time for {name}: {elapsed_time:.2f} ms")

        # Reset the starting time for the name
        del self.start_times[name]
