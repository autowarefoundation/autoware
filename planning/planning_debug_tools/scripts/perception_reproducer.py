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

import argparse
import copy
import math
import os
from subprocess import CalledProcessError
from subprocess import check_output
import time

from autoware_auto_perception_msgs.msg import DetectedObjects
from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_perception_msgs.msg import TrafficSignalArray
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import psutil
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosbag2_py import StorageFilter
from rosidl_runtime_py.utilities import get_message
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


class PerceptionReproducer(Node):
    def __init__(self, args):
        super().__init__("perception_reproducer")
        self.args = args

        # subscriber
        self.sub_odom = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.on_odom, 1
        )

        # publisher
        if self.args.predicted_object:
            self.objects_pub = self.create_publisher(
                PredictedObjects, "/perception/object_recognition/objects", 1
            )
        else:
            self.objects_pub = self.create_publisher(
                DetectedObjects, "/perception/object_recognition/detection/objects", 1
            )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, "/perception/obstacle_segmentation/pointcloud", 1
        )
        self.traffic_signals_pub = self.create_publisher(
            TrafficSignalArray, "/perception/traffic_light_recognition/traffic_signals", 1
        )

        self.ego_pose_idx = None
        self.ego_pose = None

        self.prev_traffic_signals_msg = None

        self.rosbag_objects_data = []
        self.rosbag_ego_odom_data = []
        self.rosbag_traffic_signals_data = []

        # load rosbag
        print("Stared loading rosbag")
        if args.bag:
            self.load_rosbag(args.bag)
        elif args.directory:
            for bag_file in sorted(os.listdir(args.directory)):
                self.load_rosbag(args.directory + "/" + bag_file)
        print("Ended loading rosbag")

        # wait for ready to publish/subscribe
        time.sleep(1.0)

        self.timer = self.create_timer(0.01, self.on_timer)
        print("Start timer callback")

    def on_odom(self, odom):
        self.ego_pose = odom.pose.pose

    def on_timer(self):
        # kill node if required
        kill_process_name = None
        if self.args.predicted_object:
            kill_process_name = "map_based_prediction"
        else:
            kill_process_name = "dummy_perception_publisher_node"
        if kill_process_name:
            try:
                pid = check_output(["pidof", kill_process_name])
                process = psutil.Process(int(pid[:-1]))
                process.terminate()
            except CalledProcessError:
                pass

        timestamp = self.get_clock().now().to_msg()

        if not self.args.predicted_object:
            pointcloud_msg = create_empty_pointcloud(timestamp)
            self.pointcloud_pub.publish(pointcloud_msg)

        if self.ego_pose:
            ego_odom = self.find_nearest_ego_odom_by_observation(self.ego_pose)
            pose_timestamp = ego_odom[0]
            log_ego_pose = ego_odom[1].pose.pose

            msgs = copy.deepcopy(self.find_topics_by_timestamp(pose_timestamp))
            objects_msg = msgs[0]
            traffic_signals_msg = msgs[1]
            if objects_msg:
                objects_msg.header.stamp = timestamp
                if not self.args.predicted_object:
                    objects_msg.header.frame_id = "map"
                    for o in objects_msg.objects:
                        object_pose = o.kinematics.pose_with_covariance.pose
                        ego_yaw = get_yaw_from_quaternion(log_ego_pose.orientation)
                        theta = math.atan2(object_pose.position.x, object_pose.position.y)
                        length = math.hypot(object_pose.position.x, object_pose.position.y)

                        object_pose.position.x = log_ego_pose.position.x + length * math.cos(
                            ego_yaw + theta
                        )
                        object_pose.position.y = log_ego_pose.position.y + length * math.sin(
                            ego_yaw + theta
                        )

                        obj_yaw = get_yaw_from_quaternion(object_pose.orientation)
                        object_pose.orientation = get_quaternion_from_yaw(ego_yaw + obj_yaw)

                self.objects_pub.publish(objects_msg)
            if traffic_signals_msg:
                traffic_signals_msg.header.stamp = timestamp
                self.traffic_signals_pub.publish(traffic_signals_msg)
                self.prev_traffic_signals_msg = traffic_signals_msg
            elif self.prev_traffic_signals_msg:
                self.prev_traffic_signals_msg.header.stamp = timestamp
                self.traffic_signals_pub.publish(self.prev_traffic_signals_msg)
        else:
            print("No ego pose found.")

    def find_nearest_ego_odom_by_observation(self, ego_pose):
        if self.ego_pose_idx:
            start_idx = self.ego_pose_idx - 10
            end_idx = self.ego_pose_idx + 10
        else:
            start_idx = 0
            end_idx = len(self.rosbag_ego_odom_data) - 1

        nearest_idx = 0
        nearest_dist = float("inf")
        for idx in range(start_idx, end_idx + 1):
            data = self.rosbag_ego_odom_data[idx]
            dist = calc_squared_distance(data[1].pose.pose.position, ego_pose.position)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_idx = idx

        return self.rosbag_ego_odom_data[nearest_idx]

    def find_topics_by_timestamp(self, timestamp):
        objects_data = None
        for data in self.rosbag_objects_data:
            if timestamp < data[0]:
                objects_data = data[1]
                break

        traffic_signals_data = None
        for data in self.rosbag_traffic_signals_data:
            if timestamp < data[0]:
                traffic_signals_data = data[1]
                break

        return objects_data, traffic_signals_data

    def load_rosbag(self, rosbag2_path: str):
        reader = open_reader(str(rosbag2_path))

        topic_types = reader.get_all_topics_and_types()
        # Create a map for quicker lookup
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        objects_topic = (
            "/perception/object_recognition/objects"
            if self.args.predicted_object
            else "/perception/object_recognition/detection/objects"
        )
        ego_odom_topic = "/localization/kinematic_state"
        traffic_signals_topic = "/perception/traffic_light_recognition/traffic_signals"
        topic_filter = StorageFilter(topics=[objects_topic, ego_odom_topic, traffic_signals_topic])
        reader.set_filter(topic_filter)

        while reader.has_next():
            (topic, data, stamp) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            if topic == objects_topic:
                self.rosbag_objects_data.append((stamp, msg))
            if topic == ego_odom_topic:
                self.rosbag_ego_odom_data.append((stamp, msg))
            if topic == traffic_signals_topic:
                self.rosbag_traffic_signals_data.append((stamp, msg))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bag", help="rosbag", default=None)
    parser.add_argument("-d", "--directory", help="directory of rosbags", default=None)
    parser.add_argument(
        "-p", "--predicted-object", help="publish predicted object", action="store_true"
    )
    args = parser.parse_args()

    rclpy.init()
    node = PerceptionReproducer(args)
    rclpy.spin(node)

    try:
        rclpy.init()
        node = PerceptionReproducer(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
