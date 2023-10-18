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
from subprocess import CalledProcessError
from subprocess import check_output
import time

from autoware_auto_perception_msgs.msg import DetectedObjects
from autoware_auto_perception_msgs.msg import PredictedObjects
from autoware_auto_perception_msgs.msg import TrackedObjects
from autoware_auto_perception_msgs.msg import TrafficSignalArray as AutoTrafficSignalArray
from autoware_perception_msgs.msg import TrafficSignalArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import psutil
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosbag2_py import StorageFilter
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2
from utils import open_reader


class PerceptionReplayerCommon(Node):
    def __init__(self, args, name):
        super().__init__(name)
        self.args = args

        self.ego_pose = None
        self.rosbag_objects_data = []
        self.rosbag_ego_odom_data = []
        self.rosbag_traffic_signals_data = []
        self.is_auto_traffic_signals = False

        # subscriber
        self.sub_odom = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.on_odom, 1
        )

        # publisher
        if self.args.detected_object:
            self.objects_pub = self.create_publisher(
                DetectedObjects, "/perception/object_recognition/detection/objects", 1
            )
        elif self.args.tracked_object:
            self.objects_pub = self.create_publisher(
                TrackedObjects, "/perception/object_recognition/tracking/objects", 1
            )
        else:
            self.objects_pub = self.create_publisher(
                PredictedObjects, "/perception/object_recognition/objects", 1
            )

        self.pointcloud_pub = self.create_publisher(
            PointCloud2, "/perception/obstacle_segmentation/pointcloud", 1
        )
        self.recorded_ego_pub_as_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )

        self.recorded_ego_pub = self.create_publisher(
            Odometry, "/perception_reproducer/rosbag_ego_odom", 1
        )

        # load rosbag
        print("Stared loading rosbag")
        if os.path.isdir(args.bag):
            for bag_file in sorted(os.listdir(args.bag)):
                if bag_file.endswith(self.args.rosbag_format):
                    self.load_rosbag(args.bag + "/" + bag_file)
        else:
            self.load_rosbag(args.bag)
        print("Ended loading rosbag")

        # temporary support old auto msgs
        if self.is_auto_traffic_signals:
            self.auto_traffic_signals_pub = self.create_publisher(
                AutoTrafficSignalArray, "/perception/traffic_light_recognition/traffic_signals", 1
            )
        else:
            self.traffic_signals_pub = self.create_publisher(
                TrafficSignalArray, "/perception/traffic_light_recognition/traffic_signals", 1
            )

        # wait for ready to publish/subscribe
        time.sleep(1.0)

    def on_odom(self, odom):
        self.ego_pose = odom.pose.pose

    def load_rosbag(self, rosbag2_path: str):
        reader = open_reader(str(rosbag2_path))

        topic_types = reader.get_all_topics_and_types()
        # Create a map for quicker lookup
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        objects_topic = (
            "/perception/object_recognition/detection/objects"
            if self.args.detected_object
            else "/perception/object_recognition/tracking/objects"
            if self.args.tracked_object
            else "/perception/object_recognition/objects"
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
                self.is_auto_traffic_signals = (
                    "autoware_auto_perception_msgs" in type(msg).__module__
                )

    def kill_online_perception_node(self):
        # kill node if required
        kill_process_name = None
        if self.args.detected_object:
            kill_process_name = "dummy_perception_publisher_node"
        elif self.args.tracked_object:
            kill_process_name = "multi_object_tracker"
        else:
            kill_process_name = "map_based_prediction"
        if kill_process_name:
            try:
                pid = check_output(["pidof", kill_process_name])
                process = psutil.Process(int(pid[:-1]))
                process.terminate()
            except CalledProcessError:
                pass

    def binary_search(self, data, timestamp):
        if data[-1][0] < timestamp:
            return data[-1][1]
        elif data[0][0] > timestamp:
            return data[0][1]

        low, high = 0, len(data) - 1

        while low <= high:
            mid = (low + high) // 2
            if data[mid][0] < timestamp:
                low = mid + 1
            elif data[mid][0] > timestamp:
                high = mid - 1
            else:
                return data[mid][1]

        # Return the next timestamp's data if available
        if low < len(data):
            return data[low][1]
        return None

    def find_topics_by_timestamp(self, timestamp):
        objects_data = self.binary_search(self.rosbag_objects_data, timestamp)
        traffic_signals_data = self.binary_search(self.rosbag_traffic_signals_data, timestamp)
        return objects_data, traffic_signals_data

    def find_ego_odom_by_timestamp(self, timestamp):
        return self.binary_search(self.rosbag_ego_odom_data, timestamp)
