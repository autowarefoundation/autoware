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
from autoware_auto_perception_msgs.msg import TrafficSignalArray
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
        self.traffic_signals_pub = self.create_publisher(
            TrafficSignalArray, "/perception/traffic_light_recognition/traffic_signals", 1
        )

        # load rosbag
        print("Stared loading rosbag")
        if os.path.isdir(args.bag):
            for bag_file in sorted(os.listdir(args.bag)):
                self.load_rosbag(args.bag + "/" + bag_file)
        else:
            self.load_rosbag(args.bag)
        print("Ended loading rosbag")

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

    def find_ego_odom_by_timestamp(self, timestamp):
        ego_odom_data = None
        for data in self.rosbag_ego_odom_data:
            if timestamp < data[0]:
                ego_odom_data = data[1]
                break

        return ego_odom_data
