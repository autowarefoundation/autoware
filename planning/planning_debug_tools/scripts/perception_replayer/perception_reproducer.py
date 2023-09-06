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

from perception_replayer_common import PerceptionReplayerCommon
import rclpy
from utils import calc_squared_distance
from utils import create_empty_pointcloud
from utils import translate_objects_coordinate


class PerceptionReproducer(PerceptionReplayerCommon):
    def __init__(self, args):
        super().__init__(args, "perception_reproducer")

        self.ego_pose_idx = None
        self.prev_traffic_signals_msg = None

        # start timer callback
        self.timer = self.create_timer(0.1, self.on_timer)
        print("Start timer callback")

    def on_timer(self):
        timestamp = self.get_clock().now().to_msg()

        self.kill_online_perception_node()

        if self.args.detected_object:
            pointcloud_msg = create_empty_pointcloud(timestamp)
            self.pointcloud_pub.publish(pointcloud_msg)

        if not self.ego_pose:
            print("No ego pose found.")
            return

        # find nearest ego odom by simulation observation
        ego_odom = self.find_nearest_ego_odom_by_observation(self.ego_pose)
        pose_timestamp = ego_odom[0]
        log_ego_pose = ego_odom[1].pose.pose

        # extract message by the nearest ego odom timestamp
        msgs = copy.deepcopy(self.find_topics_by_timestamp(pose_timestamp))
        objects_msg = msgs[0]
        traffic_signals_msg = msgs[1]

        # objects
        if objects_msg:
            objects_msg.header.stamp = timestamp
            if self.args.detected_object:
                translate_objects_coordinate(self.ego_pose, log_ego_pose, objects_msg)
            self.objects_pub.publish(objects_msg)

        # traffic signals
        if traffic_signals_msg:
            traffic_signals_msg.stamp = timestamp
            self.traffic_signals_pub.publish(traffic_signals_msg)
            self.prev_traffic_signals_msg = traffic_signals_msg
        elif self.prev_traffic_signals_msg:
            self.prev_traffic_signals_msg.stamp = timestamp
            self.traffic_signals_pub.publish(self.prev_traffic_signals_msg)

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


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bag", help="rosbag", default=None)
    parser.add_argument(
        "-d", "--detected-object", help="publish detected object", action="store_true"
    )
    parser.add_argument(
        "-t", "--tracked-object", help="publish tracked object", action="store_true"
    )
    args = parser.parse_args()

    rclpy.init()
    node = PerceptionReproducer(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
