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
import pickle

import numpy as np
from perception_replayer_common import PerceptionReplayerCommon
import rclpy
from utils import StopWatch
from utils import create_empty_pointcloud
from utils import translate_objects_coordinate


class PerceptionReproducer(PerceptionReplayerCommon):
    def __init__(self, args):
        super().__init__(args, "perception_reproducer")

        self.prev_traffic_signals_msg = None
        self.stopwatch = StopWatch(self.args.verbose)  # for debug

        # to make some data to accelerate computation
        self.preprocess_data()

        # start main timer callback
        self.timer = self.create_timer(0.1, self.on_timer)

        # kill perception process to avoid a conflict of the perception topics
        self.timer_check_perception_process = self.create_timer(3.0, self.on_timer_kill_perception)

        print("Start timer callback")

    def preprocess_data(self):
        # closest search with numpy data is much faster than usual
        self.rosbag_ego_odom_data_numpy = np.array(
            [
                [data[1].pose.pose.position.x, data[1].pose.pose.position.y]
                for data in self.rosbag_ego_odom_data
            ]
        )

    def on_timer_kill_perception(self):
        self.kill_online_perception_node()

    def on_timer(self):
        if self.args.verbose:
            print("\n-- on_timer start --")
        self.stopwatch.tic("total on_timer")

        timestamp = self.get_clock().now().to_msg()

        if self.args.detected_object:
            pointcloud_msg = create_empty_pointcloud(timestamp)
            self.pointcloud_pub.publish(pointcloud_msg)

        if not self.ego_pose:
            print("No ego pose found.")
            return

        # find nearest ego odom by simulation observation
        self.stopwatch.tic("find_nearest_ego_odom_by_observation")
        ego_odom = self.find_nearest_ego_odom_by_observation(self.ego_pose)
        pose_timestamp = ego_odom[0]
        log_ego_pose = ego_odom[1].pose.pose
        self.stopwatch.toc("find_nearest_ego_odom_by_observation")

        # extract message by the nearest ego odom timestamp
        self.stopwatch.tic("find_topics_by_timestamp")
        msgs_orig = self.find_topics_by_timestamp(pose_timestamp)
        self.stopwatch.toc("find_topics_by_timestamp")

        # copy the messages
        self.stopwatch.tic("message deepcopy")
        if self.args.detected_object:
            msgs = pickle.loads(pickle.dumps(msgs_orig))  # this is x5 faster than deepcopy
            objects_msg = msgs[0]
            traffic_signals_msg = msgs[1]
        else:
            # NOTE: No need to deepcopy since only timestamp will be changed and it will be changed every time correctly.
            objects_msg = msgs_orig[0]
            traffic_signals_msg = msgs_orig[1]
        self.stopwatch.toc("message deepcopy")

        self.stopwatch.tic("transform and publish")
        # objects
        if objects_msg:
            objects_msg.header.stamp = timestamp
            if self.args.detected_object:
                translate_objects_coordinate(self.ego_pose, log_ego_pose, objects_msg)
            self.objects_pub.publish(objects_msg)

        # ego odom
        self.recorded_ego_pub.publish(ego_odom[1])

        # traffic signals
        # temporary support old auto msgs
        if traffic_signals_msg:
            if self.is_auto_traffic_signals:
                traffic_signals_msg.header.stamp = timestamp
                self.auto_traffic_signals_pub.publish(traffic_signals_msg)
            else:
                traffic_signals_msg.stamp = timestamp
                self.traffic_signals_pub.publish(traffic_signals_msg)
            self.prev_traffic_signals_msg = traffic_signals_msg
        elif self.prev_traffic_signals_msg:
            if self.is_auto_traffic_signals:
                self.prev_traffic_signals_msg.header.stamp = timestamp
                self.auto_traffic_signals_pub.publish(self.prev_traffic_signals_msg)
            else:
                self.prev_traffic_signals_msg.stamp = timestamp
                self.traffic_signals_pub.publish(self.prev_traffic_signals_msg)
        self.stopwatch.toc("transform and publish")

        self.stopwatch.toc("total on_timer")

    def find_nearest_ego_odom_by_observation(self, ego_pose):
        # nearest search with numpy format is much (~ x100) faster than regular for loop
        self_pose = np.array([ego_pose.position.x, ego_pose.position.y])
        dists_squared = np.sum((self.rosbag_ego_odom_data_numpy - self_pose) ** 2, axis=1)
        nearest_idx = np.argmin(dists_squared)

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
    parser.add_argument(
        "-f", "--rosbag-format", help="rosbag data format (default is db3)", default="db3"
    )
    parser.add_argument(
        "-v", "--verbose", help="output debug data", action="store_true", default=False
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
