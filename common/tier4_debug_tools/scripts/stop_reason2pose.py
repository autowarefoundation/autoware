#! /usr/bin/env python3

# Copyright 2020 Tier IV, Inc.
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
import math
import sys

from case_converter import pascal2snake
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rtree import index
from self_pose_listener import SelfPoseListener
from tier4_planning_msgs.msg import StopReasonArray


class StopReason2PoseNode(Node):
    def __init__(self, options):
        super().__init__("stop_reason2pose_node")
        self._options = options
        self._sub_pose = self.create_subscription(
            StopReasonArray, self._options.topic_name, self._on_stop_reasons, 1
        )
        self._pub_pose_map = {}
        self._idx_map = {}
        self._pose_map = {}
        self._self_pose_listener = SelfPoseListener()
        self.timer = self.create_timer((1.0 / 100), self._self_pose_listener.get_current_pose)

    def _on_stop_reasons(self, msg):
        for stop_reason in msg.stop_reasons:
            snake_case_stop_reason = pascal2snake(stop_reason.reason)

            if len(stop_reason.stop_factors) == 0:
                self.get_logger().warn("stop_factor is null")
                return

            for stop_factor in stop_reason.stop_factors:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose = stop_factor.stop_pose

                # Get nearest pose
                th_dist = 1.0
                nearest_pose_id = self._get_nearest_pose_id(
                    snake_case_stop_reason, pose.pose, th_dist
                )
                if nearest_pose_id:
                    self._update_pose(snake_case_stop_reason, pose.pose, nearest_pose_id)
                    pose_id = nearest_pose_id
                else:
                    pose_id = self._register_pose(snake_case_stop_reason, pose.pose)

                pose_topic_name = "{snake_case_stop_reason}_{pose_id}".format(**locals())
                topic_ns = "/tier4_debug_tools/stop_reason2pose/"
                if pose_topic_name not in self._pub_pose_map:
                    self._pub_pose_map[pose_topic_name] = self.create_publisher(
                        PoseStamped, topic_ns + pose_topic_name, 1
                    )
                self._pub_pose_map[pose_topic_name].publish(pose)

            # Publish nearest stop_reason without number
            nearest_pose = PoseStamped()
            nearest_pose.header = msg.header
            nearest_pose.pose = self._get_nearest_pose_in_array(
                stop_reason, self._self_pose_listener.self_pose
            )

            if nearest_pose.pose:
                if snake_case_stop_reason not in self._pub_pose_map:
                    topic_ns = "/tier4_debug_tools/stop_reason2pose/"
                    self._pub_pose_map[snake_case_stop_reason] = self.create_publisher(
                        PoseStamped, topic_ns + snake_case_stop_reason, 1
                    )
                self._pub_pose_map[snake_case_stop_reason].publish(nearest_pose)

    def _get_nearest_pose_in_array(self, stop_reason, self_pose):
        poses = [stop_factor.stop_pose for stop_factor in stop_reason.stop_factors]
        if not poses:
            return None

        distances = [StopReason2PoseNode.calc_distance2d(p, self_pose.pose) for p in poses]
        nearest_idx = np.argmin(distances)

        return poses[nearest_idx]

    def _find_nearest_pose_id(self, name, pose):
        if name not in self._idx_map:
            self._idx_map[name] = index.Index()

        return self._idx_map[name].nearest(StopReason2PoseNode.pose2boundingbox(pose), 1)

    def _get_nearest_pose_id(self, name, pose, th_dist):
        nearest_pose_ids = list(self._find_nearest_pose_id(name, pose))
        if not nearest_pose_ids:
            return None

        nearest_pose_id = nearest_pose_ids[0]
        nearest_pose = self._get_pose(name, nearest_pose_id)
        if not nearest_pose:
            return None

        dist = StopReason2PoseNode.calc_distance2d(pose, nearest_pose)
        if dist > th_dist:
            return None

        return nearest_pose_id

    def _get_pose(self, name, pose_id):
        if name not in self._pose_map:
            return None

        return self._pose_map[name][pose_id]

    def _update_pose(self, name, pose, pose_id):
        self._pose_map[name][id] = pose
        self._idx_map[name].insert(pose_id, StopReason2PoseNode.pose2boundingbox(pose))

    def _register_pose(self, name, pose):
        if name not in self._pose_map:
            self._pose_map[name] = {}

        pose_id = len(self._pose_map[name]) + 1
        self._pose_map[name][pose_id] = pose
        self._idx_map[name].insert(pose_id, StopReason2PoseNode.pose2boundingbox(pose))
        return pose_id

    @staticmethod
    def calc_distance2d(pose1, pose2):
        p1 = pose1.position
        p2 = pose2.position
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    @staticmethod
    def pose2boundingbox(pose):
        return [pose.position.x, pose.position.y, pose.position.x, pose.position.y]


def main(args):
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("topic_name", type=str)
    ns = parser.parse_args(args)

    stop_reason2pose_node = StopReason2PoseNode(ns)
    rclpy.spin(stop_reason2pose_node)
    stop_reason2pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
