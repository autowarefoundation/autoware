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

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class SelfPoseListener(Node):
    def __init__(self):
        super().__init__("self_pose_listener")
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self)
        self.self_pose = PoseStamped()

    def get_current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            tf_time = self.tf_buffer.get_latest_common_time("map", "base_link")
            self.self_pose = SelfPoseListener.create_pose(tf_time, "map", tf)
        except LookupException as e:
            self.get_logger().warn("Required transformation not found: `{}`".format(str(e)))
            return None

    @staticmethod
    def create_pose(time, frame_id, tf):
        pose = PoseStamped()

        pose.header.stamp = time.to_msg()
        pose.header.frame_id = frame_id

        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation.x = tf.transform.rotation.x
        pose.pose.orientation.y = tf.transform.rotation.y
        pose.pose.orientation.z = tf.transform.rotation.z
        pose.pose.orientation.w = tf.transform.rotation.w

        return pose
