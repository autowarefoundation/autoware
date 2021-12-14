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
import sys

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Tf2PoseNode(Node):
    def __init__(self, options):
        super().__init__("tf2pose")

        self._options = options
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._pub_pose = self.create_publisher(PoseStamped, "/tier4_debug_tools/tf2pose/pose", 1)
        self.timer = self.create_timer((1.0 / self._options.hz), self._on_timer)

    def _on_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self._options.tf_from, self._options.tf_to, rclpy.time.Time()
            )
            time = self.tf_buffer.get_latest_common_time(self._options.tf_from, self._options.tf_to)
            pose = Tf2PoseNode.create_pose(time, self._options.tf_from, tf)
            self._pub_pose.publish(pose)
        except LookupException as e:
            print(e)

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


def main(args):
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("tf_from", type=str)
    parser.add_argument("tf_to", type=str)
    parser.add_argument("hz", type=int, default=10)
    ns = parser.parse_args(args)

    tf2pose_node = Tf2PoseNode(ns)
    rclpy.spin(tf2pose_node)
    tf2pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
