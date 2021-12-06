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
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
import tf2_ros


class Pose2TfNode(Node):
    def __init__(self, options):
        super().__init__("pose2tf")
        self._options = options
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._sub_pose = self.create_subscription(
            PoseStamped, self._options.topic_name, self._on_pose, 1
        )

    def _on_pose(self, msg):
        try:
            tfs = Pose2TfNode.create_transform(self, msg)
            transforms = []
            transforms.append(tfs)
            self._tf_broadcaster.sendTransform(transforms)
        except tf2_ros.TransformException as e:
            print(e)

    @staticmethod
    def create_transform(self, msg):
        tfs = TransformStamped()
        tfs.header.frame_id = msg.header.frame_id
        tfs.header.stamp = msg.header.stamp
        tfs.child_frame_id = self._options.tf_name
        tfs.transform.translation.x = msg.pose.position.x
        tfs.transform.translation.y = msg.pose.position.y
        tfs.transform.translation.z = msg.pose.position.z
        tfs.transform.rotation.x = msg.pose.orientation.x
        tfs.transform.rotation.y = msg.pose.orientation.y
        tfs.transform.rotation.z = msg.pose.orientation.z
        tfs.transform.rotation.w = msg.pose.orientation.w
        return tfs


def main(args):
    print("{}".format(args))
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("topic_name", type=str)
    parser.add_argument("tf_name", type=str)
    ns = parser.parse_args(args)

    pose2tf_node = Pose2TfNode(ns)
    rclpy.spin(pose2tf_node)
    pose2tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
