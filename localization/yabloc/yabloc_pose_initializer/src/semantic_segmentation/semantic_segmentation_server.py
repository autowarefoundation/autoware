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
import sys

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import semantic_segmentation_core as core
from sensor_msgs.msg import Image
from yabloc_pose_initializer.srv import SemanticSegmentation

# cspell: ignore DDOWNLOAD
ERROR_MESSAGE = """\
The yabloc_pose_initializer is not working correctly because the DNN model has not been downloaded correctly.
To download models, "-DDOWNLOAD_ARTIFACTS=ON" is required at build time.
Please see the README of yabloc_pose_initializer for more information."""


class SemanticSegmentationServer(Node):
    def __init__(self):
        super().__init__("segmentation_server_node")

        model_path = self.declare_parameter("model_path", "").value

        self.get_logger().info("model path: " + model_path)
        self.bridge_ = CvBridge()

        if os.path.exists(model_path):
            self.dnn_ = core.SemanticSegmentationCore(model_path)
        else:
            self.dnn_ = None
            self.__print_error_message()

        self.srv = self.create_service(
            SemanticSegmentation, "semantic_segmentation_srv", self.on_service
        )

    def __print_error_message(self):
        messages = ERROR_MESSAGE.split("\n")
        for message in messages:
            self.get_logger().error(message)

    def on_service(self, request, response):
        if self.dnn_:
            response.dst_image = self.__inference(request.src_image)
            response.success = True
        else:
            self.__print_error_message()
            response.success = False
            response.dst_image = request.src_image

        return response

    def __inference(self, msg: Image):
        stamp = msg.header.stamp
        self.get_logger().info("Subscribed image: " + str(stamp))
        src_image = self.bridge_.imgmsg_to_cv2(msg)

        mask = self.dnn_.inference(src_image)
        dst_msg = self.bridge_.cv2_to_imgmsg(mask)
        dst_msg.encoding = "bgr8"

        return dst_msg


def main():
    rclpy.init(args=sys.argv)

    server_node = SemanticSegmentationServer()
    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
