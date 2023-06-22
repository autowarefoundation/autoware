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

import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import semantic_segmentation_core as core
from sensor_msgs.msg import Image


class SemanticSegmentationNode(Node):
    def __init__(self):
        super().__init__("semantic_segmentation_node")

        model_path = self.declare_parameter("model_path", "").value
        self.imshow_ = self.declare_parameter("imshow", False).value

        self.get_logger().info("model path: " + model_path)

        self.sub_image_ = self.create_subscription(
            Image, "~/input/image_raw", self.imageCallback, 10
        )

        self.pub_overlay_image_ = self.create_publisher(Image, "~/output/overlay_image", 10)
        self.pub_image_ = self.create_publisher(Image, "~/output/semantic_image", 10)

        self.dnn_ = core.SemanticSegmentationCore(model_path)
        self.bridge_ = CvBridge()

    def imageCallback(self, msg: Image):
        stamp = msg.header.stamp
        self.get_logger().info("Subscribed image: " + str(stamp))

        src_image = self.bridge_.imgmsg_to_cv2(msg)
        start_time = time.time()
        mask = self.dnn_.inference(src_image)
        elapsed_time = time.time() - start_time

        show_image = self.dnn_.drawOverlay(src_image, mask)
        cv2.putText(
            show_image,
            "Inference: " + "{:.1f}".format(elapsed_time * 1000) + "ms",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # visualize
        if self.imshow_:
            cv2.imshow("segmentation", show_image)
            cv2.waitKey(1)

        # publish dst image
        self.__publish_image(mask, self.pub_image_)
        self.__publish_image(show_image, self.pub_overlay_image_)

    def __publish_image(self, image, publisher):
        out_msg = self.bridge_.cv2_to_imgmsg(image)
        out_msg.encoding = "bgr8"
        publisher.publish(out_msg)


def main():
    rclpy.init(args=sys.argv)

    segmentation_node = SemanticSegmentationNode()
    rclpy.spin(segmentation_node)
    segmentation_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
