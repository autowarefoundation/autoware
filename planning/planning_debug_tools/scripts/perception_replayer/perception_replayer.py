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
import functools
import sys

from PyQt5.QtWidgets import QApplication
from geometry_msgs.msg import PoseWithCovarianceStamped
from perception_replayer_common import PerceptionReplayerCommon
import rclpy
from time_manager_widget import TimeManagerWidget
from utils import create_empty_pointcloud
from utils import translate_objects_coordinate


class PerceptionReplayer(PerceptionReplayerCommon):
    def __init__(self, args):
        super().__init__(args, "perception_replayer")

        self.bag_timestamp = self.rosbag_objects_data[0][0]
        self.is_pause = False
        self.rate = 1.0
        self.prev_traffic_signals_msg = None

        # initialize widget
        self.widget = TimeManagerWidget(
            self.rosbag_objects_data[0][0], self.rosbag_objects_data[-1][0]
        )
        self.widget.show()
        self.widget.button.clicked.connect(self.onPushed)
        for button in self.widget.rate_button:
            button.clicked.connect(functools.partial(self.onSetRate, button))
        self.widget.pub_recorded_ego_pose_button.clicked.connect(self.publish_recorded_ego_pose)

        # start timer callback
        self.delta_time = 0.1
        self.timer = self.create_timer(self.delta_time, self.on_timer)
        print("Start timer callback")

    def on_timer(self):
        timestamp = self.get_clock().now().to_msg()

        self.kill_online_perception_node()

        if self.args.detected_object:
            pointcloud_msg = create_empty_pointcloud(timestamp)
            self.pointcloud_pub.publish(pointcloud_msg)

        # step timestamp
        # get timestamp from slider
        self.bag_timestamp = self.rosbag_objects_data[0][
            0
        ] + self.widget.slider.value() / 1000000 * (
            self.rosbag_objects_data[-1][0] - self.rosbag_objects_data[0][0]
        )
        if not self.is_pause:
            self.bag_timestamp += self.rate * self.delta_time * 1e9  # seconds to timestamp
        # update slider value from the updated timestamp
        self.widget.slider.setValue(self.widget.timestamp_to_value(self.bag_timestamp))

        # extract message by the timestamp
        msgs = copy.deepcopy(self.find_topics_by_timestamp(self.bag_timestamp))
        objects_msg = msgs[0]
        traffic_signals_msg = msgs[1]

        # objects
        if objects_msg:
            objects_msg.header.stamp = timestamp
            if self.args.detected_object:
                if not self.ego_pose:
                    print("No ego pose found.")
                    return

                ego_odom = self.find_ego_odom_by_timestamp(self.bag_timestamp)
                if not ego_odom:
                    return
                log_ego_pose = ego_odom.pose.pose

                translate_objects_coordinate(self.ego_pose, log_ego_pose, objects_msg)
            self.objects_pub.publish(objects_msg)

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

    def onPushed(self, event):
        if self.widget.button.isChecked():
            self.is_pause = True
        else:
            self.is_pause = False

    def onSetRate(self, button):
        self.rate = float(button.text())

    def publish_recorded_ego_pose(self):
        ego_odom = self.find_ego_odom_by_timestamp(self.bag_timestamp)
        if not ego_odom:
            return

        recorded_ego_pose = PoseWithCovarianceStamped()
        recorded_ego_pose.header.stamp = self.get_clock().now().to_msg()
        recorded_ego_pose.header.frame_id = "map"
        recorded_ego_pose.pose.pose = ego_odom.pose.pose
        recorded_ego_pose.pose.covariance = [
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.25,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.06853892326654787,
        ]

        self.recorded_ego_pub_as_initialpose.publish(recorded_ego_pose)
        print("Published recorded ego pose as /initialpose")


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
    args = parser.parse_args()

    app = QApplication(sys.argv)

    rclpy.init()
    node = PerceptionReplayer(args)

    try:
        while True:
            app.processEvents()
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
