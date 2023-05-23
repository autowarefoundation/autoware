#!/usr/bin/env python3

# Copyright 2023 Tier IV, Inc.
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

from control_performance_analysis.msg import DrivingMonitorStamped
from control_performance_analysis.msg import ErrorStamped
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import BoolStamped

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--interval", help="interval distance to plot")
parser.add_argument("-r", "--realtime_plot", default=True, help="Enable real-time plotting")


class PlotterNode(Node):
    def __init__(self, realtime_plot):
        super().__init__("plotter_node")

        self.realtime_plot = realtime_plot

        self.subscription_error = self.create_subscription(
            ErrorStamped, "/control_performance/performance_vars", self.error_callback, 10
        )
        self.subscription_driving = self.create_subscription(
            DrivingMonitorStamped, "driving_topic", self.driving_callback, 10
        )
        self.subscription_odometry = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odometry_callback, 10
        )
        self.subscription_plot = self.create_subscription(
            BoolStamped, "/make_plot", self.make_plot_callback, 10
        )
        self.curvature = None
        self.velocity = None
        self.lateral_error = None
        self.pose_distance_threshold = (
            0.2  # Define the pose distance threshold to trigger plot update
        )
        self.previous_pos = None

        self.abs_curvature_arr = []
        self.abs_velocity_arr = []
        self.abs_lateral_error_arr = []

        self.fig, self.ax = plt.subplots(3, 3, figsize=(12, 9))
        self.velocities = [
            (0.0, 3.0),
            (3.0, 6.0),
            (6.0, 9.0),
            (9.0, 12.0),
            (12.0, 15.0),
            (15.0, 18.0),
            (18.0, 21.0),
            (21.0, 24.0),
            (24.0, float("inf")),
        ]
        plt.pause(0.1)

    def error_callback(self, msg):
        print("error_callback!")
        self.lateral_error = msg.error.lateral_error
        self.curvature = msg.error.curvature_estimate

    def driving_callback(self, msg):
        print("driving_callback!")

    def make_plot_callback(self, msg):
        if msg.data is True:
            self.update_plot()

    def odometry_callback(self, msg):
        print("odometry_callback!")
        current_pos = msg.pose.pose.position
        self.velocity = msg.twist.twist.linear.x

        if self.previous_pos is None:
            self.previous_pos = current_pos
            return

        if self.curvature is None:
            print("waiting curvature")
            return
        if self.velocity is None:
            print("waiting velocity")
            return
        if self.lateral_error is None:
            print("waiting lateral_error")
            return

        pose_distance = self.calculate_distance(self.previous_pos, current_pos)
        print("pose_distance = ", pose_distance)
        if pose_distance >= self.pose_distance_threshold:
            print("update!")
            self.abs_curvature_arr.append(abs(self.curvature))
            self.abs_lateral_error_arr.append(abs(self.lateral_error))
            self.abs_velocity_arr.append(abs(self.velocity))
            if self.realtime_plot is True:
                self.update_plot()
            self.previous_pos = current_pos

    def calculate_distance(self, pos1, pos2):
        distance = math.sqrt((pos2.x - pos1.x) ** 2 + (pos2.y - pos1.y) ** 2)
        return distance

    # def update_plot(self):
    #     if self.curvature is not None and self.velocity is not None and self.lateral_error is not None:
    #         self.curvature_arr.append(self.curvature)
    #         self.lateral_error_arr.append(self.lateral_error)
    #         self.velocity_arr.append(self.velocity)
    #         self.ax.cla()  # Clear the existing plot
    #         self.ax.scatter(self.curvature_arr, self.velocity_arr, self.lateral_error_arr)
    #         plt.pause(0.001)
    #     else:
    #         print("waiting data in update_data")

    def update_plot(self):
        for i, (vel_min, vel_max) in enumerate(self.velocities):
            indices = [j for j, v in enumerate(self.abs_velocity_arr) if vel_min <= v <= vel_max]

            if len(indices) > 0:
                ax = self.ax[i // 3, i % 3]
                ax.cla()
                ax.scatter(
                    [self.abs_curvature_arr[j] for j in indices],
                    [self.abs_lateral_error_arr[j] for j in indices],
                )
                ax.set_xlabel("Curvature")
                ax.set_ylabel("Lateral Error")
                ax.set_title(f"Velocity: {vel_min} - {vel_max}")

        plt.tight_layout()
        plt.pause(0.1)  # Pause to allow the plot to update


def main(args=None):
    rclpy.init()

    args = parser.parse_args(args)
    realtime_plot = args.realtime_plot

    plotter_node = PlotterNode(realtime_plot)
    print("spin start!")
    rclpy.spin(plotter_node)
    print("spin end!")
    plotter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
