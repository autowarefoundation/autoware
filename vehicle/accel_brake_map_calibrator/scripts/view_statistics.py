#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2021 Tier IV, Inc. All rights reserved.
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

from ament_index_python.packages import get_package_share_directory
from calc_utils import CalcUtils
import config as CF
from csv_reader import CSVReader
from plotter import Plotter
import rclpy
from rclpy.node import Node


class ViewPlot(Node):
    def __init__(self):
        super().__init__("statistics_viewer")
        # get parameter
        package_path = get_package_share_directory("accel_brake_map_calibrator")
        self.declare_parameter("log_file", package_path + "/config/log.csv")
        log_file = self.get_parameter("log_file").get_parameter_value().string_value
        self.declare_parameter("velocity_min_threshold", 0.1)
        min_vel_thr = (
            self.get_parameter("velocity_min_threshold").get_parameter_value().double_value
        )
        self.declare_parameter("velocity_diff_threshold", 0.556)
        vel_diff_thr = (
            self.get_parameter("velocity_diff_threshold").get_parameter_value().double_value
        )
        self.declare_parameter("pedal_diff_threshold", 0.03)
        pedal_diff_thr = (
            self.get_parameter("pedal_diff_threshold").get_parameter_value().double_value
        )
        self.declare_parameter("max_steer_threshold", 0.2)
        max_steer_thr = self.get_parameter("max_steer_threshold").get_parameter_value().double_value
        self.declare_parameter("max_pitch_threshold", 0.02)
        max_pitch_thr = self.get_parameter("max_pitch_threshold").get_parameter_value().double_value
        self.declare_parameter("max_jerk_threshold", 0.7)
        max_jerk_thr = self.get_parameter("max_jerk_threshold").get_parameter_value().double_value
        self.declare_parameter("pedal_velocity_thresh", 0.15)
        max_pedal_vel_thr = (
            self.get_parameter("pedal_velocity_thresh").get_parameter_value().double_value
        )

        # read csv
        self.cr = CSVReader(log_file, csv_type="file")
        # remove unused_data
        self.csv_data = self.cr.removeUnusedData(
            min_vel_thr, max_steer_thr, max_pitch_thr, max_pedal_vel_thr, max_jerk_thr
        )

        # get statistics array
        vel_data = self.cr.getVelData()
        pedal_data = self.cr.getPedalData()
        acc_data = self.cr.getAccData()

        # get color factor (pitch) array for plotting
        color_data = self.cr.getPitchData()

        data, full_data = CalcUtils.create_2d_map(
            vel_data,
            pedal_data,
            acc_data,
            color_data,
            CF.VEL_LIST / 3.6,
            vel_diff_thr,
            CF.PEDAL_LIST,
            pedal_diff_thr,
        )

        count_map, average_map, stddev_map = CalcUtils.create_stat_map(data)

        # visualization
        plotter = Plotter(1, 3)
        plotter.subplot(1)
        plotter.imshow(
            average_map,
            CF.VEL_MIN,
            CF.VEL_MAX,
            CF.VEL_SPAN,
            CF.PEDAL_MIN,
            CF.PEDAL_MAX,
            CF.PEDAL_SPAN,
        )
        plotter.add_label("average of accel", "velocity(kmh)", "throttle")

        plotter.subplot(2)
        plotter.imshow(
            stddev_map,
            CF.VEL_MIN,
            CF.VEL_MAX,
            CF.VEL_SPAN,
            CF.PEDAL_MIN,
            CF.PEDAL_MAX,
            CF.PEDAL_SPAN,
        )
        plotter.add_label("std. dev. of accel", "velocity(kmh)", "throttle")

        plotter.subplot(3)
        plotter.imshow(
            count_map,
            CF.VEL_MIN,
            CF.VEL_MAX,
            CF.VEL_SPAN,
            CF.PEDAL_MIN,
            CF.PEDAL_MAX,
            CF.PEDAL_SPAN,
            num_data_type="int",
        )
        plotter.add_label("number of accel data", "velocity(kmh)", "throttle")
        plotter.show()


def main(args=None):
    rclpy.init(args=args)
    node = ViewPlot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
