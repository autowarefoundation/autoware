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
import numpy as np
from plotter import Plotter
import rclpy
from rclpy.node import Node


class DelayEstimator(Node):
    def __init__(self):
        # get parameter
        super().__init__("delay_estimator")
        package_path = get_package_share_directory("autoware_accel_brake_map_calibrator")
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
        self.declare_parameter("update_hz", 10.0)
        update_hz = self.get_parameter("update_hz").get_parameter_value().double_value

        self.data_span = 1.0 / update_hz

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

        # plot all data
        # value to use for statistics
        PEDAL_VALUE = 0.1
        VEL_VALUE_LIST = np.array([10, 15, 20]) / 3.6
        plotter = Plotter(3, 2)
        max_delay_step = 5
        for delay_step in range(max_delay_step + 1):
            print("data processing... " + str(delay_step) + " / " + str(max_delay_step))
            csv_data = self.cr.extractPedalRangeWithDelay(delay_step, PEDAL_VALUE, pedal_diff_thr)

            # get correlation coefficient
            # extract data of velocity is VEL_VALUE
            coef_list = []
            for vel_value in VEL_VALUE_LIST:
                ex_csv_data = csv_data[csv_data[CF.VEL] < vel_value + vel_diff_thr]
                ex_csv_data = csv_data[csv_data[CF.VEL] > vel_value - vel_diff_thr]
                pedal_speed = ex_csv_data[CF.A_PED_SPD] - ex_csv_data[CF.B_PED_SPD]
                accel = ex_csv_data[CF.ACC]
                coef = self.getCorCoef(pedal_speed, accel)
                coef_list.append(coef)

            print("delay_step: ", delay_step)
            print("coef: ", coef_list)

            ave_coef = np.average(coef_list)
            self.plotPedalSpeedAndAccel(csv_data, plotter, delay_step + 1, delay_step, ave_coef)
        plotter.show()

    def getCorCoef(self, a, b):
        coef = np.corrcoef(np.array(a), np.array(b))
        return coef[0, 1]

    def plotPedalSpeedAndAccel(self, csv_data, plotter, subplot_num, delay_step, coef):
        pedal_speed = csv_data[CF.A_PED_SPD] - csv_data[CF.B_PED_SPD]
        accel = csv_data[CF.ACC]
        velocity = csv_data[CF.VEL] * 3.6
        fig = plotter.subplot(subplot_num)
        plotter.scatter_color(pedal_speed, accel, velocity, "hsv", label=None)
        delay_time_ms = delay_step * self.data_span * 1000
        plotter.add_label(
            "pedal-spd-acc (delay = " + str(delay_time_ms) + " ms), R = " + str(coef),
            "pedal-speed",
            "accel",
        )
        plotter.set_lim(fig, [-0.4, 0.4], [-1.0, 1.0])


def main(args=None):
    rclpy.init(args=args)
    node = DelayEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
