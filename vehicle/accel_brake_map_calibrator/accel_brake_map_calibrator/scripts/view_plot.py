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

import argparse
import math

from ament_index_python.packages import get_package_share_directory
from calc_utils import CalcUtils
import config as CF
from csv_reader import CSVReader
import numpy as np
from plotter import Plotter
import rclpy
from rclpy.node import Node


class ViewPlot(Node):
    def __init__(self, args):
        super().__init__("plot_viewer")
        default_map_dir = args.default_map_dir
        calibrated_map_dir = args.calibrated_map_dir
        calibration_method = args.method
        scatter_only = args.scatter_only
        log_file = args.log_file
        min_vel_thr = args.min_vel_thr
        vel_diff_thr = args.vel_diff_thr
        pedal_diff_thr = args.pedal_diff_thr
        max_steer_thr = args.max_steer_thr
        max_pitch_thr = args.max_pitch_thr
        max_jerk_thr = args.max_jerk_thr
        max_pedal_vel_thr = args.max_pedal_vel_thr

        if default_map_dir is None:
            package_path = get_package_share_directory("raw_vehicle_cmd_converter")
            self.declare_parameter(
                "/accel_brake_map_calibrator/csv_default_map_dir", package_path + "/data/default/"
            )
            default_map_dir = (
                self.get_parameter("/accel_brake_map_calibrator/csv_default_map_dir")
                .get_parameter_value()
                .string_value
            )

        if calibrated_map_dir is None:
            package_path = get_package_share_directory("accel_brake_map_calibrator")
            self.declare_parameter(
                "/accel_brake_map_calibrator/csv_calibrated_map_dir", package_path + "/config/"
            )
            calibrated_map_dir = (
                self.get_parameter("/accel_brake_map_calibrator/csv_calibrated_map_dir")
                .get_parameter_value()
                .string_value
            )

        if calibration_method is None:
            calibration_method = "each_cell"
        elif not ((calibration_method == "each_cell") | (calibration_method == "four_cell")):
            print("invalid method.")
            calibration_method = "each_cell"

        print("default map dir: {}".format(default_map_dir))
        print("calibrated map dir: {}".format(calibrated_map_dir))
        print("calibration method: {}".format(calibration_method))

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
            calibration_method,
        )

        count_map, average_map, stddev_map = CalcUtils.create_stat_map(data)
        velocity_map_list = []
        for i in range(len(CF.VEL_LIST)):
            velocity_map_list.append(CalcUtils.extract_x_index_map(full_data, i))

        default_pedal_list, default_acc_list = self.load_map(default_map_dir)
        if len(default_pedal_list) == 0 or len(default_acc_list) == 0:
            self.get_logger().warning("No default map file was found in {}".format(default_map_dir))

        calibrated_pedal_list, calibrated_acc_list = self.load_map(calibrated_map_dir)
        if len(calibrated_pedal_list) == 0 or len(calibrated_acc_list) == 0:
            self.get_logger().warning(
                "No calibrated map file was found in {}".format(calibrated_map_dir)
            )

        # visualize point from data
        plot_width = 3
        plot_height = int(math.ceil(len(CF.VEL_LIST) / float(plot_width)))
        plotter = Plotter(plot_height, plot_width)
        for i in range(len(CF.VEL_LIST)):
            self.view_pedal_accel_graph(
                plotter,
                i,
                velocity_map_list,
                i,
                count_map,
                average_map,
                stddev_map,
                default_pedal_list,
                default_acc_list,
                calibrated_pedal_list,
                calibrated_acc_list,
                scatter_only,
            )
        plotter.show()

    def plotter_function(self):
        return self.plotter

    def view_pedal_accel_graph(
        self,
        plotter,
        subplot_num,
        velocity_map_list,
        vel_list_idx,
        count_map,
        average_map,
        stddev_map,
        default_pedal_list,
        default_acc_list,
        calibrated_pedal_list,
        calibrated_acc_list,
        scatter_only,
    ):

        fig = plotter.subplot_more(subplot_num)

        if not scatter_only:
            # calibrated map
            if len(calibrated_pedal_list) != 0 and len(calibrated_acc_list) != 0:
                plotter.plot(
                    calibrated_pedal_list[vel_list_idx],
                    calibrated_acc_list[vel_list_idx],
                    color="blue",
                    label="calibrated",
                )

            # default map
            if len(default_pedal_list) != 0 and len(default_acc_list) != 0:
                plotter.plot(
                    default_pedal_list[vel_list_idx],
                    default_acc_list[vel_list_idx],
                    color="orange",
                    label="default",
                    linestyle="dashed",
                )

        # plot all data
        pedal_list = [0 for i in range(len(CF.PEDAL_LIST))]
        if velocity_map_list[vel_list_idx] is not None:
            plotter.scatter_color(
                velocity_map_list[vel_list_idx][:, 1],
                velocity_map_list[vel_list_idx][:, 2],
                color=velocity_map_list[vel_list_idx][:, 3],
                label="all",
            )

            for pedal in velocity_map_list[vel_list_idx][:, 1]:
                min_pedal = 10
                for pedal_idx, ref_pedal in enumerate(CF.PEDAL_LIST):
                    if min_pedal > abs(pedal - ref_pedal):
                        min_pedal = abs(pedal - ref_pedal)
                        min_pedal_idx = pedal_idx
                pedal_list[min_pedal_idx] += 1

        # plot average data
        plotter.scatter(CF.PEDAL_LIST, average_map[:, vel_list_idx], "red", label="average")

        # add label of standard deviation
        plotter.scatter([], [], "black", label="std dev")

        # plot average text
        for i in range(len(CF.PEDAL_LIST)):
            if count_map[i, vel_list_idx] == 0:
                continue
            x = CF.PEDAL_LIST[i]
            y = average_map[i, vel_list_idx]
            y2 = stddev_map[i, vel_list_idx]
            # plot average
            plotter.plot_text(x, y + 1, y, color="red")

            # plot standard deviation
            plotter.plot_text(x, y - 1, y2, color="black")

            # plot the number of all data
            plotter.plot_text(
                x, y - 2, "{}\npts".format(pedal_list[i]), num_data_type="str", color="green"
            )

        pedal_lim = [CF.PEDAL_LIST[0] - 0.05, CF.PEDAL_LIST[-1] + 0.05]
        accel_lim = [-5.0, 5.0]

        plotter.set_lim(fig, pedal_lim, accel_lim)
        plotter.add_label(
            str(CF.VEL_LIST[vel_list_idx]) + "kmh; pedal-accel relation", "pedal", "accel"
        )

    def load_map(self, csv_dir):
        try:
            accel_pedal_list = []
            accel_acc_list = []
            with open(csv_dir + "accel_map.csv") as f:
                for l_idx, l in enumerate(f.readlines()):
                    w = l.split(",")
                    w[-1] = w[-1][:-1]
                    if l_idx != 0:
                        accel_pedal_list.append([float(w[0]) for e in w[1:]])
                        accel_acc_list.append([float(e) for e in w[1:]])

            brake_pedal_list = []
            brake_acc_list = []
            with open(csv_dir + "brake_map.csv") as f:
                for l_idx, l in enumerate(f.readlines()):
                    w = l.split(",")
                    w[-1] = w[-1][:-1]
                    if l_idx != 0:
                        brake_pedal_list.append([-float(w[0]) for e in w[1:]])
                        brake_acc_list.append([float(e) for e in w[1:]])

            return np.hstack(
                [np.fliplr(np.array(accel_pedal_list).T), np.array(brake_pedal_list).T.tolist()]
            ), np.hstack([np.fliplr(np.array(accel_acc_list).T), np.array(brake_acc_list).T])
        except OSError as e:
            print(e)
            return [], []


def main(args=None):
    rclpy.init(args=None)
    node = ViewPlot(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    package_path = get_package_share_directory("accel_brake_map_calibrator")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d", "--default-map-dir", default=None, type=str, help="directory of default map"
    )
    parser.add_argument(
        "-c", "--calibrated-map-dir", default=None, type=str, help="directory of calibrated map"
    )
    parser.add_argument(
        "-m",
        "--method",
        default="None",
        type=str,
        help="calibration method : each_cell or four_cell",
    )
    parser.add_argument("-s", "--scatter-only", action="store_true", help="show only scatters")
    parser.add_argument(
        "-l",
        "--log-file",
        default=package_path + "/config/log.csv",
        type=str,
        help="path of log.csv",
    )
    parser.add_argument(
        "--min-vel-thr", default=0.1, type=float, help="valid min velocity threshold"
    )
    parser.add_argument(
        "--vel-diff-thr", default=0.556, type=float, help="valid velocity diff threshold"
    )
    parser.add_argument(
        "--pedal-diff-thr", default=0.03, type=float, help="valid pedal diff threshold"
    )
    parser.add_argument(
        "--max-steer-thr", default=0.2, type=float, help="valid max steer threshold"
    )
    parser.add_argument(
        "--max-pitch-thr", default=0.02, type=float, help="valid max pitch threshold"
    )
    parser.add_argument("--max-jerk-thr", default=0.7, type=float, help="valid max jerk threshold")
    parser.add_argument(
        "--max-pedal-vel-thr", default=0.7, type=float, help="valid max pedal velocity threshold"
    )

    args = parser.parse_args()
    main(args)
