#! /usr/bin/env python3

# Copyright 2022 Tier IV, Inc.
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

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from calc_utils import CalcUtils
import config as CF
from csv_reader import CSVReader
import matplotlib.pyplot as plt
import numpy as np
from plotter import Plotter
import rclpy
from rclpy.node import Node
from tier4_external_api_msgs.srv import GetAccelBrakeMapCalibrationData as CalibData
import yaml


class DrawGraph(Node):
    calibrated_map_dir = ""

    def __init__(self):
        super().__init__("plot_server")

        self.srv = self.create_service(
            CalibData, "/accel_brake_map_calibrator/get_data_service", self.get_data_callback
        )

        default_map_path = get_package_share_directory("autoware_raw_vehicle_cmd_converter")
        self.declare_parameter(
            "/accel_brake_map_calibrator/csv_default_map_dir", default_map_path + "/data/default/"
        )
        self.default_map_dir = (
            self.get_parameter("/accel_brake_map_calibrator/csv_default_map_dir")
            .get_parameter_value()
            .string_value
        )

        package_path = get_package_share_directory("autoware_accel_brake_map_calibrator")
        self.declare_parameter(
            "/accel_brake_map_calibrator/csv_calibrated_map_dir", package_path + "/config/"
        )
        self.calibrated_map_dir = (
            self.get_parameter("/accel_brake_map_calibrator/csv_calibrated_map_dir")
            .get_parameter_value()
            .string_value
        )

        self.declare_parameter("calibration_method", "each_cell")
        self.calibration_method = (
            self.get_parameter("calibration_method").get_parameter_value().string_value
        )
        if self.calibration_method is None:
            self.calibration_method = "each_cell"
        elif not (
            (self.calibration_method == "each_cell") | (self.calibration_method == "four_cell")
        ):
            print("invalid method.")
            self.calibration_method = "each_cell"

        self.log_file = package_path + "/config/log.csv"

        config_file = package_path + "/config/accel_brake_map_calibrator.param.yaml"
        if Path(config_file).exists():
            self.get_logger().info("config file exists")
            with open(config_file) as yml:
                data = yaml.safe_load(yml)
            self.min_vel_thr = data["/**"]["ros__parameters"]["velocity_min_threshold"]
            self.vel_diff_thr = data["/**"]["ros__parameters"]["velocity_diff_threshold"]
            self.pedal_diff_thr = data["/**"]["ros__parameters"]["pedal_diff_threshold"]
            self.max_steer_thr = data["/**"]["ros__parameters"]["max_steer_threshold"]
            self.max_pitch_thr = data["/**"]["ros__parameters"]["max_pitch_threshold"]
            self.max_jerk_thr = data["/**"]["ros__parameters"]["max_jerk_threshold"]
        else:
            self.get_logger().warning("config file is not found in {}".format(config_file))
            self.min_vel_thr = 0.1
            self.vel_diff_thr = 0.556
            self.pedal_diff_thr = 0.03
            self.max_steer_thr = 0.2
            self.max_pitch_thr = 0.02
            self.max_jerk_thr = 0.7

        self.max_pedal_vel_thr = 0.7

        # debug
        self.get_logger().info("default map dir: {}".format(self.default_map_dir))
        self.get_logger().info("calibrated map dir: {}".format(self.calibrated_map_dir))
        self.get_logger().info("calibrated method: {}".format(self.calibration_method))
        self.get_logger().info("log file :{}".format(self.log_file))
        self.get_logger().info("min_vel_thr : {}".format(self.min_vel_thr))
        self.get_logger().info("vel_diff_thr : {}".format(self.vel_diff_thr))
        self.get_logger().info("pedal_diff_thr : {}".format(self.pedal_diff_thr))
        self.get_logger().info("max_steer_thr : {}".format(self.max_steer_thr))
        self.get_logger().info("max_pitch_thr : {}".format(self.max_pitch_thr))
        self.get_logger().info("max_jerk_thr : {}".format(self.max_jerk_thr))
        self.get_logger().info("max_pedal_vel_thr : {}".format(self.max_pedal_vel_thr))

    def get_data_callback(self, request, response):
        # read csv
        # If log file doesn't exist, return empty data
        if not Path(self.log_file).exists():
            response.graph_image = []
            self.get_logger().info("svg data is empty")

            response.accel_map = ""
            self.get_logger().info("accel map is empty")

            response.brake_map = ""
            self.get_logger().info("brake map is empty")

            return response

        self.cr = CSVReader(self.log_file, csv_type="file")

        # remove unused_data
        self.csv_data = self.cr.removeUnusedData(
            self.min_vel_thr,
            self.max_steer_thr,
            self.max_pitch_thr,
            self.max_pedal_vel_thr,
            self.max_jerk_thr,
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
            self.vel_diff_thr,
            CF.PEDAL_LIST,
            self.pedal_diff_thr,
            self.calibration_method,
        )

        count_map, average_map, stddev_map = CalcUtils.create_stat_map(data)
        velocity_map_list = []
        for i in range(len(CF.VEL_LIST)):
            velocity_map_list.append(CalcUtils.extract_x_index_map(full_data, i))

        default_pedal_list, default_acc_list = self.load_map(self.default_map_dir)
        if len(default_pedal_list) == 0 or len(default_acc_list) == 0:
            self.get_logger().warning(
                "No default map file was found in {}".format(self.default_map_dir)
            )

            response.graph_image = []
            self.get_logger().info("svg data is empty")

            response.accel_map = ""
            self.get_logger().info("accel map is empty")

            response.brake_map = ""
            self.get_logger().info("brake map is empty")

            return response

        calibrated_pedal_list, calibrated_acc_list = self.load_map(self.calibrated_map_dir)
        if len(calibrated_pedal_list) == 0 or len(calibrated_acc_list) == 0:
            self.get_logger().warning(
                "No calibrated map file was found in {}".format(self.calibrated_map_dir)
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
            )
        plt.savefig("plot.svg")
        self.get_logger().info("svg saved")

        # pack response data
        text = Path("plot.svg").read_text()
        if text == "":
            response.graph_image = []
            self.get_logger().info("svg data is empty")
        else:
            byte = text.encode()
            for b in byte:
                response.graph_image.append(b)
            self.get_logger().info("svg data is packed")

        accel_map_name = Path(self.calibrated_map_dir + "accel_map.csv")
        if accel_map_name.exists():
            with open(self.calibrated_map_dir + "accel_map.csv", "r") as calibrated_accel_map:
                for accel_data in calibrated_accel_map:
                    response.accel_map += accel_data
            self.get_logger().info("accel map is packed")
        else:
            response.accel_map = ""
            self.get_logger().info("accel map is empty")

        brake_map_name = Path(self.calibrated_map_dir + "brake_map.csv")
        if brake_map_name.exists():
            with open(self.calibrated_map_dir + "brake_map.csv", "r") as calibrated_brake_map:
                for brake_data in calibrated_brake_map:
                    response.brake_map += brake_data
            self.get_logger().info("brake map is packed")
        else:
            response.brake_map = ""
            self.get_logger().info("brake map is empty")

        return response

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
    ):
        fig = plotter.subplot_more(subplot_num)

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
    node = DrawGraph()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
