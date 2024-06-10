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

import argparse
import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from tier4_external_api_msgs.srv import GetAccelBrakeMapCalibrationData as CalibData


class CalibrationDataRelay(Node):
    def __init__(self, args):
        super().__init__("plot_client")
        self.cli = self.create_client(
            CalibData, "/api/external/get/accel_brake_map_calibrator/data"
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.request = CalibData.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.request)


def main(args=None):
    rclpy.init(args=None)
    client = CalibrationDataRelay(args)
    client.send_request()
    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                save_dir = "./test_data_save"
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                svg_name = save_dir + "/graph.svg"

                f_svg = open(svg_name, "w")
                svg_byte = bytes(response.graph_image)
                text = svg_byte.decode()
                f_svg.write(text)

                print("svg done")

                acc_map_name = save_dir + "/accel_map.csv"
                f_acc = open(acc_map_name, "w")
                f_acc.write(response.accel_map)

                print("accel map done")

                brk_map_name = save_dir + "/brake_map.csv"
                f_brk = open(brk_map_name, "w")
                f_brk.write(response.brake_map)

                print("brake map done")

            except Exception as e:
                client.get_logger().info("Service call failed %r" % (e,))

            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    package_path = get_package_share_directory("autoware_accel_brake_map_calibrator")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d", "--default-map-dir", default=None, type=str, help="directory of default map"
    )
    parser.add_argument(
        "-c", "--calibrated-map-dir", default=None, type=str, help="directory of calibrated map"
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
