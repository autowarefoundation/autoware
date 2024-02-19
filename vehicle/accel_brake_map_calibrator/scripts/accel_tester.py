#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2022 Tier IV, Inc. All rights reserved.
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

import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import Float32Stamped

MAX_ACCEL = 1.0  # [-]
MIN_ACCEL = 0.0  # [-]


class AccelTester(Node):
    def __init__(self):
        super().__init__("vehicle_accel_tester")
        self.pub = self.create_publisher(Float32Stamped, "/vehicle/tester/accel", 1)

    def run(self):
        while rclpy.ok():
            value = float(input(f"target accel [{MIN_ACCEL} ~ {MAX_ACCEL} -] > "))
            if value > MAX_ACCEL:
                print("input value is larger than max accel!" + f"input: {value} max: {MAX_ACCEL}")
                value = MAX_ACCEL
            elif value < MIN_ACCEL:
                print("input value is smaller than min accel!" + f"input: {value} min: {MIN_ACCEL}")
                value = MIN_ACCEL

            msg = Float32Stamped(stamp=self.get_clock().now().to_msg(), data=value)

            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    accel_tester = AccelTester()
    accel_tester.run()

    accel_tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
