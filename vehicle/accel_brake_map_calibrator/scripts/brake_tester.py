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

MAX_BRAKE = 1.0  # [-]
MIN_BRAKE = 0.0  # [-]


class BrakeTester(Node):
    def __init__(self):
        super().__init__("vehicle_brake_tester")
        self.pub = self.create_publisher(Float32Stamped, "/vehicle/tester/brake", 1)

    def run(self):
        while rclpy.ok():
            value = float(
                input("target brake [" + str(MIN_BRAKE) + " ~ " + str(MAX_BRAKE) + "] > ")
            )

            if value > MAX_BRAKE:
                print("input value is larger than max brake!" + f"input: {value} max: {MAX_BRAKE}")
                value = MAX_BRAKE
            elif value < MIN_BRAKE:
                print("input value is smaller than min brake!" + f"input: {value} min: {MIN_BRAKE}")
                value = MIN_BRAKE

            msg = Float32Stamped(stamp=self.get_clock().now().to_msg(), data=value)

            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    brake_tester = BrakeTester()
    brake_tester.run()

    brake_tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
