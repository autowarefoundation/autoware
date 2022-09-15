#!/usr/bin/env python3

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
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import StringStamped


class CalculationCostAnalyzer(Node):
    def __init__(self, args):
        super().__init__("calculation_cost_analyzer")

        self.functions_name = args.functions.split(",")

        self.calculation_cost_hist = []
        self.sub_calculation_cost = self.create_subscription(
            StringStamped,
            "/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/debug/calculation_time",
            self.CallbackCalculationCost,
            1,
        )

        plt.ion()
        plt.figure()

        self.lines = []
        self.y_vec = []
        for f_idx, function_name in enumerate(self.functions_name):
            (line,) = plt.plot([], [], label="{}".format(function_name))
            self.lines.append(line)
            self.y_vec.append(deque())

        plt.legend(loc="lower left")
        plt.show()

    def CallbackCalculationCost(self, msg):
        max_y = 0
        max_x = 0
        for f_idx, function_name in enumerate(self.functions_name):
            for line in msg.data.split("\n"):
                if function_name in line:
                    y = float(line.split(":=")[1].split("[ms]")[0])
                    self.y_vec[f_idx].append(y)

            if len(self.y_vec[f_idx]) > 100:
                self.y_vec[f_idx].popleft()

            x_vec = list(range(len(self.y_vec[f_idx])))

            self.lines[f_idx].set_xdata(x_vec)
            self.lines[f_idx].set_ydata(self.y_vec[f_idx])

            if len(self.y_vec[f_idx]) > 0:
                max_x = max(max_x, max(x_vec))
                max_y = max(max_y, max(self.y_vec[f_idx]))

        plt.xlim(0, max_x)
        plt.ylim(0.0, max_y)

        plt.draw()
        plt.pause(0.01)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--functions", type=str, default="solveOsqp")
    args = parser.parse_args()

    rclpy.init()
    node = CalculationCostAnalyzer(args)
    rclpy.spin(node)
