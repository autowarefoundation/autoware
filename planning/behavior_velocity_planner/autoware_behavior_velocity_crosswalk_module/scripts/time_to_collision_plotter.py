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
from collections import deque
from copy import deepcopy

from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import StringStamped
import yaml


def expand_margin(x_vec, y_vec, min_x=0.0, max_x=10.0):
    expanded_x_vec = deepcopy(x_vec)
    expanded_y_vec = deepcopy(y_vec)

    if min_x < expanded_x_vec[0]:
        expanded_x_vec = [min_x] + expanded_x_vec
        min_y = expanded_y_vec[0] + min_x - expanded_x_vec[0]
        expanded_y_vec = [min_y] + expanded_y_vec

    if expanded_x_vec[-1] < max_x:
        expanded_x_vec.append(max_x)
        max_y = expanded_y_vec[-1] + max_x - expanded_x_vec[-2]
        expanded_y_vec.append(max_y)

    return expanded_x_vec, expanded_y_vec


class CollisionInfo:
    def __init__(self, module_id, obj_id, ttc, ttv, state):
        self.module_id = module_id
        self.obj_id = obj_id
        self.ttc = ttc
        self.ttv = ttv
        self.state = state


class TimeToCollisionPlotter(Node):
    def __init__(self, args):
        super().__init__("time_to_collision_plotter")

        self.depth = args.depth

        self.sub_calculation_cost = self.create_subscription(
            StringStamped,
            "/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/collision_info",
            self.on_collision_info,
            1,
        )

        self.time_to_collision_hist = {}
        self.max_hist_size = args.depth

        crosswalk_config = (
            get_package_share_directory("autoware_launch")
            + "/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/crosswalk.param.yaml"
        )
        with open(crosswalk_config) as f:
            crosswalk_config_obj = yaml.safe_load(f)["/**"]["ros__parameters"]

            ego_pass_first_x = crosswalk_config_obj["crosswalk"]["pass_judge"][
                "ego_pass_first_margin_x"
            ]
            ego_pass_first_y = crosswalk_config_obj["crosswalk"]["pass_judge"][
                "ego_pass_first_margin_y"
            ]
            self.ego_pass_first_margin = [ego_pass_first_x, ego_pass_first_y]

            ego_pass_later_x = crosswalk_config_obj["crosswalk"]["pass_judge"][
                "ego_pass_later_margin_x"
            ]
            ego_pass_later_y = crosswalk_config_obj["crosswalk"]["pass_judge"][
                "ego_pass_later_margin_y"
            ]
            self.ego_pass_later_margin = [ego_pass_later_x, ego_pass_later_y]

        plt.ion()
        _, self.ax = plt.subplots()
        plt.show()
        plt.connect("key_press_event", self.on_key)

        self.timer = self.create_timer(0.3, self.on_timer)

    def on_key(self, event):
        if event.key == "c":
            self.time_to_collision_hist = {}

    def on_collision_info(self, msg):
        collision_info_data_vec = []
        collision_info_str_vec = msg.data[:-1].split(",")
        if len(collision_info_str_vec) < 5:
            return

        for i in range(int(len(collision_info_str_vec) / 5)):
            collision_info_data_vec.append(
                CollisionInfo(*collision_info_str_vec[i * 5 : i * 5 + 5])
            )

        # memorize data in the history dictionary
        for collision_info_data in collision_info_data_vec:
            uuid = collision_info_data.module_id + collision_info_data.obj_id
            if uuid not in self.time_to_collision_hist:
                self.time_to_collision_hist[uuid] = deque([])

            self.time_to_collision_hist[uuid].append(
                [float(collision_info_data.ttv), float(collision_info_data.ttc)]
            )

    def on_timer(self):
        # make the size of history not exceed the maximum value
        for uuid in self.time_to_collision_hist:
            while self.max_hist_size < len(self.time_to_collision_hist[uuid]):
                self.time_to_collision_hist[uuid].popleft()

        plt.cla()

        # plot "ego_pass_later" border
        ego_pass_later_x = []
        ego_pass_later_y = []
        for i in range(len(self.ego_pass_later_margin[0])):
            ego_pass_later_x.append(self.ego_pass_later_margin[0][i])
            ego_pass_later_y.append(self.ego_pass_later_margin[1][i])
        expanded_ego_pass_later_x, expanded_ego_pass_later_y = expand_margin(
            ego_pass_later_x, ego_pass_later_y
        )
        plt.fill_between(
            expanded_ego_pass_later_x,
            expanded_ego_pass_later_y,
            [expanded_ego_pass_later_y[-1] for i in expanded_ego_pass_later_y],
            alpha=0.2,
            color="green",
        )

        # plot "ego_pass_first" border
        ego_pass_first_x = []
        ego_pass_first_y = []
        for i in range(len(self.ego_pass_first_margin[0])):
            ego_pass_first_x.append(self.ego_pass_first_margin[0][i])
            ego_pass_first_y.append(self.ego_pass_first_margin[1][i])
        expanded_ego_pass_first_x, expanded_ego_pass_first_y = expand_margin(
            ego_pass_first_x, ego_pass_first_y
        )
        plt.fill_between(
            expanded_ego_pass_first_y,
            [expanded_ego_pass_first_x[0] for i in expanded_ego_pass_first_x],
            expanded_ego_pass_first_x,
            alpha=0.2,
            color="blue",
        )
        plt.text(
            expanded_ego_pass_later_x[0],
            expanded_ego_pass_later_y[-1],
            "Ego passes later.",
            va="top",
            ha="left",
            color="green",
        )
        plt.text(
            expanded_ego_pass_first_y[-1],
            expanded_ego_pass_first_x[0],
            "Ego passes first.",
            va="bottom",
            ha="right",
            color="blue",
        )
        plt.text(0, 0, "Ego yields.", va="bottom", ha="left", color="red")

        # plot history
        for uuid in self.time_to_collision_hist:
            hist = self.time_to_collision_hist[uuid]
            plt.plot(
                [val[0] for val in hist],
                [val[1] for val in hist],
                label=uuid[0:4] + "-" + uuid[4:8],
            )

        plt.legend()
        plt.draw()
        plt.title("Time to collision")
        plt.xlabel("Pedestrian's time to collision")
        plt.ylabel("Ego's time to collision")
        plt.pause(0.01)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--depth",
        type=float,
        default=100,
    )
    args = parser.parse_args()

    rclpy.init()
    node = TimeToCollisionPlotter(args)
    rclpy.spin(node)
