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
from dataclasses import dataclass
from itertools import cycle
import math
from threading import Lock
import time

import imageio
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import Float64MultiArrayStamped

matplotlib.use("TKAgg")


@dataclass
class NPC:
    x: float
    y: float
    th: float
    width: float
    height: float
    speed: float
    dangerous: bool
    ref_object_enter_time: float
    ref_object_exit_time: float
    collision_start_time: float
    collision_start_dist: float
    collision_end_time: float
    collision_end_dist: float
    pred_x: list[float]
    pred_y: list[float]

    def __init__(self, data: list[float]):
        self.x = data[0]
        self.y = data[1]
        self.th = data[2]
        self.width = data[3]
        self.height = data[4]
        self.speed = data[5]
        self.dangerous = bool(int(data[6]))
        self.ref_object_enter_time = data[7]
        self.ref_object_exit_time = data[8]
        self.collision_start_time = data[9]
        self.collision_start_dist = data[10]
        self.collision_end_time = data[11]
        self.collision_end_dist = data[12]
        self.first_collision_x = data[13]
        self.first_collision_y = data[14]
        self.last_collision_x = data[15]
        self.last_collision_y = data[16]
        self.pred_x = data[17:58:2]
        self.pred_y = data[18:58:2]


class TTCVisualizer(Node):
    def __init__(self, args):
        super().__init__("ttc_visualizer")
        self.ttc_dist_pub = self.create_subscription(
            Float64MultiArrayStamped,
            "/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/intersection/ego_ttc",
            self.on_ego_ttc,
            1,
        )
        self.ttc_time_pub = self.create_subscription(
            Float64MultiArrayStamped,
            "/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/intersection/object_ttc",
            self.on_object_ttc,
            1,
        )
        self.args = args
        self.lane_id = args.lane_id
        self.ego_ttc_data = None
        self.object_ttc_data = None
        self.npc_vehicles = []
        self.images = []
        self.last_sub = time.time()

        self.plot_timer = self.create_timer(0.2, self.on_plot_timer)
        self.fig = plt.figure(figsize=(13, 6))
        self.ttc_ax = self.fig.add_subplot(1, 2, 1)
        self.ttc_vel_ax = self.ttc_ax.twinx()
        self.world_ax = self.fig.add_subplot(1, 2, 2)
        self.lock = Lock()
        self.color_list = [
            "#e41a1c",
            "#377eb8",
            "#4daf4a",
            "#984ea3",
            "#ff7f00",
            "#ffff33",
            "#a65628",
            "#f781bf",
        ]
        plt.ion()
        plt.show(block=False)

    def plot_ttc(self):
        self.ttc_ax.cla()
        self.ttc_vel_ax.cla()

        n_ttc_data = int(self.ego_ttc_data.layout.dim[1].size)
        ego_ttc_time = self.ego_ttc_data.data[n_ttc_data : 2 * n_ttc_data]
        ego_ttc_dist = self.ego_ttc_data.data[2 * n_ttc_data : 3 * n_ttc_data]

        self.ttc_ax.grid()
        self.ttc_ax.set_xlabel("ego time")
        self.ttc_ax.set_ylabel("ego dist")
        time_dist_plot = self.ttc_ax.plot(ego_ttc_time, ego_ttc_dist, label="time-dist", c="orange")
        self.ttc_ax.set_xlim(
            min(ego_ttc_time) - 2.0,
            min(max(ego_ttc_time) + 3.0, self.args.max_time),
        )
        # self.ttc_ax.set_ylim(min(ego_ttc_dist) - 2.0, max(ego_ttc_dist) + 3.0)
        for npc, color in zip(self.npc_vehicles, cycle(self.color_list)):
            t0, t1 = npc.collision_start_time, npc.collision_end_time
            d0, d1 = npc.collision_start_dist, npc.collision_end_dist
            self.ttc_ax.fill(
                [t0, t0, t1, t1, 0, 0],
                [d0, 0, 0, d1, d1, d0],
                c=color,
                alpha=0.2,
            )
        dd = [d1 - d0 for d0, d1 in zip(ego_ttc_dist, ego_ttc_dist[1:])]
        dt = [t1 - t0 for t0, t1 in zip(ego_ttc_time, ego_ttc_time[1:])]
        v = [d / t for d, t in zip(dd, dt)]
        self.ttc_vel_ax.yaxis.set_label_position("right")
        self.ttc_vel_ax.set_ylabel("ego velocity")
        # self.ttc_vel_ax.set_ylim(0.0, max(v) + 1.0)
        time_velocity_plot = self.ttc_vel_ax.plot(ego_ttc_time[1:], v, label="time-v", c="red")
        lines = time_dist_plot + time_velocity_plot
        labels = [line.get_label() for line in lines]
        self.ttc_ax.legend(lines, labels, loc="upper left")

    def plot_world(self):
        detect_range = self.args.range
        self.world_ax.cla()
        n_ttc_data = int(self.ego_ttc_data.layout.dim[1].size)
        ego_path_x = self.ego_ttc_data.data[3 * n_ttc_data : 4 * n_ttc_data]
        ego_path_y = self.ego_ttc_data.data[4 * n_ttc_data : 5 * n_ttc_data]
        self.world_ax.set_aspect("equal")
        self.world_ax.scatter(ego_path_x[0], ego_path_y[0], marker="x", c="red", s=15)
        min_x, max_x = min(ego_path_x), max(ego_path_x)
        min_y, max_y = min(ego_path_y), max(ego_path_y)
        x_dir = 1 if ego_path_x[-1] > ego_path_x[0] else -1
        y_dir = 1 if ego_path_y[-1] > ego_path_y[0] else -1
        min_x = min_x - detect_range if x_dir == 1 else min_x - 20
        max_x = max_x + detect_range if x_dir == -1 else max_x + 20
        min_y = min_y - detect_range if y_dir == 1 else min_y - 20
        max_y = max_y + detect_range if y_dir == -1 else max_y + 20
        self.world_ax.set_xlim(min_x, max_x)
        self.world_ax.set_ylim(min_y, max_y)
        arrows = [
            (x0, y0, math.atan2(x1 - x0, y1 - y0))
            for (x0, y0, x1, y1) in zip(
                ego_path_x[0:-1:5],
                ego_path_y[0:-1:5],
                ego_path_x[4:-1:5],
                ego_path_y[4:-1:5],
            )
        ]
        for x, y, th in arrows:
            self.world_ax.arrow(
                x,
                y,
                math.sin(th) * 0.5,
                math.cos(th) * 0.5,
                head_width=0.1,
                head_length=0.1,
            )

        for npc, color in zip(self.npc_vehicles, cycle(self.color_list)):
            x, y, th, w, h = npc.x, npc.y, npc.th, npc.width, npc.height
            bbox = np.array(
                [
                    [-w / 2, -h / 2],
                    [+w / 2, -h / 2],
                    [+w / 2, +h / 2],
                    [-w / 2, +h / 2],
                    [-w / 2, -h / 2],
                ]
            ).transpose()
            Rth = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            bbox_rot = Rth @ bbox
            self.world_ax.fill(bbox_rot[0, :] + x, bbox_rot[1, :] + y, color, alpha=0.5)
            self.world_ax.plot(
                [npc.first_collision_x, npc.last_collision_x],
                [npc.first_collision_y, npc.last_collision_y],
                c=color,
                linewidth=3.0,
            )
            if npc.dangerous:
                self.world_ax.plot(npc.pred_x, npc.pred_y, c=color, linewidth=1.5)
            else:
                self.world_ax.plot(npc.pred_x, npc.pred_y, c=color, linestyle="--")

        self.world_ax.plot(ego_path_x, ego_path_y, c="k", linestyle="--")

    def cleanup(self):
        if self.args.save:
            kwargs_write = {"fps": self.args.fps, "quantizer": "nq"}
            imageio.mimsave("./" + self.args.gif + ".gif", self.images, **kwargs_write)
        rclpy.shutdown()

    def on_plot_timer(self):
        with self.lock:
            if (not self.ego_ttc_data) or (not self.object_ttc_data):
                return

            if not self.last_sub:
                return

            now = time.time()
            if (now - self.last_sub) > 1.0:
                print("elapsed more than 1sec from last sub, exit/save fig")
                self.cleanup()

            self.plot_ttc()
            self.plot_world()
            self.fig.canvas.flush_events()

            if self.args.save:
                image = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype="uint8")
                image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
                self.images.append(image)

    def on_ego_ttc(self, msg):
        with self.lock:
            if int(msg.data[0]) == self.lane_id:
                self.ego_ttc_data = msg
                self.last_sub = time.time()

    def parse_npc_vehicles(self):
        self.npc_vehicles = []
        n_npc_vehicles = int(self.object_ttc_data.layout.dim[0].size)
        npc_data_size = int(self.object_ttc_data.layout.dim[1].size)
        for i in range(1, n_npc_vehicles):
            data = self.object_ttc_data.data[i * npc_data_size : (i + 1) * npc_data_size]
            self.npc_vehicles.append(NPC(data))

    def on_object_ttc(self, msg):
        with self.lock:
            if int(msg.data[0]) == self.lane_id:
                self.object_ttc_data = msg
                self.parse_npc_vehicles()
                self.last_sub = time.time()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--lane_id",
        type=int,
        required=True,
        help="lane_id to analyze",
    )
    parser.add_argument(
        "--range",
        type=float,
        default=60,
        help="detect range for drawing",
    )
    parser.add_argument("--max_time", type=float, default=100, help="max plot limit for time")
    parser.add_argument("-s", "--save", action="store_true", help="flag to save gif")
    parser.add_argument("--gif", type=str, default="ttc", help="filename of gif file")
    parser.add_argument("--fps", type=float, default=5, help="fps of gif")
    args = parser.parse_args()

    rclpy.init()
    visualizer = TTCVisualizer(args)
    rclpy.spin(visualizer)
