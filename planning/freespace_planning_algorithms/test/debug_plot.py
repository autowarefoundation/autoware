#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2021 Tier IV, Inc.
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

import csv
from math import cos
from math import sin

from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
import numpy as np


class CarModel(object):
    def __init__(self, length=5.5, width=2.75, base2back=1.5):
        self.length = length
        self.width = width
        self.base2back = base2back

    def _get_four_points(self):
        back = -1.0 * self.base2back
        front = self.length - self.base2back
        right = -0.5 * self.width
        left = 0.5 * self.width
        P = np.array([[back, left], [back, right], [front, right], [front, left]])
        return P

    def get_four_points(self, pos, yaw=0.0):
        R_mat = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
        pos = np.array(pos)
        P_ = self._get_four_points()
        P = P_.dot(R_mat.T) + pos[None, :]
        return P


def create_costmap_msg():
    costmap_msg = OccupancyGrid()

    origin = Pose()
    origin.orientation.w = 1.0

    info = MapMetaData()
    info.width = 150
    info.height = 150
    info.resolution = 0.2
    info.origin = origin
    costmap_msg.info = info

    data = np.zeros((info.height, info.width), dtype=int)
    data[:, :10] = 100
    data[:, -10:] = 100
    data[:10, :] = 100
    data[-10:, :] = 100
    costmap_msg.data = data.flatten().tolist()
    return costmap_msg


class Plotter(object):
    def __init__(self, msg):
        info = msg.info
        n_grid = np.array([info.width, info.height])
        res = info.resolution
        origin = info.origin

        tmp = np.array(msg.data).reshape((n_grid[1], n_grid[0]))  # about to be transposed!!

        self.arr = tmp  # [IMPORTANT] !!
        self.b_min = np.array([origin.position.x, origin.position.y])
        self.b_max = self.b_min + n_grid * res
        self.n_grid = n_grid
        self.origin = origin

    def plot(self, pose_start=None, pose_goal=None, pose_seq=None, plan_duration=None):
        fig, ax = plt.subplots()
        x_lin, y_lin = [np.linspace(self.b_min[i], self.b_max[i], self.n_grid[i]) for i in range(2)]
        X, Y = np.meshgrid(x_lin, y_lin)
        ax.contourf(X, Y, self.arr, cmap="Greys")

        car = CarModel()

        def plot_pose(pose, color, lw=1.0):
            pos_xy = pose[:2]
            yaw = pose[2]
            P = car.get_four_points(pos_xy, yaw=yaw)
            ax.scatter(pos_xy[0], pos_xy[1], c=color, s=2)
            for idx_pair in [[0, 1], [1, 2], [2, 3], [3, 0]]:
                i, j = idx_pair
                ax.plot([P[i, 0], P[j, 0]], [P[i, 1], P[j, 1]], color=color, linewidth=lw)

        if pose_start:
            plot_pose(pose_start, "green")
        if pose_goal:
            plot_pose(pose_goal, "red")

        if pose_seq is not None:
            for pose in pose_seq:
                plot_pose(pose, "blue", lw=0.5)

        ax.axis("equal")
        ax.text(4, 25, "elapsed : {0} [sec]".format(plan_duration * 1e-3), fontsize=12)


if __name__ == "__main__":
    for postfix in ["single", "multi"]:
        for idx in range(4):
            pose_list = []
            file_base = "/tmp/result_{0}{1}".format(postfix, idx)
            with open(file_base + ".txt", "r") as f:
                reader = csv.reader(f)
                plan_duration = float(next(reader)[0])
                pose_start = [float(e) for e in next(reader)]
                pose_goal = [float(e) for e in next(reader)]
                for row in reader:
                    pose_list.append([float(e) for e in row])

            costmap_msg = create_costmap_msg()
            plotter = Plotter(costmap_msg)
            plotter.plot(pose_start, pose_goal, pose_list, plan_duration)
            plt.savefig(file_base + ".png")
