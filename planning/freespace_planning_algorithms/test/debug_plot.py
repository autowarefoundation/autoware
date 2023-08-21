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

import argparse
from dataclasses import dataclass
from math import asin
from math import atan2
from math import cos
from math import sin
import os
import re
import subprocess
from typing import Dict
from typing import Tuple

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float64


@dataclass
class ProblemDescription:
    costmap: OccupancyGrid
    start: Pose
    goal: Pose
    trajectory: PoseArray
    vehicle_length: Float64
    vehicle_width: Float64
    vehicle_base2back: Float64
    elapsed_time: Float64

    @classmethod
    def from_rosbag_path(cls, path: str) -> "ProblemDescription":
        # ref: rosbag2/rosbag2_py/test/test_sequential_reader.py
        storage_options, converter_options = cls.get_rosbag_options(bag_path)
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        topic_types = reader.get_all_topics_and_types()

        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
        message_map = {}

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            message_map[topic] = msg

        return cls(**message_map)

    @staticmethod
    def get_rosbag_options(path: str, serialization_format="cdr"):
        # copied from rosbag2/rosbag2_py/test/test_sequential_reader.py
        storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format,
        )

        return storage_options, converter_options


@dataclass
class VehicleModel:
    length: float
    width: float
    base2back: float

    @classmethod
    def from_problem_description(cls, pd: ProblemDescription) -> "VehicleModel":
        return cls(pd.vehicle_length.data, pd.vehicle_width.data, pd.vehicle_base2back.data)

    # cspell: ignore nparr
    # nparr means "numpy array" (maybe)
    def get_vertices(self, pose: Pose) -> np.ndarray:
        x, y, yaw = self.pose_msg_to_nparr(pose)

        back = -1.0 * self.base2back
        front = self.length - self.base2back
        right = -0.5 * self.width
        left = 0.5 * self.width
        vertices_local = np.array([[back, left], [back, right], [front, right], [front, left]])

        R_mat = np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])
        vertices_global = vertices_local.dot(R_mat.T) + np.array([x, y])
        return vertices_global

    def plot_pose(self, pose: Pose, ax, color="black", lw=1):
        x = pose.position.x
        y = pose.position.y
        V = self.get_vertices(pose)
        ax.scatter(x, y, c=color, s=2)
        for idx_pair in [[0, 1], [1, 2], [2, 3], [3, 0]]:
            i, j = idx_pair
            ax.plot([V[i, 0], V[j, 0]], [V[i, 1], V[j, 1]], color=color, linewidth=lw)

    @staticmethod
    def euler_from_quaternion(quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sin_roll_cos_pitch = 2 * (w * x + y * z)
        cos_roll_cos_pitch = 1 - 2 * (x * x + y * y)
        roll = atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

        sin_pitch = 2 * (w * y - z * x)
        pitch = asin(sin_pitch)

        sin_yaw_cos_pitch = 2 * (w * z + x * y)
        cos_yaw_cos_pitch = 1 - 2 * (y * y + z * z)
        yaw = atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
        return roll, pitch, yaw

    @staticmethod
    def pose_msg_to_nparr(pose_msg: Pose) -> Tuple[float, float, float]:
        _, _, yaw = VehicleModel.euler_from_quaternion(pose_msg.orientation)
        return pose_msg.position.x, pose_msg.position.y, yaw


def plot_problem(pd: ProblemDescription, ax, meta_info):
    info = pd.costmap.info
    n_grid = np.array([info.width, info.height])
    res = info.resolution
    origin = info.origin
    arr = np.array(pd.costmap.data).reshape((n_grid[1], n_grid[0]))
    b_min = np.array([origin.position.x, origin.position.y])
    b_max = b_min + n_grid * res

    x_lin, y_lin = [np.linspace(b_min[i], b_max[i], n_grid[i]) for i in range(2)]
    X, Y = np.meshgrid(x_lin, y_lin)
    ax.contourf(X, Y, arr, cmap="Greys")

    vehicle_model = VehicleModel.from_problem_description(pd)
    vehicle_model.plot_pose(pd.start, ax, "green")
    vehicle_model.plot_pose(pd.goal, ax, "red")

    for pose in pd.trajectory.poses:
        vehicle_model.plot_pose(pose, ax, "blue", 0.5)

    text = "elapsed : {0} [msec]".format(int(round(pd.elapsed_time.data)))
    ax.text(0.3, 0.3, text, fontsize=15, color="red")

    ax.text(0.3, b_max[1] - 1.5, meta_info, fontsize=15, color="red")

    ax.axis("equal")
    ax.set_xlim([b_min[0], b_max[0]])
    ax.set_ylim([b_min[1], b_max[1]])


def create_concat_png(src_list, dest, is_horizontal):
    opt = "+append" if is_horizontal else "-append"
    cmd = ["convert", opt]
    for src in src_list:
        cmd.append(src)
    cmd.append(dest)
    subprocess.Popen(cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--concat", action="store_true", help="concat png images (requires imagemagick)"
    )
    args = parser.parse_args()
    concat = args.concat

    dir_name_table: Dict[Tuple[str, int], str] = {}
    # cspell: ignore fpalgos, cand
    prefix = "fpalgos"
    for cand_dir in os.listdir("/tmp"):
        if cand_dir.startswith(prefix):
            m = re.match(r"{}-(\w+)-case([0-9])".format(prefix), cand_dir)
            assert m is not None
            algo_name = m.group(1)
            case_number = int(m.group(2))
            dir_name_table[(algo_name, case_number)] = cand_dir

    algo_names = sorted({key[0] for key in dir_name_table.keys()})
    case_indices = sorted({key[1] for key in dir_name_table.keys()})
    n_algo = len(algo_names)
    n_case = len(case_indices)

    for i in range(n_algo):
        algo_name = algo_names[i]
        algo_png_images = []
        for j in range(n_case):
            fig, ax = plt.subplots()

            result_dir = dir_name_table[(algo_name, j)]
            bag_path = os.path.join("/tmp", result_dir)

            pd = ProblemDescription.from_rosbag_path(bag_path)

            meta_info = "{}-case{}".format(algo_name, j)
            plot_problem(pd, ax, meta_info)
            fig.tight_layout()

            file_name = os.path.join("/tmp", "plot-{}.png".format(meta_info))
            algo_png_images.append(file_name)
            plt.savefig(file_name)
            print("saved to {}".format(file_name))

        algo_summary_file = os.path.join("/tmp", "summary-{}.png".format(algo_name))
        if concat:
            create_concat_png(algo_png_images, algo_summary_file, True)
