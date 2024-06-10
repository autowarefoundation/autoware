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
import sys

from calc_utils import CalcUtils
import config as CF
from csv_reader import CSVReader
import numpy as np
from plotter import Plotter

# parameter for statistics
SCATTER_GRAPH_PEDAL_LIM = [-0.55, 0.55]
SCATTER_GRAPH_ACCEL_LIM = [-1.8, 1.8]
CONSIDER_PEDAL_VAL_FOR_STAT = [0.0, 0.1, 0.2]

parser = argparse.ArgumentParser()
parser.add_argument("--csv-file-dir", help="log file directory")
parser.add_argument("--min-vel-thr", type=float, default=0.1, help="thresh of minimum velocity")
parser.add_argument(
    "--max-pedal-vel-thr", type=float, default=0.15, help="thresh of maximum pedal velocity"
)
parser.add_argument("--max-steer-thr", type=float, default=0.2, help="thresh of maximum steer")
parser.add_argument("--max-pitch-thr", type=float, default=0.02, help="thresh of maximum pitch")
parser.add_argument("--max-jerk-thr", type=float, default=0.7, help="thresh of maximum jerk")
parser.add_argument(
    "--pedal-diff-thr",
    type=float,
    default=0.03,
    help="thresh of max delta-pedal to add statistics map",
)
parser.add_argument(
    "--vel-diff-thr",
    type=float,
    default=0.556,
    help="thresh of max delta-velocity to add statistics map",
)
parser.add_argument(
    "--view-velocity-1", type=int, default=5, help="velocity(kmh) to visualize by plot (1)"
)
parser.add_argument(
    "--view-velocity-2", type=int, default=20, help="velocity(kmh) to visualize by plot (2)"
)
parser.add_argument(
    "--view-velocity-3", type=int, default=30, help="velocity(kmh) to visualize by plot (3)"
)
parser.add_argument("--output-stat-csv", default="stat.csv", help="output csv file name")
parser.add_argument("--output-plot-file", default="result.png", help="output plot file name")
parser.add_argument("--no-plot", action="store_true")

args = parser.parse_args()
# get path
csv_file_dir = args.csv_file_dir
min_vel_thr = args.min_vel_thr
max_pedal_vel_thr = args.max_pedal_vel_thr
max_steer_thr = args.max_steer_thr
max_pitch_thr = args.max_pitch_thr
max_jerk_thr = args.max_jerk_thr
pedal_diff_thr = args.pedal_diff_thr
vel_diff_thr = args.vel_diff_thr
view_velocity_list = []
view_velocity_list.append(args.view_velocity_1)
view_velocity_list.append(args.view_velocity_2)
view_velocity_list.append(args.view_velocity_3)
output_stat_csv = args.output_stat_csv
output_plot_file = args.output_plot_file
remove_by_invalid_pedal = True
remove_by_vel = True
remove_by_steer = True
remove_by_pitch = True
remove_by_pedal_speed = True
remove_by_jerk = True
is_plot = not args.no_plot

view_velocity_idx_list = []
for vv in view_velocity_list:
    if not (vv in CF.VEL_LIST):
        print("invalid view_velocity. velocity list is: ", CF.VEL_LIST)
        sys.exit()
    view_velocity_idx_list.append(CF.VEL_LIST.tolist().index(vv))

# search index of pedal to use for statistics
stat_pedal_list = []
for val in CONSIDER_PEDAL_VAL_FOR_STAT:
    if not (val in CF.PEDAL_LIST):
        print("invalid CONSIDER_PEDAL_VAL_FOR_STAT. pedal list is:", CF.PEDAL_LIST)
    stat_pedal_list.append(CF.PEDAL_LIST.tolist().index(val))

# read file
cr = CSVReader(csv_file_dir)

# remove unused_data
csv_data = cr.removeUnusedData(
    min_vel_thr,
    max_steer_thr,
    max_pitch_thr,
    max_pedal_vel_thr,
    max_jerk_thr,
    remove_by_invalid_pedal,
    remove_by_vel,
    remove_by_steer,
    remove_by_pitch,
    remove_by_pedal_speed,
    remove_by_jerk,
)


# get statistics array
vel_data = cr.getVelData()
pedal_data = cr.getPedalData()
acc_data = cr.getAccData()

# get color factor (pitch) array for plotting
color_data = cr.getPitchData()

data, full_data = CalcUtils.create_2d_map(
    vel_data,
    pedal_data,
    acc_data,
    color_data,
    CF.VEL_LIST / 3.6,
    vel_diff_thr,
    CF.PEDAL_LIST,
    pedal_diff_thr,
)

count_map, average_map, stddev_map = CalcUtils.create_stat_map(data)
velocity_map_list = []
for v_idx in view_velocity_idx_list:
    velocity_map_list.append(CalcUtils.extract_x_index_map(full_data, v_idx))

# output statistics
f = open(output_stat_csv, "a")
count_list_over_v = []
stddev_list_over_v = []
for vi in view_velocity_idx_list:
    count_list = []
    stddev_list = []
    for pi in stat_pedal_list:
        count_list.append(count_map[pi][vi])
        if count_map[pi][vi] > 5:
            stddev_list.append(stddev_map[pi][vi])
        else:
            print("Warning: count is fewer than 5.")
    count_sum = int(np.sum(count_list))
    stddev_ave = np.average(stddev_list)
    count_list_over_v.append(count_sum)
    stddev_list_over_v.append(stddev_ave)
    f.write(str(count_sum) + ",")
    f.write(str(stddev_ave) + ",")
    print("velocity index:", vi)
    print("\tcount: ", count_sum)
    print("\tstddev: ", stddev_ave)
count_sum_over_v = int(np.sum(count_list_over_v))
stddev_ave_over_v = np.average(stddev_list_over_v)
f.write(str(count_sum_over_v) + ",")
f.write(str(stddev_ave_over_v) + "\n")
f.close()
print("full data:")
print("\tcount: ", count_sum_over_v)
print("\tstddev: ", stddev_ave_over_v)

# visualization
plotter = Plotter(2, 3)
plotter.subplot(1)
plotter.imshow(
    average_map, CF.VEL_MIN, CF.VEL_MAX, CF.VEL_SPAN, CF.PEDAL_MIN, CF.PEDAL_MAX, CF.PEDAL_SPAN
)
plotter.add_label("average of accel", "velocity(kmh)", "throttle")

plotter.subplot(2)
plotter.imshow(
    stddev_map, CF.VEL_MIN, CF.VEL_MAX, CF.VEL_SPAN, CF.PEDAL_MIN, CF.PEDAL_MAX, CF.PEDAL_SPAN
)
plotter.add_label("std. dev. of accel", "velocity(kmh)", "throttle")

plotter.subplot(3)
plotter.imshow(
    count_map,
    CF.VEL_MIN,
    CF.VEL_MAX,
    CF.VEL_SPAN,
    CF.PEDAL_MIN,
    CF.PEDAL_MAX,
    CF.PEDAL_SPAN,
    num_data_type="int",
)
plotter.add_label("number of accel data", "velocity(kmh)", "throttle")

# view pedal-accel graph


def view_pedal_accel_graph(subplot_num, vel_list_idx):
    # plot all data
    fig = plotter.subplot(subplot_num)
    plotter.scatter(
        velocity_map_list[vel_list_idx][:, 1],
        velocity_map_list[vel_list_idx][:, 2],
        "blue",
        label="all",
    )

    # plot average data
    # remove average score of no data
    average_map_v_avoid_0 = (
        average_map[:, view_velocity_idx_list[vel_list_idx]]
        + np.where(average_map[:, view_velocity_idx_list[vel_list_idx]] == 0, True, False) * 1e03
    )

    plotter.scatter(CF.PEDAL_LIST, average_map_v_avoid_0, "red", label="average")

    plotter.set_lim(fig, SCATTER_GRAPH_PEDAL_LIM, SCATTER_GRAPH_ACCEL_LIM)
    plotter.add_label(
        str(view_velocity_list[vel_list_idx]) + "kmh; pedal-accel relation", "pedal", "accel"
    )


view_pedal_accel_graph(4, 0)
view_pedal_accel_graph(5, 1)
view_pedal_accel_graph(6, 2)

if is_plot:
    plotter.show()
plotter.save(output_plot_file)


# pedal-pitch plot
"""
cr = CSVReader(csv_file_dir)
csv_data = cr.removeUnusedData(
    min_vel_thr, max_steer_thr, max_pitch_thr,
    max_pedal_vel_thr, max_jerk_thr,
    remove_by_invalid_pedal,
    remove_by_vel=True,
    remove_by_steer=False,
    remove_by_pitch=False,
    remove_by_pedal_speed=False,
    remove_by_jerk=False)
pitch = csv_data[CF.PITCH]
pedal = csv_data[CF.A_PED] - csv_data[CF.B_PED]
velocity = csv_data[CF.VEL] * 3.6  # color
plotter.scatter_color(pedal, pitch, velocity, (0.0, 35.0), "hsv", label=None)
plotter.add_label("pedal-pitch relation", "pedal", "pitch")
plotter.show()
"""

"""
# pedal-speed-accel plot
cr = CSVReader(csv_file_dir)
csv_data = cr.removeUnusedData(
    min_vel_thr, max_steer_thr, max_pitch_thr,
    max_pedal_vel_thr, max_jerk_thr,
    remove_by_invalid_pedal,
    remove_by_vel=True,
    remove_by_steer=True,
    remove_by_pitch=True,
    remove_by_pedal_speed=False,
    remove_by_jerk=False)

csv_data = csv_data.reset_index(drop=True)
delay = 3  # accel delay (*100ms)
for i in range(delay, len(csv_data))[::-1]:
    pedal = csv_data[CF.A_PED][i-delay] - csv_data[CF.B_PED][i-delay]
    # extract data of pedal = 0.1
    tgt_ped = 0.1
    if pedal > tgt_ped + pedal_diff_thr or \
            pedal < tgt_ped - pedal_diff_thr:
        csv_data = csv_data.drop(i)
        continue

    velocity = csv_data[CF.VEL][i]
    if velocity > 12.0 / 3.6 or \
            velocity < 8.0 / 3.6:
        csv_data = csv_data.drop(i)
        continue

pedal_speed = csv_data[CF.A_PED_SPD] - csv_data[CF.B_PED_SPD]
accel = csv_data[CF.ACC]
velocity = csv_data[CF.VEL] * 3.6
plotter = Plotter(1, 1)
fig = plotter.subplot(1)
plotter.scatter_color(pedal_speed, accel, velocity,
                      (0.0, 35.0), "hsv", label=None)
plotter.add_label("pedal-speed-accel relation (only pedal = 0.1)",
                  "pedal-speed", "accel")
plotter.set_lim(fig, [-0.4, 0.4], [-0.5, 1.0])
plotter.show()
"""
