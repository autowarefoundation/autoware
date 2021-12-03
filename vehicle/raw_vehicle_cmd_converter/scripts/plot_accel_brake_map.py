#!/usr/bin/env python3

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

from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import numpy as np


def main(dimension, map_dir, accel_or_brake):
    if map_dir is None:
        script_dir = get_package_share_directory("raw_vehicle_cmd_converter")
        csv_dir = script_dir + "/data/default"
    else:
        csv_dir = map_dir

    vel_list = []
    stroke_list = []
    acc_list = []
    try:
        with open(csv_dir + "/{}_map.csv".format(accel_or_brake)) as f:
            for l_idx, l in enumerate(f.readlines()):
                w = l.split(",")
                w[-1] = w[-1][:-1]
                if l_idx == 0:
                    vel_list = [float(e) * 3600 / 1000 for e in w[1:]]
                else:
                    stroke_list.append(float(w[0]))
                    acc_list.append([float(e) for e in w[1:]])

        plt.rcParams["font.size"] = 12
        if plot_dimension == 2:
            plt.title("{} map".format(accel_or_brake))
            plt.xlabel("stroke")
            plt.ylabel("acceleration [m/s^2]")

            for vel_idx, vel in enumerate(vel_list):
                plt.plot(
                    stroke_list, np.array(acc_list).T[vel_idx], label="vel={}[km/h]".format(vel)
                )
            plt.legend()
        else:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")
            ax.set_title("{} map".format(accel_or_brake))
            ax.set_xlabel("stroke")
            ax.set_ylabel("vel [km/h]")
            ax.set_zlabel("acceleration [m/s^2]")

            surf = ax.plot_surface(
                np.array([stroke_list for e in vel_list]).T,
                [vel_list for e in stroke_list],
                np.array(acc_list),
                cmap="bwr",
            )
            fig.colorbar(surf, shrink=0.5, aspect=10)

        plt.show()

    except Exception:  # noqa: B902
        print("No map file found in {}".format(map_dir))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p", "--plot-dimension", default=3, type=int, help="dimension of plotting map"
    )
    parser.add_argument("-a", "--accel", action="store_true", help="show accel map")
    parser.add_argument("-b", "--brake", action="store_true", help="show brake map")
    parser.add_argument("-d", "--dir", default=None, help="map directory")
    args = parser.parse_args()

    plot_dimension = args.plot_dimension
    accel = args.accel
    brake = args.brake
    map_dir = args.dir

    if accel:
        main(plot_dimension, map_dir, "accel")
    elif brake:
        main(plot_dimension, map_dir, "brake")
    else:
        print('Please add argument of "-a" or "-b"')
        parser.print_help()
