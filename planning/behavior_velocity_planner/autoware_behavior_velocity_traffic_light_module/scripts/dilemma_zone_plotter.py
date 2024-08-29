#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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
import matplotlib.pyplot as plt
import numpy as np
import yaml


def get_params_from_yaml():
    autoware_launch_package_path = get_package_share_directory("autoware_launch")

    # get parameters from traffic light
    traffic_light_yaml_file_path = os.path.join(
        autoware_launch_package_path,
        "config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light.param.yaml",
    )
    with open(traffic_light_yaml_file_path, "r") as yaml_file:
        params = yaml.safe_load(yaml_file)
        yellow_lamp_period = params["/**"]["ros__parameters"]["traffic_light"]["yellow_lamp_period"]

    # get parameters from behavior velocity planner
    behavior_vel_yaml_file_path = os.path.join(
        autoware_launch_package_path,
        "config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/behavior_velocity_planner.param.yaml",
    )
    with open(behavior_vel_yaml_file_path, "r") as yaml_file:
        params = yaml.safe_load(yaml_file)
        max_accel = params["/**"]["ros__parameters"]["max_accel"]
        delay_response_time = params["/**"]["ros__parameters"]["delay_response_time"]

    return yellow_lamp_period, max_accel, delay_response_time


def plot(max_ego_vel, params):
    yellow_lamp_period, max_accel, delay_response_time = params

    # create data to plot
    ego_vel = np.linspace(0, round(max_ego_vel), 100)
    moving_distance_during_yellow_light = yellow_lamp_period * ego_vel
    stop_distance = delay_response_time * ego_vel - ego_vel**2 / (2 * max_accel)

    # plot
    fig, ax = plt.subplots()
    ax.plot(moving_distance_during_yellow_light, ego_vel, color="blue")
    ax.plot(stop_distance, ego_vel, color="red")

    # label for the plot
    ax.set_xlabel("moving distance [m]")
    ax.set_ylabel("ego velocity [m/s]")

    # colorize the regions
    min_distance = np.minimum(moving_distance_during_yellow_light, stop_distance)
    max_distance = np.maximum(moving_distance_during_yellow_light, stop_distance)
    ax.fill_betweenx(
        ego_vel,
        stop_distance,
        moving_distance_during_yellow_light,
        where=(moving_distance_during_yellow_light >= stop_distance),
        facecolor="lightgreen",
        interpolate=True,
        alpha=0.3,
    )
    ax.fill_betweenx(
        ego_vel,
        stop_distance,
        moving_distance_during_yellow_light,
        where=(moving_distance_during_yellow_light < stop_distance),
        facecolor="orange",
        interpolate=True,
        alpha=0.3,
    )
    ax.fill_betweenx(ego_vel, min_distance, facecolor="yellow", interpolate=True, alpha=0.1)
    ax.fill_between(max_distance, ego_vel, facecolor="red", interpolate=True, alpha=0.05)

    # init dot lines for the cursor
    hline = ax.axhline(y=0, color="gray", linestyle="--")
    vline = ax.axvline(x=0, color="gray", linestyle="--")

    # capture the cursor
    def on_mouse_move(event):
        if event.inaxes:
            hline.set_ydata(event.ydata)
            vline.set_xdata(event.xdata)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect("motion_notify_event", on_mouse_move)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--vel", default=16.7, type=float, help="maximum ego velocity [m/s]")
    args = parser.parse_args()

    params = get_params_from_yaml()

    max_ego_vel = args.vel
    plot(max_ego_vel, params)
