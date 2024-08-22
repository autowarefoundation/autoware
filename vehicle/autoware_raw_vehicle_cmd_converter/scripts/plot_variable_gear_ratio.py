#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc. All rights reserved.
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

import matplotlib.pyplot as plt
import numpy as np


def calculate_variable_gear_ratio(vel, steer_wheel, a, b, c):
    return max(1e-5, a + b * vel * vel - c * abs(steer_wheel))


def plot_vgr_vs_steer_wheel(ax, velocity, a, b, c):
    steer_wheel_values = np.linspace(-10.0, 10.0, 100)
    vgr_values = [
        calculate_variable_gear_ratio(velocity, steer_wheel, a, b, c)
        for steer_wheel in steer_wheel_values
    ]

    ax.plot(
        steer_wheel_values,
        vgr_values,
        linewidth=2,
    )
    ax.set_xlabel("Steer Wheel [rad]", fontsize=14)
    ax.set_ylabel("Variable Gear Ratio", fontsize=14)
    ax.set_title(
        f"VGR vs Steer Wheel\n(Velocity = {velocity} m/s, a = {a}, b = {b}, c = {c})",
        fontsize=16,
        fontweight="bold",
    )
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.tick_params(axis="both", which="major", labelsize=12)


def plot_vgr_vs_velocity(ax, steer_wheel, a, b, c):
    velocity_values = np.linspace(0, 16.66, 100)
    vgr_values = [
        calculate_variable_gear_ratio(velocity, steer_wheel, a, b, c)
        for velocity in velocity_values
    ]

    ax.plot(velocity_values, vgr_values, linewidth=2)
    ax.set_xlabel("Velocity [m/s]", fontsize=14)
    ax.set_ylabel("Variable Gear Ratio", fontsize=14)
    ax.set_title(
        f"VGR vs Velocity\n(Steer Wheel = {steer_wheel} rad, a = {a}, b = {b}, c = {c})",
        fontsize=16,
        fontweight="bold",
    )
    ax.grid(True, linestyle="--", alpha=0.7)
    ax.tick_params(axis="both", which="major", labelsize=12)


def main():
    parser = argparse.ArgumentParser(
        description="Plot Variable Gear Ratio (VGR) based on velocity and steer wheel angle."
    )
    parser.add_argument("--vgr_coef_a", type=float, default=15.713, help="Coefficient a for VGR")
    parser.add_argument("--vgr_coef_b", type=float, default=0.053, help="Coefficient b for VGR")
    parser.add_argument("--vgr_coef_c", type=float, default=0.042, help="Coefficient c for VGR")
    parser.add_argument(
        "--velocity",
        type=float,
        default=8.33,
        help="Fixed velocity value for plotting VGR vs Steer Wheel",
    )
    parser.add_argument(
        "--steer_wheel",
        type=float,
        default=0.0,
        help="Fixed steer wheel value for plotting VGR vs Velocity",
    )

    args = parser.parse_args()

    plt.style.use("seaborn-whitegrid")

    fig, axs = plt.subplots(1, 2, figsize=(16, 8))

    plot_vgr_vs_steer_wheel(
        axs[0], args.velocity, args.vgr_coef_a, args.vgr_coef_b, args.vgr_coef_c
    )
    plot_vgr_vs_velocity(
        axs[1], args.steer_wheel, args.vgr_coef_a, args.vgr_coef_b, args.vgr_coef_c
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
