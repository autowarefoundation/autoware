# Copyright 2024 Proxima Technology Inc, TIER IV
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

import datetime
import os

from assets import ControlType  # type: ignore
from autoware_smart_mpc_trajectory_follower.scripts import drive_controller
from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
import data_collection_utils  # type: ignore
from density_estimation import KinematicStates  # type: ignore
from density_estimation import visualize_speed_acc
from density_estimation import visualize_speed_steer
import matplotlib.pyplot as plt  # type: ignore
from numba import njit  # type: ignore
import numpy as np
import pandas as pd  # type: ignore
import pure_pursuit_gain_updater  # type: ignore
import scipy.interpolate  # type: ignore
import simplejson as json  # type: ignore

# cSpell:ignore numba simplejson
print("\n\n### import python_simulator.py ###")

ACTIVATE_LIMITS = False
UPDATE_PP_GAIN = True

ctrl_time_step = drive_functions.ctrl_time_step

sim_dt = 0.003333  # Simulation time step
ctrl_freq = 10  # int, controlled once in 10 simulations.
acc_time_delay = drive_functions.acc_time_delay
steer_time_delay = drive_functions.steer_time_delay
acc_delay_step_sim = round(acc_time_delay / sim_dt)
steer_delay_step_sim = round(steer_time_delay / sim_dt)
acc_time_constant = drive_functions.acc_time_constant
steer_time_constant = drive_functions.steer_time_constant
steer_dead_band = 0.0012
steer_scaling = 1.0
acc_scaling = 1.0
measurement_steer_bias = 0.00
L = drive_functions.L
N = drive_functions.N

if ACTIVATE_LIMITS:
    steer_rate_lim = 0.35
    vel_rate_lim = 7.0
else:
    steer_rate_lim = 10.35
    vel_rate_lim = 107.0

mpc_freq = drive_functions.mpc_freq


use_accel_map = False

vgr_coef_a1 = 15.713
vgr_coef_b1 = 0.053
vgr_coef_c1 = 0.042
vgr_coef_a2 = 15.713
vgr_coef_b2 = 0.053
vgr_coef_c2 = 0.042


perturbed_sim_flag = False
if os.path.isfile("sim_setting.json"):
    with open("sim_setting.json", "r") as file:
        sim_setting_dict = json.load(file)
        print("load sim_setting.json")
        print("sim_setting_dict", sim_setting_dict)
        perturbed_sim_flag = True
else:
    print("sim_setting.json not found")

# Rewriting for para-search
nominal_setting_dict_display = {}
if perturbed_sim_flag:
    if "steer_bias" in sim_setting_dict.keys():
        nominal_setting_dict_display["steer_bias"] = 1 * measurement_steer_bias
        measurement_steer_bias = sim_setting_dict["steer_bias"]
        print("nominal steer_bias", nominal_setting_dict_display["steer_bias"])
        print("perturbed steer_bias", measurement_steer_bias)
        print("perturbed steer_bias_deg", measurement_steer_bias * 180 / np.pi)

    if "steer_rate_lim" in sim_setting_dict.keys():
        nominal_setting_dict_display["steer_rate_lim"] = 1 * steer_rate_lim
        steer_rate_lim = sim_setting_dict["steer_rate_lim"]
        print("nominal steer_rate_lim", nominal_setting_dict_display["steer_rate_lim"])
        print("perturbed steer_rate_lim", steer_rate_lim)

    if "vel_rate_lim" in sim_setting_dict.keys():
        nominal_setting_dict_display["vel_rate_lim"] = 1 * vel_rate_lim
        vel_rate_lim = sim_setting_dict["vel_rate_lim"]
        print("nominal vel_rate_lim", nominal_setting_dict_display["vel_rate_lim"])
        print("perturbed vel_rate_lim", vel_rate_lim)

    if "wheel_base" in sim_setting_dict.keys():
        nominal_setting_dict_display["wheel_base"] = 1 * L
        L = sim_setting_dict["wheel_base"]
        print("nominal wheel_base", nominal_setting_dict_display["wheel_base"])
        print("perturbed wheel_base", L)

    if "steer_dead_band" in sim_setting_dict.keys():
        nominal_setting_dict_display["steer_dead_band"] = 1 * steer_dead_band
        steer_dead_band = sim_setting_dict["steer_dead_band"]
        print("nominal steer_dead_band", nominal_setting_dict_display["steer_dead_band"])
        print("perturbed steer_dead_band", steer_dead_band)

    if "adaptive_gear_ratio_coef" in sim_setting_dict.keys():
        nominal_setting_dict_display["adaptive_gear_ratio_coef"] = [
            1 * vgr_coef_a1,
            1 * vgr_coef_b1,
            1 * vgr_coef_c1,
            1 * vgr_coef_a2,
            1 * vgr_coef_b2,
            1 * vgr_coef_c2,
        ]  # type: ignore
        vgr_coef_a1 = sim_setting_dict["adaptive_gear_ratio_coef"][0]
        vgr_coef_b1 = sim_setting_dict["adaptive_gear_ratio_coef"][1]
        vgr_coef_c1 = sim_setting_dict["adaptive_gear_ratio_coef"][2]
        vgr_coef_a2 = sim_setting_dict["adaptive_gear_ratio_coef"][3]
        vgr_coef_b2 = sim_setting_dict["adaptive_gear_ratio_coef"][4]
        vgr_coef_c2 = sim_setting_dict["adaptive_gear_ratio_coef"][5]
        print(
            "nominal adaptive_gear_ratio_coef",
            nominal_setting_dict_display["adaptive_gear_ratio_coef"],
        )
        print(
            "perturbed: vgr_coef_a1",
            vgr_coef_a1,
            "vgr_coef_b1",
            vgr_coef_b1,
            "vgr_coef_c1",
            vgr_coef_c1,
        )
        print(
            "perturbed: vgr_coef_a2",
            vgr_coef_a2,
            "vgr_coef_b2",
            vgr_coef_b2,
            "vgr_coef_c2",
            vgr_coef_c2,
        )

    if "acc_time_delay" in sim_setting_dict.keys():
        nominal_setting_dict_display["acc_time_delay"] = sim_dt * acc_delay_step_sim
        print("nominal acc_time_delay", nominal_setting_dict_display["acc_time_delay"])
        print("nominal acc_delay_step_sim", acc_delay_step_sim)
        acc_delay_step_sim = round(sim_setting_dict["acc_time_delay"] / sim_dt)
        print("perturbed acc_time_delay", sim_setting_dict["acc_time_delay"])
        print("perturbed acc_delay_step_sim", acc_delay_step_sim)

    if "steer_time_delay" in sim_setting_dict.keys():
        nominal_setting_dict_display["steer_time_delay"] = sim_dt * steer_delay_step_sim
        print("nominal steer_time_delay", nominal_setting_dict_display["steer_time_delay"])
        print("nominal steer_delay_step_sim", steer_delay_step_sim)
        steer_delay_step_sim = round(sim_setting_dict["steer_time_delay"] / sim_dt)
        print("perturbed steer_time_delay", sim_setting_dict["steer_time_delay"])
        print("perturbed steer_delay_step_sim", steer_delay_step_sim)

    if "acc_time_constant" in sim_setting_dict.keys():
        nominal_setting_dict_display["acc_time_constant"] = 1 * acc_time_constant
        acc_time_constant = sim_setting_dict["acc_time_constant"]
        print("nominal acc_time_constant", nominal_setting_dict_display["acc_time_constant"])
        print("perturbed acc_time_constant", acc_time_constant)

    if "steer_time_constant" in sim_setting_dict.keys():
        nominal_setting_dict_display["steer_time_constant"] = 1 * steer_time_constant
        steer_time_constant = sim_setting_dict["steer_time_constant"]
        print("nominal steer_time_constant", nominal_setting_dict_display["steer_time_constant"])
        print("perturbed steer_time_constant", steer_time_constant)

    if "accel_map_scale" in sim_setting_dict.keys():
        nominal_setting_dict_display["accel_map_scale"] = None  # type: ignore
        use_accel_map = True
        accel_map_scale = sim_setting_dict["accel_map_scale"]
        df = pd.read_csv("accel_map.csv", header=None)
        current_vel_axis = np.array(
            [float(df.loc[0].values[1 + i]) for i in range(len(df.loc[0].values[1:]))]
        )
        accel_cmd_axis = np.array(
            [float(df[0].values[1 + i]) for i in range(len(df[0].values[1:]))]
        )
        accel_map_data = np.zeros((len(accel_cmd_axis), len(current_vel_axis)))
        for i in range(len(current_vel_axis)):
            for j in range(len(accel_cmd_axis)):
                accel_map_data[j, i] = float(df[1 + i].values[1 + j]) * (
                    accel_map_scale ** (i * 1.0 / (len(current_vel_axis) - 1))
                )
        X1, X2 = np.meshgrid(current_vel_axis, accel_cmd_axis)
        xy = [(x, y) for x in current_vel_axis for y in accel_cmd_axis]
        z = [
            accel_map_data[j, i]
            for i in range(len(current_vel_axis))
            for j in range(len(accel_cmd_axis))
        ]
        accel_map_f_ = scipy.interpolate.LinearNDInterpolator(xy, z)

        def accel_map_f(x, y):
            res = accel_map_f_(
                [np.clip(x, a_min=current_vel_axis.min(), a_max=current_vel_axis.max())],
                [np.clip(y, a_min=accel_cmd_axis.min(), a_max=accel_cmd_axis.max())],
            )
            return [res]

        print("current_vel_axis", current_vel_axis)
        print("accel_cmd_axis", accel_cmd_axis)
        print(
            "accel_map_f(current_vel_axis[-1],accel_cmd_axis[-1])",
            accel_map_f(current_vel_axis[-1], accel_cmd_axis[-1]),
            "accel_map_data[-1,-1]",
            accel_map_data[-1, -1],
        )
        print(
            "accel_map_f(current_vel_axis[-1],accel_cmd_axis[-2])",
            accel_map_f(current_vel_axis[-1], accel_cmd_axis[-2]),
            "accel_map_data[-2,-1]",
            accel_map_data[-2, -1],
        )
        print("perturbed use_accel_map", use_accel_map)

    if "steer_scaling" in sim_setting_dict.keys():
        nominal_setting_dict_display["steer_scaling"] = 1 * steer_scaling
        steer_scaling = sim_setting_dict["steer_scaling"]
        print("nominal steer_scaling", nominal_setting_dict_display["steer_scaling"])
        print("perturbed steer_scaling", steer_scaling)

    if "acc_scaling" in sim_setting_dict.keys():
        nominal_setting_dict_display["acc_scaling"] = 1 * acc_scaling
        acc_scaling = sim_setting_dict["acc_scaling"]
        print("nominal acc_scaling", nominal_setting_dict_display["acc_scaling"])
        print("perturbed acc_scaling", acc_scaling)

    if "vehicle_type" in sim_setting_dict.keys():
        nominal_setting_dict_display["steer_rate_lim"] = 1 * steer_rate_lim
        nominal_setting_dict_display["vel_rate_lim"] = 1 * vel_rate_lim
        nominal_setting_dict_display["wheel_base"] = 1 * L
        nominal_setting_dict_display["acc_time_delay"] = sim_dt * acc_delay_step_sim
        nominal_setting_dict_display["steer_time_delay"] = sim_dt * steer_delay_step_sim
        nominal_setting_dict_display["acc_time_constant"] = 1 * acc_time_constant
        nominal_setting_dict_display["steer_time_constant"] = 1 * steer_time_constant
        nominal_setting_dict_display["acc_scaling"] = 1 * acc_scaling
        if sim_setting_dict["vehicle_type"] == 0:
            pass
        elif sim_setting_dict["vehicle_type"] == 1:
            # heavy-weight bus
            if ACTIVATE_LIMITS:
                steer_rate_lim = 5.0
                vel_rate_lim = 7.0
            L = 4.76
            acc_delay_step_sim = round(1.0 / sim_dt)
            steer_delay_step_sim = round(1.0 / sim_dt)
            acc_time_constant = 1.0
            steer_time_constant = 1.0
            acc_scaling = 0.2
        elif sim_setting_dict["vehicle_type"] == 2:
            # light-weight bus
            if ACTIVATE_LIMITS:
                steer_rate_lim = 5.0
                vel_rate_lim = 7.0
            L = 4.76
            acc_delay_step_sim = round(0.5 / sim_dt)
            steer_delay_step_sim = round(0.5 / sim_dt)
            acc_time_constant = 0.5
            steer_time_constant = 0.5
            acc_scaling = 0.5
        elif sim_setting_dict["vehicle_type"] == 3:
            # small vehicle
            if ACTIVATE_LIMITS:
                steer_rate_lim = 5.0
                vel_rate_lim = 7.0
            L = 1.335
            acc_delay_step_sim = round(0.3 / sim_dt)
            steer_delay_step_sim = round(0.3 / sim_dt)
            acc_time_constant = 0.3
            steer_time_constant = 0.3
            acc_scaling = 1.5
        elif sim_setting_dict["vehicle_type"] == 4:
            # small robot
            if ACTIVATE_LIMITS:
                steer_rate_lim = 60.0 * (np.pi / 180.0)
                vel_rate_lim = 3.0
            L = 0.395
            acc_delay_step_sim = round(0.2 / sim_dt)
            steer_delay_step_sim = round(0.2 / sim_dt)
            acc_time_constant = 0.2
            steer_time_constant = 0.2
            acc_scaling = 3.0

        print("steer_rate_lim", nominal_setting_dict_display["steer_rate_lim"], steer_rate_lim)
        print("vel_rate_lim", nominal_setting_dict_display["vel_rate_lim"], vel_rate_lim)
        print("wheel_base", nominal_setting_dict_display["wheel_base"], L)
        print(
            "acc_time_delay",
            nominal_setting_dict_display["acc_time_delay"],
            sim_dt * acc_delay_step_sim,
        )
        print(
            "steer_time_delay",
            nominal_setting_dict_display["steer_time_delay"],
            sim_dt * steer_delay_step_sim,
        )
        print(
            "acc_time_constant",
            nominal_setting_dict_display["acc_time_constant"],
            acc_time_constant,
        )
        print(
            "steer_time_constant",
            nominal_setting_dict_display["steer_time_constant"],
            steer_time_constant,
        )
        print("acc_scaling", nominal_setting_dict_display["acc_scaling"], acc_scaling)

    if "test_vehicle" in sim_setting_dict.keys():
        nominal_setting_dict_display["steer_rate_lim"] = 1 * steer_rate_lim
        nominal_setting_dict_display["vel_rate_lim"] = 1 * vel_rate_lim
        nominal_setting_dict_display["wheel_base"] = 1 * L
        nominal_setting_dict_display["acc_time_delay"] = sim_dt * acc_delay_step_sim
        nominal_setting_dict_display["steer_time_delay"] = sim_dt * steer_delay_step_sim
        nominal_setting_dict_display["acc_time_constant"] = 1 * acc_time_constant
        nominal_setting_dict_display["steer_time_constant"] = 1 * steer_time_constant
        nominal_setting_dict_display["acc_scaling"] = 1 * acc_scaling
        if sim_setting_dict["test_vehicle"] == 0:
            L = 5.76
            acc_delay_step_sim = round(0.2 / sim_dt)
            steer_delay_step_sim = round(0.9 / sim_dt)
            acc_time_constant = 0.4
            steer_time_constant = 0.8
            steer_scaling = 3.0
            acc_scaling = 0.5
            steer_dead_band = 0.002
            measurement_steer_bias = 0.2 * np.pi / 180.0
        elif sim_setting_dict["test_vehicle"] == 1:
            L = 1.76
            acc_delay_step_sim = round(0.7 / sim_dt)
            steer_delay_step_sim = round(0.8 / sim_dt)
            acc_time_constant = 0.2
            steer_time_constant = 0.9
            steer_scaling = 0.3
            acc_scaling = 3.0
            steer_dead_band = 0.001
            measurement_steer_bias = 0.3 * np.pi / 180.0
        elif sim_setting_dict["test_vehicle"] == 2:
            L = 4.56
            acc_delay_step_sim = round(0.4 / sim_dt)
            steer_delay_step_sim = round(0.8 / sim_dt)
            acc_time_constant = 0.4
            steer_time_constant = 0.9
            steer_scaling = 0.6
            acc_scaling = 0.3
            steer_dead_band = 0.004
            measurement_steer_bias = 0.7 * np.pi / 180.0
        elif sim_setting_dict["test_vehicle"] == 3:
            # heavy-weight bus
            L = 4.76
            acc_delay_step_sim = round(1.0 / sim_dt)
            steer_delay_step_sim = round(1.0 / sim_dt)
            acc_time_constant = 1.0
            steer_time_constant = 1.0
            acc_scaling = 0.2
        elif sim_setting_dict["test_vehicle"] == 4:
            L = 3.56
            acc_delay_step_sim = round(0.3 / sim_dt)
            steer_delay_step_sim = round(0.8 / sim_dt)
            acc_time_constant = 0.3
            steer_time_constant = 0.9
            steer_scaling = 0.3
            acc_scaling = 0.3
            steer_dead_band = 0.003
            measurement_steer_bias = 0.3 * np.pi / 180.0
        elif sim_setting_dict["test_vehicle"] == 5:
            L = 4.76
            acc_delay_step_sim = round(1.0 / sim_dt)
            steer_delay_step_sim = round(0.7 / sim_dt)
            acc_time_constant = 1.0
            steer_time_constant = 0.7
            acc_scaling = 0.2
        elif sim_setting_dict["test_vehicle"] == 6:
            L = 4.76
            acc_delay_step_sim = round(1.0 / sim_dt)
            steer_delay_step_sim = round(0.6 / sim_dt)
            acc_time_constant = 1.0
            steer_time_constant = 0.6
            acc_scaling = 0.2
        elif sim_setting_dict["test_vehicle"] == 7:
            L = 4.76
            acc_delay_step_sim = round(1.0 / sim_dt)
            steer_delay_step_sim = round(0.5 / sim_dt)
            acc_time_constant = 1.0
            steer_time_constant = 0.5
            acc_scaling = 0.2
        print("steer_rate_lim", nominal_setting_dict_display["steer_rate_lim"], steer_rate_lim)
        print("vel_rate_lim", nominal_setting_dict_display["vel_rate_lim"], vel_rate_lim)
        print("wheel_base", nominal_setting_dict_display["wheel_base"], L)
        print(
            "acc_time_delay",
            nominal_setting_dict_display["acc_time_delay"],
            sim_dt * acc_delay_step_sim,
        )
        print(
            "steer_time_delay",
            nominal_setting_dict_display["steer_time_delay"],
            sim_dt * steer_delay_step_sim,
        )
        print(
            "acc_time_constant",
            nominal_setting_dict_display["acc_time_constant"],
            acc_time_constant,
        )
        print(
            "steer_time_constant",
            nominal_setting_dict_display["steer_time_constant"],
            steer_time_constant,
        )
        print("acc_scaling", nominal_setting_dict_display["acc_scaling"], acc_scaling)


# dynamics
# @njit(cache=False, fastmath=True) # Commented out because we want to use scipy.interpolate2d
def f_sim(
    states,
    inputs,
    L=L,
    acc_time_constant=acc_time_constant,
    steer_time_constant=steer_time_constant,
    steer_dead_band=steer_dead_band,
    steer_scaling=steer_scaling,
    acc_scaling=acc_scaling,
    measurement_steer_bias=measurement_steer_bias,
):
    """Time derivative of sim model dynamics."""
    v = states[2]
    theta = states[3]
    alpha = states[4]
    delta = states[5]

    delta_diff = steer_scaling * inputs[1] - delta
    if delta_diff >= steer_dead_band:
        delta_diff = delta_diff - steer_dead_band
    elif delta_diff <= -steer_dead_band:
        delta_diff = delta_diff + steer_dead_band
    else:
        delta_diff = 0.0

    actual_alpha_input = acc_scaling * inputs[0]
    actual_delta = delta - measurement_steer_bias

    # Acceleration input value -> Actual acceleration input value distortion applied
    if use_accel_map:
        if actual_alpha_input > 0:
            actual_alpha_input = accel_map_f(v, actual_alpha_input)[0]  # vel, acc_cmd

    # Tire angle input value -> Actual tire angle input value distortion application
    steer_tire_angle_cmd = 1 * delta
    coeff_from_tire_to_wheel = (
        20.0  # Assuming a fixed gear ratio of 20 (moving in a range of roughly 15-30)
    )
    steer_wheel = coeff_from_tire_to_wheel * steer_tire_angle_cmd
    adaptive_gear_ratio1 = vgr_coef_a1 + vgr_coef_b1 * v * v - vgr_coef_c1 * abs(steer_wheel)
    adaptive_gear_ratio2 = vgr_coef_a2 + vgr_coef_b2 * v * v - vgr_coef_c2 * abs(steer_wheel)
    actual_delta = (
        steer_tire_angle_cmd * (adaptive_gear_ratio1 / adaptive_gear_ratio2)
        - measurement_steer_bias
    )

    states_dot = np.zeros(6)
    states_dot[0] = v * np.cos(theta)
    states_dot[1] = v * np.sin(theta)
    states_dot[2] = alpha
    states_dot[3] = v * np.tan(actual_delta) / L
    states_dot[4] = (actual_alpha_input - alpha) / acc_time_constant
    states_dot[5] = delta_diff / steer_time_constant
    return states_dot


def F_sim(
    states,
    inputs,
    L=L,
    acc_time_constant=acc_time_constant,
    steer_time_constant=steer_time_constant,
    steer_dead_band=steer_dead_band,
):
    """Sim model dynamics based on the Euler method."""
    states_next = (
        states
        + f_sim(states, inputs, L, acc_time_constant, steer_time_constant, steer_dead_band) * sim_dt
    )
    return states_next


# cSpell:ignore njit fastmath
@njit(cache=False, fastmath=True)
def d_inputs_to_inputs(u_old, d_inputs):
    """Compute the sequence of input values from the sequence of input change rates."""
    inputs = np.zeros((d_inputs.shape[0] * ctrl_freq * mpc_freq, 2))
    for j in range(mpc_freq):
        inputs[ctrl_freq * j : ctrl_freq * (j + 1)] += (
            u_old + (j + 1) * d_inputs[0] * ctrl_time_step
        )
    for i in range(1, d_inputs.shape[0]):
        for j in range(mpc_freq):
            inputs[ctrl_freq * (mpc_freq * i + j) : ctrl_freq * (mpc_freq * i + j + 1)] += (
                inputs[ctrl_freq * mpc_freq * i - 1] + (j + 1) * d_inputs[i] * ctrl_time_step
            )
    return inputs


def F_true_predict(
    x_current,
    u_old,
    d_inputs,
    acc_des_queue,
    steer_des_queue,
    L=L,
    acc_time_constant=acc_time_constant,
    steer_time_constant=steer_time_constant,
    steer_dead_band=steer_dead_band,
):
    """Compute the true predicted trajectory based on simulator dynamics from the input values."""
    inputs = d_inputs_to_inputs(u_old, d_inputs)
    predicted_traj = np.zeros((d_inputs.shape[0] + 1, x_current.shape[0]))
    predicted_traj[0] += x_current
    acc_queue = np.concatenate((np.array(acc_des_queue), inputs[:, 0]))
    steer_queue = np.concatenate((np.array(steer_des_queue), inputs[:, 1]))
    for i in range(d_inputs.shape[0]):
        x_tmp = predicted_traj[i].copy()
        for j in range(ctrl_freq * mpc_freq):
            x_tmp = F_sim(
                x_tmp,
                np.array(
                    [
                        acc_queue[ctrl_freq * mpc_freq * i + j],
                        steer_queue[ctrl_freq * mpc_freq * i + j],
                    ]
                ),
                L,
                acc_time_constant,
                steer_time_constant,
                steer_dead_band,
            )
        predicted_traj[i + 1] = x_tmp.copy()
    return predicted_traj, inputs[:: ctrl_freq * mpc_freq]


def get_mpc_trajectory(x_current, trajectory_data, trajectory_interpolator_list):
    """Calculate the target trajectory to be used in MPC from the given data."""
    nearest_index = np.argmin(
        ((trajectory_data[:, 1:3] - x_current[:2].reshape(1, 2)) ** 2).sum(axis=1)
    )
    timestamp_mpc_trajectory = (
        np.arange(N + 1) * mpc_freq * ctrl_freq * sim_dt + trajectory_data[nearest_index, 0]
    )
    break_flag = False
    if timestamp_mpc_trajectory[-1] >= trajectory_data[-1, 0]:
        timestamp_mpc_trajectory -= timestamp_mpc_trajectory[-1] - trajectory_data[-1, 0] - 1e-16
        break_flag = True

    mpc_trajectory = np.array(
        [
            trajectory_interpolator_list[i](timestamp_mpc_trajectory)
            for i in range(trajectory_data.shape[1] - 1)
        ]
    ).T
    mpc_trajectory = np.hstack(
        [
            timestamp_mpc_trajectory.reshape(-1, 1) - trajectory_data[nearest_index, 0],
            mpc_trajectory,
        ]
    )

    X_des = np.zeros((N + 1, 8))
    X_des[:, :6] = mpc_trajectory[:, 1:7]
    X_des[:, [6, 7]] = mpc_trajectory[:, [5, 6]]
    U_des = np.zeros((N, 2))
    return X_des, U_des, x_current[:6] - trajectory_data[nearest_index, 1:7], break_flag


def drive_sim(
    save_file=True,
    save_dir=None,
    load_dir=drive_functions.load_dir,
    visualize=True,
    use_trained_model=False,
    use_trained_model_diff=None,
    use_memory_diff=None,
    control_type: ControlType = ControlType.mpc,
    initial_error=np.zeros(6),
    t_range=[0, 100],
    seed=1,
    acc_amp_range=0.05,
    acc_period_range=[5.0, 20.0],
    steer_amp_range=0.005,
    steer_period_range=[5.0, 30.0],
    large_steer_amp_range=0.00,
    large_steer_period_range=[5.0, 20.0],
    start_large_steer_time=40.0,
    acc_max=1.2,
    constant_vel_time=5.0,
    split_size=5,
    y_length=60.0,
    x_length=120.0,
    step_response_max_input=0.01,
    step_response_max_length=1.5,
    step_response_start_time_ratio=0.0,
    step_response_interval=5.0,
    step_response_min_length=0.5,
    smoothing_trajectory_data_flag=True,
):
    """Perform a slalom driving simulation."""
    if control_type != ControlType.mpc:
        print(f"\n[run {control_type.value}]\n")
    else:
        print("\n[run slalom_drive]\n")
    if save_file:
        if save_dir is None:
            save_dir_ = "python_sim_log_" + str(datetime.datetime.now())
        else:
            save_dir_ = save_dir
        if not os.path.isdir(save_dir_):
            os.mkdir(save_dir_)
    if control_type == ControlType.mpc:
        controller = drive_controller.drive_controller(
            model_file_name=(load_dir + "/model_for_test_drive.pth"),
            load_GP_dir=load_dir,
            load_polynomial_reg_dir=load_dir,
            use_trained_model=use_trained_model,
            use_trained_model_diff=use_trained_model_diff,
            use_memory_diff=use_memory_diff,
            load_train_data_dir=load_dir,
        )
        mode = controller.mode
        print("mode:", mode)

    # cSpell:ignore interp
    plt.rcParams["figure.figsize"] = (8, 8)
    t_eval = np.arange(t_range[0], t_range[1], sim_dt)
    trajectory_data = np.loadtxt("slalom_course_data.csv", delimiter=",")
    trajectory_interpolator_list = [
        scipy.interpolate.interp1d(trajectory_data[:, 0], trajectory_data[:, 1 + i])
        for i in range(trajectory_data.shape[1] - 1)
    ]
    x_init = trajectory_data[0, 1:7].copy()

    acc_des_queue = [0] * acc_delay_step_sim

    initial_steer_input = (
        measurement_steer_bias + np.sign(measurement_steer_bias) * steer_dead_band
    ) / steer_scaling
    steer_des_queue = [initial_steer_input] * steer_delay_step_sim

    calculated = 0
    break_flag = False

    tracking_error_list = []
    target_vel_list = []

    total_abs_max_lateral_deviation = -1
    straight_line_abs_max_lateral_deviation = -1
    total_abs_max_velocity_error = -1
    total_abs_max_yaw_error = -1
    total_abs_max_acc_error = -1
    total_abs_max_steer_error = -1
    prev_u_actual_input = np.zeros(2)
    log_updater = data_collection_utils.driving_log_updater()

    pp_gain_updater = pure_pursuit_gain_updater.pure_pursuit_gain_updater()
    acc_gain_scaling = 1.0
    steer_gain_scaling = 1.0  # L/2.79

    acc_gain_scaling_decay = 0.9
    steer_gain_scaling_decay = 0.9

    # used for "pp_eight"
    previous_pp_index = 0

    if control_type != ControlType.mpc:  # feedforward_test:
        (
            t_acc_array,
            amp_acc_array,
            t_steer_array,
            amp_steer_array,
            t_large_steer_array,
            amp_large_steer_array,
        ) = data_collection_utils.create_additional_sine_data(
            seed,
            t_range,
            acc_amp_range,
            acc_period_range,
            steer_amp_range,
            steer_period_range,
            large_steer_amp_range,
            large_steer_period_range,
            start_large_steer_time,
        )
    if control_type in [ControlType.pp_eight, ControlType.pp_straight, ControlType.npp_eight]:
        if control_type in [ControlType.pp_eight, ControlType.npp_eight]:
            figure_eight = data_collection_utils.FigureEight(
                y_length,
                x_length,
                acc_max=acc_max,
                constant_vel_time=constant_vel_time,
                split_size=split_size,
                smoothing_trajectory_data_flag=smoothing_trajectory_data_flag,
            )
            (
                trajectory_position_data,
                trajectory_yaw_data,
                curvature_radius,
                parts,
                achievement_rates,
            ) = figure_eight.get_trajectory_points(0.01)

            x_init[:2] = trajectory_position_data[0]
            x_init[3] = trajectory_yaw_data[0]
            x_init[2] = figure_eight.v_start
        else:
            straight_line = data_collection_utils.StraightLine(
                acc_max=acc_max, constant_vel_time=constant_vel_time, split_size=split_size
            )
            x_init[2] = straight_line.v_mid

    x_init += initial_error

    x_current = x_init.copy()

    for i in range(t_eval.size):
        if i % ctrl_freq == 0:  # update u_opt
            if control_type in [ControlType.pp_eight, ControlType.pp_straight]:
                pp_gain_updater.state_queue_updater(
                    x_current[2], x_current[3], x_current[4], x_current[5]
                )

            if control_type == ControlType.mpc:  # not feedforward_test:
                X_des, U_des, tracking_error, break_flag = get_mpc_trajectory(
                    x_current, trajectory_data, trajectory_interpolator_list
                )
                tracking_error_list.append(tracking_error.copy())
                u_opt = controller.update_input_queue_and_get_optimal_control(
                    log_updater.control_cmd_time_stamp_list,
                    log_updater.control_cmd_acc_list,
                    log_updater.control_cmd_steer_list,
                    x_current,
                    X_des,
                    U_des,
                    t_eval[i],
                    t_eval[i],
                )
                if t_eval[i] < 5.0:
                    u_opt[1] = initial_steer_input
            elif control_type == ControlType.ff:
                u_opt = data_collection_utils.get_current_additional_sine(
                    t_eval[i],
                    t_acc_array,
                    amp_acc_array,
                    t_steer_array,
                    amp_steer_array,
                    t_large_steer_array,
                    amp_large_steer_array,
                )
                u_opt[0] += data_collection_utils.get_feedforward_nominal_input(
                    t_eval[i], trajectory_data
                )[0]

            elif control_type == ControlType.pp_eight:
                (
                    target_position,
                    target_yaw,
                    previous_pp_index,
                ) = data_collection_utils.get_pure_pursuit_info(
                    x_current, trajectory_position_data, trajectory_yaw_data, previous_pp_index
                )
                target_vel = figure_eight.get_current_velocity(t_eval[i])
                target_vel_list.append(np.array([t_eval[i], target_vel]))
                u_opt = drive_functions.pure_pursuit_control(
                    pos_xy_obs=x_current[:2],
                    pos_yaw_obs=x_current[3],
                    longitudinal_vel_obs=x_current[2],
                    pos_xy_ref=target_position[:2],
                    pos_yaw_ref=target_yaw,
                    longitudinal_vel_ref=target_vel,
                    acc_gain_scaling=acc_gain_scaling,
                    steer_gain_scaling=steer_gain_scaling,
                )
                u_opt += data_collection_utils.get_current_additional_sine(
                    t_eval[i],
                    t_acc_array,
                    amp_acc_array,
                    t_steer_array,
                    amp_steer_array,
                    t_large_steer_array,
                    amp_large_steer_array,
                )
                u_opt[1] += data_collection_utils.step_response(
                    t_eval[i],
                    step_response_start_time_ratio * t_range[1],
                    step_response_interval,
                    step_response_max_input,
                    step_response_max_length,
                    step_response_min_length,
                )
                break_flag = figure_eight.break_flag
            elif control_type == ControlType.pp_straight:
                target_vel = straight_line.get_current_velocity(t_eval[i])
                target_vel_list.append(np.array([t_eval[i], target_vel]))
                target_position = np.array([x_current[0], 0.0])
                target_yaw = 0.0
                u_opt = drive_functions.pure_pursuit_control(
                    pos_xy_obs=x_current[:2],
                    pos_yaw_obs=x_current[3],
                    longitudinal_vel_obs=x_current[2],
                    pos_xy_ref=target_position[:2],
                    pos_yaw_ref=target_yaw,
                    longitudinal_vel_ref=target_vel,
                    acc_gain_scaling=acc_gain_scaling,
                    steer_gain_scaling=steer_gain_scaling,
                )
                u_opt += data_collection_utils.get_current_additional_sine(
                    t_eval[i],
                    t_acc_array,
                    amp_acc_array,
                    t_steer_array,
                    amp_steer_array,
                    t_large_steer_array,
                    amp_large_steer_array,
                )
                u_opt[1] += data_collection_utils.step_response(
                    t_eval[i],
                    step_response_start_time_ratio * t_range[1],
                    step_response_interval,
                    step_response_max_input,
                    step_response_max_length,
                    step_response_min_length,
                )
                break_flag = straight_line.break_flag

            elif control_type == ControlType.npp_eight:
                (
                    target_position,
                    target_yaw,
                    previous_pp_index,
                    target_position_ahead,
                ) = data_collection_utils.get_naive_pure_pursuit_info(
                    x_current, trajectory_position_data, trajectory_yaw_data, previous_pp_index
                )
                target_vel = figure_eight.get_current_velocity(t_eval[i])

                applying_velocity_limit_by_lateral_acc_flag = True
                if applying_velocity_limit_by_lateral_acc_flag:
                    max_lateral_accel = 0.3
                    velocity_limit_by_lateral_acc = np.sqrt(
                        max_lateral_accel * curvature_radius[previous_pp_index]
                    )
                    target_vel = min(target_vel, velocity_limit_by_lateral_acc)

                applying_velocity_limit_by_tracking_error_flag = True
                if applying_velocity_limit_by_tracking_error_flag:
                    lateral_error = np.sqrt(((x_current[:2] - target_position[:2]) ** 2).sum())
                    yaw_error = x_current[3] - target_yaw
                    if yaw_error > np.pi:
                        yaw_error -= 2 * np.pi
                    if yaw_error < -np.pi:
                        yaw_error += 2 * np.pi

                    safety_velocity = 3.0
                    lateral_error_threshold = 2.0
                    yaw_error_threshold = 0.5
                    if lateral_error_threshold < lateral_error or yaw_error_threshold < np.abs(
                        yaw_error
                    ):
                        target_vel = safety_velocity

                target_vel_list.append(np.array([t_eval[i], target_vel]))
                u_opt = drive_functions.naive_pure_pursuit_control(
                    pos_xy_obs=x_current[:2],
                    pos_yaw_obs=x_current[3],
                    longitudinal_vel_obs=x_current[2],
                    pos_xy_ref_target=target_position_ahead[:2],
                    longitudinal_vel_ref_nearest=target_vel,
                )
                u_opt += data_collection_utils.get_current_additional_sine(
                    t_eval[i],
                    t_acc_array,
                    amp_acc_array,
                    t_steer_array,
                    amp_steer_array,
                    t_large_steer_array,
                    amp_large_steer_array,
                )
                u_opt[1] += data_collection_utils.step_response(
                    t_eval[i],
                    step_response_start_time_ratio * t_range[1],
                    step_response_interval,
                    step_response_max_input,
                    step_response_max_length,
                    step_response_min_length,
                )
                break_flag = figure_eight.break_flag
            calculated += 1
            log_updater.update(t_eval[i], x_current, u_opt)
            if control_type in [ControlType.pp_eight, ControlType.pp_straight]:
                pp_gain_updater.input_queue_updater(u_opt[0], u_opt[1])
                if i % (100 * ctrl_freq) == 0 and UPDATE_PP_GAIN:  # update u_opt
                    # pp_gain_updater.get_acc_gain_scaling()
                    acc_gain_scaling = (
                        acc_gain_scaling_decay * acc_gain_scaling
                        + (1 - acc_gain_scaling_decay) * pp_gain_updater.get_acc_gain_scaling()
                    )
                    if control_type == ControlType.pp_eight:
                        steer_gain_scaling = (
                            steer_gain_scaling_decay * steer_gain_scaling
                            + (1 - steer_gain_scaling_decay)
                            * pp_gain_updater.get_steer_gain_scaling()
                        )

        if visualize and (control_type != ControlType.mpc) and (i == t_eval.size - 1 or break_flag):
            X = np.array(log_updater.X_history)
            U = np.array(log_updater.U_history)

            if not use_trained_model:
                title_str = "control_using_nominal_model: "
            else:
                title_str = "control_using_trained_model: "
            if perturbed_sim_flag:
                title_str += (
                    "perturbed_sim: "
                    + str(sim_setting_dict)
                    + ", nominal_model: "
                    + str(nominal_setting_dict_display)
                )
            # cSpell:ignore nrows ncols suptitle
            fig, axes = plt.subplots(nrows=3, ncols=3, figsize=(18, 12), tight_layout=True)
            fig.suptitle(title_str)

            ax1: plt.Axes = axes[0, 0]
            ax1.plot(X[:, 0], X[:, 1], label="trajectory")
            ax1.legend()
            ax1.set_xlabel("x_position [m]")
            ax1.set_ylabel("y_position [m]")

            time_normalize_1 = ctrl_freq * sim_dt
            f_s = int(1.0 / time_normalize_1)

            # cSpell:ignore numba simplejson fftfreq basefmt markerfmt
            ax2: plt.Axes = axes[0, 1]
            X_acc = np.fft.fft(U[:, 0]) / len(U[:, 0])  # Fourier transform of waveforms
            freqs = np.fft.fftfreq(len(U[:, 0])) * f_s
            ax2.stem(
                freqs,
                np.abs(X_acc),
                basefmt="k-",
                markerfmt="cx",
                label="acc input",
            )
            ax2.legend()
            ax2.set_xlim([-1.0, 1.0])
            ax2.set_xlabel("freq")
            ax2.set_ylabel("amplitude")

            ax3: plt.Axes = axes[0, 2]
            X_steer = np.fft.fft(U[:, 1]) / len(U[:, 1])  # Fourier transform of waveforms
            freqs = np.fft.fftfreq(len(U[:, 1])) * f_s
            ax3.stem(
                freqs,
                np.abs(X_steer),
                basefmt="k-",
                markerfmt="cx",
                label="steer input",
            )
            ax3.legend()
            ax3.set_xlim([-1.0, 1.0])
            ax3.set_xlabel("freq")
            ax3.set_ylabel("amplitude")

            if control_type in [
                ControlType.pp_eight,
                ControlType.pp_straight,
                ControlType.npp_eight,
            ]:
                ax4: plt.Axes = axes[1, 0]
                ax4.plot(
                    np.array(target_vel_list)[:, 0],
                    np.array(target_vel_list)[:, 1],
                    label="vel target",
                )
                ax4.plot(time_normalize_1 * np.arange(X.shape[0]), X[:, 2], label="vel")
                ax4.legend()

            ax5: plt.Axes = axes[1, 1]
            ax5.plot(time_normalize_1 * np.arange(U.shape[0]), U[:, 0], label="acc input")
            ax5.legend()

            ax6: plt.Axes = axes[1, 2]
            ax6.plot(time_normalize_1 * np.arange(U.shape[0]), U[:, 1], label="steer input")
            ax6.legend()

            kinematic_states = KinematicStates(
                speed=X[:, 2],
                acc=X[:, 4],
                steer=X[:, 5],
            )

            ax7: plt.Axes = axes[2, 0]
            fig, ax7 = visualize_speed_acc(fig=fig, ax=ax7, kinematic_states=kinematic_states)
            ax7.plot()

            ax8: plt.Axes = axes[2, 1]
            fig, ax8 = visualize_speed_steer(fig=fig, ax=ax8, kinematic_states=kinematic_states)
            ax8.plot()

            ax9: plt.Axes = axes[2, 2]
            ax9.axis("off")

            if save_file:
                png_save_dir = save_dir_
            else:
                png_save_dir = "."
            plt.savefig(f"{png_save_dir}/python_simulator_{control_type.value}_drive.png")
            plt.close()

        if (
            visualize
            and (control_type == ControlType.mpc)
            and (i % 1000 * ctrl_freq == 999 * ctrl_freq or i == t_eval.size - 1 or break_flag)
        ):
            fig = plt.figure(figsize=(24, 15), tight_layout=True)
            if not use_trained_model:
                title_str = "control_using_nominal_model: "
            else:
                title_str = "control_using_trained_model: "
            if perturbed_sim_flag:
                title_str += (
                    "perturbed_sim: "
                    + str(sim_setting_dict)
                    + ", nominal_model: "
                    + str(nominal_setting_dict_display)
                )
            fig.suptitle(title_str)

            plt.subplot(3, 3, 1)

            ax1 = plt.subplot(3, 3, 1)
            X = np.array(log_updater.X_history)
            U = np.array(log_updater.U_history)
            X_des_hist = np.array(controller.X_des_history)
            time_normalize_1 = ctrl_freq * sim_dt
            time_normalize_2 = ctrl_freq * mpc_freq * sim_dt
            ax1.scatter(
                trajectory_data[:, 1],
                trajectory_data[:, 2],
                s=4,
                c="orange",
                label="reference_trajectory",
            )
            ax1.plot(X[:, 0], X[:, 1], label="trajectory", color="tab:blue")
            if mode != "mppi":
                ax1.scatter(
                    controller.nominal_traj_ilqr[:, 0],
                    controller.nominal_traj_ilqr[:, 1],
                    s=4,
                    c="red",
                    label="pred_trajectory_ilqr",
                )
                true_prediction, nominal_inputs = F_true_predict(
                    x_current,
                    controller.u_old,
                    controller.nominal_inputs_ilqr,
                    acc_des_queue,
                    steer_des_queue,
                )
                ax1.scatter(
                    true_prediction[:, 0],
                    true_prediction[:, 1],
                    s=4,
                    c="green",
                    label="true_prediction",
                )
            if mode != "ilqr":
                ax1.scatter(
                    controller.nominal_traj_mppi[:, 0],
                    controller.nominal_traj_mppi[:, 1],
                    s=4,
                    c="pink",
                    label="pred_trajectory_mppi",
                )

            ax1.legend()
            ax1.set_xlabel("x_position [m]")
            ax1.set_ylabel("y_position [m]")

            ax2 = plt.subplot(3, 3, 2)
            tracking_error_array = np.array(tracking_error_list)
            lateral_deviation = (
                -np.sin(X_des_hist[:, 3]) * tracking_error_array[:, 0]
                + np.cos(X_des_hist[:, 3]) * tracking_error_array[:, 1]
            )

            ax2.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                lateral_deviation,
                label="lateral_deviation",
            )

            ax2_coef = [0.00, 0.05, -0.05, 0.10, -0.10, 0.15, -0.15, 0.20, -0.20]
            for coe in ax2_coef:
                ax2.plot(
                    time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                    coe * np.ones(tracking_error_array.shape[0]),
                    linestyle="dashed",
                )

            ax2.set_xlabel("Time [s]")
            ax2.set_ylabel("Lateral deviation [m]")
            ax2.legend()

            straight_line_index = np.where(X_des_hist[:, 0] < 250.0)[
                0
            ].max()  # Before 250 [m] is considered to be a straight run.
            total_abs_max_lateral_deviation = np.abs(lateral_deviation).max()
            total_abs_mean_lateral_deviation = np.abs(lateral_deviation).mean()
            straight_line_abs_max_lateral_deviation = np.abs(
                lateral_deviation[: straight_line_index + 1]
            ).max()
            ax2.set_title(
                "abs_max(lateral_dev) = "
                + str(total_abs_max_lateral_deviation)
                + "\nabs_max(lateral_dev[straight_line]) = "
                + str(straight_line_abs_max_lateral_deviation)
            )

            ax3 = plt.subplot(3, 3, 4)
            ax3.plot(
                time_normalize_1 * np.arange(X.shape[0]),
                X[:, 2],
                label="velocity",
                color="tab:blue",
            )
            ax3.plot(
                time_normalize_1 * np.arange(X.shape[0]),
                X_des_hist[:, 2],
                label="velocity_target",
                color="orange",
            )
            ax3.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -tracking_error_array[:, 2],
                label="velocity_error",
                color="lightgrey",
            )
            if mode != "mppi":
                ax3.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(true_prediction.shape[0]),
                    true_prediction[:, 2],
                    s=4,
                    label="velocity_true_prediction",
                    c="green",
                )
                ax3.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(controller.nominal_traj_ilqr.shape[0]),
                    controller.nominal_traj_ilqr[:, 2],
                    s=4,
                    label="velocity_prediction_ilqr",
                    c="red",
                )
            ax3.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                np.zeros(tracking_error_array.shape[0]),
                linestyle="dashed",
                color="darkred",
            )
            ax3.set_xlabel("Time [s]")
            ax3.set_ylabel("velocity [m/s]")
            ax3.legend()
            total_abs_max_velocity_error = np.abs(tracking_error_array[:, 2]).max()
            total_abs_mean_velocity_error = np.abs(tracking_error_array[:, 2]).mean()
            straight_line_abs_max_velocity_error = np.abs(
                tracking_error_array[: straight_line_index + 1, 2]
            ).max()
            ax3.set_title(
                "abs_max(velocity_error) = "
                + str(total_abs_max_velocity_error)
                + "\nabs_max(velocity_error[straight_line]) = "
                + str(straight_line_abs_max_velocity_error)
            )

            ax4 = plt.subplot(3, 3, 5)
            ax4.plot(
                time_normalize_1 * np.arange(X.shape[0]), X[:, 3], label="yaw", color="tab:blue"
            )
            ax4.plot(
                time_normalize_1 * np.arange(X.shape[0]),
                X_des_hist[:, 3],
                label="yaw_target",
                color="orange",
            )
            ax4.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -tracking_error_array[:, 3],
                label="yaw_error",
                color="lightgrey",
            )
            if mode != "mppi":
                ax4.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(true_prediction.shape[0]),
                    true_prediction[:, 3],
                    s=4,
                    label="yaw_true_prediction",
                    c="green",
                )
                ax4.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(controller.nominal_traj_ilqr.shape[0]),
                    controller.nominal_traj_ilqr[:, 3],
                    s=4,
                    label="yaw_prediction_ilqr",
                    c="red",
                )
            ax4.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                np.zeros(tracking_error_array.shape[0]),
                linestyle="dashed",
                c="darkred",
            )
            ax4.set_xlabel("Time [s]")
            ax4.set_ylabel("yaw [rad]")
            ax4.legend()
            total_abs_max_yaw_error = np.abs(tracking_error_array[:, 3]).max()
            total_abs_mean_yaw_error = np.abs(tracking_error_array[:, 3]).mean()
            straight_line_abs_max_yaw_error = np.abs(
                tracking_error_array[: straight_line_index + 1, 3]
            ).max()
            ax4.set_title(
                "abs_max(yaw_error) = "
                + str(total_abs_max_yaw_error)
                + "\nabs_max(yaw_error[straight_line]) = "
                + str(straight_line_abs_max_yaw_error)
            )

            ax5 = plt.subplot(3, 3, 7)
            ax5.plot(
                time_normalize_1 * np.arange(X.shape[0]), X[:, 4], label="acc", color="tab:blue"
            )
            ax5.plot(
                time_normalize_1 * np.arange(X.shape[0]), U[:, 0], label="acc_input", color="violet"
            )
            ax5.plot(
                time_normalize_1 * np.arange(X.shape[0]),
                X_des_hist[:, 4],
                label="acc_target",
                color="orange",
            )
            ax5.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -tracking_error_array[:, 4],
                label="acc_error",
                color="lightgrey",
            )
            if mode != "mppi":
                ax5.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(true_prediction.shape[0]),
                    true_prediction[:, 4],
                    s=4,
                    label="acc_true_prediction",
                    c="green",
                )
                ax5.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(controller.nominal_traj_ilqr.shape[0]),
                    controller.nominal_traj_ilqr[:, 4],
                    s=4,
                    label="acc_prediction_ilqr",
                    c="red",
                )
                ax5.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(nominal_inputs.shape[0]),
                    nominal_inputs[:, 0],
                    s=4,
                    label="acc_input_schedule",
                    c="plum",
                )
            ax5.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                np.zeros(tracking_error_array.shape[0]),
                linestyle="dashed",
                color="darkred",
            )
            ax5.set_xlabel("Time [s]")
            ax5.set_ylabel("acc [m/s^2]")
            ax5.legend()
            total_abs_max_acc_error = np.abs(tracking_error_array[:, 4]).max()
            total_abs_mean_acc_error = np.abs(tracking_error_array[:, 4]).mean()
            straight_line_abs_max_acc_error = np.abs(
                tracking_error_array[: straight_line_index + 1, 4]
            ).max()
            ax5.set_title(
                "abs_max(acc_error) = "
                + str(total_abs_max_acc_error)
                + "\nabs_max(acc_error[straight_line]) = "
                + str(straight_line_abs_max_acc_error)
            )

            ax6 = plt.subplot(3, 3, 8)
            ax6.plot(
                time_normalize_1 * np.arange(X.shape[0]), X[:, 5], label="steer", color="tab:blue"
            )
            ax6.plot(
                time_normalize_1 * np.arange(X.shape[0]),
                U[:, 1],
                label="steer_input",
                color="violet",
            )
            ax6.plot(
                time_normalize_1 * np.arange(X.shape[0]),
                X_des_hist[:, 7],
                label="steer_mpc_target",
                color="orange",
            )
            ax6.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -tracking_error_array[:, 5],
                label="steer_error",
                color="lightgrey",
            )
            if mode != "mppi":
                ax6.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(true_prediction.shape[0]),
                    true_prediction[:, 5],
                    s=4,
                    label="steer_true_prediction",
                    c="green",
                )
                ax6.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(controller.nominal_traj_ilqr.shape[0]),
                    controller.nominal_traj_ilqr[:, 5],
                    s=4,
                    label="steer_prediction_ilqr",
                    c="red",
                )
                ax6.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(nominal_inputs.shape[0]),
                    nominal_inputs[:, 1],
                    s=4,
                    label="steer_input_schedule",
                    c="plum",
                )
            ax6.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                np.zeros(tracking_error_array.shape[0]),
                linestyle="dashed",
                color="darkred",
            )
            ax6.set_xlabel("Time [s]")
            ax6.set_ylabel("steer [rad]")
            ax6.legend()
            total_abs_max_steer_error = np.abs(tracking_error_array[:, 5]).max()
            total_abs_mean_steer_error = np.abs(tracking_error_array[:, 5]).mean()
            straight_line_abs_max_steer_error = np.abs(
                tracking_error_array[: straight_line_index + 1, 5]
            ).max()
            ax6.set_title(
                "abs_max(steer_error) = "
                + str(total_abs_max_steer_error)
                + "\nabs_max(steer_error[straight_line]) = "
                + str(straight_line_abs_max_steer_error)
            )

            ax7 = plt.subplot(3, 3, 3)
            f_s = int(1.0 / time_normalize_1)

            steer_state = X[:, 5]
            X_steer = np.fft.fft(steer_state)  # Fourier transform of waveforms
            freqs = np.fft.fftfreq(len(steer_state)) * f_s
            ax7.stem(
                freqs,
                np.abs(X_steer) / len(steer_state),
                basefmt="k-",
                markerfmt="rx",
                label="steer_state",
            )

            lateral_acc = X[:, 2] * X[:, 2] * np.tan(X[:, 5]) / L
            steer_dot = np.zeros(X.shape[0])
            steer_dot[0] = (X[1, 5] - X[0, 5]) / time_normalize_1
            steer_dot[-1] = (X[-1, 5] - X[-2, 5]) / time_normalize_1
            steer_dot[1:-1] = 0.5 * (X[2:, 5] - X[:-2, 5]) / time_normalize_1
            lateral_jerk = 2 * X[:, 2] * np.tan(X[:, 5]) * X[:, 4] / L + X[:, 2] * X[
                :, 2
            ] * steer_dot / (L * np.cos(X[:, 5]) * np.cos(X[:, 5]))

            X_lateral_acc = np.fft.fft(lateral_acc)  # Fourier transform of waveforms
            freqs = np.fft.fftfreq(len(lateral_acc)) * f_s
            ax7.stem(
                freqs,
                np.abs(X_lateral_acc) / len(lateral_acc),
                basefmt="k-",
                markerfmt="gx",
                label="lateral_acc",
            )

            X_lateral_jerk = np.fft.fft(lateral_jerk)  # Fourier transform of waveforms
            freqs = np.fft.fftfreq(len(lateral_jerk)) * f_s
            ax7.stem(
                freqs,
                np.abs(X_lateral_jerk) / len(lateral_jerk),
                basefmt="k-",
                markerfmt="bx",
                label="lateral_jerk",
            )

            ax7.set_xlabel("Frequency in Hertz[Hz]")
            ax7.set_ylabel("Amplitude (normalized dividing by data length)")
            ax7.set_xlim(-1, 1)
            ax7.legend()

            if use_accel_map:
                ax9 = plt.subplot(3, 3, 9, projection="3d")
                X1, X2 = np.meshgrid(current_vel_axis, accel_cmd_axis)
                ax9.plot_surface(X1, X2, accel_map_data)

            if save_file:
                png_save_dir = save_dir_
            else:
                png_save_dir = "."
            if not use_trained_model:
                plt.savefig(png_save_dir + "/python_simulator_nominal_model_fig_" + str(i) + ".png")
            else:
                plt.savefig(png_save_dir + "/python_simulator_trained_model_fig_" + str(i) + ".png")
            plt.close()

            np.savetxt(
                save_dir_ + "/lateral_deviation.csv",
                np.hstack(
                    [time_normalize_1 * np.arange(tracking_error_array.shape[0]), lateral_deviation]
                ),
                delimiter=",",
            )

            np.savetxt(
                save_dir_ + "/steer_state.csv",
                np.hstack(
                    [time_normalize_1 * np.arange(tracking_error_array.shape[0]), steer_state]
                ),
                delimiter=",",
            )

        acc_des_queue.append(u_opt[0])
        steer_des_queue.append(u_opt[1])
        u_actual_input = np.array([acc_des_queue.pop(0), steer_des_queue.pop(0)])

        # Logic-level restrictions on control inputs
        delta_steer_max = steer_rate_lim * sim_dt
        u_actual_input[0] = np.clip(u_actual_input[0], -vel_rate_lim, vel_rate_lim)
        u_actual_input[1] = np.clip(
            u_actual_input[1],
            prev_u_actual_input[1] - delta_steer_max,
            prev_u_actual_input[1] + delta_steer_max,
        )

        x_next = F_sim(x_current, u_actual_input)
        x_current = x_next.copy()
        prev_u_actual_input = u_actual_input.copy()
        if break_flag:
            if control_type == ControlType.mpc:
                controller.send_initialize_input_queue()
                controller.stop_model_update()
            break
    if save_file:
        log_updater.save(save_dir_)

    if control_type == ControlType.mpc and perturbed_sim_flag:
        auto_test_performance_result_dict = {
            "total_abs_max_lateral_deviation": total_abs_max_lateral_deviation,
            "straight_line_abs_max_lateral_deviation": straight_line_abs_max_lateral_deviation,
            "total_abs_max_velocity_error": total_abs_max_velocity_error,
            "total_abs_max_yaw_error": total_abs_max_yaw_error,
            "total_abs_max_acc_error": total_abs_max_acc_error,
            "total_abs_max_steer_error": total_abs_max_steer_error,
            "use_trained_model": use_trained_model,
            "sim_setting_dict": sim_setting_dict,
            "nominal_setting_dict_display": nominal_setting_dict_display,
        }
        auto_test_performance_result_list = [
            total_abs_max_lateral_deviation,
            total_abs_max_velocity_error,
            total_abs_max_yaw_error,
            total_abs_max_acc_error,
            total_abs_max_steer_error,
            total_abs_mean_lateral_deviation,
            total_abs_mean_velocity_error,
            total_abs_mean_yaw_error,
            total_abs_mean_acc_error,
            total_abs_mean_steer_error,
            straight_line_abs_max_lateral_deviation,
        ]

        if save_file:
            with open(save_dir_ + "/auto_test_performance_result.json", "w") as json_f:
                json.dump(auto_test_performance_result_dict, json_f)
        print("how_many_time_controlled", calculated)

        return auto_test_performance_result_list
