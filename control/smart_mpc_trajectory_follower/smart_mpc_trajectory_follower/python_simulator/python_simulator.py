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

# cspell: ignore numba njit simplejson fastmath interp suptitle fftfreq basefmt markerfmt

import csv
import datetime
import os

import matplotlib.pyplot as plt
from numba import njit
import numpy as np
import pandas as pd
import scipy.interpolate
import simplejson as json
from smart_mpc_trajectory_follower.scripts import drive_controller
from smart_mpc_trajectory_follower.scripts import drive_functions

print("\n\n### import python_simulator.py ###")

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

steer_rate_lim = 0.35
vel_rate_lim = 7.0

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
        ]
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
        nominal_setting_dict_display["accel_map_scale"] = None
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

    actual_alpha = alpha
    actual_delta = delta - measurement_steer_bias

    # Acceleration input value -> Actual acceleration input value distortion applied
    if use_accel_map:
        if actual_alpha > 0:
            actual_alpha = accel_map_f(v, actual_alpha)[0]  # vel, acc_cmd

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
    states_dot[2] = actual_alpha
    states_dot[3] = v * np.tan(actual_delta) / L
    states_dot[4] = (acc_scaling * inputs[0] - alpha) / acc_time_constant
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


def get_feedforward_nominal_input(t, trajectory_data):
    """Calculate the nominal input for feed-forward driving."""
    total_time = trajectory_data[-1, 0]
    t_current = t - (t // total_time) * total_time
    nearest_index = np.argmin(np.abs(trajectory_data[:, 0] - t_current))
    return trajectory_data[nearest_index, [5, 6]]


def create_additional_sine_data(
    seed,
    t_range,
    acc_width_range,
    acc_period_range,
    steer_width_range,
    steer_period_range,
    large_steer_width_range,
    large_steer_period_range,
    start_large_steer_time,
):
    """Create sine wave data to be added randomly to feed-forward runs."""
    np.random.seed(seed=seed)
    t_acc = 0.0
    t_steer = 0.0
    t_large_steer = 0.0
    t_acc_list = []
    t_steer_list = []
    t_large_steer_list = []
    t_acc_list.append(t_acc)
    t_steer_list.append(t_steer)
    t_large_steer_list.append(t_large_steer)
    t_large_steer += start_large_steer_time
    t_large_steer_list.append(t_large_steer)
    width_acc_list = []
    width_steer_list = []
    width_large_steer_list = []
    width_large_steer_list.append(0)
    while True:
        if max(t_acc, t_large_steer) >= t_steer:
            period = (
                steer_period_range[1] - steer_period_range[0]
            ) * np.random.uniform() + steer_period_range[0]
            t_steer += period
            t_steer_list.append(t_steer)
            width_steer_list.append(steer_width_range * np.random.uniform())
        elif t_large_steer >= t_acc:
            period = (
                acc_period_range[1] - acc_period_range[0]
            ) * np.random.uniform() + acc_period_range[0]
            t_acc += period
            t_acc_list.append(t_acc)
            width_acc_list.append(acc_width_range * np.random.uniform())
        else:
            period = (
                large_steer_period_range[1] - large_steer_period_range[0]
            ) * np.random.uniform() + large_steer_period_range[0]
            t_large_steer += period
            t_large_steer_list.append(t_large_steer)
            width_large_steer_list.append(large_steer_width_range * np.random.uniform())
        if t_acc >= t_range[1] and t_steer >= t_range[1] and t_large_steer >= t_range[1]:
            break
    return (
        np.array(t_acc_list),
        np.array(width_acc_list),
        np.array(t_steer_list),
        np.array(width_steer_list),
        np.array(t_large_steer_list),
        np.array(width_large_steer_list),
    )


@njit(cache=False, fastmath=True)
def get_current_additional_sine(
    t,
    t_acc_array,
    width_acc_array,
    t_steer_array,
    width_steer_array,
    t_large_steer_array,
    width_large_steer_array,
):
    """Calculate current values from already created sine wave data."""
    acc_index = 0
    steer_index = 0
    large_steer_index = 0
    for i in range(t_acc_array.shape[0] - 1):
        if t < t_acc_array[i + 1]:
            break
        acc_index += 1
    for i in range(t_steer_array.shape[0] - 1):
        if t < t_steer_array[i + 1]:
            break
        steer_index += 1
    for i in range(t_large_steer_array.shape[0] - 1):
        if t < t_large_steer_array[i + 1]:
            break
        large_steer_index += 1
    acc = width_acc_array[acc_index] * np.sin(
        2
        * np.pi
        * (t - t_acc_array[acc_index])
        / (t_acc_array[acc_index + 1] - t_acc_array[acc_index])
    )
    steer = width_steer_array[steer_index] * np.sin(
        2
        * np.pi
        * (t - t_steer_array[steer_index])
        / (t_steer_array[steer_index + 1] - t_steer_array[steer_index])
    )
    steer += width_large_steer_array[large_steer_index] * np.sin(
        2
        * np.pi
        * (t - t_large_steer_array[large_steer_index])
        / (t_large_steer_array[large_steer_index + 1] - t_large_steer_array[large_steer_index])
    )
    return np.array([acc, steer])


def create_vel_sine_data(seed, t_range, vel_width_range, vel_period_range):
    """Create sine wave data for target velocity."""
    np.random.seed(seed=seed)
    t_vel = 0.0
    t_vel_list = []
    t_vel_list.append(t_vel)
    width_vel_list = []
    while True:
        period = (
            vel_period_range[1] - vel_period_range[0]
        ) * np.random.uniform() + vel_period_range[0]
        t_vel += period
        t_vel_list.append(t_vel)
        width_vel_list.append(vel_width_range * np.random.uniform())
        if t_vel >= t_range[1]:
            break
    return (
        np.array(t_vel_list),
        np.array(width_vel_list),
    )


@njit(cache=False, fastmath=True)
def get_current_vel_sine(t, t_vel_array, width_vel_array, v_mid):
    """Calculate current target velocity values from already created sine wave data."""
    vel_index = 0
    for i in range(t_vel_array.shape[0] - 1):
        if t < t_vel_array[i + 1]:
            break
        vel_index += 1
    vel = v_mid + width_vel_array[vel_index] * np.sin(
        2
        * np.pi
        * (t - t_vel_array[vel_index])
        / (t_vel_array[vel_index + 1] - t_vel_array[vel_index])
    )
    return vel


def get_figure_eight_point(t, circle_radius):
    """
    Get the position and yaw angle in world coordinates of the figure eight given the circle radius.

    Here t is a 1-dimensional array of numpy and each t[i] represents the distance traveled.
    The return value is a 2-dimensional array of positions and a 1-dimensional array of yaw angles corresponding to t.
    """
    sign = -2 * (np.floor(t / (2 * np.pi * circle_radius)) % 2).astype(int) + 1
    x = circle_radius * np.sin(t / circle_radius)
    y = sign * circle_radius * (1 - np.cos(t / circle_radius))
    yaw = (
        sign * ((t / circle_radius) % (2 * np.pi) - np.pi) + np.pi
    )  # if sign == 1, then yaw = (t/circle_radius)%(2*np.pi). Else, yaw = 2*np.pi - (t/circle_radius)%(2*np.pi).
    return np.array([x, y]).T, yaw


def pure_pursuit(
    x_current,
    target_position,
    target_vel,
    target_yaw,
    acc_lim=7.0,
    steer_lim=1.0,
    wheel_base=drive_functions.L,
    lookahead_time=3.0,
    min_lookahead=3.0,
    acc_kp=0.5,
):
    """Calculate acceleration and steer angle input values based on pure pursuit."""
    present_position = x_current[:2]
    present_longitudinal_velocity = x_current[2]
    present_point_yaw = x_current[3]
    longitudinal_vel_err = present_longitudinal_velocity - target_vel
    acc_kp = 0.5
    acc_cmd = np.clip(-acc_kp * longitudinal_vel_err, -acc_lim, acc_lim)

    # compute steer cmd
    cos_yaw = np.cos(target_yaw)
    sin_yaw = np.sin(target_yaw)
    diff_position = present_position - target_position
    lat_err = -sin_yaw * diff_position[0] + cos_yaw * diff_position[1]
    yaw_err = present_point_yaw - target_yaw
    while True:
        if yaw_err > np.pi:
            yaw_err -= 2.0 * np.pi
        if yaw_err < (-np.pi):
            yaw_err += 2.0 * np.pi
        if np.abs(yaw_err) < np.pi:
            break

    lookahead = min_lookahead + lookahead_time * np.abs(present_longitudinal_velocity)
    steer_kp = 2.0 * wheel_base / (lookahead * lookahead)
    steer_kd = 2.0 * wheel_base / lookahead
    steer_cmd = np.clip(-steer_kp * lat_err - steer_kd * yaw_err, -steer_lim, steer_lim)
    return np.array([acc_cmd, steer_cmd])


def get_pure_pursuit_info(x_current, trajectory_position_data, trajectory_yaw_data, previous_index):
    """Calculate the target position and yaw angle required for pure pursuit."""
    search_range = (
        np.arange(
            previous_index - trajectory_position_data.shape[0] // 4,
            previous_index + trajectory_position_data.shape[0] // 4,
        )
        % trajectory_position_data.shape[0]
    )
    nearest_index = np.argmin(
        ((trajectory_position_data[search_range] - x_current[:2].reshape(1, 2)) ** 2).sum(axis=1)
    )
    return (
        trajectory_position_data[search_range[nearest_index]],
        trajectory_yaw_data[search_range[nearest_index]],
        search_range[nearest_index],
    )


class driving_log_updater:
    """Class for updating logs when driving on the Python simulator."""

    def __init__(self):
        self.X_history = []
        self.U_history = []
        self.control_cmd_time_stamp_list = []
        self.control_cmd_steer_list = []
        self.control_cmd_acc_list = []
        self.kinematic_state_list = []
        self.acceleration_list = []
        self.steering_status_list = []
        self.control_cmd_orig_list = []
        self.operation_mode_list = []

    def update(self, t_current, x_current, u_current):
        """Update logs."""
        self.X_history.append(x_current)
        self.U_history.append(u_current)
        self.control_cmd_time_stamp_list.append(t_current)
        self.control_cmd_steer_list.append(u_current[1])
        self.control_cmd_acc_list.append(u_current[0])
        if self.control_cmd_time_stamp_list[-1] - self.control_cmd_time_stamp_list[0] > 3.0:
            self.control_cmd_time_stamp_list.pop(0)
            self.control_cmd_steer_list.pop(0)
            self.control_cmd_acc_list.pop(0)
        t_sec = int(t_current)
        t_n_sec = int(1e9 * (t_current - t_sec))
        kinematic_state = np.zeros(7)
        acceleration = np.zeros(4)
        steering_status = np.zeros(3)
        control_cmd_orig = np.zeros(10)
        operation_mode = np.zeros(3)
        kinematic_state[0] = t_sec
        kinematic_state[1] = t_n_sec
        kinematic_state[2] = x_current[0]
        kinematic_state[3] = x_current[1]
        kinematic_state[4] = np.sin(0.5 * x_current[3])
        kinematic_state[5] = np.cos(0.5 * x_current[3])
        kinematic_state[6] = x_current[2]
        self.kinematic_state_list.append(kinematic_state)
        acceleration[0] = t_sec
        acceleration[1] = t_n_sec
        acceleration[3] = x_current[4]
        self.acceleration_list.append(acceleration)
        steering_status[0] = t_sec
        steering_status[1] = t_n_sec
        steering_status[2] = x_current[5]
        self.steering_status_list.append(steering_status)
        control_cmd_orig[0] = t_sec
        control_cmd_orig[1] = t_n_sec
        control_cmd_orig[4] = u_current[1]
        control_cmd_orig[9] = u_current[0]
        self.control_cmd_orig_list.append(control_cmd_orig)
        operation_mode[0] = t_sec
        operation_mode[1] = t_n_sec
        operation_mode[2] = 2.0
        self.operation_mode_list.append(operation_mode)

    def save(self, save_dir):
        """Save logs in csv format."""
        kinematic_states = np.zeros((len(self.kinematic_state_list), 48))
        kinematic_states[:, [0, 1, 4, 5, 9, 10, 47]] = np.array(self.kinematic_state_list)
        np.savetxt(save_dir + "/kinematic_state.csv", kinematic_states, delimiter=",")
        np.savetxt(save_dir + "/acceleration.csv", np.array(self.acceleration_list), delimiter=",")
        np.savetxt(
            save_dir + "/steering_status.csv",
            np.array(self.steering_status_list),
            delimiter=",",
        )
        np.savetxt(
            save_dir + "/control_cmd_orig.csv",
            np.array(self.control_cmd_orig_list),
            delimiter=",",
        )
        np.savetxt(
            save_dir + "/system_operation_mode_state.csv",
            np.array(self.operation_mode_list),
            delimiter=",",
        )
        with open(save_dir + "/system_operation_mode_state.csv", "w") as f:
            writer = csv.writer(f)
            for i in range(len(self.operation_mode_list)):
                operation_mode_plus_true = self.operation_mode_list[i].tolist()
                operation_mode_plus_true.append("True")
                writer.writerow(operation_mode_plus_true)


def slalom_drive(
    save_file=True,
    save_dir=None,
    load_dir=drive_functions.load_dir,
    visualize=True,
    use_trained_model=False,
    use_trained_model_diff=None,
    control_type="mpc",  # feedforward_test=False,
    straight_line_test=False,
    initial_error=np.zeros(6),
    t_range=[0, 100],
    seed=1,
    acc_width_range=0.005,
    acc_period_range=[5.0, 20.0],
    steer_width_range=0.005,
    steer_period_range=[5.0, 20.0],
    large_steer_width_range=0.00,
    large_steer_period_range=[5.0, 20.0],
    start_large_steer_time=40.0,
    vel_width_range=5.0,
    vel_period_range=[5.0, 20.0],
    v_mid=6.0,
    circle_radius=30.0,
):
    """Perform a slalom driving simulation."""
    if control_type == "pp":
        print("\n[run figure_eight_drive]\n")
    elif control_type == "ff":
        print("\n[run feedforward_drive]\n")
    else:
        if not straight_line_test:
            print("\n[run slalom_drive]\n")
        else:
            print("\n[straight_line_test]\n")
    if save_file:
        if save_dir is None:
            save_dir_ = "python_sim_log_" + str(datetime.datetime.now())
        else:
            save_dir_ = save_dir
        if not os.path.isdir(save_dir_):
            os.mkdir(save_dir_)
    controller = drive_controller.drive_controller(
        model_file_name=(load_dir + "/model_for_test_drive.pth"),
        load_GP_dir=load_dir,
        load_polynomial_reg_dir=load_dir,
        use_trained_model=use_trained_model,
        use_trained_model_diff=use_trained_model_diff,
        load_train_data_dir=load_dir,
    )
    mode = controller.mode
    if control_type == "mpc":
        print("mode:", mode)

    plt.rcParams["figure.figsize"] = (8, 8)
    t_eval = np.arange(*t_range, sim_dt)
    if not straight_line_test:
        trajectory_data = np.loadtxt("slalom_course_data.csv", delimiter=",")
    else:
        # Test by straight_line. To run, the following create_straight_line_test_csv() must be executed
        trajectory_data = np.loadtxt("straight_line.csv", delimiter=",")
    trajectory_interpolator_list = [
        scipy.interpolate.interp1d(trajectory_data[:, 0], trajectory_data[:, 1 + i])
        for i in range(trajectory_data.shape[1] - 1)
    ]
    x_init = trajectory_data[0, 1:7] + initial_error

    x_current = x_init.copy()

    acc_des_queue = [0] * acc_delay_step_sim
    steer_des_queue = [0] * steer_delay_step_sim

    calculated = 0
    break_flag = False

    tracking_error_list = []

    total_abs_max_lateral_deviation = -1
    straight_line_abs_max_lateral_deviation = -1
    total_abs_max_velocity_error = -1
    total_abs_max_yaw_error = -1
    total_abs_max_acc_error = -1
    total_abs_max_steer_error = -1
    prev_u_actual_input = np.zeros(2)
    log_updater = driving_log_updater()

    previous_pp_index = 0

    if control_type != "mpc":  # feedforward_test:
        (
            t_acc_array,
            width_acc_array,
            t_steer_array,
            width_steer_array,
            t_large_steer_array,
            width_large_steer_array,
        ) = create_additional_sine_data(
            seed,
            t_range,
            acc_width_range,
            acc_period_range,
            steer_width_range,
            steer_period_range,
            large_steer_width_range,
            large_steer_period_range,
            start_large_steer_time,
        )
    if control_type == "pp":
        t_figure_eight = np.arange(*[0, 4 * np.pi * circle_radius], 0.01)
        trajectory_position_data, trajectory_yaw_data = get_figure_eight_point(
            t_figure_eight, circle_radius
        )
        # plt.plot(trajectory_position_data[:,0],trajectory_position_data[:,1])
        # plt.show()
        t_vel_array, width_vel_array = create_vel_sine_data(
            seed, t_range, vel_width_range, vel_period_range
        )

    for i in range(t_eval.size):
        if i % ctrl_freq == 0:  # update u_opt
            if control_type == "mpc":  # not feedforward_test:
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
                )
            elif control_type == "ff":
                u_opt = get_current_additional_sine(
                    t_eval[i],
                    t_acc_array,
                    width_acc_array,
                    t_steer_array,
                    width_steer_array,
                    t_large_steer_array,
                    width_large_steer_array,
                )
                u_opt[0] += get_feedforward_nominal_input(t_eval[i], trajectory_data)[0]
            elif control_type == "pp":
                target_position, target_yaw, previous_pp_index = get_pure_pursuit_info(
                    x_current, trajectory_position_data, trajectory_yaw_data, previous_pp_index
                )
                target_vel = get_current_vel_sine(t_eval[i], t_vel_array, width_vel_array, v_mid)
                u_opt = pure_pursuit(x_current, target_position, target_vel, target_yaw)
                u_opt += get_current_additional_sine(
                    t_eval[i],
                    t_acc_array,
                    width_acc_array,
                    t_steer_array,
                    width_steer_array,
                    t_large_steer_array,
                    width_large_steer_array,
                )
            calculated += 1
            log_updater.update(t_eval[i], x_current, u_opt)
        if (
            visualize and (control_type in ["ff", "pp"]) and (i == t_eval.size - 1 or break_flag)
        ):  # feedforward_test and (i == t_eval.size - 1 or break_flag):
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
            fig = plt.figure(figsize=(18, 12), tight_layout=True)
            fig.suptitle(title_str)
            plt.subplot(231)

            ax1 = plt.subplot(2, 3, 1)
            ax1.plot(X[:, 0], X[:, 1], label="trajectory")
            ax1.legend()
            ax1.set_xlabel("x_position [m]")
            ax1.set_ylabel("y_position [m]")

            time_normalize_1 = ctrl_freq * sim_dt
            f_s = int(1.0 / time_normalize_1)

            ax2 = plt.subplot(2, 3, 2)
            X_acc = np.fft.fft(U[:, 0]) / len(U[:, 0])  # Fourier transform of waveforms
            freqs = np.fft.fftfreq(len(U[:, 0])) * f_s
            ax2.stem(
                freqs,
                np.abs(X_acc),
                use_line_collection=True,
                basefmt="k-",
                markerfmt="cx",
                label="acc input",
            )
            ax2.legend()
            ax2.set_xlim([-1.0, 1.0])
            ax2.set_xlabel("freq")
            ax2.set_ylabel("amplitude")

            ax3 = plt.subplot(2, 3, 3)
            X_steer = np.fft.fft(U[:, 1]) / len(U[:, 1])  # Fourier transform of waveforms
            freqs = np.fft.fftfreq(len(U[:, 1])) * f_s
            ax3.stem(
                freqs,
                np.abs(X_steer),
                use_line_collection=True,
                basefmt="k-",
                markerfmt="cx",
                label="steer input",
            )
            ax3.legend()
            ax3.set_xlim([-1.0, 1.0])
            ax3.set_xlabel("freq")
            ax3.set_ylabel("amplitude")

            ax5 = plt.subplot(2, 3, 5)
            ax5.plot(time_normalize_1 * np.arange(U.shape[0]), U[:, 0], label="acc input")
            ax5.legend()

            ax6 = plt.subplot(2, 3, 6)
            ax6.plot(time_normalize_1 * np.arange(U.shape[0]), U[:, 1], label="steer input")
            ax6.legend()

            if save_file:
                png_save_dir = save_dir_
            else:
                png_save_dir = "."
            if control_type == "ff":
                plt.savefig(png_save_dir + "/python_simulator_feedforward_drive.png")
            elif control_type == "pp":
                plt.savefig(png_save_dir + "/python_simulator_pure_pursuit_drive.png")
            plt.close()

        if (
            visualize
            and (control_type == "mpc")
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

            plt.subplot(331)

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
            ax1.plot(X[:, 0], X[:, 1], label="trajectory")
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
            ax2.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                np.zeros(tracking_error_array.shape[0]),
                linestyle="dashed",
            )
            ax2.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                0.05 * np.ones(tracking_error_array.shape[0]),
                linestyle="dashed",
            )
            ax2.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -0.05 * np.ones(tracking_error_array.shape[0]),
                linestyle="dashed",
            )
            ax2.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                0.1 * np.ones(tracking_error_array.shape[0]),
                linestyle="dashed",
            )
            ax2.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -0.1 * np.ones(tracking_error_array.shape[0]),
                linestyle="dashed",
            )
            ax2.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                0.15 * np.ones(tracking_error_array.shape[0]),
                linestyle="dashed",
            )
            ax2.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -0.15 * np.ones(tracking_error_array.shape[0]),
                linestyle="dashed",
            )
            ax2.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                0.2 * np.ones(tracking_error_array.shape[0]),
                linestyle="dashed",
            )
            ax2.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -0.2 * np.ones(tracking_error_array.shape[0]),
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
            ax3.plot(time_normalize_1 * np.arange(X.shape[0]), X[:, 2], label="velocity")
            ax3.plot(
                time_normalize_1 * np.arange(X.shape[0]), X_des_hist[:, 2], label="velocity_target"
            )
            ax3.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -tracking_error_array[:, 2],
                label="velocity_error",
            )
            if mode != "mppi":
                ax3.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(true_prediction.shape[0]),
                    true_prediction[:, 2],
                    s=4,
                    label="velocity_true_prediction",
                )
                ax3.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(controller.nominal_traj_ilqr.shape[0]),
                    controller.nominal_traj_ilqr[:, 2],
                    s=4,
                    label="velocity_prediction_ilqr",
                )
            ax3.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                np.zeros(tracking_error_array.shape[0]),
                linestyle="dashed",
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
            ax4.plot(time_normalize_1 * np.arange(X.shape[0]), X[:, 3], label="yaw")
            ax4.plot(time_normalize_1 * np.arange(X.shape[0]), X_des_hist[:, 3], label="yaw_target")
            ax4.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -tracking_error_array[:, 3],
                label="yaw_error",
            )
            if mode != "mppi":
                ax4.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(true_prediction.shape[0]),
                    true_prediction[:, 3],
                    s=4,
                    label="yaw_true_prediction",
                )
                ax4.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(controller.nominal_traj_ilqr.shape[0]),
                    controller.nominal_traj_ilqr[:, 3],
                    s=4,
                    label="yaw_prediction_ilqr",
                )
            ax4.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                np.zeros(tracking_error_array.shape[0]),
                linestyle="dashed",
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
            ax5.plot(time_normalize_1 * np.arange(X.shape[0]), X[:, 4], label="acc")
            ax5.plot(time_normalize_1 * np.arange(X.shape[0]), U[:, 0], label="acc_input")
            ax5.plot(time_normalize_1 * np.arange(X.shape[0]), X_des_hist[:, 4], label="acc_target")
            ax5.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -tracking_error_array[:, 4],
                label="acc_error",
            )
            if mode != "mppi":
                ax5.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(true_prediction.shape[0]),
                    true_prediction[:, 4],
                    s=4,
                    label="acc_true_prediction",
                )
                ax5.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(controller.nominal_traj_ilqr.shape[0]),
                    controller.nominal_traj_ilqr[:, 4],
                    s=4,
                    label="acc_prediction_ilqr",
                )
                ax5.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(nominal_inputs.shape[0]),
                    nominal_inputs[:, 0],
                    s=4,
                    label="acc_input_schedule",
                )
            ax5.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                np.zeros(tracking_error_array.shape[0]),
                linestyle="dashed",
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
            ax6.plot(time_normalize_1 * np.arange(X.shape[0]), X[:, 5], label="steer")
            ax6.plot(time_normalize_1 * np.arange(X.shape[0]), U[:, 1], label="steer_input")
            ax6.plot(
                time_normalize_1 * np.arange(X.shape[0]), X_des_hist[:, 7], label="steer_mpc_target"
            )
            ax6.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                -tracking_error_array[:, 5],
                label="steer_error",
            )
            if mode != "mppi":
                ax6.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(true_prediction.shape[0]),
                    true_prediction[:, 5],
                    s=4,
                    label="steer_true_prediction",
                )
                ax6.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(controller.nominal_traj_ilqr.shape[0]),
                    controller.nominal_traj_ilqr[:, 5],
                    s=4,
                    label="steer_prediction_ilqr",
                )
                ax6.scatter(
                    time_normalize_1 * (X.shape[0] - 1)
                    + time_normalize_2 * np.arange(nominal_inputs.shape[0]),
                    nominal_inputs[:, 1],
                    s=4,
                    label="steer_input_schedule",
                )
            ax6.plot(
                time_normalize_1 * np.arange(tracking_error_array.shape[0]),
                np.zeros(tracking_error_array.shape[0]),
                linestyle="dashed",
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
                use_line_collection=True,
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
                use_line_collection=True,
                basefmt="k-",
                markerfmt="gx",
                label="lateral_acc",
            )

            X_lateral_jerk = np.fft.fft(lateral_jerk)  # Fourier transform of waveforms
            freqs = np.fft.fftfreq(len(lateral_jerk)) * f_s
            ax7.stem(
                freqs,
                np.abs(X_lateral_jerk) / len(lateral_jerk),
                use_line_collection=True,
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
            controller.send_initialize_input_queue()
            controller.stop_model_update()
            break
    if save_file:
        log_updater.save(save_dir_)

    if control_type == "mpc" and perturbed_sim_flag:
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


def create_straight_line_test_csv(
    jerk=0.3, starting_vel=5.0, interval_1=2.0, interval_2=3.0, interval_3=10.0
):
    """Generate data for a straight line driving test."""
    t_1 = 10.0
    t_2 = t_1 + interval_1  # Increasing acceleration.
    t_3 = t_2 + interval_2  # Constant acceleration
    t_4 = t_3 + interval_1  # Decreasing acceleration.
    t_5 = t_4 + interval_3  # acceleration = 0
    t_6 = t_5 + interval_1  # Decreasing acceleration.
    t_7 = t_6 + interval_2  # Constant acceleration
    t_8 = t_7 + interval_1  # Decreasing acceleration.
    vel_t1 = starting_vel
    x_t1 = starting_vel * t_1
    vel_t2 = vel_t1 + 0.5 * jerk * interval_1 * interval_1
    x_t2 = x_t1 + vel_t1 * interval_1 + 0.5 * jerk * interval_1 * interval_1 * interval_1 / 3
    vel_t3 = vel_t2 + jerk * interval_1 * interval_2
    x_t3 = x_t2 + vel_t2 * interval_2 + 0.5 * jerk * interval_1 * interval_2 * interval_2
    vel_t4 = vel_t3 + 0.5 * jerk * interval_1 * interval_1
    x_t4 = x_t3 + vel_t4 * interval_1 - 0.5 * jerk * interval_1 * interval_1 * interval_1 / 3
    vel_t5 = vel_t4
    x_t5 = x_t4 + vel_t4 * interval_3
    vel_t6 = vel_t5 - 0.5 * jerk * interval_1 * interval_1
    x_t6 = x_t5 + vel_t5 * interval_1 - 0.5 * jerk * interval_1 * interval_1 * interval_1 / 3
    vel_t7 = vel_t6 - jerk * interval_1 * interval_2
    x_t7 = x_t6 + vel_t6 * interval_2 - 0.5 * jerk * interval_1 * interval_2 * interval_2
    vel_t8 = vel_t7 - 0.5 * jerk * interval_1 * interval_1
    x_t8 = x_t7 + vel_t8 * interval_1 + 0.5 * jerk * interval_1 * interval_1 * interval_1 / 3

    def calc_longitudinal_state(t):
        if t < t_1:
            return starting_vel * t, starting_vel, 0.0
        elif t < t_2:
            return (
                x_t1 + vel_t1 * (t - t_1) + 0.5 * jerk * (t - t_1) * (t - t_1) * (t - t_1) / 3,
                vel_t1 + 0.5 * jerk * (t - t_1) * (t - t_1),
                jerk * (t - t_1),
            )
        elif t < t_3:
            return (
                x_t2 + vel_t2 * (t - t_2) + 0.5 * jerk * (t_2 - t_1) * (t - t_2) * (t - t_2),
                vel_t2 + jerk * (t_2 - t_1) * (t - t_2),
                jerk * (t_2 - t_1),
            )
        elif t < t_4:
            return (
                x_t4 - vel_t4 * (t_4 - t) + 0.5 * jerk * (t_4 - t) * (t_4 - t) * (t_4 - t) / 3,
                vel_t4 - 0.5 * jerk * (t_4 - t) * (t_4 - t),
                jerk * (t_4 - t),
            )
        elif t < t_5:
            return x_t4 + vel_t4 * (t - t_4), vel_t4, 0.0
        elif t < t_6:
            return (
                x_t5 + vel_t5 * (t - t_5) - 0.5 * jerk * (t - t_5) * (t - t_5) * (t - t_5) / 3,
                vel_t5 - 0.5 * jerk * (t - t_5) * (t - t_5),
                -jerk * (t - t_5),
            )
        elif t < t_7:
            return (
                x_t6 + vel_t6 * (t - t_6) - 0.5 * jerk * (t_6 - t_5) * (t - t_6) * (t - t_6),
                vel_t6 - jerk * (t_6 - t_5) * (t - t_6),
                -jerk * (t_6 - t_5),
            )
        elif t < t_8:
            return (
                x_t8 - vel_t8 * (t_8 - t) - 0.5 * jerk * (t_8 - t) * (t_8 - t) * (t_8 - t) / 3,
                vel_t8 + 0.5 * jerk * (t_8 - t) * (t_8 - t),
                -jerk * (t_8 - t),
            )
        else:
            return x_t8 + vel_t8 * (t - t_8), vel_t8, 0.0

    t_range = [0, 100]
    time_stamps = np.arange(*t_range, 0.001)
    trajectory_data = np.zeros((time_stamps.shape[0], 9))
    trajectory_data[:, 0] = time_stamps
    for i in range(time_stamps.shape[0]):
        (
            trajectory_data[i, 1],
            trajectory_data[i, 3],
            trajectory_data[i, 5],
        ) = calc_longitudinal_state(time_stamps[i])
    np.savetxt("straight_line.csv", trajectory_data, delimiter=",")
