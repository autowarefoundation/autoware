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

# cspell: ignore numba njit fastmath ndim


"""Define primary parameters and functions to be used elsewhere."""
from functools import partial
import os
from pathlib import Path
import sys
from typing import Callable

from ament_index_python.packages import get_package_share_directory
from numba import njit
import numpy as np
import yaml

PACKAGE_NAME = "autoware_smart_mpc_trajectory_follower"

param_path = Path(get_package_share_directory(PACKAGE_NAME)) / "param"

mpc_param_path = param_path / "mpc_param.yaml"
with open(mpc_param_path, "r") as yml:
    mpc_param = yaml.safe_load(yml)

mpc_freq = int(
    mpc_param["mpc_parameter"]["mpc_setting"]["mpc_freq"]
)  # Int, mpc horizon interval/control interval

# information on MPC
ctrl_time_step = float(
    mpc_param["mpc_parameter"]["mpc_setting"]["ctrl_time_step"]
)  # Time interval of control.
mpc_time_step = mpc_freq * ctrl_time_step  # MPC Horizon interval.
N = int(mpc_param["mpc_parameter"]["mpc_setting"]["N"])  # Length of MPC Horizon.
steer_ctrl_queue_size = int(
    mpc_param["mpc_parameter"]["mpc_setting"]["steer_ctrl_queue_size"]
)  # Queue for training and control
steer_ctrl_queue_size_core = int(
    mpc_param["mpc_parameter"]["mpc_setting"]["steer_ctrl_queue_size_core"]
)
acc_ctrl_queue_size = int(mpc_param["mpc_parameter"]["mpc_setting"]["acc_ctrl_queue_size"])
nx_0 = int(mpc_param["mpc_parameter"]["mpc_setting"]["nx_0"])  # dimension of the state
nu_0 = int(mpc_param["mpc_parameter"]["mpc_setting"]["nu_0"])  # dimension of the input

#################################################################

# Control parameters that may be changed in the experiment##

#################################################################
mode = str(mpc_param["mpc_parameter"]["system"]["mode"])
# cost function weights
Q = np.diag(np.array(mpc_param["mpc_parameter"]["cost_parameters"]["Q"], dtype=float))  # stage cost
Q_c = np.diag(
    np.array(mpc_param["mpc_parameter"]["cost_parameters"]["Q_c"], dtype=float)
)  # Stage costs in MPC `timing_Q_c`th horizon.
Q_f = np.diag(
    np.array(mpc_param["mpc_parameter"]["cost_parameters"]["Q_f"], dtype=float)
)  # terminal cost
R = np.diag(
    np.array(mpc_param["mpc_parameter"]["cost_parameters"]["R"], dtype=float)
)  # Cost for control inputs. Here acceleration rate of change and steer rate of change.
timing_Q_c = np.array(
    mpc_param["mpc_parameter"]["mpc_setting"]["timing_Q_c"], dtype=int
)  # Horizon numbers to change stage costs.
reference_horizon = int(mpc_param["mpc_parameter"]["preprocessing"]["reference_horizon"])  # 50

acc_lim_weight = float(mpc_param["mpc_parameter"]["cost_parameters"]["acc_lim_weight"])
steer_lim_weight = float(mpc_param["mpc_parameter"]["cost_parameters"]["steer_lim_weight"])
acc_rate_lim_weight = float(mpc_param["mpc_parameter"]["cost_parameters"]["acc_rate_lim_weight"])
steer_rate_lim_weight = float(
    mpc_param["mpc_parameter"]["cost_parameters"]["steer_rate_lim_weight"]
)

# iLQR parameters
ls_step = float(mpc_param["mpc_parameter"]["ilqr"]["ls_step"])  # 0.9
max_iter_ls = int(mpc_param["mpc_parameter"]["ilqr"]["max_iter_ls"])  # 10
max_iter_ilqr = int(mpc_param["mpc_parameter"]["ilqr"]["max_iter_ilqr"])  # 1
ilqr_tol = float(mpc_param["mpc_parameter"]["ilqr"]["ilqr_tol"])  # 0.01

# MPPI parameters
lam = float(mpc_param["mpc_parameter"]["mppi"]["lam"])  # 0.1
Sigma = np.array(mpc_param["mpc_parameter"]["mppi"]["Sigma"], dtype=float)
max_iter_mppi = int(mpc_param["mpc_parameter"]["mppi"]["max_iter_mppi"])
sample_num = int(mpc_param["mpc_parameter"]["mppi"]["sample_num"])
mppi_tol = float(mpc_param["mpc_parameter"]["mppi"]["mppi_tol"])
mppi_step = int(mpc_param["mpc_parameter"]["mppi"]["mppi_step"])

cap_pred_error = np.array(mpc_param["mpc_parameter"]["preprocessing"]["cap_pred_error"])

nominal_param_path = param_path / "nominal_param.yaml"
with open(nominal_param_path, "r") as yml:
    nominal_param = yaml.safe_load(yml)
# Vehicle body information given by default.
L = float(nominal_param["nominal_parameter"]["vehicle_info"]["wheel_base"])  # Length of vehicle [m]
steer_dead_band_for_ctrl = 0.00  # Size of the steering dead band zone given to the control side.

acc_time_delay = float(nominal_param["nominal_parameter"]["acceleration"]["acc_time_delay"])
acc_delay_step = min(round(acc_time_delay / ctrl_time_step), acc_ctrl_queue_size - mpc_freq)
acc_time_constant = float(nominal_param["nominal_parameter"]["acceleration"]["acc_time_constant"])

steer_time_delay = float(nominal_param["nominal_parameter"]["steering"]["steer_time_delay"])
steer_delay_step = min(round(steer_time_delay / ctrl_time_step), steer_ctrl_queue_size - mpc_freq)
steer_time_constant = float(nominal_param["nominal_parameter"]["steering"]["steer_time_constant"])


vel_steer_cost_coef_table = np.array(
    mpc_param["mpc_parameter"]["cost_parameters"]["vel_steer_cost_coef_table"], dtype=float
)
vel_steer_table = np.array(
    mpc_param["mpc_parameter"]["cost_parameters"]["vel_steer_table"], dtype=float
)


lateral_cost_coef_table = np.array(
    mpc_param["mpc_parameter"]["cost_parameters"]["lateral_cost_coef_table"], dtype=float
)
lateral_error_table = np.array(
    mpc_param["mpc_parameter"]["cost_parameters"]["lateral_error_table"], dtype=float
)
yaw_cost_coef_table = np.array(
    mpc_param["mpc_parameter"]["cost_parameters"]["yaw_cost_coef_table"], dtype=float
)
yaw_error_table = np.array(
    mpc_param["mpc_parameter"]["cost_parameters"]["yaw_error_table"], dtype=float
)


steer_rate_cost_table = np.array(
    mpc_param["mpc_parameter"]["cost_parameters"]["steer_rate_cost_table"], dtype=float
)
curvature_table = np.array(
    mpc_param["mpc_parameter"]["cost_parameters"]["curvature_table"], dtype=float
)

use_max_curvature = str(mpc_param["mpc_parameter"]["cost_parameters"]["use_max_curvature"])

use_sg_for_nominal_inputs = bool(
    mpc_param["mpc_parameter"]["preprocessing"]["use_sg_for_nominal_inputs"]
)
sg_deg_for_nominal_inputs = int(
    mpc_param["mpc_parameter"]["preprocessing"]["sg_deg_for_nominal_inputs"]
)
sg_window_size_for_nominal_inputs = int(
    mpc_param["mpc_parameter"]["preprocessing"]["sg_window_size_for_nominal_inputs"]
)

acc_fb_decay = float(mpc_param["mpc_parameter"]["compensation"]["acc_fb_decay"])
acc_fb_gain = float(mpc_param["mpc_parameter"]["compensation"]["acc_fb_gain"])
acc_fb_sec_order_ratio = float(mpc_param["mpc_parameter"]["compensation"]["acc_fb_sec_order_ratio"])
max_error_acc = float(mpc_param["mpc_parameter"]["compensation"]["max_error_acc"])

steer_fb_decay = float(mpc_param["mpc_parameter"]["compensation"]["steer_fb_decay"])
steer_fb_gain = float(mpc_param["mpc_parameter"]["compensation"]["steer_fb_gain"])
steer_fb_sec_order_ratio = float(
    mpc_param["mpc_parameter"]["compensation"]["steer_fb_sec_order_ratio"]
)
max_error_steer = float(mpc_param["mpc_parameter"]["compensation"]["max_error_steer"])

trained_model_param_path = param_path / "trained_model_param.yaml"
with open(trained_model_param_path, "r") as yml:
    trained_model_param = yaml.safe_load(yml)
use_trained_model_diff = bool(
    trained_model_param["trained_model_parameter"]["control_application"]["use_trained_model_diff"]
)
minimum_steer_diff = float(
    trained_model_param["trained_model_parameter"]["control_application"]["minimum_steer_diff"]
)
use_memory_diff = bool(
    trained_model_param["trained_model_parameter"]["memory_for_training"]["use_memory_diff"]
)

reflect_only_poly_diff = bool(
    trained_model_param["trained_model_parameter"]["control_application"]["reflect_only_poly_diff"]
)
use_sg_for_trained_model_diff = bool(
    trained_model_param["trained_model_parameter"]["control_application"][
        "use_sg_for_trained_model_diff"
    ]
)
sg_deg_for_trained_model_diff = int(
    trained_model_param["trained_model_parameter"]["control_application"][
        "sg_deg_for_trained_model_diff"
    ]
)
sg_window_size_for_trained_model_diff = int(
    trained_model_param["trained_model_parameter"]["control_application"][
        "sg_window_size_for_trained_model_diff"
    ]
)

use_sg_for_memory_diff = bool(
    trained_model_param["trained_model_parameter"]["memory_for_training"]["use_sg_for_memory_diff"]
)
sg_deg_for_trained_model_diff = int(
    trained_model_param["trained_model_parameter"]["memory_for_training"]["sg_deg_for_memory_diff"]
)
sg_window_size_for_memory_diff = int(
    trained_model_param["trained_model_parameter"]["memory_for_training"][
        "sg_window_size_for_memory_diff"
    ]
)

use_sg_for_noise = bool(
    trained_model_param["trained_model_parameter"]["control_application"]["use_sg_for_noise"]
)
sg_deg_for_noise = int(
    trained_model_param["trained_model_parameter"]["control_application"]["sg_deg_for_noise"]
)
sg_window_size_for_noise = int(
    trained_model_param["trained_model_parameter"]["control_application"][
        "sg_window_size_for_noise"
    ]
)

use_memory_for_training = bool(
    trained_model_param["trained_model_parameter"]["memory_for_training"]["use_memory_for_training"]
)

load_dir = os.environ["HOME"]
save_dir = os.environ["HOME"]  # +"/autoware"
use_trained_model = bool(
    trained_model_param["trained_model_parameter"]["control_application"]["use_trained_model"]
)
update_trained_model = bool(
    trained_model_param["trained_model_parameter"]["control_application"]["update_trained_model"]
)
max_train_data_size = int(
    trained_model_param["trained_model_parameter"]["control_application"]["max_train_data_size"]
)

error_decay = np.array(
    trained_model_param["trained_model_parameter"]["control_application"]["error_decay"],
    dtype=float,
)

Error_decay = error_decay[:6]

use_x_noise = bool(
    trained_model_param["trained_model_parameter"]["control_application"]["use_x_noise"]
)
use_y_noise = bool(
    trained_model_param["trained_model_parameter"]["control_application"]["use_y_noise"]
)
use_v_noise = bool(
    trained_model_param["trained_model_parameter"]["control_application"]["use_v_noise"]
)
use_theta_noise = bool(
    trained_model_param["trained_model_parameter"]["control_application"]["use_theta_noise"]
)
use_acc_noise = bool(
    trained_model_param["trained_model_parameter"]["control_application"]["use_acc_noise"]
)
use_steer_noise = bool(
    trained_model_param["trained_model_parameter"]["control_application"]["use_steer_noise"]
)

# smoothing parameter in training
acc_sigma_for_learning = float(
    trained_model_param["trained_model_parameter"]["smoothing"]["acc_sigma_for_learning"]
)
steer_sigma_for_learning = float(
    trained_model_param["trained_model_parameter"]["smoothing"]["steer_sigma_for_learning"]
)
acc_des_sigma_for_learning = float(
    trained_model_param["trained_model_parameter"]["smoothing"]["acc_des_sigma_for_learning"]
)
steer_des_sigma_for_learning = float(
    trained_model_param["trained_model_parameter"]["smoothing"]["steer_des_sigma_for_learning"]
)

x_out_sigma_for_learning = float(
    trained_model_param["trained_model_parameter"]["smoothing"]["x_out_sigma_for_learning"]
)
y_out_sigma_for_learning = float(
    trained_model_param["trained_model_parameter"]["smoothing"]["y_out_sigma_for_learning"]
)
v_out_sigma_for_learning = float(
    trained_model_param["trained_model_parameter"]["smoothing"]["v_out_sigma_for_learning"]
)
theta_out_sigma_for_learning = float(
    trained_model_param["trained_model_parameter"]["smoothing"]["theta_out_sigma_for_learning"]
)
acc_out_sigma_for_learning = float(
    trained_model_param["trained_model_parameter"]["smoothing"]["acc_out_sigma_for_learning"]
)
steer_out_sigma_for_learning = float(
    trained_model_param["trained_model_parameter"]["smoothing"]["steer_out_sigma_for_learning"]
)


vel_normalize = float(trained_model_param["trained_model_parameter"]["normalize"]["vel_normalize"])
acc_normalize = float(trained_model_param["trained_model_parameter"]["normalize"]["acc_normalize"])
steer_normalize = float(
    trained_model_param["trained_model_parameter"]["normalize"]["steer_normalize"]
)

NN_x_weight = float(trained_model_param["trained_model_parameter"]["weight"]["NN_x_weight"])
NN_y_weight = float(trained_model_param["trained_model_parameter"]["weight"]["NN_y_weight"])
NN_v_weight = float(trained_model_param["trained_model_parameter"]["weight"]["NN_v_weight"])
NN_yaw_weight = float(trained_model_param["trained_model_parameter"]["weight"]["NN_yaw_weight"])
NN_acc_weight = float(trained_model_param["trained_model_parameter"]["weight"]["NN_acc_weight"])
NN_steer_weight = float(trained_model_param["trained_model_parameter"]["weight"]["NN_steer_weight"])

NN_x_weight_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_x_weight_diff"]
)
NN_y_weight_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_y_weight_diff"]
)
NN_v_weight_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_v_weight_diff"]
)
NN_yaw_weight_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_yaw_weight_diff"]
)
NN_acc_weight_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_acc_weight_diff"]
)
NN_steer_weight_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_steer_weight_diff"]
)

NN_x_weight_two_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_x_weight_two_diff"]
)
NN_y_weight_two_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_y_weight_two_diff"]
)
NN_v_weight_two_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_v_weight_two_diff"]
)
NN_yaw_weight_two_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_yaw_weight_two_diff"]
)
NN_acc_weight_two_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_acc_weight_two_diff"]
)
NN_steer_weight_two_diff = float(
    trained_model_param["trained_model_parameter"]["weight"]["NN_steer_weight_two_diff"]
)

finalize_x_weight = float(
    trained_model_param["trained_model_parameter"]["weight"]["finalize_x_weight"]
)
finalize_y_weight = float(
    trained_model_param["trained_model_parameter"]["weight"]["finalize_y_weight"]
)
finalize_v_weight = float(
    trained_model_param["trained_model_parameter"]["weight"]["finalize_v_weight"]
)
# upper limit of input

read_limit_file = bool(mpc_param["mpc_parameter"]["limit"]["read_limit_file"])
if read_limit_file:
    limitter_package_path = Path(get_package_share_directory("autoware_vehicle_cmd_gate"))
    limit_yaml_path = limitter_package_path / "config" / "vehicle_cmd_gate.param.yaml"
    with open(limit_yaml_path, "r") as yml:
        limit_params = yaml.safe_load(yml)

    if limit_params is not None:
        reference_speed_points = np.array(
            limit_params["/**"]["ros__parameters"]["nominal"]["reference_speed_points"]
        )
        steer_lim_points = np.array(limit_params["/**"]["ros__parameters"]["nominal"]["steer_lim"])
        steer_rate_lim_points = np.array(
            limit_params["/**"]["ros__parameters"]["nominal"]["steer_rate_lim"]
        )
        acc_lim_points = np.array(limit_params["/**"]["ros__parameters"]["nominal"]["lon_acc_lim"])
        acc_rate_lim_points = np.array(
            limit_params["/**"]["ros__parameters"]["nominal"]["lon_jerk_lim"]
        )
        lat_acc_lim_points = np.array(
            limit_params["/**"]["ros__parameters"]["nominal"]["lat_acc_lim"]
        )
        lat_jerk_lim_points = np.array(
            limit_params["/**"]["ros__parameters"]["nominal"]["lat_jerk_lim"]
        )
    else:
        print("Error: limit_params is None")
        sys.exit(1)
else:
    reference_speed_points = np.array([20.0, 30.0])
    steer_lim_points = np.array([10.0, 10.0])
    steer_rate_lim_points = np.array([10.0, 10.0])
    acc_lim_points = np.array([20.0, 20.0])
    acc_rate_lim_points = np.array([20.0, 20.0])
    lat_acc_lim_points = np.array([20.0, 20.0])
    lat_jerk_lim_points = np.array([20.0, 20.0])


@njit(cache=True, fastmath=True)
def calc_limits(
    vel: float,
    acc: float,
    steer: float,
    reference_speed_points=reference_speed_points,
    steer_lim_points=steer_lim_points,
    steer_rate_lim_points=steer_rate_lim_points,
    acc_lim_points=acc_lim_points,
    lat_acc_lim_points=lat_acc_lim_points,
    lat_jerk_lim_points=lat_jerk_lim_points,
) -> tuple[float, float, float, float, float]:
    """Calculate the constraints according to the current velocity, acceleration and steer.

    Returns:
        float: upper and lower limit of steer
        float: upper limit of steer change rate
        float: lower limit of steer change rate
        float: upper and lower limit of acceleration
        float: upper and lower limit of acceleration change rate

    However, we write the upper limit of the "upper and lower limits". lower limit is equal to `(-1) * upper limit`.
    """
    interval_index = 0
    for i in range(reference_speed_points.shape[0]):
        if np.abs(vel) > reference_speed_points[i]:
            interval_index += 1
        else:
            break
    if interval_index == 0:
        steer_lim = steer_lim_points[0]
        steer_rate_lim = steer_rate_lim_points[0]
        acc_lim = acc_lim_points[0]
        acc_rate_lim = acc_rate_lim_points[0]
        lat_acc_lim = lat_acc_lim_points[0]
        lat_jerk_lim = lat_jerk_lim_points[0]
    elif interval_index == reference_speed_points.shape[0]:
        steer_lim = steer_lim_points[-1]
        steer_rate_lim = steer_rate_lim_points[-1]
        acc_lim = acc_lim_points[-1]
        acc_rate_lim = acc_rate_lim_points[-1]
        lat_acc_lim = lat_acc_lim_points[-1]
        lat_jerk_lim = lat_jerk_lim_points[-1]
    else:
        r = (np.abs(vel) - reference_speed_points[interval_index - 1]) / (
            reference_speed_points[interval_index] - reference_speed_points[interval_index - 1]
        )
        steer_lim = (1 - r) * steer_lim_points[interval_index - 1] + r * steer_lim_points[
            interval_index
        ]
        steer_rate_lim = (1 - r) * steer_rate_lim_points[
            interval_index - 1
        ] + r * steer_rate_lim_points[interval_index]
        acc_lim = (1 - r) * acc_lim_points[interval_index - 1] + r * acc_lim_points[interval_index]
        acc_rate_lim = (1 - r) * acc_rate_lim_points[interval_index - 1] + r * acc_rate_lim_points[
            interval_index
        ]
        lat_acc_lim = (1 - r) * lat_acc_lim_points[interval_index - 1] + r * lat_acc_lim_points[
            interval_index
        ]
        lat_jerk_lim = (1 - r) * lat_jerk_lim_points[interval_index - 1] + r * lat_jerk_lim_points[
            interval_index
        ]
    lat_jerk_lim_coef = L * np.cos(steer) * np.cos(steer) / (vel * vel + 1e-16)
    lat_jerk_lim_const = vel * acc * np.sin(2 * steer) / (vel * vel + 1e-16)
    steer_rate_lat_jerk_lim_lb = np.array(
        [-lat_jerk_lim_coef * lat_jerk_lim - lat_jerk_lim_const, -0.01]
    ).min()
    steer_rate_lat_jerk_lim_ub = np.array(
        [lat_jerk_lim_coef * lat_jerk_lim - lat_jerk_lim_const, 0.01]
    ).max()

    steer_rate_lb = np.array([-steer_rate_lim, steer_rate_lat_jerk_lim_lb]).max()
    steer_rate_ub = np.array([steer_rate_lim, steer_rate_lat_jerk_lim_ub]).min()

    return (
        np.array([steer_lim, np.arctan(lat_acc_lim * L / (vel * vel + 1e-16))]).min(),
        steer_rate_lb,
        steer_rate_ub,
        acc_lim,
        acc_rate_lim,
    )


@njit(cache=True, fastmath=True)
def calc_steer_rate_cost_coef(curvature: float) -> float:
    """Calc steer rate cost coefficient."""
    interval_index = 0
    for i in range(curvature_table.shape[0]):
        if curvature > curvature_table[i]:
            interval_index += 1
        else:
            break
    if interval_index == 0:
        steer_rate_cost_coef = steer_rate_cost_table[0]

    elif interval_index == steer_rate_cost_table.shape[0]:
        steer_rate_cost_coef = steer_rate_cost_table[-1]

    else:
        r = (curvature - curvature_table[interval_index - 1]) / (
            curvature_table[interval_index] - curvature_table[interval_index - 1]
        )
        steer_rate_cost_coef = (1 - r) * steer_rate_cost_table[
            interval_index - 1
        ] + r * steer_rate_cost_table[interval_index]

    return steer_rate_cost_coef


@njit(cache=True, fastmath=True)
def calc_table_value(x: float, table_domain, table_target) -> float:
    interval_index = 0
    for i in range(table_domain.shape[0]):
        if x > table_domain[i]:
            interval_index += 1
        else:
            break
    if interval_index == 0:
        target_val = table_target[0]

    elif interval_index == table_target.shape[0]:
        target_val = table_target[-1]

    else:
        r = (x - table_domain[interval_index - 1]) / (
            table_domain[interval_index] - table_domain[interval_index - 1]
        )
        target_val = (1 - r) * table_target[interval_index - 1] + r * table_target[interval_index]

    return target_val


@njit(cache=True, fastmath=True)
def transform_yaw(yaw_old: float, yaw_current_: float) -> float:
    """Transform the yaw angle so that the difference in yaw angle is less than or equal to π."""
    rotate_num = (yaw_current_ - yaw_old) // (2 * np.pi)
    if yaw_current_ - yaw_old - 2 * rotate_num * np.pi < np.pi:
        return yaw_current_ - 2 * rotate_num * np.pi
    else:
        return yaw_current_ - 2 * (rotate_num + 1) * np.pi


@njit(cache=True, fastmath=True)
def transform_yaw_for_x_current(x_old: np.ndarray, x_current_: np.ndarray) -> np.ndarray:
    """Apply `transform_yaw` to x_current_ and return the result."""
    x_current = x_current_.copy()
    x_current[3] = transform_yaw(x_old[3], x_current_[3])
    return x_current


def transform_yaw_for_X_des(x_current: np.ndarray, X_des_: np.ndarray) -> tuple[np.ndarray, float]:
    """Transform the yaw angle with respect to the target trajectory.

    X_des[0] is set to the current state.
    """
    X_des = X_des_.copy()

    initial_error = X_des[0, :3] - x_current[:3]

    len_journey = max(mpc_time_step * X_des[:, 2].sum(), 5.0)
    theta_des = X_des[0, 3]
    diff_position = -initial_error[:2]
    signed_lat_err = -np.sin(theta_des) * diff_position[0] + np.cos(theta_des) * diff_position[1]

    X_des[: reference_horizon + 1, :3] -= (
        np.arange(reference_horizon + 1)[::-1].reshape(-1, 1)
        @ initial_error.reshape(1, -1)
        / reference_horizon
    )
    X_des[0, 3] = transform_yaw(x_current[3], X_des[0, 3])
    yaw_initial_error = X_des[0, 3] - x_current[3]
    X_des[0, 3] -= yaw_initial_error

    diff_theta = -yaw_initial_error + np.arctan(signed_lat_err / len_journey)
    diff_delta = L * diff_theta / len_journey

    for i in range(X_des.shape[0] - 1):
        if i < reference_horizon:
            X_des[i + 1, 3] = (
                transform_yaw(X_des[i, 3], X_des[i + 1, 3])
                - yaw_initial_error * (reference_horizon - i - 1) / reference_horizon
            )
        else:
            X_des[i + 1, 3] = transform_yaw(X_des[i, 3], X_des[i + 1, 3])

    return X_des, diff_delta


@njit(cache=True, fastmath=True)
def U_des_from_X_des(u_opt: np.ndarray, X_des: np.ndarray, diff_delta: float) -> np.ndarray:
    """Calculate the initial value of the MPC from the target trajectory.

    This is used for the first control or when the error calculated by calc_maximum_trajectory_error is large.
    """
    U_des = np.zeros((X_des.shape[0] - 1, nu_0))
    for i in range(U_des.shape[0]):
        U_des[i] = (X_des[i + 1, 6:8] - X_des[i, 6:8]) / mpc_time_step
    init_error = X_des[0, 6:8] - u_opt
    U_des = U_des + init_error / ((X_des.shape[0] - 1) * mpc_time_step) - diff_delta
    return U_des


@njit(cache=True, fastmath=True)
def calc_maximum_trajectory_error(traj: np.ndarray, X_des: np.ndarray) -> float:
    """Calculate the maximum trajectory error.

    It calculates
    * predicted trajectory produced by the MPC,
    * the lateral error of the target trajectory
    * the maximum value of the yaw error.
    """
    err = 0.0
    for i in range(traj.shape[0]):
        theta_des = X_des[i, 3]
        diff_position = traj[i, 0:2] - X_des[i, 0:2]
        lat_err = np.abs(
            -np.sin(theta_des) * diff_position[0] + np.cos(theta_des) * diff_position[1]
        )
        err = max(err, lat_err)
        err = max(err, np.abs(theta_des - traj[i, 3]))
    return err


@njit(cache=False, fastmath=True)
def transform_Q_R(
    X_des: np.ndarray,
    U_des: np.ndarray,
    nominal_traj: np.ndarray,
    nominal_input: np.ndarray,
    steer_rate_coef: float,
) -> tuple:
    """Calculate the MPC cost weight matrix from the current predicted trajectory and target trajectory.

    This calculates the information to add the constraints to the MPC cost.
    """
    Q_total = np.zeros((X_des.shape[0], Q.shape[0], Q.shape[1]))
    R_total = np.zeros((U_des.shape[0], R.shape[0], R.shape[1]))
    acc_lim_weights = np.zeros(X_des.shape[0])
    acc_lim_center = np.zeros(X_des.shape[0])
    steer_lim_weights = np.zeros(X_des.shape[0])
    steer_lim_center = np.zeros(X_des.shape[0])
    acc_rate_lim_weights = np.zeros(U_des.shape[0])
    acc_rate_lim_center = np.zeros(U_des.shape[0])
    steer_rate_lim_weights = np.zeros(U_des.shape[0])
    steer_rate_lim_center = np.zeros(U_des.shape[0])
    for i in range(X_des.shape[0]):
        if i == X_des.shape[0] - 1:
            Q_ = Q_f.copy()
        elif i in timing_Q_c:
            Q_ = Q_c.copy()
        else:
            Q_ = Q.copy()
        x_current = nominal_traj[i]
        v = x_current[2]
        acc = X_des[i, 4]
        theta = X_des[i, 3]

        cos = np.cos(theta)
        sin = np.sin(theta)
        Rot = np.array([[cos, -sin], [sin, cos]])

        lateral_error = np.abs((Rot.T @ (X_des[i, :2] - x_current[:2]))[1])
        yaw_error = np.abs(X_des[i, 3] - x_current[3])

        cost_tr_steer_rate = 1 / calc_table_value(
            np.abs(v), vel_steer_table, vel_steer_cost_coef_table
        )

        if i in timing_Q_c or i == X_des.shape[0] - 1:
            lateral_cost_coef = calc_table_value(
                lateral_error, lateral_error_table, lateral_cost_coef_table
            )
            Q_[1, 1] = lateral_cost_coef * Q_[1, 1]
            yaw_cost_coef = calc_table_value(yaw_error, yaw_error_table, yaw_cost_coef_table)
            Q_[3, 3] = yaw_cost_coef * Q_[3, 3]

        Q_[:2, :2] = Rot @ Q_[:2, :2] @ Rot.T
        Q_total[i] = Q_

        steer_lim, steer_rate_lim_lb, steer_rate_lim_ub, acc_lim, acc_rate_lim = calc_limits(
            v, x_current[4], x_current[5]
        )

        if x_current[2] <= 0.0 and x_current[nx_0] <= 0.0:
            acc_lim_weights[i] = acc_lim_weight
            acc_lim_center[i] = 0.0
        else:
            if x_current[nx_0] > acc_lim:  # acc inputs
                acc_lim_weights[i] = acc_lim_weight
                acc_lim_center[i] = acc_lim
            elif x_current[nx_0] < -acc_lim:
                acc_lim_weights[i] = acc_lim_weight
                acc_lim_center[i] = -acc_lim

        if x_current[nx_0 + acc_ctrl_queue_size] > steer_lim:  # steer inputs
            steer_lim_weights[i] = steer_lim_weight
            steer_lim_center[i] = steer_lim
        elif x_current[nx_0 + acc_ctrl_queue_size] < -steer_lim:
            steer_lim_weights[i] = steer_lim_weight
            steer_lim_center[i] = -steer_lim
        if i != X_des.shape[0] - 1:
            R_ = R.copy()
            R_[1, 1] = steer_rate_coef * R_[1, 1]
            if acc >= 0:
                R_[1, 1] = cost_tr_steer_rate * R_[1, 1]
            R_total[i] = R_

            if nominal_input[i, 0] > acc_rate_lim:
                acc_rate_lim_weights[i] = acc_rate_lim_weight
                acc_rate_lim_center[i] = acc_rate_lim
            elif nominal_input[i, 0] < -acc_rate_lim:
                acc_rate_lim_weights[i] = acc_rate_lim_weight
                acc_rate_lim_center[i] = -acc_rate_lim

            if nominal_input[i, 1] > steer_rate_lim_ub:
                steer_rate_lim_weights[i] = steer_rate_lim_weight
                steer_rate_lim_center[i] = steer_rate_lim_ub
            elif nominal_input[i, 1] < steer_rate_lim_lb:
                steer_rate_lim_weights[i] = steer_rate_lim_weight
                steer_rate_lim_center[i] = steer_rate_lim_lb

    return (
        Q_total,
        R_total,
        acc_lim_weights,
        acc_lim_center,
        steer_lim_weights,
        steer_lim_center,
        acc_rate_lim_weights,
        acc_rate_lim_center,
        steer_rate_lim_weights,
        steer_rate_lim_center,
    )


@njit(cache=False, fastmath=True)
def calc_cost(
    X_des: np.ndarray,
    U_des: np.ndarray,
    Traj: np.ndarray,
    Inputs: np.ndarray,
    n: int,
    steer_rate_coef: float,
) -> np.ndarray:
    """Calculate the MPC cost of the nth horizon from several candidate predicted trajectories and target trajectories."""
    Cost = np.zeros(Traj.shape[0])
    if n == N:
        Q_ = Q_f.copy()
    elif n in timing_Q_c:
        Q_ = Q_c.copy()
    else:
        Q_ = Q.copy()
    acc = X_des[4]
    theta = X_des[3]
    cos = np.cos(theta)
    sin = np.sin(theta)
    Rot = np.array([[cos, -sin], [sin, cos]])
    for i in range(Traj.shape[0]):
        x_current = Traj[i]
        v = x_current[2]

        Qi = Q_.copy()
        lateral_error = np.abs((Rot.T @ (X_des[:2] - x_current[:2]))[1])
        yaw_error = np.abs(X_des[3] - x_current[3])

        cost_tr_steer_rate = 1 / calc_table_value(
            np.abs(v), vel_steer_table, vel_steer_cost_coef_table
        )

        if n in timing_Q_c or n == N:
            lateral_cost_coef = calc_table_value(
                lateral_error, lateral_error_table, lateral_cost_coef_table
            )
            Qi[1, 1] = lateral_cost_coef * Qi[1, 1]
            yaw_cost_coef = calc_table_value(yaw_error, yaw_error_table, yaw_cost_coef_table)
            Qi[3, 3] = yaw_cost_coef * Qi[3, 3]
        Qi[:2, :2] = Rot @ Qi[:2, :2] @ Rot.T
        Cost[i] += 0.5 * np.dot(Qi @ (x_current - X_des), x_current - X_des)

        steer_lim, steer_rate_lim_lb, steer_rate_lim_ub, acc_lim, acc_rate_lim = calc_limits(
            v, x_current[4], x_current[5]
        )
        if x_current[2] <= 0.0 and x_current[nx_0] <= 0.0:
            Cost[i] += 0.5 * acc_lim_weight * (x_current[nx_0] - 0.0) * (x_current[nx_0] - 0.0)

        else:
            if x_current[nx_0] > acc_lim:
                Cost[i] += (
                    0.5 * acc_lim_weight * (x_current[nx_0] - acc_lim) * (x_current[nx_0] - acc_lim)
                )
            elif x_current[nx_0] < -acc_lim:
                Cost[i] += (
                    0.5 * acc_lim_weight * (x_current[nx_0] + acc_lim) * (x_current[nx_0] + acc_lim)
                )

        if x_current[nx_0 + 1] > steer_lim:
            Cost[i] += (
                0.5
                * steer_lim_weight
                * (x_current[nx_0 + 1] - steer_lim)
                * (x_current[nx_0 + 1] - steer_lim)
            )

        elif x_current[nx_0 + 1] < -steer_lim:
            Cost[i] += (
                0.5
                * steer_lim_weight
                * (x_current[nx_0 + 1] + steer_lim)
                * (x_current[nx_0 + 1] + steer_lim)
            )

        if n != N:
            R_ = R.copy()
            R_[1, 1] = steer_rate_coef * R_[1, 1]

            if acc >= 0:
                R_[1, 1] = cost_tr_steer_rate * R_[1, 1]

            Cost[i] += 0.5 * np.dot(R_ @ (Inputs[i] - U_des), Inputs[i] - U_des)

            if Inputs[i, 0] > acc_rate_lim:
                Cost[i] += (
                    0.5
                    * acc_rate_lim_weight
                    * (Inputs[i, 0] - acc_rate_lim)
                    * (Inputs[i, 0] - acc_rate_lim)
                )
            elif Inputs[i, 0] < -acc_rate_lim:
                Cost[i] += (
                    0.5
                    * acc_rate_lim_weight
                    * (Inputs[i, 0] + acc_rate_lim)
                    * (Inputs[i, 0] + acc_rate_lim)
                )

            if Inputs[i, 1] > steer_rate_lim_ub:
                Cost[i] += (
                    0.5
                    * steer_rate_lim_weight
                    * (Inputs[i, 1] - steer_rate_lim_ub)
                    * (Inputs[i, 1] - steer_rate_lim_ub)
                )
            elif Inputs[i, 1] < steer_rate_lim_lb:
                Cost[i] += (
                    0.5
                    * steer_rate_lim_weight
                    * (Inputs[i, 1] - steer_rate_lim_lb)
                    * (Inputs[i, 1] - steer_rate_lim_lb)
                )

    return Cost


@njit(cache=False, fastmath=True)
def calc_cost_only_for_states(X_des: np.ndarray, Traj: np.ndarray, n: int) -> np.ndarray:
    """Calculate the MPC cost for states of the nth horizon from several candidate predicted trajectories and target trajectories."""
    Cost = np.zeros(Traj.shape[0])
    if n == N:
        Q_ = Q_f.copy()
    elif n in timing_Q_c:
        Q_ = Q_c.copy()
    else:
        Q_ = Q.copy()
    theta = X_des[3]
    cos = np.cos(theta)
    sin = np.sin(theta)
    Rot = np.array([[cos, -sin], [sin, cos]])
    for i in range(Traj.shape[0]):
        x_current = Traj[i]
        v = x_current[2]

        Qi = Q_.copy()
        if n in timing_Q_c or n == N:
            lateral_error = np.abs((Rot.T @ (X_des[:2] - x_current[:2]))[1])
            lateral_cost_coef = calc_table_value(
                lateral_error, lateral_error_table, lateral_cost_coef_table
            )
            Qi[1, 1] = lateral_cost_coef * Qi[1, 1]
            yaw_error = np.abs(X_des[3] - x_current[3])
            yaw_cost_coef = calc_table_value(yaw_error, yaw_error_table, yaw_cost_coef_table)

            Qi[3, 3] = yaw_cost_coef * Qi[3, 3]
        Qi[:2, :2] = Rot @ Qi[:2, :2] @ Rot.T
        Cost[i] += 0.5 * np.dot(Qi @ (x_current - X_des), x_current - X_des)

        steer_lim, _, _, acc_lim, _ = calc_limits(v, x_current[4], x_current[5])
        if x_current[2] <= 0.0 and x_current[nx_0] <= 0.0:
            Cost[i] += 0.5 * acc_lim_weight * (x_current[nx_0] - 0.0) * (x_current[nx_0] - 0.0)

        else:
            if x_current[nx_0] > acc_lim:
                Cost[i] += (
                    0.5 * acc_lim_weight * (x_current[nx_0] - acc_lim) * (x_current[nx_0] - acc_lim)
                )
            elif x_current[nx_0] < -acc_lim:
                Cost[i] += (
                    0.5 * acc_lim_weight * (x_current[nx_0] + acc_lim) * (x_current[nx_0] + acc_lim)
                )

        if x_current[nx_0 + 1] > steer_lim:
            Cost[i] += (
                0.5
                * steer_lim_weight
                * (x_current[nx_0 + 1] - steer_lim)
                * (x_current[nx_0 + 1] - steer_lim)
            )

        elif x_current[nx_0 + 1] < -steer_lim:
            Cost[i] += (
                0.5
                * steer_lim_weight
                * (x_current[nx_0 + 1] + steer_lim)
                * (x_current[nx_0 + 1] + steer_lim)
            )

    return Cost


def rotate_data(x_old: np.ndarray, var_dot: np.ndarray) -> np.ndarray:
    """Apply a rotation that converts the x, y velocity relative to the world to the x, y velocity relative to the vehicle body."""
    theta_old = x_old[3]
    cos = np.cos(theta_old)
    sin = np.sin(theta_old)

    var_dot_ = var_dot.copy()
    var_dot_[:2] = np.array([[cos, sin], [-sin, cos]]) @ var_dot[:2]

    return var_dot_


@njit(cache=True, fastmath=True)
def f_init(
    states: np.ndarray,
    inputs: np.ndarray,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
    steer_dead_band_for_ctrl=steer_dead_band_for_ctrl,
) -> np.ndarray:
    """Time derivative of nominal model dynamics."""
    v = states[2]
    theta = states[3]
    alpha = states[4]
    delta = states[5]

    delta_diff = inputs[1] - delta
    if delta_diff >= steer_dead_band_for_ctrl:
        delta_diff = delta_diff - steer_dead_band_for_ctrl
    elif delta_diff <= -steer_dead_band_for_ctrl:
        delta_diff = delta_diff + steer_dead_band_for_ctrl
    else:
        delta_diff = 0.0

    states_dot = np.zeros(nx_0)
    states_dot[0] = v * np.cos(theta)
    states_dot[1] = v * np.sin(theta)
    states_dot[2] = alpha
    states_dot[3] = v * np.tan(delta) / L
    states_dot[4] = (inputs[0] - alpha) / acc_time_constant_ctrl
    states_dot[5] = delta_diff / steer_time_constant_ctrl
    return states_dot


@njit(cache=True, fastmath=True)
def F_init(
    states: np.ndarray,
    inputs: np.ndarray,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
    steer_dead_band_for_ctrl=steer_dead_band_for_ctrl,
) -> np.ndarray:
    """Nominal model dynamics."""
    states_next = (
        states
        + f_init(
            states,
            inputs,
            acc_time_constant_ctrl,
            steer_time_constant_ctrl,
            steer_dead_band_for_ctrl,
        )
        * ctrl_time_step
    )
    return states_next


@njit(cache=True, fastmath=True)
def F_multiple(
    states: np.ndarray,
    inputs: np.ndarray,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
    steer_dead_band_for_ctrl=steer_dead_band_for_ctrl,
) -> np.ndarray:
    """Discretize and integrate up to the MPC time width according to the nominal model dynamics."""
    states_next = states
    for i in range(mpc_freq):
        states_next = F_init(
            states_next,
            inputs[i],
            acc_time_constant_ctrl,
            steer_time_constant_ctrl,
            steer_dead_band_for_ctrl,
        )
    return states_next


@njit(cache=True, fastmath=True)
def F_init_with_diff(
    states: np.ndarray,
    inputs: np.ndarray,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
    steer_dead_band_for_ctrl=steer_dead_band_for_ctrl,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute the dynamics of the nominal model while computing derivatives."""
    states_next = (
        states
        + f_init(
            states,
            inputs,
            acc_time_constant_ctrl,
            steer_time_constant_ctrl,
            steer_dead_band_for_ctrl,
        )
        * ctrl_time_step
    )
    dF_dx = np.eye(nx_0)
    dF_du = np.zeros((nx_0, nu_0))
    v = states[2]
    theta = states[3]
    delta = states[5]

    dF_dx[0, 2] += np.cos(theta) * ctrl_time_step
    dF_dx[0, 3] -= v * np.sin(theta) * ctrl_time_step
    dF_dx[1, 2] += np.sin(theta) * ctrl_time_step
    dF_dx[1, 3] += v * np.cos(theta) * ctrl_time_step
    dF_dx[2, 4] += ctrl_time_step
    dF_dx[3, 2] += np.tan(delta) * ctrl_time_step / L
    dF_dx[3, 5] += v * ctrl_time_step / (L * np.cos(delta) * np.cos(delta))
    dF_dx[4, 4] -= ctrl_time_step / acc_time_constant_ctrl

    dF_du[4, 0] = ctrl_time_step / acc_time_constant_ctrl
    if abs(inputs[1] - delta) >= steer_dead_band_for_ctrl:
        dF_dx[5, 5] -= ctrl_time_step / steer_time_constant_ctrl
        dF_du[5, 1] = ctrl_time_step / steer_time_constant_ctrl
    return states_next, dF_dx, dF_du


@njit(cache=True, fastmath=True)
def F_multiple_with_diff(
    states: np.ndarray,
    inputs: np.ndarray,
    acc_time_constant_ctrl: float = ctrl_time_step,
    steer_time_constant_ctrl: float = ctrl_time_step,
    steer_dead_band_for_ctrl=steer_dead_band_for_ctrl,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Integrate up to the MPC time width according to the nominal model while calculating the derivative."""
    F_0, dFx_0, dFu_0 = F_init_with_diff(
        states,
        inputs[0],
        acc_time_constant_ctrl=acc_time_constant_ctrl,
        steer_time_constant_ctrl=steer_time_constant_ctrl,
        steer_dead_band_for_ctrl=steer_dead_band_for_ctrl,
    )
    F_1, dFx_1, dFu_1 = F_init_with_diff(
        F_0,
        inputs[1],
        acc_time_constant_ctrl=acc_time_constant_ctrl,
        steer_time_constant_ctrl=steer_time_constant_ctrl,
        steer_dead_band_for_ctrl=steer_dead_band_for_ctrl,
    )
    F_2, dFx_2, dFu_2 = F_init_with_diff(
        F_1,
        inputs[2],
        acc_time_constant_ctrl=acc_time_constant_ctrl,
        steer_time_constant_ctrl=steer_time_constant_ctrl,
        steer_dead_band_for_ctrl=steer_dead_band_for_ctrl,
    )
    dF_dx = dFx_2 @ dFx_1 @ dFx_0

    dF_du = np.zeros((nx_0, 3 * nu_0))
    dF_du[:, :nu_0] = dFx_2 @ dFx_1 @ dFu_0
    dF_du[:, nu_0 : 2 * nu_0] = dFx_2 @ dFu_1
    dF_du[:, 2 * nu_0 : 3 * nu_0] = dFu_2
    return F_2, dF_dx, dF_du


@njit(cache=True, fastmath=True)
def f_init_for_candidates(
    States: np.ndarray,
    Inputs: np.ndarray,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
    steer_dead_band_for_ctrl=steer_dead_band_for_ctrl,
) -> np.ndarray:
    """Time derivative of nominal model dynamics for multiple candidates."""
    v = States[:, 2]
    theta = States[:, 3]
    alpha = States[:, 4]
    delta = States[:, 5]
    smaller_than_band = Inputs[:, 1] - delta < steer_dead_band_for_ctrl
    larger_than_band = Inputs[:, 1] - delta > -steer_dead_band_for_ctrl

    delta_diff = (1 - smaller_than_band * larger_than_band) * (
        (Inputs[:, 1] - delta)
        - (1 * larger_than_band - 1 * smaller_than_band) * steer_dead_band_for_ctrl
    )
    States_dot = np.zeros(States.shape)
    States_dot[:, 0] = v * np.cos(theta)
    States_dot[:, 1] = v * np.sin(theta)
    States_dot[:, 2] = alpha
    States_dot[:, 3] = v * np.tan(delta) / L
    States_dot[:, 4] = (Inputs[:, 0] - alpha) / acc_time_constant_ctrl
    States_dot[:, 5] = delta_diff / steer_time_constant_ctrl
    return States_dot


@njit(cache=True, fastmath=True)
def F_multiple_for_candidates(
    States: np.ndarray,
    Inputs: np.ndarray,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
    steer_dead_band_for_ctrl=steer_dead_band_for_ctrl,
) -> np.ndarray:
    """Discretize and integrate up to the MPC time width according to the nominal model dynamics for several candidates."""
    # Euler method
    States_next = States.copy()
    for i in range(mpc_freq):
        States_next = (
            States_next
            + f_init_for_candidates(
                States_next,
                Inputs[:, i, :],
                acc_time_constant_ctrl,
                steer_time_constant_ctrl,
                steer_dead_band_for_ctrl,
            )
            * ctrl_time_step
        )
    return States_next


@njit(cache=True, fastmath=True)
def F_with_history(
    states: np.ndarray,
    inputs: np.ndarray,
    previous_error=None,
    k: int = 0,
    i: int = acc_delay_step,
    j: int = steer_delay_step,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
    steer_dead_band_for_ctrl=steer_dead_band_for_ctrl,
) -> tuple[np.ndarray, None]:
    """Integrate up to the MPC time width according to the nominal model.

    This includes the history of the input to the state.
    """
    # Function from extended state
    states_next = np.zeros(states.shape)
    states_next[nx_0 + mpc_freq : nx_0 + acc_ctrl_queue_size] = states[
        nx_0 : nx_0 + acc_ctrl_queue_size - mpc_freq
    ].copy()
    states_next[
        nx_0 + acc_ctrl_queue_size + mpc_freq : nx_0 + acc_ctrl_queue_size + steer_ctrl_queue_size
    ] = states[
        nx_0 + acc_ctrl_queue_size : nx_0 + acc_ctrl_queue_size + steer_ctrl_queue_size - mpc_freq
    ].copy()
    for index in range(mpc_freq):
        states_next[nx_0 + mpc_freq - index - 1] = (
            states_next[nx_0 + mpc_freq - index] + ctrl_time_step * inputs[0]
        )
        states_next[nx_0 + acc_ctrl_queue_size + mpc_freq - index - 1] = (
            states_next[nx_0 + acc_ctrl_queue_size + mpc_freq - index] + ctrl_time_step * inputs[1]
        )

    actual_inputs = np.zeros((mpc_freq, inputs.shape[0]))
    actual_inputs[:, 0] = states_next[nx_0 + i + np.arange(mpc_freq)[::-1]]
    actual_inputs[:, 1] = states_next[nx_0 + acc_ctrl_queue_size + j + np.arange(mpc_freq)[::-1]]

    states_next[:nx_0] = F_multiple(
        states[:nx_0],
        actual_inputs,
        acc_time_constant_ctrl,
        steer_time_constant_ctrl,
        steer_dead_band_for_ctrl,
    )

    return states_next, None


def F_with_history_and_diff(
    states: np.ndarray,
    inputs: np.ndarray,
    previous_error: None = None,
    k: int = 0,
    i: int = acc_delay_step,
    j: int = steer_delay_step,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, None]:
    """While computing the derivative, integrate up to the MPC time width according to the nominal model.

    This includes the history of the input to the state.
    """
    nx = states.shape[0]
    states_next = np.zeros(nx)
    dF_dx = np.zeros((nx, nx))
    dF_du = np.zeros((nx, nu_0))

    states_next[nx_0 + mpc_freq : nx_0 + acc_ctrl_queue_size] = states[
        nx_0 : nx_0 + acc_ctrl_queue_size - mpc_freq
    ].copy()
    states_next[
        nx_0 + acc_ctrl_queue_size + mpc_freq : nx_0 + acc_ctrl_queue_size + steer_ctrl_queue_size
    ] = states[
        nx_0 + acc_ctrl_queue_size : nx_0 + acc_ctrl_queue_size + steer_ctrl_queue_size - mpc_freq
    ].copy()

    dF_dx[
        nx_0 + mpc_freq : nx_0 + acc_ctrl_queue_size, nx_0 : nx_0 + acc_ctrl_queue_size - mpc_freq
    ] = np.eye(acc_ctrl_queue_size - mpc_freq)
    dF_dx[
        nx_0 + acc_ctrl_queue_size + mpc_freq : nx_0 + acc_ctrl_queue_size + steer_ctrl_queue_size,
        nx_0 + acc_ctrl_queue_size : nx_0 + acc_ctrl_queue_size + steer_ctrl_queue_size - mpc_freq,
    ] = np.eye(steer_ctrl_queue_size - mpc_freq)

    for index in range(mpc_freq):
        states_next[nx_0 + mpc_freq - index - 1] = (
            states_next[nx_0 + mpc_freq - index] + ctrl_time_step * inputs[0]
        )
        states_next[nx_0 + acc_ctrl_queue_size + mpc_freq - index - 1] = (
            states_next[nx_0 + acc_ctrl_queue_size + mpc_freq - index] + ctrl_time_step * inputs[1]
        )
        dF_dx[nx_0 + mpc_freq - index - 1, nx_0] = 1
        dF_dx[nx_0 + acc_ctrl_queue_size + mpc_freq - index - 1, nx_0 + acc_ctrl_queue_size] = 1

        dF_du[nx_0 + mpc_freq - index - 1, 0] = (index + 1) * ctrl_time_step
        dF_du[nx_0 + acc_ctrl_queue_size + mpc_freq - index - 1, 1] = (index + 1) * ctrl_time_step

    actual_inputs = np.zeros((mpc_freq, inputs.shape[0]))
    actual_inputs[:, 0] = states_next[nx_0 + i + np.arange(mpc_freq)[::-1]].copy()

    actual_inputs[:, 1] = states_next[
        nx_0 + acc_ctrl_queue_size + j + np.arange(mpc_freq)[::-1]
    ].copy()
    control_index = np.vstack(
        (
            nx_0 + max(i - mpc_freq, 0) + np.arange(mpc_freq)[::-1],
            nx_0 + acc_ctrl_queue_size + max(j - mpc_freq, 0) + np.arange(mpc_freq)[::-1],
        )
    ).T.flatten()
    states_next[:nx_0], dF_dx[:nx_0, :nx_0], dF_dx[:nx_0, control_index] = F_multiple_with_diff(
        states[:nx_0], actual_inputs, acc_time_constant_ctrl, steer_time_constant_ctrl
    )

    return states_next, dF_dx, dF_du, None


@njit(cache=True, fastmath=True)
def F_with_history_for_candidates(
    States: np.ndarray,
    Inputs: np.ndarray,
    Previous_error=None,
    k=0,
    i: int = acc_delay_step,
    j: int = steer_delay_step,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
) -> tuple[np.ndarray, None]:
    """For several candidates, discretize and integrate up to the MPC time-width according to the nominal model dynamics.

    This includes the history of the input to the state.
    """
    States_next = np.zeros(States.shape)

    States_next[:, nx_0 + mpc_freq : nx_0 + acc_ctrl_queue_size] = States[
        :, nx_0 : nx_0 + acc_ctrl_queue_size - mpc_freq
    ].copy()
    States_next[
        :,
        nx_0 + acc_ctrl_queue_size + mpc_freq : nx_0 + acc_ctrl_queue_size + steer_ctrl_queue_size,
    ] = States[
        :,
        nx_0 + acc_ctrl_queue_size : nx_0 + acc_ctrl_queue_size + steer_ctrl_queue_size - mpc_freq,
    ].copy()
    for index in range(mpc_freq):
        States_next[:, nx_0 + mpc_freq - index - 1] = (
            States_next[:, nx_0 + mpc_freq - index] + ctrl_time_step * Inputs[:, 0]
        )
        States_next[:, nx_0 + acc_ctrl_queue_size + mpc_freq - index - 1] = (
            States_next[:, nx_0 + acc_ctrl_queue_size + mpc_freq - index]
            + ctrl_time_step * Inputs[:, 1]
        )
    actual_Inputs = np.zeros((Inputs.shape[0], mpc_freq, nu_0))

    actual_Inputs[:, :, 0] = States_next[:, nx_0 + i + np.arange(mpc_freq)[::-1]].copy()
    actual_Inputs[:, :, 1] = States_next[
        :, nx_0 + acc_ctrl_queue_size + j + np.arange(mpc_freq)[::-1]
    ].copy()
    States_next[:, :nx_0] = F_multiple_for_candidates(
        States[:, :nx_0], actual_Inputs, acc_time_constant_ctrl, steer_time_constant_ctrl
    )
    return States_next, None


def F_with_model(
    states: np.ndarray,
    inputs: np.ndarray,
    previous_error: np.ndarray,
    k: int,
    pred: Callable,
    i: int = acc_delay_step,
    j: int = steer_delay_step,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
) -> tuple[np.ndarray, np.ndarray]:
    """Integrate up to the MPC time width according to the trained model."""
    if k == 0:
        error = Error_decay * previous_error + (1 - Error_decay) * pred(states)
    else:
        error = Error_decay * Error_decay * Error_decay * previous_error + (
            1 - Error_decay * Error_decay * Error_decay
        ) * pred(states)
    previous_error_ = error
    states_next, _ = F_with_history(
        states,
        inputs,
        i=i,
        j=j,
        acc_time_constant_ctrl=acc_time_constant_ctrl,
        steer_time_constant_ctrl=steer_time_constant_ctrl,
    )

    states_next[:nx_0] += error * mpc_time_step

    return states_next, previous_error_


def F_with_model_initial_diff(
    states: np.ndarray,
    inputs: np.ndarray,
    previous_error: np.ndarray,
    k: int,
    pred: Callable,
    i: int = acc_delay_step,
    j: int = steer_delay_step,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """While computing the derivative, integrate up to the MPC time width according to the trained model.

    This includes the history of the input to the state.
    The derivative follows the nominal model.
    This verifies whether the learning is working on the predictions or on the derivatives.
    """
    if k == 0:
        error = error_decay * previous_error + (1 - error_decay) * pred(states)
    else:
        error = error_decay * error_decay * error_decay * previous_error + (
            1 - error_decay * error_decay * error_decay
        ) * pred(states)
    previous_error_ = error

    rot_error, d_rot_error = np.split(error, [nx_0])
    states_next, dF_dx, dF_du, _ = F_with_history_and_diff(
        states,
        inputs,
        i=i,
        j=j,
        acc_time_constant_ctrl=acc_time_constant_ctrl,
        steer_time_constant_ctrl=steer_time_constant_ctrl,
    )

    dF_dx[:2, 3] += d_rot_error * mpc_time_step
    states_next[:nx_0] += rot_error * mpc_time_step

    return states_next, dF_dx, dF_du, previous_error_


def F_with_model_diff(
    states: np.ndarray,
    inputs: np.ndarray,
    previous_error: np.ndarray,
    k: int,
    pred: Callable,
    i: int = acc_delay_step,
    j: int = steer_delay_step,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """While computing the derivative, it integrates up to the MPC time width according to the trained model.

    This includes the history of the input to the state.
    The derivatives also follow a trained model.
    """
    pred_with_diff = pred(states)

    if k == 0:
        rot_error = (
            error_decay[:nx_0] * previous_error[:nx_0]
            + (1 - error_decay[:nx_0]) * pred_with_diff[:, 0]
        )
        d_rot_error = (
            error_decay[nx_0:] * previous_error[nx_0:]
            + (1 - error_decay[nx_0:]) * pred_with_diff[:2, 1]
        )
    else:
        rot_error = (
            error_decay[:nx_0] * error_decay[:nx_0] * error_decay[:nx_0] * previous_error[:nx_0]
            + (1 - error_decay[:nx_0] * error_decay[:nx_0] * error_decay[:nx_0])
            * pred_with_diff[:, 0]
        )
        d_rot_error = (
            error_decay[nx_0:] * error_decay[nx_0:] * error_decay[nx_0:] * previous_error[nx_0:]
            + (1 - error_decay[nx_0:] * error_decay[nx_0:] * error_decay[nx_0:])
            * pred_with_diff[:2, 1]
        )
    pred_error_ = np.concatenate((rot_error, d_rot_error))
    states_next, dF_dx, dF_du, _ = F_with_history_and_diff(
        states,
        inputs,
        i=i,
        j=j,
        acc_time_constant_ctrl=acc_time_constant_ctrl,
        steer_time_constant_ctrl=steer_time_constant_ctrl,
    )

    dF_dx[:2, 3] += d_rot_error * mpc_time_step
    states_next[:nx_0] += rot_error * mpc_time_step
    C = pred_with_diff[:, 2:] * mpc_time_step
    steer_diff = (
        dF_dx[5, nx_0 + acc_ctrl_queue_size :].sum() + C[5, nx_0 + acc_ctrl_queue_size :].sum()
    )
    if steer_diff < minimum_steer_diff:
        C[5, -1] += minimum_steer_diff - steer_diff
    return states_next, dF_dx, dF_du, C, pred_error_


def F_with_model_for_candidates(
    States: np.ndarray,
    Inputs: np.ndarray,
    Previous_error: np.ndarray,
    k: int,
    Pred: Callable,
    i: int = acc_delay_step,
    j: int = steer_delay_step,
    acc_time_constant_ctrl: float = acc_time_constant,
    steer_time_constant_ctrl: float = steer_time_constant,
) -> tuple[np.ndarray, np.ndarray]:
    """For several candidates, integrate up to the MPC time width according to the trained model.

    This includes the history of the input to the state.
    """
    if k == 0:
        Error = Error_decay * Previous_error + (1 - Error_decay) * Pred(States.T).T
    else:
        Error = (
            Error_decay * Error_decay * Error_decay * Previous_error
            + (1 - Error_decay * Error_decay * Error_decay) * Pred(States.T).T
        )
    Previous_error_ = Error

    States_next, _ = F_with_history_for_candidates(
        States,
        Inputs,
        i=i,
        j=j,
        acc_time_constant_ctrl=acc_time_constant_ctrl,
        steer_time_constant_ctrl=steer_time_constant_ctrl,
    )
    States_next[:, :nx_0] += Error * mpc_time_step

    return States_next, Previous_error_


def acc_prediction_error_compensation(
    states: np.ndarray,
    previous_states: np.ndarray,
    inputs: np.ndarray,
    previous_error: np.ndarray,
    acc_fb_1: float,
    acc_fb_2: float,
    acc_time_stamp_interval: float,
):
    actual_acc_input = np.concatenate(
        (np.array([inputs[0]]), previous_states[nx_0 : nx_0 + acc_ctrl_queue_size])
    )[acc_delay_step]

    predicted_acc = previous_states[4] + acc_time_stamp_interval * (
        previous_error[4] + (actual_acc_input - previous_states[4]) / acc_time_constant
    )
    prediction_error = predicted_acc - states[4]
    new_acc_fb_1 = np.exp(-acc_fb_decay) * acc_fb_1 + acc_fb_decay * prediction_error
    new_acc_fb_2 = (
        acc_fb_decay * np.exp(-acc_fb_decay) * acc_fb_1 + np.exp(-acc_fb_decay) * acc_fb_2
    )
    return new_acc_fb_1, new_acc_fb_2


def steer_prediction_error_compensation(
    states: np.ndarray,
    previous_states: np.ndarray,
    inputs: np.ndarray,
    previous_error: np.ndarray,
    steer_fb_1: float,
    steer_fb_2: float,
    steer_time_stamp_interval: float,
):
    actual_steer_input = np.concatenate(
        (np.array([inputs[1]]), previous_states[nx_0 + acc_ctrl_queue_size :])
    )[steer_delay_step]

    delta_diff = actual_steer_input - previous_states[5]
    if delta_diff >= steer_dead_band_for_ctrl:
        delta_diff = delta_diff - steer_dead_band_for_ctrl
    elif delta_diff <= -steer_dead_band_for_ctrl:
        delta_diff = delta_diff + steer_dead_band_for_ctrl
    else:
        delta_diff = 0.0

    predicted_steer = previous_states[5] + steer_time_stamp_interval * (
        previous_error[5] + delta_diff / steer_time_constant
    )
    prediction_error = predicted_steer - states[5]

    new_steer_fb_1 = np.exp(-steer_fb_decay) * steer_fb_1 + steer_fb_decay * (prediction_error)
    new_steer_fb_2 = (
        steer_fb_decay * np.exp(-steer_fb_decay) * steer_fb_1 + np.exp(-steer_fb_decay) * steer_fb_2
    )
    return new_steer_fb_1, new_steer_fb_2


def pure_pursuit_control(
    pos_xy_obs,
    pos_yaw_obs,
    longitudinal_vel_obs,
    pos_xy_ref,
    pos_yaw_ref,
    longitudinal_vel_ref,
    acc_gain_scaling=1.0,
    steer_gain_scaling=1.0,
):
    pure_pursuit_acc_kp = acc_gain_scaling * mpc_param["mpc_parameter"]["pure_pursuit"]["acc_kp"]
    pure_pursuit_lookahead_time = mpc_param["mpc_parameter"]["pure_pursuit"]["lookahead_time"]
    pure_pursuit_min_lookahead = mpc_param["mpc_parameter"]["pure_pursuit"]["min_lookahead"]
    pure_pursuit_steer_kp_param = (
        steer_gain_scaling * mpc_param["mpc_parameter"]["pure_pursuit"]["steer_kp_param"]
    )
    pure_pursuit_steer_kd_param = (
        steer_gain_scaling * mpc_param["mpc_parameter"]["pure_pursuit"]["steer_kd_param"]
    )

    longitudinal_vel_err = longitudinal_vel_obs - longitudinal_vel_ref
    pure_pursuit_acc_cmd = -pure_pursuit_acc_kp * longitudinal_vel_err

    cos_yaw = np.cos(pos_yaw_ref)
    sin_yaw = np.sin(pos_yaw_ref)
    diff_position = pos_xy_obs - pos_xy_ref
    lat_err = -sin_yaw * diff_position[0] + cos_yaw * diff_position[1]
    yaw_err = pos_yaw_obs - pos_yaw_ref
    while True:
        if yaw_err > np.pi:
            yaw_err -= 2.0 * np.pi
        if yaw_err < (-np.pi):
            yaw_err += 2.0 * np.pi
        if np.abs(yaw_err) < np.pi:
            break

    lookahead = pure_pursuit_min_lookahead + pure_pursuit_lookahead_time * np.abs(
        longitudinal_vel_obs
    )
    pure_pursuit_steer_kp = pure_pursuit_steer_kp_param * L / (lookahead * lookahead)
    pure_pursuit_steer_kd = pure_pursuit_steer_kd_param * L / lookahead
    pure_pursuit_steer_cmd = -pure_pursuit_steer_kp * lat_err - pure_pursuit_steer_kd * yaw_err
    return np.array([pure_pursuit_acc_cmd, pure_pursuit_steer_cmd])


naive_pure_pursuit_lookahead_coef = float(
    mpc_param["mpc_parameter"]["naive_pure_pursuit"]["lookahead_coef"]
)
naive_pure_pursuit_lookahead_intercept = float(
    mpc_param["mpc_parameter"]["naive_pure_pursuit"]["lookahead_intercept"]
)


def naive_pure_pursuit_control(
    pos_xy_obs,
    pos_yaw_obs,
    longitudinal_vel_obs,
    pos_xy_ref_target,
    longitudinal_vel_ref_nearest,
):
    pure_pursuit_acc_kp = mpc_param["mpc_parameter"]["naive_pure_pursuit"]["acc_kp"]
    wheel_base = L
    longitudinal_vel_err = longitudinal_vel_obs - longitudinal_vel_ref_nearest
    pure_pursuit_acc_cmd = -pure_pursuit_acc_kp * longitudinal_vel_err

    alpha = (
        np.arctan2(pos_xy_ref_target[1] - pos_xy_obs[1], pos_xy_ref_target[0] - pos_xy_obs[0])
        - pos_yaw_obs
    )
    angular_velocity_z = 2.0 * longitudinal_vel_ref_nearest * np.sin(alpha) / wheel_base
    steer = np.arctan(angular_velocity_z * wheel_base / max(0.1, longitudinal_vel_ref_nearest))
    return np.array([pure_pursuit_acc_cmd, steer])


def sg_filter(
    inputs: np.ndarray,
    use_sg: bool,
    sg_window_size: int,
    sg_vector_left_edge: list[np.ndarray],
    sg_vector_right_edge: list[np.ndarray],
    sg_vector: np.ndarray,
) -> np.ndarray:
    """SG Filter. Used for smoothing."""
    if use_sg:
        input_len = inputs.shape[0]
        filtered_inputs = np.zeros(inputs.shape)
        for i in range(input_len):
            if i < sg_window_size:
                filtered_inputs[i] = (
                    (inputs[: sg_window_size + i + 1].T * sg_vector_left_edge[i])
                    .sum(axis=inputs.ndim - 1)
                    .T
                )
            elif i > input_len - sg_window_size - 1:
                filtered_inputs[i] = (
                    (inputs[i - sg_window_size :].T * sg_vector_right_edge[input_len - i - 1])
                    .sum(axis=inputs.ndim - 1)
                    .T
                )
            else:
                filtered_inputs[i] = (
                    (inputs[i - sg_window_size : i + sg_window_size + 1].T * sg_vector)
                    .sum(axis=inputs.ndim - 1)
                    .T
                )
        return filtered_inputs
    else:
        return inputs


def calc_sg_filter_weight(sg_deg, sg_window_size):
    """Calculate weighted average weights from SG filter degree and window size."""
    sg_vector_left_edge = []
    sg_vector_right_edge = []
    e_0 = np.zeros(sg_deg + 1)
    e_0[0] = 1.0
    for i in range(sg_window_size):
        reg_times = (np.arange(sg_window_size + i + 1) - i) * mpc_time_step
        T = np.ones((sg_window_size + i + 1, sg_deg + 1))
        W = np.eye(T.shape[0])
        for j in range(sg_deg):
            T[:, j + 1] = T[:, j] * reg_times
        sg_vector_left_edge.append(W @ T @ np.linalg.inv(T.T @ W @ T) @ e_0)
        reg_times = (np.arange(sg_window_size + i + 1) - sg_window_size) * mpc_time_step
        T = np.ones((sg_window_size + i + 1, sg_deg + 1))
        W = np.eye(T.shape[0])
        for j in range(sg_deg):
            T[:, j + 1] = T[:, j] * reg_times
        sg_vector_right_edge.append(W @ T @ np.linalg.inv(T.T @ W @ T) @ e_0)
    reg_times = (np.arange(2 * sg_window_size + 1) - sg_window_size) * mpc_time_step
    T = np.ones((2 * sg_window_size + 1, sg_deg + 1))
    for j in range(sg_deg):
        T[:, j + 1] = T[:, j] * reg_times
    sg_vector = T @ np.linalg.inv(T.T @ T) @ e_0
    return sg_vector, sg_vector_left_edge, sg_vector_right_edge


(
    sg_vector_for_nominal_inputs,
    sg_vector_left_edge_for_nominal_inputs,
    sg_vector_right_edge_for_nominal_inputs,
) = calc_sg_filter_weight(sg_deg_for_nominal_inputs, sg_window_size_for_nominal_inputs)
(
    sg_vector_for_trained_model_diff,
    sg_vector_left_edge_for_trained_model_diff,
    sg_vector_right_edge_for_trained_model_diff,
) = calc_sg_filter_weight(sg_deg_for_trained_model_diff, sg_window_size_for_trained_model_diff)
(
    sg_vector_for_memory_diff,
    sg_vector_left_edge_for_memory_diff,
    sg_vector_right_edge_for_memory_diff,
) = calc_sg_filter_weight(0, sg_window_size_for_memory_diff)
(
    sg_vector_for_noise,
    sg_vector_left_edge_for_noise,
    sg_vector_right_edge_for_noise,
) = calc_sg_filter_weight(sg_deg_for_noise, sg_window_size_for_noise)

sg_filter_for_nominal_inputs = partial(
    sg_filter,
    use_sg=use_sg_for_nominal_inputs,
    sg_window_size=sg_window_size_for_nominal_inputs,
    sg_vector_left_edge=sg_vector_left_edge_for_nominal_inputs,
    sg_vector_right_edge=sg_vector_right_edge_for_nominal_inputs,
    sg_vector=sg_vector_for_nominal_inputs,
)

sg_filter_for_trained_model_diff = partial(
    sg_filter,
    use_sg=use_sg_for_trained_model_diff,
    sg_window_size=sg_window_size_for_trained_model_diff,
    sg_vector_left_edge=sg_vector_left_edge_for_trained_model_diff,
    sg_vector_right_edge=sg_vector_right_edge_for_trained_model_diff,
    sg_vector=sg_vector_for_trained_model_diff,
)

sg_filter_for_memory_diff = partial(
    sg_filter,
    use_sg=use_sg_for_memory_diff,
    sg_window_size=sg_window_size_for_memory_diff,  # sg_window_size_for_trained_model_diff,
    sg_vector_left_edge=sg_vector_left_edge_for_memory_diff,  # sg_vector_left_edge_for_trained_model_diff,
    sg_vector_right_edge=sg_vector_right_edge_for_memory_diff,  # sg_vector_right_edge_for_trained_model_diff,
    sg_vector=sg_vector_for_memory_diff,
)


sg_filter_for_noise = partial(
    sg_filter,
    use_sg=use_sg_for_noise,
    sg_window_size=sg_window_size_for_noise,
    sg_vector_left_edge=sg_vector_left_edge_for_noise,
    sg_vector_right_edge=sg_vector_right_edge_for_noise,
    sg_vector=sg_vector_for_noise,
)


def calc_gf_kernel(sigma):
    """Calculate the kernel of the Gaussian filter from the variance."""
    kernel = np.zeros(int(round(4 * sigma)) * 2 + 1)
    for i in range(kernel.shape[0]):
        kernel[i] = np.exp(-((i - round(4 * sigma)) ** 2) / (2 * sigma**2))

    kernel = kernel / kernel.sum()
    return kernel


kernel_acc_for_learning = calc_gf_kernel(acc_sigma_for_learning)
kernel_steer_for_learning = calc_gf_kernel(steer_sigma_for_learning)
kernel_acc_des_for_learning = calc_gf_kernel(acc_des_sigma_for_learning)
kernel_steer_des_for_learning = calc_gf_kernel(steer_des_sigma_for_learning)

max_input_queue_size_for_learning = max(
    [
        kernel_acc_for_learning.shape[0],
        kernel_steer_for_learning.shape[0],
        kernel_acc_des_for_learning.shape[0],
        kernel_steer_des_for_learning.shape[0],
    ]
)

kernel_x_out_for_learning = calc_gf_kernel(x_out_sigma_for_learning)
kernel_y_out_for_learning = calc_gf_kernel(y_out_sigma_for_learning)
kernel_v_out_for_learning = calc_gf_kernel(v_out_sigma_for_learning)
kernel_theta_out_for_learning = calc_gf_kernel(theta_out_sigma_for_learning)
kernel_acc_out_for_learning = calc_gf_kernel(acc_out_sigma_for_learning)
kernel_steer_out_for_learning = calc_gf_kernel(steer_out_sigma_for_learning)

max_output_queue_size_for_learning = max(
    [
        kernel_x_out_for_learning.shape[0],
        kernel_y_out_for_learning.shape[0],
        kernel_v_out_for_learning.shape[0],
        kernel_theta_out_for_learning.shape[0],
        kernel_acc_out_for_learning.shape[0],
        kernel_steer_out_for_learning.shape[0],
    ]
)


def u_cut_off(
    u_input: np.ndarray,
    u_old: np.ndarray,
    steer_lim: float,
    steer_rate_lim_lb: float,
    steer_rate_lim_ub: float,
    acc_lim: float,
    acc_rate_lim: float,
) -> np.ndarray:
    """Process input values to satisfy constraints."""
    u_input_ = u_input.copy()
    lb_0 = max(-acc_lim, u_old[0] - acc_rate_lim * ctrl_time_step)
    ub_0 = min(acc_lim, u_old[0] + acc_rate_lim * ctrl_time_step)
    lb_1 = max(-steer_lim, u_old[1] + steer_rate_lim_lb * ctrl_time_step)
    ub_1 = min(steer_lim, u_old[1] + steer_rate_lim_ub * ctrl_time_step)
    if u_input[0] >= ub_0:
        u_input_[0] = ub_0
    elif u_input[0] < lb_0:
        u_input_[0] = lb_0
    if u_input[1] >= ub_1:
        u_input_[1] = ub_1
    elif u_input[1] < lb_1:
        u_input_[1] = lb_1
    return u_input_
