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

# cspell: ignore numba njit fastmath

from functools import partial

from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
from numba import njit
import numpy as np

sqrt_mpc_time_step = np.sqrt(drive_functions.mpc_time_step)


@njit(cache=True, fastmath=True)
def get_pred(x_input, theta_1, theta_2, Z, mean_mtr, cov_mtr):
    """Calculate the prediction and its derivative, the deviation and its derivative based on the Gaussian process."""
    Z_norm = (Z - x_input) * theta_2
    k_ast = theta_1 * np.exp(-0.5 * ((Z - x_input) * Z_norm).sum(axis=1))
    k_ = cov_mtr @ k_ast
    Z_k = Z_norm.T * k_ast
    deviation = np.sqrt(theta_1 - np.dot(k_ast, k_))
    return np.dot(mean_mtr, k_ast), deviation, Z_k @ mean_mtr, -Z_k @ k_ / deviation


@njit(cache=True, fastmath=True)
def get_pred_deviation(x_input, theta_1, theta_2, Z, cov_mtr):
    """Calculate the deviation of the prediction and its derivative based on the Gaussian process."""
    coef = np.abs(x_input[0])
    coef = coef * coef * coef * coef * coef * coef * coef
    if coef > 1.0:
        coef = 1.0
    Z_norm = (Z - x_input) * theta_2
    k_ast = theta_1 * np.exp(-0.5 * ((Z - x_input) * Z_norm).sum(axis=1))
    k_ = cov_mtr @ k_ast
    Z_k = Z_norm.T * k_ast
    deviation = np.sqrt(theta_1 - np.dot(k_ast, k_))
    return coef * deviation, -coef * Z_k @ k_ / deviation


@njit(cache=True, fastmath=True)
def get_pred_deviations(x_inputs, theta_1, theta_2, Z, cov_mtr, indices):
    """Calculate the deviation of the predictions and their derivatives for multiple inputs based on the Gaussian process."""
    deviation = np.zeros(x_inputs.shape[0])
    deviation_dot = np.zeros((x_inputs.shape[0], x_inputs.shape[1] + 3))
    for i in range(x_inputs.shape[0]):
        deviation[i], deviation_dot[i, indices] = get_pred_deviation(
            x_inputs[i], theta_1, theta_2, Z, cov_mtr
        )
    return sqrt_mpc_time_step * deviation, sqrt_mpc_time_step * deviation_dot


class transform_GP_to_numba:
    """Class for fast computation of necessary calculations based on Gaussian processes with numba."""

    def __init__(self, i, dir_name="."):
        indices = np.concatenate(
            (
                np.array([2, 4, 5]),
                np.arange(
                    drive_functions.acc_ctrl_queue_size + drive_functions.steer_ctrl_queue_size
                )
                + 6,
            )
        )
        file_names = [
            "/GP_x_info.npz",
            "/GP_y_info.npz",
            "/GP_v_info.npz",
            "/GP_theta_info.npz",
            "/GP_acc_info.npz",
            "/GP_steer_info.npz",
        ]
        file_name = file_names[i]
        GP_info = np.load(dir_name + file_name)
        theta_1 = 1 * GP_info["theta_1"]
        theta_2 = GP_info["theta_2"]
        Z = GP_info["Z"]
        mean_mtr = GP_info["mean_mtr"]
        cov_mtr = GP_info["cov_mtr"]
        self.get_pred = partial(
            get_pred,
            theta_1=theta_1,
            theta_2=theta_2,
            Z=Z,
            mean_mtr=mean_mtr,
            cov_mtr=cov_mtr,
        )
        self.pred_deviation = partial(
            get_pred_deviation, theta_1=theta_1, theta_2=theta_2, Z=Z, cov_mtr=cov_mtr
        )
        self.pred_deviations = partial(
            get_pred_deviations,
            theta_1=theta_1,
            theta_2=theta_2,
            Z=Z,
            cov_mtr=cov_mtr,
            indices=indices,
        )
