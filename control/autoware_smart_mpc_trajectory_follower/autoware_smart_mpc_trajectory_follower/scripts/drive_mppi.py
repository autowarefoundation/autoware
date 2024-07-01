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

from typing import Callable

from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
from numba import njit
import numpy as np

index_cost = np.concatenate(
    (
        np.arange(drive_functions.nx_0 + 1),
        np.array([drive_functions.nx_0 + drive_functions.acc_ctrl_queue_size]),
    )
)


@njit(cache=False, fastmath=True)
def generate_input_perturbation(N, sample_num, step, Sigma_0, Sigma_1):
    """Generate random perturbations of the input in MPPI."""
    change_input_index = np.concatenate((np.arange(N, step=step), np.array([N])))
    acc_perturbation = np.random.normal(
        loc=0, scale=Sigma_0, size=(sample_num - 1, change_input_index.shape[0] - 1)
    )
    steer_perturbation = np.random.normal(
        loc=0, scale=Sigma_1, size=(sample_num - 1, change_input_index.shape[0] - 1)
    )
    Input_perturbation = np.zeros((sample_num - 1, N, 2))

    for i in range(change_input_index.shape[0] - 1):
        Input_perturbation[
            :, change_input_index[i] : change_input_index[i + 1], 0
        ] += acc_perturbation[:, np.array([i])]
        Input_perturbation[
            :, change_input_index[i] : change_input_index[i + 1], 1
        ] += steer_perturbation[:, np.array([i])]

    return Input_perturbation


class drive_mppi:
    """Class that runs MPPI to compute optimal input values."""

    def __init__(
        self,
        lam,
        Sigma,
        max_iter_mppi,
        sample_num,
        mppi_tol,
        mppi_step,
    ):
        self.lam = lam
        self.Sigma = Sigma

        self.max_iter_mppi = max_iter_mppi
        self.sample_num = sample_num
        self.mppi_tol = mppi_tol
        self.mppi_step = mppi_step

    def generate_sample_inputs(self, inputs):
        """Generate input samples in MPPI."""
        N = inputs.shape[0]
        nu = inputs.shape[1]
        Inputs = np.zeros((self.sample_num, N, nu))

        Inputs += inputs
        Inputs[1:] += generate_input_perturbation(
            N, self.sample_num, self.mppi_step, self.Sigma[0], self.Sigma[1]
        )
        return Inputs

    def calc_forward_trajectories_with_cost(self, x_current, Inputs, X_des, U_des, Previous_error):
        """Calculate the predicted trajectory and cost.

        Given the current state and the candidate input columns in the line search.
        """
        N = Inputs.shape[1]
        samples = Inputs.shape[0]
        Traj = np.zeros((samples, N + 1, x_current.shape[0]))
        Traj[:, 0, :] += x_current
        Cost = np.zeros(samples)
        for k in range(N):
            Cost += drive_functions.calc_cost_only_for_states(
                X_des[k], Traj[:, k, index_cost], k
            ) + self.lam * (Inputs[:, k, :] - Inputs[0, k, :]) @ (
                (1 / self.Sigma) * (Inputs[0, k, :] - U_des[k])
            )
            Traj[:, k + 1], Previous_error = self.F_for_candidates(
                Traj[:, k], Inputs[:, k], Previous_error, k
            )
        Cost += drive_functions.calc_cost_only_for_states(X_des[N], Traj[:, N, index_cost], N)
        return Traj, Cost

    def compute_optimal_control(self, x_current, inputs, X_des, U_des, previous_error):
        """Proceed with MPPI iteration one time."""
        N = inputs.shape[0]

        Inputs = self.generate_sample_inputs(inputs)

        Previous_error = np.tile(previous_error, (self.sample_num, 1))
        Traj, Cost = self.calc_forward_trajectories_with_cost(
            x_current, Inputs, X_des, U_des, Previous_error
        )
        original_cost = Cost[0]
        best_cost = Cost.min()
        Exps = np.exp(-(Cost - best_cost) / self.lam)
        Exps = Exps / Exps.sum()
        new_inputs = (Inputs.T @ Exps).T
        previous_error_ = previous_error.copy()
        new_traj = np.zeros((N + 1, x_current.shape[0]))
        new_traj[0] = x_current.copy()
        for k in range(N):
            new_traj[k + 1], previous_error_ = self.F(
                new_traj[k], new_inputs[k], previous_error_, k
            )
        if np.dot(Exps, Cost) < (1 - self.mppi_tol) * original_cost:
            proceed = True
        else:
            proceed = False
        return new_inputs, new_inputs[0], new_traj, Traj, proceed

    def receive_model(self, F_for_candidates: Callable, F):
        """Receive vehicle model for control."""
        self.F_for_candidates = F_for_candidates
        self.F = F
