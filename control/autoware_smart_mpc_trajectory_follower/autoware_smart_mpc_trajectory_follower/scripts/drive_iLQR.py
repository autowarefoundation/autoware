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

# cspell: ignore numba njit fastmath Riccati Jacobians

"""Optimize input based on iLQR."""


from importlib import reload as ir
import time
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
acc_index = drive_functions.nx_0
steer_index = drive_functions.nx_0 + drive_functions.acc_ctrl_queue_size
actual_state_dim = drive_functions.nx_0
input_dim = drive_functions.nu_0
mpc_freq = drive_functions.mpc_freq
acc_delay_step = drive_functions.acc_delay_step
steer_delay_step = drive_functions.steer_delay_step
acc_ctrl_queue_size = drive_functions.acc_ctrl_queue_size
steer_ctrl_queue_size = drive_functions.steer_ctrl_queue_size
ctrl_time_step = drive_functions.ctrl_time_step


@njit(cache=True, fastmath=True)
def sparse_right_action_for_state_diff(
    Mat: np.ndarray,
    A: np.ndarray,
    actual_state_dim=actual_state_dim,
    acc_ctrl_queue_size=acc_ctrl_queue_size,
    steer_ctrl_queue_size=steer_ctrl_queue_size,
) -> np.ndarray:
    """Receives the sparse matrix `A₀` and computes `Mat @ A₀`.

    The argument A is part of A₀ .
    The remaining A₀ components are known a priori.
    """
    result = np.zeros(
        (Mat.shape[0], actual_state_dim + acc_ctrl_queue_size + steer_ctrl_queue_size)
    )
    result += Mat[:, :actual_state_dim] @ A

    result[:, actual_state_dim : actual_state_dim + acc_ctrl_queue_size - mpc_freq] += Mat[
        :, actual_state_dim + mpc_freq : actual_state_dim + acc_ctrl_queue_size
    ]
    result[
        :,
        actual_state_dim
        + acc_ctrl_queue_size : actual_state_dim
        + acc_ctrl_queue_size
        + steer_ctrl_queue_size
        - mpc_freq,
    ] += Mat[
        :,
        actual_state_dim
        + acc_ctrl_queue_size
        + mpc_freq : actual_state_dim
        + acc_ctrl_queue_size
        + steer_ctrl_queue_size,
    ]
    for i in range(mpc_freq):
        result[:, actual_state_dim] += Mat[:, actual_state_dim + i]
        result[:, actual_state_dim + acc_ctrl_queue_size] += Mat[
            :, actual_state_dim + acc_ctrl_queue_size + i
        ]
    return result


@njit(cache=True, fastmath=True)
def sparse_left_action_for_state_diff(
    A: np.ndarray,
    Mat: np.ndarray,
    actual_state_dim=actual_state_dim,
    acc_ctrl_queue_size=acc_ctrl_queue_size,
    steer_ctrl_queue_size=steer_ctrl_queue_size,
) -> np.ndarray:
    """Receives the sparse matrix A₀ and computes A₀.T @ Mat .

    The argument A is part of A₀ .
    The remaining A₀ components are known a priori.
    """
    result = np.zeros(
        (actual_state_dim + acc_ctrl_queue_size + steer_ctrl_queue_size, Mat.shape[1])
    )
    result += A.T @ Mat[:actual_state_dim]

    result[actual_state_dim : actual_state_dim + acc_ctrl_queue_size - mpc_freq] += Mat[
        actual_state_dim + mpc_freq : actual_state_dim + acc_ctrl_queue_size
    ]
    result[
        actual_state_dim
        + acc_ctrl_queue_size : actual_state_dim
        + acc_ctrl_queue_size
        + steer_ctrl_queue_size
        - mpc_freq
    ] += Mat[
        actual_state_dim
        + acc_ctrl_queue_size
        + mpc_freq : actual_state_dim
        + acc_ctrl_queue_size
        + steer_ctrl_queue_size
    ]
    for i in range(mpc_freq):
        result[actual_state_dim] += Mat[actual_state_dim + i]
        result[actual_state_dim + acc_ctrl_queue_size] += Mat[
            actual_state_dim + acc_ctrl_queue_size + i
        ]
    return result


@njit(cache=True, fastmath=True)
def vector_sparse_left_action_for_state_diff(
    A: np.ndarray,
    vec: np.ndarray,
    actual_state_dim=actual_state_dim,
    acc_ctrl_queue_size=acc_ctrl_queue_size,
    steer_ctrl_queue_size=steer_ctrl_queue_size,
) -> np.ndarray:
    """Receives the sparse matrix A₀ and computes A₀.T @ vec.

    The argument A is part of A₀ .
    The remaining A₀ components are known a priori.
    """
    result = np.zeros(actual_state_dim + acc_ctrl_queue_size + steer_ctrl_queue_size)
    result += A.T @ vec[:actual_state_dim]

    result[actual_state_dim : actual_state_dim + acc_ctrl_queue_size - mpc_freq] += vec[
        actual_state_dim + mpc_freq : actual_state_dim + acc_ctrl_queue_size
    ]
    result[
        actual_state_dim
        + acc_ctrl_queue_size : actual_state_dim
        + acc_ctrl_queue_size
        + steer_ctrl_queue_size
        - mpc_freq
    ] += vec[
        actual_state_dim
        + acc_ctrl_queue_size
        + mpc_freq : actual_state_dim
        + acc_ctrl_queue_size
        + steer_ctrl_queue_size
    ]
    for i in range(mpc_freq):
        result[actual_state_dim] += vec[actual_state_dim + i]
        result[actual_state_dim + acc_ctrl_queue_size] += vec[
            actual_state_dim + acc_ctrl_queue_size + i
        ]
    return result


@njit(cache=True, fastmath=True)
def sparse_right_action_for_input_diff(
    Mat: np.ndarray,
    input_dim=input_dim,
    actual_state_dim=actual_state_dim,
    acc_ctrl_queue_size=acc_ctrl_queue_size,
) -> np.ndarray:
    """Compute Mat @ B for some sparse matrix B known a priori."""
    result = np.zeros((Mat.shape[0], input_dim))
    for i in range(mpc_freq):
        result[:, 0] += (mpc_freq - i) * ctrl_time_step * Mat[:, actual_state_dim + i]
        result[:, 1] += (
            (mpc_freq - i) * ctrl_time_step * Mat[:, actual_state_dim + acc_ctrl_queue_size + i]
        )
    return result


@njit(cache=True, fastmath=True)
def sparse_left_action_for_input_diff(
    Mat: np.ndarray,
    input_dim=input_dim,
    actual_state_dim=actual_state_dim,
    acc_ctrl_queue_size=acc_ctrl_queue_size,
) -> np.ndarray:
    """Compute B.T @ Mat for some sparse matrix B known a priori."""
    result = np.zeros((input_dim, Mat.shape[1]))
    for i in range(mpc_freq):
        result[0] += (mpc_freq - i) * ctrl_time_step * Mat[actual_state_dim + i]
        result[1] += (
            (mpc_freq - i) * ctrl_time_step * Mat[actual_state_dim + acc_ctrl_queue_size + i]
        )
    return result


@njit(cache=True, fastmath=True)
def vector_sparse_left_action_for_input_diff(
    vec: np.ndarray,
    input_dim=input_dim,
    actual_state_dim=actual_state_dim,
    acc_ctrl_queue_size=acc_ctrl_queue_size,
) -> np.ndarray:
    """Compute B.T @ vec for some sparse matrix B known a priori."""
    result = np.zeros(input_dim)
    for i in range(mpc_freq):
        result[0] += (mpc_freq - i) * ctrl_time_step * vec[actual_state_dim + i]
        result[1] += (
            (mpc_freq - i) * ctrl_time_step * vec[actual_state_dim + acc_ctrl_queue_size + i]
        )
    return result


@njit(cache=False, fastmath=True)
def compute_iLQR_coef(
    traj,
    inputs,
    A,
    B,
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
    N,
    X_des,
    U_des,
    Sigma_x=None,
    dSigma_x=None,
    Sigma_y=None,
    dSigma_y=None,
    Sigma_v=None,
    dSigma_v=None,
    Sigma_theta=None,
    dSigma_theta=None,
    Sigma_acc=None,
    dSigma_acc=None,
    Sigma_steer=None,
    dSigma_steer=None,
    acc_input_index=acc_index,
    steer_input_index=steer_index,
    state_dim=actual_state_dim,
):
    """Perform the main part of iLQR.

    Performs a Riccati recursion and returns the coefficients needed for the final output.
    """
    nx = A.shape[2]
    nu = B.shape[2]
    P = np.zeros((N + 1, nx, nx))
    w = np.zeros((N + 1, nx))
    H_inv_G = np.zeros((N, nu, nx))
    H_inv_g = np.zeros((N, nu))
    P[N][:6, :6] = Q_total[-1][:6, :6]
    P[N][acc_input_index, acc_input_index] = Q_total[-1][6, 6]
    P[N][steer_input_index, steer_input_index] = Q_total[-1][7, 7]

    P[N][acc_input_index, acc_input_index] += acc_lim_weights[N]
    P[N][steer_input_index, steer_input_index] += steer_lim_weights[N]

    Q_N_x = Q_total[-1] @ (traj[N] - X_des[N])

    w[N][:6] = Q_N_x[:6]
    w[N][acc_input_index] = Q_N_x[6]
    w[N][steer_input_index] = Q_N_x[7]

    w[N, acc_input_index] += acc_lim_weights[N] * (traj[N, acc_index] - acc_lim_center[N])
    w[N, steer_input_index] += steer_lim_weights[N] * (traj[N, acc_index + 1] - steer_lim_center[N])

    for i in range(N):
        j = N - i - 1
        Qj = Q_total[j]
        Rj = R_total[j]
        Aj = A[j]
        Pj1Aj = sparse_right_action_for_state_diff(
            P[j + 1], Aj[:state_dim], actual_state_dim=state_dim
        )
        G = sparse_left_action_for_input_diff(Pj1Aj, actual_state_dim=state_dim)

        H = (
            sparse_left_action_for_input_diff(
                sparse_right_action_for_input_diff(P[j + 1], actual_state_dim=state_dim),
                actual_state_dim=state_dim,
            )
            + Rj
        )
        H[0, 0] += acc_rate_lim_weights[j]
        H[1, 1] += steer_rate_lim_weights[j]

        g_ = vector_sparse_left_action_for_input_diff(w[j + 1], actual_state_dim=state_dim) + Rj @ (
            inputs[j] - U_des[j]
        )
        g_[0] += acc_rate_lim_weights[j] * (inputs[j, 0] - acc_rate_lim_center[j])
        g_[1] += steer_rate_lim_weights[j] * (inputs[j, 1] - steer_rate_lim_center[j])

        one_over_det = 1 / (H[0, 0] * H[1, 1] - H[1, 0] * H[1, 0])

        H_inv = one_over_det * np.array([[H[1, 1], -H[1, 0]], [-H[1, 0], H[0, 0]]])

        H_inv_G[j] = H_inv @ G
        H_inv_g[j] = H_inv @ g_

        P[j] = (
            sparse_left_action_for_state_diff(Aj[:state_dim], Pj1Aj, actual_state_dim=state_dim)
            - G.T @ H_inv_G[j]
        )
        P[j][:6, :6] += Qj[:6, :6]
        P[j][acc_input_index, acc_input_index] += Qj[6, 6]
        P[j][steer_input_index, steer_input_index] += Qj[7, 7]

        P[j][acc_input_index, acc_input_index] += acc_lim_weights[j]
        P[j][steer_input_index, steer_input_index] += steer_lim_weights[j]

        w[j] = (
            vector_sparse_left_action_for_state_diff(
                Aj[:state_dim], w[j + 1], actual_state_dim=state_dim
            )
            - H_inv_G[j].T @ g_
        )

        Q_j_x = Qj @ (traj[j] - X_des[j])
        w[j, :6] += Q_j_x[:6]
        w[j, acc_input_index] += Q_j_x[6]
        w[j, steer_input_index] += Q_j_x[7]

        w[j, acc_input_index] += acc_lim_weights[j] * (traj[j, acc_index] - acc_lim_center[j])
        w[j, steer_input_index] += steer_lim_weights[j] * (
            traj[j, acc_index + 1] - steer_lim_center[j]
        )
        P_noise = np.zeros((nx, nx))
        w_noise = np.zeros(nx)
        if Sigma_x is not None:
            sigma_x = Sigma_x[j]
            d_sigma_x = dSigma_x[j]
            vec_part = P[j][0, 0] * d_sigma_x
            P_noise += d_sigma_x.reshape(-1, 1) @ vec_part.reshape(1, -1)
            w_noise += sigma_x * vec_part
        if Sigma_y is not None:
            sigma_y = Sigma_y[j]
            d_sigma_y = dSigma_y[j]
            vec_part = P[j][1, 1] * d_sigma_y
            P_noise += d_sigma_y.reshape(-1, 1) @ vec_part.reshape(1, -1)
            w_noise += sigma_y * vec_part
        if Sigma_v is not None:
            sigma_v = Sigma_v[j]
            d_sigma_v = dSigma_v[j]
            vec_part = P[j][2, 2] * d_sigma_v
            P_noise += d_sigma_v.reshape(-1, 1) @ vec_part.reshape(1, -1)
            w_noise += sigma_v * vec_part
        if Sigma_theta is not None:
            sigma_theta = Sigma_theta[j]
            d_sigma_theta = dSigma_theta[j]
            vec_part = P[j][3, 3] * d_sigma_theta
            P_noise += d_sigma_theta.reshape(-1, 1) @ vec_part.reshape(1, -1)
            w_noise += sigma_theta * vec_part
        if Sigma_acc is not None:
            sigma_acc = Sigma_acc[j]
            d_sigma_acc = dSigma_acc[j]
            vec_part = P[j][4, 4] * d_sigma_acc
            P_noise += d_sigma_acc.reshape(-1, 1) @ vec_part.reshape(1, -1)
            w_noise += sigma_acc * vec_part
        if Sigma_steer is not None:
            sigma_steer = Sigma_steer[j]
            d_sigma_steer = dSigma_steer[j]
            vec_part = P[j][5, 5] * d_sigma_steer
            P_noise += d_sigma_steer.reshape(-1, 1) @ vec_part.reshape(1, -1)
            w_noise += sigma_steer * vec_part

        P[j] += P_noise
        w[j] += w_noise
    return P, w, H_inv_G, H_inv_g


@njit(cache=True, fastmath=True)
def calc_line_search_candidates(A, B, H_inv_g, H_inv_G, inputs, ls):
    """Compute the candidate input columns for the line search in the final stage of iLQR."""
    ls_num = ls.shape[0]
    nx = A.shape[2]
    nu = B.shape[2]
    N = A.shape[0]

    Inputs = np.zeros((ls_num, N, nu))
    delta_x_k = np.zeros((ls_num, nx))
    for k in range(N):
        delta_u_k = -ls.reshape(-1, 1) @ H_inv_g[k].reshape(1, -1) - delta_x_k @ H_inv_G[k].T
        Inputs[:, k, :] = inputs[k] + delta_u_k
        delta_x_k = delta_x_k @ A[k].T + delta_u_k @ B[k].T
    return Inputs


class drive_iLQR:
    """Class that runs iLQR to compute optimal input values."""

    ls_step: float
    """ Line search step width."""

    max_iter_ls: int
    """ Number of candidates for line search."""

    max_iter_ilqr: int
    """ Maximum number of iLQR iterations."""

    ilqr_tol: float
    """ Tolerance to terminate iLQR iterations."""

    use_trained_model_diff: bool
    """ Whether to use the derivative of the trained model."""

    def __init__(
        self,
        ls_step,
        max_iter_ls,
        max_iter_ilqr,
        ilqr_tol,
        use_trained_model_diff,
    ):
        ir(drive_functions)
        self.ls_step = ls_step
        self.max_iter_ls = max_iter_ls
        self.max_iter_ilqr = max_iter_ilqr
        self.ilqr_tol = ilqr_tol
        self.use_trained_model_diff = use_trained_model_diff

        ls = np.ones(max_iter_ls + 1)
        for i in range(max_iter_ls - 1):
            ls[i + 1] = ls_step * ls[i]
        ls[-1] = 0.0
        self.ls = ls
        self.add_state_hc = False
        self.state_dim = actual_state_dim
        self.acc_index = acc_index
        self.steer_index = steer_index

    def calc_forward_trajectories_with_cost(
        self,
        x_current: np.ndarray,
        Inputs: np.ndarray,
        X_des: np.ndarray,
        U_des: np.ndarray,
        previous_error: np.ndarray,
        steer_rate_cost_coef: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Calculate the predicted trajectory and cost.

        Given the current state and the candidate input columns in the line search.
        """
        N = Inputs.shape[1]
        samples = Inputs.shape[0]
        Traj = np.zeros((samples, N + 1, x_current.shape[0]))
        Traj[:, 0, :] += x_current
        Cost = np.zeros(samples)

        Previous_error = np.tile(previous_error[:6], (samples, 1))
        for k in range(N):
            Cost += drive_functions.calc_cost(
                X_des[k],
                U_des[k],
                Traj[:, k, index_cost],
                Inputs[:, k, :],
                k,
                steer_rate_cost_coef,
            )
            Traj[:, k + 1], Previous_error = self.F_for_candidates(
                Traj[:, k], Inputs[:, k], Previous_error, k
            )
        Cost += drive_functions.calc_cost(
            X_des[N],
            U_des[N - 1],
            Traj[:, N, index_cost],
            Inputs[:, N - 1, :],
            N,
            steer_rate_cost_coef,
        )
        return Traj, Cost

    def calc_forward_trajectory_with_diff(
        self, x_current: np.ndarray, inputs: np.ndarray, previous_error: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Calculate the derivatives (Jacobians) of the predicted trajectory t with diff.

        This also calculates "the function that returns the predicted trajectory from the current state and the input sequence",
        given the current state and the input sequence.
        """
        N = inputs.shape[0]
        nx = x_current.shape[0]
        nu = inputs.shape[1]
        traj = np.zeros((N + 1, nx))
        traj[0] = x_current
        A = np.zeros((N, nx, nx))
        B = np.zeros((N, nx, nu))
        C = np.zeros((N, 6, nx))
        if self.add_state_hc:
            D = np.zeros((N, 6 + self.h_dim_double, nx + self.h_dim_double))
        previous_error_ = previous_error.copy()
        for k in range(N):
            if self.use_trained_model_diff:
                traj[k + 1], A[k], B[k], C[k], previous_error_ = self.F_with_diff(
                    traj[k], inputs[k], previous_error_, k
                )
                if self.add_state_hc:
                    D[k][:6, 6 : 6 + self.h_dim_double] = (
                        self.get_dy_dhc() * drive_functions.mpc_time_step
                    )
                    dhc_dx = self.get_dhc_dx()
                    D[k][6:, :6] = dhc_dx[:, :6]
                    D[k][6:, 6 + self.h_dim_double :] = dhc_dx[:, 6:]
                    D[k][6:, 6 : 6 + self.h_dim_double] = self.get_dhc_dhc()
            else:
                traj[k + 1], A[k], B[k], previous_error_ = self.F_with_initial_diff(
                    traj[k], inputs[k], previous_error_, k
                )
        if self.use_trained_model_diff:
            A[:, :6] += drive_functions.sg_filter_for_trained_model_diff(C)
            if self.add_state_hc:
                D = drive_functions.sg_filter_for_memory_diff(D)
                A_ = np.zeros((N, nx + self.h_dim_double, nx + self.h_dim_double))
                A_[:, :6, :6] = A[:, :6, :6]
                A_[:, 6 + self.h_dim_double :, :6] = A[:, 6:, :6]
                A_[:, :6, 6 + self.h_dim_double :] = A[:, :6, 6:]
                A_[:, 6 + self.h_dim_double :, 6 + self.h_dim_double :] = A[:, 6:, 6:]
                A_[:, : 6 + self.h_dim_double] += D
                B_ = np.zeros((N, nx + self.h_dim_double, nu))
                B_[:, :6] = B[:, :6]
                B_[:, 6 + self.h_dim_double :] = B[:, 6:]
                return traj, A_, B_

        return traj, A, B

    def compute_optimal_control(
        self,
        x_current: np.ndarray,
        inputs: np.ndarray,
        X_des: np.ndarray,
        U_des: np.ndarray,
        previous_error: np.ndarray,
        x_noise: Callable | None,
        y_noise: Callable | None,
        v_noise: Callable | None,
        theta_noise: Callable | None,
        acc_noise: Callable | None,
        steer_noise: Callable | None,
        steer_rate_cost_coef: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, bool]:
        """Calculate the optimal predictive trajectory and the input sequence at that time.

        It also determines whether the tolerance has been reached and returns whether the optimization should continue or not.
        Performs one iteration of iLQR.
        """
        N = inputs.shape[0]
        self.time_1 = time.time()
        traj, A, B = self.calc_forward_trajectory_with_diff(x_current, inputs, previous_error)
        (
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
        ) = drive_functions.transform_Q_R(X_des, U_des, traj, inputs, steer_rate_cost_coef)
        self.time_2 = time.time()
        if x_noise is None:
            Sigma_x = None
            dSigma_x = None
        else:
            Sigma_x, dSigma_x = x_noise(np.hstack((traj[:-1, [2, 3, 5]], traj[:-1, 6:])))
            Sigma_x = drive_functions.sg_filter_for_noise(Sigma_x)
            dSigma_x = drive_functions.sg_filter_for_noise(dSigma_x)
        if y_noise is None:
            Sigma_y = None
            dSigma_y = None
        else:
            Sigma_y, dSigma_y = y_noise(np.hstack((traj[:-1, [2, 3, 5]], traj[:-1, 6:])))
            Sigma_y = drive_functions.sg_filter_for_noise(Sigma_y)
            dSigma_y = drive_functions.sg_filter_for_noise(dSigma_y)
        if v_noise is None:
            Sigma_v = None
            dSigma_v = None
        else:
            Sigma_v, dSigma_v = v_noise(np.hstack((traj[:-1, [2, 3, 5]], traj[:-1, 6:])))
            Sigma_v = drive_functions.sg_filter_for_noise(Sigma_v)
            dSigma_v = drive_functions.sg_filter_for_noise(dSigma_v)
        if theta_noise is None:
            Sigma_theta = None
            dSigma_theta = None
        else:
            Sigma_theta, dSigma_theta = theta_noise(
                np.hstack((traj[:-1, [2, 3, 5]], traj[:-1, 6:]))
            )
            Sigma_theta = drive_functions.sg_filter_for_noise(Sigma_theta)
            dSigma_theta = drive_functions.sg_filter_for_noise(dSigma_theta)
        if acc_noise is None:
            Sigma_acc = None
            dSigma_acc = None
        else:
            Sigma_acc, dSigma_acc = acc_noise(np.hstack((traj[:-1, [2, 3, 5]], traj[:-1, 6:])))
            Sigma_acc = drive_functions.sg_filter_for_noise(Sigma_acc)
            dSigma_acc = drive_functions.sg_filter_for_noise(dSigma_acc)

        if steer_noise is None:
            Sigma_steer = None
            dSigma_steer = None
        else:
            Sigma_steer, dSigma_steer = steer_noise(
                np.hstack((traj[:-1, [2, 3, 5]], traj[:-1, 6:]))
            )
            Sigma_steer = drive_functions.sg_filter_for_noise(Sigma_steer)
            dSigma_steer = drive_functions.sg_filter_for_noise(dSigma_steer)
        P, w, H_inv_G, H_inv_g = compute_iLQR_coef(
            traj=traj[:, index_cost],
            inputs=inputs,
            A=A,
            B=B,
            Q_total=Q_total,
            R_total=R_total,
            acc_lim_weights=acc_lim_weights,
            acc_lim_center=acc_lim_center,
            steer_lim_weights=steer_lim_weights,
            steer_lim_center=steer_lim_center,
            acc_rate_lim_weights=acc_rate_lim_weights,
            acc_rate_lim_center=acc_rate_lim_center,
            steer_rate_lim_weights=steer_rate_lim_weights,
            steer_rate_lim_center=steer_rate_lim_center,
            N=N,
            X_des=X_des,
            U_des=U_des,
            Sigma_x=Sigma_x,
            dSigma_x=dSigma_x,
            Sigma_y=Sigma_y,
            dSigma_y=dSigma_y,
            Sigma_v=Sigma_v,
            dSigma_v=dSigma_v,
            Sigma_theta=Sigma_theta,
            dSigma_theta=dSigma_theta,
            Sigma_acc=Sigma_acc,
            dSigma_acc=dSigma_acc,
            Sigma_steer=Sigma_steer,
            dSigma_steer=dSigma_steer,
            acc_input_index=self.acc_index,
            steer_input_index=self.steer_index,
            state_dim=self.state_dim,
        )
        self.time_3 = time.time()
        # start line search
        Inputs = calc_line_search_candidates(A, B, H_inv_g, H_inv_G, inputs, self.ls)
        self.time_4 = time.time()
        Traj, Cost = self.calc_forward_trajectories_with_cost(
            x_current, Inputs, X_des, U_des, previous_error, steer_rate_cost_coef
        )
        best_index = Cost.argmin()
        self.time_5 = time.time()

        if Cost[best_index] < (1 - self.ilqr_tol) * Cost[-1]:
            proceed = True
        else:
            proceed = False

        best_traj = Traj[best_index]
        best_inputs = Inputs[best_index]
        return best_inputs, best_inputs[0], best_traj, proceed

    def receive_model(
        self, F_with_initial_diff: Callable, F_with_diff: Callable, F_for_candidates: Callable
    ):
        """Receive vehicle model for control."""
        self.F_with_initial_diff = F_with_initial_diff
        self.F_with_diff = F_with_diff
        self.F_for_candidates = F_for_candidates

    def receive_memory_diff(
        self, get_dhc_dx: Callable, get_dhc_dhc: Callable, get_dy_dhc: Callable
    ):
        """Receive getter hc diff."""
        self.get_dhc_dx = get_dhc_dx
        self.get_dhc_dhc = get_dhc_dhc
        self.get_dy_dhc = get_dy_dhc
        self.h_dim_double = get_dhc_dx().shape[0]
        self.add_state_hc = True
        print(self.h_dim_double)
        self.state_dim += self.h_dim_double
        self.acc_index += self.h_dim_double
        self.steer_index += self.h_dim_double
