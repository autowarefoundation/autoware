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

# cspell: ignore optim numba interp


"""Define a drive_controller class that calculates the final input of the Proxima-side MPC."""
from functools import partial
from importlib import reload as ir
import threading
import time
from typing import Literal

from autoware_smart_mpc_trajectory_follower.scripts import drive_GP
from autoware_smart_mpc_trajectory_follower.scripts import drive_NN
from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
from autoware_smart_mpc_trajectory_follower.scripts import drive_iLQR
from autoware_smart_mpc_trajectory_follower.scripts import drive_mppi
import numpy as np
import scipy.interpolate  # type: ignore
from sklearn.preprocessing import PolynomialFeatures
import torch

ctrl_index_for_polynomial_reg = np.concatenate(
    (
        np.arange(3),
        [3 + drive_functions.acc_delay_step],
        [3 + drive_functions.acc_ctrl_queue_size + drive_functions.steer_delay_step],
    )
)


class drive_controller:
    """Calculate the final input of the Proxima-side MPC.

    Fields that can take None follow these specifications unless otherwise noted.
    * If None, read from drive_functions.py
    * If Bool, the input value is used
    """

    use_trained_model: bool | None
    """Whether to use trained models """

    use_trained_model_diff: bool | None
    """Assuming a trained model is used, whether or not to use even its derivatives."""

    update_trained_model: bool | None
    """Whether to update the model online with the data under control."""

    mode: Literal["pure_pursuit", "naive_pure_pursuit", "mppi", "ilqr", "mppi_ilqr"] | None
    """which algorithm to use"""

    use_x_noise: bool | None
    """Whether to reflect the uncertainty of the straight-line position when performing iLQG"""

    use_y_noise: bool | None
    """Whether to reflect the uncertainty of the lateral position when performing iLQG"""

    use_v_noise: bool | None
    """ Whether to reflect the uncertainty of the straight-line velocity when performing iLQG"""

    use_theta_noise: bool | None
    """ Whether to reflect the uncertainty of the yaw angle when performing iLQG"""

    use_acc_noise: bool | None
    """ Whether to reflect the uncertainty of the straight-line acceleration when performing iLQG"""

    use_steer_noise: bool | None
    """ Whether to reflect the uncertainty of the steer when performing iLQG"""

    tanh_gain: float
    """ Gain of the tanh term in the loss function in learning. """

    lam: float
    """ Weight fir the tanh term in the loss function in learning."""

    alpha_1: float
    """ Coefficients of the L¹ regularization term in online learning."""

    alpha_2: float
    """ Coefficients of the L² regularization term in online learning."""

    drive_batch_size: int
    """ Online learning batch sizes """

    drive_learning_rate: float
    """ Learning rate for online learning """

    model_file_name: str
    """ file name of the trained model to be loaded """

    load_train_data_dir: str
    """ directory for training data to be loaded during online learning """

    load_GP_dir: str | None
    """ directory to load the GP (Gaussian process regression) models for iLQG. """

    def __init__(
        self,
        use_trained_model: bool | None = None,
        use_trained_model_diff=None,
        use_memory_diff=None,
        update_trained_model=None,
        mode=None,
        use_x_noise=None,
        use_y_noise=None,
        use_v_noise=None,
        use_theta_noise=None,
        use_acc_noise=None,
        use_steer_noise=None,
        tanh_gain=10.0,
        lam=0.1,
        alpha_1=1e-7,
        alpha_2=0.0,
        drive_batch_size=200,
        drive_learning_rate=1e-6,
        model_file_name="model_for_test_drive.pth",
        load_train_data_dir=".",
        load_GP_dir=None,
        load_polynomial_reg_dir=None,
    ):
        """Initialize.

        Some arguments have a default value of None in order to reload drive_functions and update the parameters.
        """
        # reload drive_functions
        ir(drive_functions)
        ir(drive_NN)
        ir(drive_iLQR)
        ir(drive_mppi)

        if mode is None:
            self.mode = drive_functions.mode
        else:
            self.mode = mode

        if mode == "pure_pursuit" or mode == "naive_pure_pursuit":
            use_trained_model = False

        if use_trained_model is None:
            self.use_trained_model = drive_functions.use_trained_model
        else:
            self.use_trained_model = use_trained_model
        if not self.use_trained_model:
            self.use_trained_model_diff = False
            self.update_trained_model = False
            self.use_memory_diff = False
        else:
            if use_trained_model_diff is None:
                self.use_trained_model_diff = drive_functions.use_trained_model_diff
            else:
                self.use_trained_model_diff = use_trained_model_diff
            if update_trained_model is None:
                self.update_trained_model = drive_functions.update_trained_model
            else:
                self.update_trained_model = update_trained_model
            if use_memory_diff is None:
                self.use_memory_diff = drive_functions.use_memory_diff
            else:
                self.use_memory_diff = use_memory_diff

        self.acc_delay_step = drive_functions.acc_delay_step
        self.steer_delay_step = drive_functions.steer_delay_step
        self.acc_time_constant_ctrl = drive_functions.acc_time_constant
        self.steer_time_constant_ctrl = drive_functions.steer_time_constant

        # Term to consider noise in iLQG
        if use_x_noise is None:
            self.use_x_noise = drive_functions.use_x_noise
        else:
            self.use_x_noise = use_x_noise
        if use_y_noise is None:
            self.use_y_noise = drive_functions.use_y_noise
        else:
            self.use_y_noise = use_y_noise
        if use_v_noise is None:
            self.use_v_noise = drive_functions.use_v_noise
        else:
            self.use_v_noise = use_v_noise
        if use_theta_noise is None:
            self.use_theta_noise = drive_functions.use_theta_noise
        else:
            self.use_theta_noise = use_theta_noise
        if use_acc_noise is None:
            self.use_acc_noise = drive_functions.use_acc_noise
        else:
            self.use_acc_noise = use_acc_noise
        if use_steer_noise is None:
            self.use_steer_noise = drive_functions.use_steer_noise
        else:
            self.use_steer_noise = use_steer_noise
        if not self.use_trained_model:
            self.use_x_noise = False
            self.use_y_noise = False
            self.use_v_noise = False
            self.use_theta_noise = False
            self.use_acc_noise = False
            self.use_steer_noise = False

        if load_GP_dir is None:
            self.load_GP_dir = drive_functions.load_dir
        else:
            self.load_GP_dir = load_GP_dir
        if load_polynomial_reg_dir is None:
            self.load_polynomial_reg_dir = drive_functions.load_dir
        else:
            self.load_polynomial_reg_dir = load_polynomial_reg_dir

        self.drive_batch_size = drive_batch_size
        self.alpha_1 = alpha_1
        self.alpha_2 = alpha_2
        self.tanh_gain = tanh_gain
        self.lam = lam

        self.init = True
        self.initialize_input_queue = True
        self.mppi = drive_mppi.drive_mppi(
            drive_functions.lam,
            drive_functions.Sigma,
            drive_functions.max_iter_mppi,
            drive_functions.sample_num,
            drive_functions.mppi_tol,
            drive_functions.mppi_step,
        )
        self.ilqr = drive_iLQR.drive_iLQR(
            drive_functions.ls_step,
            drive_functions.max_iter_ls,
            drive_functions.max_iter_ilqr,
            drive_functions.ilqr_tol,
            self.use_trained_model_diff,
        )
        self.err = 0
        if self.use_trained_model and drive_functions.use_memory_for_training:
            print("use_memory_diff", self.use_memory_diff)
        if self.use_trained_model:
            self.model = torch.load(model_file_name)
            polynomial_reg_info = np.load(self.load_polynomial_reg_dir + "/polynomial_reg_info.npz")
            self.A_for_polynomial_reg = polynomial_reg_info["A"]
            self.b_for_polynomial_reg = polynomial_reg_info["b"]
            self.deg = int(polynomial_reg_info["deg"])
            self.polynomial_features = PolynomialFeatures(degree=self.deg, include_bias=False)
            if drive_functions.use_memory_for_training:
                self.transform_model = drive_NN.transform_model_with_memory_to_c(
                    self.model,
                    self.A_for_polynomial_reg,
                    self.b_for_polynomial_reg,
                    self.deg,
                    self.acc_delay_step,
                    self.steer_delay_step,
                    drive_functions.acc_ctrl_queue_size,
                    drive_functions.steer_ctrl_queue_size,
                    drive_functions.steer_ctrl_queue_size_core,
                )
                self.h = np.zeros(self.model.lstm.weight_hh_l0.shape[1])
                self.c = (self.h).copy()
                if self.use_memory_diff:
                    self.ilqr.receive_memory_diff(
                        self.transform_model.transform.get_dhc_dx,
                        self.transform_model.transform.get_dhc_dhc,
                        self.transform_model.transform.get_dy_dhc,
                    )
            else:
                self.transform_model = drive_NN.transform_model_to_c(
                    self.model,
                    self.A_for_polynomial_reg,
                    self.b_for_polynomial_reg,
                    self.deg,
                    self.acc_delay_step,
                    self.steer_delay_step,
                    drive_functions.acc_ctrl_queue_size,
                    drive_functions.steer_ctrl_queue_size,
                    drive_functions.steer_ctrl_queue_size_core,
                )
            self.pred = self.transform_model.pred
            if drive_functions.reflect_only_poly_diff:
                self.pred_with_diff = self.transform_model.pred_with_poly_diff
            elif self.use_memory_diff and drive_functions.use_memory_for_training:
                self.pred_with_diff = self.transform_model.pred_with_memory_diff
            else:
                self.pred_with_diff = self.transform_model.pred_with_diff
            self.loss_fn = torch.nn.L1Loss()
            self.drive_optimizer = torch.optim.Adam(
                params=self.model.parameters(), lr=drive_learning_rate
            )
            self.F_N_initial_diff = partial(
                drive_functions.F_with_model_initial_diff,
                pred=self.transform_model.pred,
                i=self.acc_delay_step,
                j=self.steer_delay_step,
                acc_time_constant_ctrl=self.acc_time_constant_ctrl,
                steer_time_constant_ctrl=self.steer_time_constant_ctrl,
            )

            self.F_N_diff = partial(
                drive_functions.F_with_model_diff,
                pred=self.pred_with_diff,
                i=self.acc_delay_step,
                j=self.steer_delay_step,
                acc_time_constant_ctrl=self.acc_time_constant_ctrl,
                steer_time_constant_ctrl=self.steer_time_constant_ctrl,
            )
            self.F_N_only_state = partial(
                drive_functions.F_with_model,
                pred=self.transform_model.pred_only_state,
                i=self.acc_delay_step,
                j=self.steer_delay_step,
                acc_time_constant_ctrl=self.acc_time_constant_ctrl,
                steer_time_constant_ctrl=self.steer_time_constant_ctrl,
            )
            self.F_N_for_candidates = partial(
                drive_functions.F_with_model_for_candidates,
                Pred=self.transform_model.Pred,
                i=self.acc_delay_step,
                j=self.steer_delay_step,
                acc_time_constant_ctrl=self.acc_time_constant_ctrl,
                steer_time_constant_ctrl=self.steer_time_constant_ctrl,
            )
        else:
            self.F_N_diff = partial(
                drive_functions.F_with_history_and_diff,
                i=self.acc_delay_step,
                j=self.steer_delay_step,
                acc_time_constant_ctrl=self.acc_time_constant_ctrl,
                steer_time_constant_ctrl=self.steer_time_constant_ctrl,
            )
            self.F_N_initial_diff = self.F_N_diff
            self.F_N_only_state = partial(
                drive_functions.F_with_history,
                i=self.acc_delay_step,
                j=self.steer_delay_step,
                acc_time_constant_ctrl=self.acc_time_constant_ctrl,
                steer_time_constant_ctrl=self.steer_time_constant_ctrl,
            )
            self.F_N_for_candidates = partial(
                drive_functions.F_with_history_for_candidates,
                i=self.acc_delay_step,
                j=self.steer_delay_step,
                acc_time_constant_ctrl=self.acc_time_constant_ctrl,
                steer_time_constant_ctrl=self.steer_time_constant_ctrl,
            )

        if self.use_x_noise:
            self.x_noise = drive_GP.transform_GP_to_numba(
                0, dir_name=self.load_GP_dir
            ).pred_deviations
        else:
            self.x_noise = None
        if self.use_y_noise:
            self.y_noise = drive_GP.transform_GP_to_numba(
                1, dir_name=self.load_GP_dir
            ).pred_deviations
        else:
            self.y_noise = None
        if self.use_v_noise:
            self.v_noise = drive_GP.transform_GP_to_numba(
                2, dir_name=self.load_GP_dir
            ).pred_deviations
        else:
            self.v_noise = None
        if self.use_theta_noise:
            self.theta_noise = drive_GP.transform_GP_to_numba(
                3, dir_name=self.load_GP_dir
            ).pred_deviations
        else:
            self.theta_noise = None
        if self.use_acc_noise:
            self.acc_noise = drive_GP.transform_GP_to_numba(
                4, dir_name=self.load_GP_dir
            ).pred_deviations
        else:
            self.acc_noise = None
        if self.use_steer_noise:
            self.steer_noise = drive_GP.transform_GP_to_numba(
                5, dir_name=self.load_GP_dir
            ).pred_deviations
        else:
            self.steer_noise = None

        self.X_des_history: list[np.ndarray] = []

        self.X_input_list = []
        self.Y_output_list = []
        # Initial data during online training

        self.emergency = False

        self.counter = 0
        self.model_update_flag = True
        if self.update_trained_model:
            train_data = np.load(load_train_data_dir + "/train_data.npz")
            self.X_input_list = train_data["X_input"][
                -drive_functions.max_train_data_size :
            ].tolist()
            self.Y_output_list = (
                train_data["Y_output"][-drive_functions.max_train_data_size :]
                - self.polynomial_features.fit_transform(
                    train_data["X_input"][
                        -drive_functions.max_train_data_size :, ctrl_index_for_polynomial_reg
                    ]
                )
                @ (self.A_for_polynomial_reg).T
                - self.b_for_polynomial_reg
            ).tolist()
            self.model_updater = threading.Thread(target=self.update_model)
            self.model_updater.start()

        self.x_old = np.empty(6)

        self.X_queue_for_learning: list[np.ndarray] = []
        self.acc_input_queue_for_learning: list[np.ndarray] = []
        self.steer_input_queue_for_learning: list[np.ndarray] = []
        self.time_stamp_queue_for_learning: list[float] = []
        self.X_smoothed_queue: list[np.ndarray] = []

        self.pre_X_input_list: list[np.ndarray] = []
        self.pre_Y_output_list: list[np.ndarray] = []

        self.time_stamp_for_update_lstm: list[float] = []
        self.X_queue_for_update_lstm: list[np.ndarray] = []

    def __del__(self):
        print("control finished")

    def get_optimal_control(
        self,
        x_current_: np.ndarray,
        time_stamp: list[float],
        X_des_: np.ndarray,
        U_des: np.ndarray = np.zeros((drive_functions.N, drive_functions.nu_0)),
    ) -> np.ndarray:
        """Calculate the optimal control input from the current state, target trajectory, and target input trajectory.

        The return value is an np.ndarray of the form `(drive_functions.nu_0, )`.
        """
        if self.initialize_input_queue:
            x_current = x_current_.copy()
            X_des, diff_delta = drive_functions.transform_yaw_for_X_des(
                x_current,
                X_des_,
            )
            self.u_opt = X_des[0, drive_functions.nx_0 + np.arange(drive_functions.nu_0)]
            self.acc_input_queue = self.u_opt[0] * np.ones(drive_functions.acc_ctrl_queue_size)
            self.steer_input_queue = self.u_opt[1] * np.ones(drive_functions.steer_ctrl_queue_size)
            self.nominal_inputs = drive_functions.U_des_from_X_des(self.u_opt, X_des, diff_delta)
            self.previous_error = np.zeros(8)

            self.X_queue_for_learning.clear()
            self.acc_input_queue_for_learning.clear()
            self.steer_input_queue_for_learning.clear()
            self.time_stamp_queue_for_learning.clear()
            self.X_smoothed_queue.clear()

            self.pre_X_input_list.clear()
            self.pre_Y_output_list.clear()

            self.time_stamp_for_update_lstm.clear()
            self.X_queue_for_update_lstm.clear()
            if self.use_trained_model and drive_functions.use_memory_for_training:
                self.h = np.zeros(self.model.lstm.weight_hh_l0.shape[1])
                self.c = (self.h).copy()

            self.initialize_X_smoothing_time_stamp = True
            self.acc_fb_1 = 0.0
            self.acc_fb_2 = 0.0
            self.steer_fb_1 = 0.0
            self.steer_fb_2 = 0.0

        else:
            x_current = drive_functions.transform_yaw_for_x_current(self.x_old, x_current_)
            X_des, diff_delta = drive_functions.transform_yaw_for_X_des(
                x_current,
                X_des_,
            )
        if drive_functions.use_max_curvature:
            curvature = np.abs(X_des[:, 5]).max() / drive_functions.L
        else:
            curvature = np.abs(X_des[:, 5]).mean() / drive_functions.L
        steer_rate_cost_coef = drive_functions.calc_steer_rate_cost_coef(curvature)
        if self.init:
            self.X_current_queue = []
            self.init = False
        self.X_des = X_des

        self.X_current = np.concatenate(
            (x_current, self.acc_input_queue[::-1], self.steer_input_queue[::-1])
        )
        self.u_old = np.array([self.acc_input_queue[-1], self.steer_input_queue[-1]])
        if self.use_trained_model and drive_functions.use_memory_for_training:
            if len(time_stamp) > 0:
                self.update_lstm_info(time_stamp[-1])

        self.X_current_queue.append(self.X_current.copy())
        if len(self.X_current_queue) > drive_functions.mpc_freq:
            self.X_current_queue.pop(0)
        if self.mode == "mppi":  # Avoid duplicate filtering for "mppi_ilqr" mode
            self.nominal_inputs = drive_functions.sg_filter_for_nominal_inputs(self.nominal_inputs)
        if self.mode == "mppi" or self.mode == "mppi_ilqr":  # Run mppi
            self.start_mppi = time.time()
            proceed = True
            for k in range(drive_functions.max_iter_mppi):
                if not proceed:
                    break
                if drive_functions.use_memory_for_training and self.use_trained_model:
                    self.transform_model.transform.set_lstm(self.h, self.c)
                    self.transform_model.transform.set_lstm_for_candidate(
                        self.h, self.c, drive_functions.sample_num
                    )
                self.mppi.receive_model(self.F_N_for_candidates, self.F_N_only_state)
                self.nominal_inputs, self.u_opt_dot, nominal_traj, self.mppi_candidates, proceed = (
                    self.mppi
                ).compute_optimal_control(
                    self.X_current,
                    self.nominal_inputs,
                    X_des,
                    U_des,
                    self.previous_error[:6],
                )
                self.nominal_traj_mppi = nominal_traj.copy()
            self.end_mppi = time.time()
        if self.mode == "ilqr" or self.mode == "mppi_ilqr":
            self.start_ilqr = time.time()
            proceed = True
            for k in range(drive_functions.max_iter_ilqr):
                if not proceed:
                    break
                if drive_functions.use_memory_for_training and self.use_trained_model:
                    self.transform_model.transform.set_lstm(self.h, self.c)
                    self.transform_model.transform.set_lstm_for_candidate(
                        self.h, self.c, drive_functions.max_iter_ls + 1
                    )
                self.nominal_inputs = drive_functions.sg_filter_for_nominal_inputs(
                    self.nominal_inputs
                )
                self.ilqr.receive_model(
                    self.F_N_initial_diff, self.F_N_diff, self.F_N_for_candidates
                )
                self.nominal_inputs, self.u_opt_dot, nominal_traj, proceed = (
                    self.ilqr
                ).compute_optimal_control(
                    self.X_current,
                    self.nominal_inputs,
                    X_des,
                    U_des,
                    self.previous_error,
                    self.x_noise,
                    self.y_noise,
                    self.v_noise,
                    self.theta_noise,
                    self.acc_noise,
                    self.steer_noise,
                    steer_rate_cost_coef,
                )
                self.nominal_traj_ilqr = nominal_traj.copy()
                self.nominal_inputs_ilqr = self.nominal_inputs.copy()
                err = drive_functions.calc_maximum_trajectory_error(self.nominal_traj_ilqr, X_des)
                if err > drive_functions.cap_pred_error[0]:
                    if err < drive_functions.cap_pred_error[1]:
                        mix_ratio = (drive_functions.cap_pred_error[1] - err) / (
                            drive_functions.cap_pred_error[1] - drive_functions.cap_pred_error[0]
                        )
                    else:
                        mix_ratio = 0.0
                    self.nominal_inputs = mix_ratio * self.nominal_inputs + (
                        1 - mix_ratio
                    ) * drive_functions.U_des_from_X_des(self.u_old, X_des, diff_delta)
                self.err = err
            if err > 1.0:
                self.emergency = True

            self.time_1 = self.ilqr.time_1
            self.time_2 = self.ilqr.time_2
            self.time_3 = self.ilqr.time_3
            self.time_4 = self.ilqr.time_4
            self.time_5 = self.ilqr.time_5
            self.end_ilqr = time.time()

        if self.mode == "pure_pursuit":
            nominal_traj = np.zeros((drive_functions.N + 1, x_current_.shape[0]))

            u_opt = drive_functions.pure_pursuit_control(
                pos_xy_obs=x_current_[:2],
                pos_yaw_obs=x_current_[3],
                longitudinal_vel_obs=x_current_[2],
                pos_xy_ref=X_des_[0, :2],
                pos_yaw_ref=X_des_[0, 3],
                longitudinal_vel_ref=X_des_[0, 2],
            )
            self.u_opt_dot = (u_opt - self.u_old) / drive_functions.ctrl_time_step

        if self.mode == "naive_pure_pursuit":
            nominal_traj = np.zeros((drive_functions.N + 1, x_current_.shape[0]))
            lookahead_distance = (
                drive_functions.naive_pure_pursuit_lookahead_coef * x_current_[2]
                + drive_functions.naive_pure_pursuit_lookahead_intercept
            )
            targetIndex = np.abs(
                ((X_des_[:, :2] - x_current_[:2]) ** 2).sum(axis=1) - lookahead_distance**2
            ).argmin()

            u_opt = drive_functions.naive_pure_pursuit_control(
                pos_xy_obs=x_current_[:2],
                pos_yaw_obs=x_current_[3],
                longitudinal_vel_obs=x_current_[2],
                pos_xy_ref_target=X_des_[targetIndex, :2],
                longitudinal_vel_ref_nearest=X_des_[0, 2],
            )
            self.u_opt_dot = (u_opt - self.u_old) / drive_functions.ctrl_time_step

        if self.use_trained_model:
            if drive_functions.use_memory_for_training:
                self.transform_model.transform.set_lstm(self.h, self.c)
            self.previous_error = drive_functions.error_decay * self.previous_error + (
                1 - drive_functions.error_decay
            ) * self.pred(self.X_current)

        self.nominal_traj = nominal_traj

        self.u_opt = self.u_old + self.u_opt_dot * drive_functions.ctrl_time_step
        (
            steer_lim,
            steer_rate_lim_lb,
            steer_rate_lim_ub,
            acc_lim,
            acc_rate_lim,
        ) = drive_functions.calc_limits(x_current[2], x_current[4], x_current[5])
        if not self.initialize_input_queue and (
            self.mode != "pure_pursuit" and self.mode != "naive_pure_pursuit"
        ):
            self.acc_fb_1, self.acc_fb_2 = np.clip(
                drive_functions.acc_prediction_error_compensation(
                    self.X_current,
                    self.X_old,
                    self.u_old,
                    self.previous_error,
                    self.acc_fb_1,
                    self.acc_fb_2,
                    self.acc_time_stamp - self.acc_time_stamp_old,
                ),
                -drive_functions.max_error_acc,
                drive_functions.max_error_acc,
            )
            self.steer_fb_1, self.steer_fb_2 = np.clip(
                drive_functions.steer_prediction_error_compensation(
                    self.X_current,
                    self.X_old,
                    self.u_old,
                    self.previous_error,
                    self.steer_fb_1,
                    self.steer_fb_2,
                    self.steer_time_stamp - self.steer_time_stamp_old,
                ),
                -drive_functions.max_error_steer,
                drive_functions.max_error_steer,
            )
            self.u_opt[0] += drive_functions.acc_fb_gain * (
                2 * self.acc_fb_1 - drive_functions.acc_fb_sec_order_ratio * self.acc_fb_2
            )
            self.u_opt[1] += drive_functions.steer_fb_gain * (
                2 * self.steer_fb_1 - drive_functions.steer_fb_sec_order_ratio * self.steer_fb_2
            )

        self.u_opt = drive_functions.u_cut_off(
            self.u_opt,
            self.u_old,
            steer_lim,
            steer_rate_lim_lb,
            steer_rate_lim_ub,
            acc_lim,
            acc_rate_lim,
        )

        self.initialize_input_queue = False
        self.X_des_history.append(X_des_[0])
        self.initial_guess_for_debug = (self.nominal_inputs).copy()

        self.x_old = x_current
        self.X_old = (self.X_current).copy()
        self.acc_time_stamp_old = self.acc_time_stamp
        self.steer_time_stamp_old = self.steer_time_stamp
        return self.u_opt

    def update_input_queue(
        self,
        time_stamp: list[float],
        acc_history: list[float],
        steer_history: list[float],
        x_current_: np.ndarray,
        acc_time_stamp: float,
        steer_time_stamp: float,
    ) -> None:
        """Receives the history of the acceleration and steer inputs and their time stamps.

        And interpolates them to be the history of the time steps used by the controller.
        The return value can be passed to get_optimal_control.
        """
        self.acc_time_stamp = acc_time_stamp
        self.steer_time_stamp = steer_time_stamp
        if not self.initialize_input_queue:
            if len(time_stamp) == 1:
                self.acc_input_queue[-1] = acc_history[0]
                self.steer_input_queue[-1] = steer_history[0]
            elif len(time_stamp) > 1:
                ctrl_num = int((time_stamp[-1] - time_stamp[0]) / drive_functions.ctrl_time_step)
                acc_num = min(drive_functions.acc_ctrl_queue_size, ctrl_num)
                steer_num = min(drive_functions.steer_ctrl_queue_size, ctrl_num)
                time_stamp_acc = (
                    drive_functions.ctrl_time_step * np.arange(acc_num)
                    - (acc_num - 1) * drive_functions.ctrl_time_step
                    + time_stamp[-1]
                )
                time_stamp_steer = (
                    drive_functions.ctrl_time_step * np.arange(steer_num)
                    - (steer_num - 1) * drive_functions.ctrl_time_step
                    + time_stamp[-1]
                )
                acc_interpolate = scipy.interpolate.interp1d(
                    np.array(time_stamp), np.array(acc_history)
                )
                steer_interpolate = scipy.interpolate.interp1d(
                    np.array(time_stamp), np.array(steer_history)
                )

                self.acc_input_queue[drive_functions.acc_ctrl_queue_size - acc_num :] = (
                    acc_interpolate(time_stamp_acc)
                )
                self.steer_input_queue[drive_functions.steer_ctrl_queue_size - steer_num :] = (
                    steer_interpolate(time_stamp_steer)
                )

                if (
                    acc_num == drive_functions.acc_ctrl_queue_size
                    and steer_num == drive_functions.steer_ctrl_queue_size
                ):
                    self.X_queue_for_learning.append(x_current_)
                    self.acc_input_queue_for_learning.append(np.array(self.acc_input_queue)[::-1])
                    self.steer_input_queue_for_learning.append(
                        np.array(self.steer_input_queue)[::-1]
                    )
                    self.time_stamp_queue_for_learning.append(time_stamp[-1])
                    if (
                        self.initialize_X_smoothing_time_stamp
                        and self.time_stamp_queue_for_learning[-1]
                        - self.time_stamp_queue_for_learning[0]
                        > drive_functions.ctrl_time_step
                        * drive_functions.max_input_queue_size_for_learning
                    ):
                        self.initialize_X_smoothing_time_stamp = False
                        self.X_smoothing_time_stamp = (
                            time_stamp[-1] - drive_functions.ctrl_time_step
                        )

                    if not self.initialize_X_smoothing_time_stamp:
                        while (
                            self.time_stamp_queue_for_learning[-1]
                            > drive_functions.ctrl_time_step + self.X_smoothing_time_stamp
                        ):
                            self.X_smoothing_time_stamp += drive_functions.ctrl_time_step
                            time_stamp_smoothing = (
                                drive_functions.ctrl_time_step
                                * np.arange(drive_functions.max_input_queue_size_for_learning)
                                - (drive_functions.max_input_queue_size_for_learning - 1)
                                * drive_functions.ctrl_time_step
                                + self.X_smoothing_time_stamp
                            )
                            x_queue = scipy.interpolate.interp1d(
                                np.array(self.time_stamp_queue_for_learning),
                                np.array(self.X_queue_for_learning).T,
                            )(time_stamp_smoothing).T
                            acc_input_queue = scipy.interpolate.interp1d(
                                np.array(self.time_stamp_queue_for_learning),
                                np.array(self.acc_input_queue_for_learning).T,
                            )(time_stamp_smoothing).T
                            steer_input_queue = scipy.interpolate.interp1d(
                                np.array(self.time_stamp_queue_for_learning),
                                np.array(self.steer_input_queue_for_learning).T,
                            )(time_stamp_smoothing).T
                            X_smoothed = np.zeros(
                                drive_functions.nx_0
                                + drive_functions.acc_ctrl_queue_size
                                + drive_functions.steer_ctrl_queue_size
                            )
                            X_smoothed[0:4] = x_queue[
                                drive_functions.max_input_queue_size_for_learning // 2, 0:4
                            ]
                            X_smoothed[4] = np.dot(
                                x_queue[
                                    drive_functions.max_input_queue_size_for_learning // 2
                                    - drive_functions.kernel_acc_for_learning.shape[0]
                                    // 2 : drive_functions.max_input_queue_size_for_learning
                                    // 2
                                    + drive_functions.kernel_acc_for_learning.shape[0] // 2
                                    + 1,
                                    4,
                                ],
                                drive_functions.kernel_acc_for_learning,
                            )
                            X_smoothed[5] = np.dot(
                                x_queue[
                                    drive_functions.max_input_queue_size_for_learning // 2
                                    - drive_functions.kernel_steer_for_learning.shape[0]
                                    // 2 : drive_functions.max_input_queue_size_for_learning
                                    // 2
                                    + drive_functions.kernel_steer_for_learning.shape[0] // 2
                                    + 1,
                                    5,
                                ],
                                drive_functions.kernel_steer_for_learning,
                            )
                            X_smoothed[
                                drive_functions.nx_0 : drive_functions.nx_0
                                + drive_functions.acc_ctrl_queue_size
                            ] = (
                                acc_input_queue[
                                    drive_functions.max_input_queue_size_for_learning // 2
                                    - drive_functions.kernel_acc_des_for_learning.shape[0]
                                    // 2 : drive_functions.max_input_queue_size_for_learning
                                    // 2
                                    + drive_functions.kernel_acc_des_for_learning.shape[0] // 2
                                    + 1
                                ].T
                                * drive_functions.kernel_acc_des_for_learning
                            ).sum(
                                axis=1
                            )
                            X_smoothed[
                                drive_functions.nx_0
                                + drive_functions.acc_ctrl_queue_size : drive_functions.nx_0
                                + drive_functions.acc_ctrl_queue_size
                                + drive_functions.steer_ctrl_queue_size
                            ] = (
                                steer_input_queue[
                                    drive_functions.max_input_queue_size_for_learning // 2
                                    - drive_functions.kernel_steer_des_for_learning.shape[0]
                                    // 2 : drive_functions.max_input_queue_size_for_learning
                                    // 2
                                    + drive_functions.kernel_steer_des_for_learning.shape[0] // 2
                                    + 1
                                ].T
                                * drive_functions.kernel_steer_des_for_learning
                            ).sum(
                                axis=1
                            )
                            self.X_smoothed_queue.append(X_smoothed)
                            if len(self.X_smoothed_queue) > drive_functions.mpc_freq + 1:
                                self.X_smoothed_queue.pop()
                            if len(self.X_smoothed_queue) == drive_functions.mpc_freq + 1:
                                acc_start = (
                                    drive_functions.acc_ctrl_queue_size
                                    - drive_functions.acc_delay_step
                                )
                                acc_end = (
                                    drive_functions.acc_ctrl_queue_size
                                    - drive_functions.acc_delay_step
                                    + drive_functions.mpc_freq
                                )
                                steer_start = (
                                    drive_functions.steer_ctrl_queue_size
                                    - drive_functions.steer_delay_step
                                )
                                steer_end = (
                                    drive_functions.steer_ctrl_queue_size
                                    - drive_functions.steer_delay_step
                                    + drive_functions.mpc_freq
                                )

                                u_for_predict_x_current = np.zeros((drive_functions.mpc_freq, 2))
                                u_for_predict_x_current[:, 0] = self.X_smoothed_queue[0][
                                    6 : 6 + drive_functions.acc_ctrl_queue_size
                                ][::-1][acc_start:acc_end]
                                u_for_predict_x_current[:, 1] = self.X_smoothed_queue[0][
                                    6 + drive_functions.acc_ctrl_queue_size :
                                ][::-1][steer_start:steer_end]
                                var_dot = self.X_smoothed_queue[-1][
                                    :6
                                ] - drive_functions.F_multiple(
                                    self.X_smoothed_queue[0][:6],
                                    u_for_predict_x_current,
                                    drive_functions.acc_time_constant,
                                    drive_functions.steer_time_constant,
                                )
                                var_dot /= drive_functions.mpc_time_step

                                self.pre_X_input_list.append(
                                    np.concatenate(
                                        (
                                            self.X_smoothed_queue[0][[2, 4, 5]],
                                            self.X_smoothed_queue[0][6:],
                                        )
                                    )
                                )
                                self.pre_Y_output_list.append(
                                    drive_functions.rotate_data(
                                        self.X_smoothed_queue[0][:6], var_dot
                                    )
                                )
                                if (
                                    len(self.pre_X_input_list)
                                    > drive_functions.max_output_queue_size_for_learning
                                ):
                                    self.pre_X_input_list.pop()
                                    self.pre_Y_output_list.pop()
                                if (
                                    len(self.pre_X_input_list)
                                    == drive_functions.max_output_queue_size_for_learning
                                    and self.update_trained_model
                                ):
                                    X_input = self.pre_X_input_list[
                                        drive_functions.max_output_queue_size_for_learning // 2
                                    ]
                                    self.X_input_list.append(X_input)
                                    pre_Y_output = np.array(np.array(self.pre_Y_output_list))
                                    Y_output = np.zeros(self.pre_Y_output_list[0].shape)
                                    Y_output[0] = np.dot(
                                        pre_Y_output[
                                            drive_functions.max_output_queue_size_for_learning // 2
                                            - drive_functions.kernel_x_out_for_learning.shape[0]
                                            // 2 : drive_functions.max_output_queue_size_for_learning
                                            // 2
                                            + drive_functions.kernel_x_out_for_learning.shape[0]
                                            // 2
                                            + 1,
                                            0,
                                        ],
                                        drive_functions.kernel_x_out_for_learning,
                                    )
                                    Y_output[1] = np.dot(
                                        pre_Y_output[
                                            drive_functions.max_output_queue_size_for_learning // 2
                                            - drive_functions.kernel_y_out_for_learning.shape[0]
                                            // 2 : drive_functions.max_output_queue_size_for_learning
                                            // 2
                                            + drive_functions.kernel_y_out_for_learning.shape[0]
                                            // 2
                                            + 1,
                                            1,
                                        ],
                                        drive_functions.kernel_y_out_for_learning,
                                    )
                                    Y_output[2] = np.dot(
                                        pre_Y_output[
                                            drive_functions.max_output_queue_size_for_learning // 2
                                            - drive_functions.kernel_v_out_for_learning.shape[0]
                                            // 2 : drive_functions.max_output_queue_size_for_learning
                                            // 2
                                            + drive_functions.kernel_v_out_for_learning.shape[0]
                                            // 2
                                            + 1,
                                            2,
                                        ],
                                        drive_functions.kernel_v_out_for_learning,
                                    )
                                    Y_output[3] = np.dot(
                                        pre_Y_output[
                                            drive_functions.max_output_queue_size_for_learning // 2
                                            - drive_functions.kernel_theta_out_for_learning.shape[0]
                                            // 2 : drive_functions.max_output_queue_size_for_learning
                                            // 2
                                            + drive_functions.kernel_theta_out_for_learning.shape[0]
                                            // 2
                                            + 1,
                                            3,
                                        ],
                                        drive_functions.kernel_theta_out_for_learning,
                                    )
                                    Y_output[4] = np.dot(
                                        pre_Y_output[
                                            drive_functions.max_output_queue_size_for_learning // 2
                                            - drive_functions.kernel_acc_out_for_learning.shape[0]
                                            // 2 : drive_functions.max_output_queue_size_for_learning
                                            // 2
                                            + drive_functions.kernel_acc_out_for_learning.shape[0]
                                            // 2
                                            + 1,
                                            4,
                                        ],
                                        drive_functions.kernel_acc_out_for_learning,
                                    )
                                    Y_output[5] = np.dot(
                                        pre_Y_output[
                                            drive_functions.max_output_queue_size_for_learning // 2
                                            - drive_functions.kernel_steer_out_for_learning.shape[0]
                                            // 2 : drive_functions.max_output_queue_size_for_learning
                                            // 2
                                            + drive_functions.kernel_steer_out_for_learning.shape[0]
                                            // 2
                                            + 1,
                                            5,
                                        ],
                                        drive_functions.kernel_steer_out_for_learning,
                                    )
                                    self.Y_output_list.append(
                                        Y_output
                                        - self.A_for_polynomial_reg
                                        @ self.polynomial_features.fit_transform(
                                            X_input[ctrl_index_for_polynomial_reg].reshape(1, -1)
                                        )[0]
                                        - self.b_for_polynomial_reg
                                    )
                                    if len(self.X_input_list) > drive_functions.max_train_data_size:
                                        self.X_input_list.pop(0)
                                        self.Y_output_list.pop(0)
                        while (
                            self.X_smoothing_time_stamp - self.time_stamp_queue_for_learning[1]
                            > drive_functions.ctrl_time_step
                            * drive_functions.max_input_queue_size_for_learning
                        ):
                            self.X_queue_for_learning.pop(0)
                            self.acc_input_queue_for_learning.pop(0)
                            self.steer_input_queue_for_learning.pop(0)
                            self.time_stamp_queue_for_learning.pop(0)

    def update_lstm_info(self, time_stamp):
        self.transform_model.transform.set_lstm(0 * self.h, 0 * self.c)
        self.time_stamp_for_update_lstm.append(time_stamp)
        self.X_queue_for_update_lstm.append(self.X_current)
        if len(self.X_queue_for_update_lstm) > 1:
            while self.time_stamp_for_update_lstm[-1] - self.time_stamp_for_update_lstm[
                1
            ] > drive_functions.mpc_time_step * (drive_functions.N + 1):
                self.time_stamp_for_update_lstm.pop(0)
                self.X_queue_for_update_lstm.pop(0)
            if (
                self.time_stamp_for_update_lstm[-1] - self.time_stamp_for_update_lstm[0]
                > drive_functions.mpc_time_step
            ):
                horizon_num = min(
                    int(
                        (self.time_stamp_for_update_lstm[-1] - self.time_stamp_for_update_lstm[0])
                        / drive_functions.mpc_time_step
                    ),
                    drive_functions.N,
                )
                time_stamp_interp = (
                    self.time_stamp_for_update_lstm[-1]
                    - drive_functions.mpc_time_step * np.arange(1, horizon_num + 1)[::-1]
                )
                X_interp = scipy.interpolate.interp1d(
                    np.array(self.time_stamp_for_update_lstm),
                    np.array(self.X_queue_for_update_lstm).T,
                )(time_stamp_interp)
                self.transform_model.transform.update_memory_by_state_history(X_interp)
                self.h = self.transform_model.transform.get_h()
                self.c = self.transform_model.transform.get_c()

    def update_input_queue_and_get_optimal_control(
        self,
        time_stamp,
        acc_history,
        steer_history,
        x_current_,
        X_des_,
        U_des=np.zeros((drive_functions.N, drive_functions.nu_0)),
        acc_time_stamp=0.0,
        steer_time_stamp=0.0,
    ):
        """Run update_input_queue and get_optimal_control all at once."""
        self.update_input_queue(
            time_stamp, acc_history, steer_history, x_current_, acc_time_stamp, steer_time_stamp
        )
        u_opt = self.get_optimal_control(x_current_, time_stamp, X_des_, U_des)
        return u_opt

    def update_model(self):
        """Update the learning model online."""
        while True:
            if not self.model_update_flag:
                break
            if not self.initialize_input_queue and len(self.X_input_list) > self.drive_batch_size:
                X_input = np.array(self.X_input_list)
                Y_output = np.array(self.Y_output_list)
                batch_index = np.random.choice(
                    X_input.shape[0], self.drive_batch_size, replace=False
                )
                X_batch = torch.tensor(X_input[batch_index].astype(np.float32)).clone()
                Y_batch = torch.tensor(Y_output[batch_index].astype(np.float32)).clone()
                Y_pred = self.model(X_batch)
                loss = drive_NN.loss_fn_plus_tanh(
                    self.loss_fn, Y_pred, Y_batch, self.tanh_gain, self.lam
                )
                for w in self.model.parameters():
                    loss = (
                        loss + self.alpha_1 * torch.norm(w, p=1) + self.alpha_2 * torch.norm(w) ** 2
                    )
                self.drive_optimizer.zero_grad()
                loss.backward()
                self.drive_optimizer.step()

                self.transform_model = drive_NN.transform_model_to_c(
                    self.model,
                    self.A_for_polynomial_reg,
                    self.b_for_polynomial_reg,
                    self.deg,
                    self.acc_delay_step,
                    self.steer_delay_step,
                    drive_functions.acc_ctrl_queue_size,
                    drive_functions.steer_ctrl_queue_size,
                    drive_functions.steer_ctrl_queue_size_core,
                )
                self.pred = self.transform_model.pred
                if drive_functions.reflect_only_poly_diff:
                    self.pred_with_diff = self.transform_model.pred_with_poly_diff
                elif self.use_memory_diff and drive_functions.use_memory_for_training:
                    self.pred_with_diff = self.transform_model.pred_with_memory_diff
                else:
                    self.pred_with_diff = self.transform_model.pred_with_diff
                self.F_N_initial_diff = partial(
                    drive_functions.F_with_model_initial_diff,
                    pred=self.transform_model.pred,
                    i=self.acc_delay_step,
                    j=self.steer_delay_step,
                    acc_time_constant_ctrl=self.acc_time_constant_ctrl,
                    steer_time_constant_ctrl=self.steer_time_constant_ctrl,
                )
                self.F_N_diff = partial(
                    drive_functions.F_with_model_diff,
                    pred=self.pred_with_diff,
                    i=self.acc_delay_step,
                    j=self.steer_delay_step,
                    acc_time_constant_ctrl=self.acc_time_constant_ctrl,
                    steer_time_constant_ctrl=self.steer_time_constant_ctrl,
                )
                self.F_N_only_state = partial(
                    drive_functions.F_with_model,
                    pred=self.transform_model.pred_only_state,
                    i=self.acc_delay_step,
                    j=self.steer_delay_step,
                    acc_time_constant_ctrl=self.acc_time_constant_ctrl,
                    steer_time_constant_ctrl=self.steer_time_constant_ctrl,
                )
                self.F_N_for_candidates = partial(
                    drive_functions.F_with_model_for_candidates,
                    Pred=self.transform_model.Pred,
                    i=self.acc_delay_step,
                    j=self.steer_delay_step,
                    acc_time_constant_ctrl=self.acc_time_constant_ctrl,
                    steer_time_constant_ctrl=self.steer_time_constant_ctrl,
                )

                print("updated!!!!!!!!!!!")
            time.sleep(0.1)

    def send_initialize_input_queue(self):
        """Flag initialization of the history of inputs passed to the control side."""
        self.initialize_input_queue = True

    def stop_model_update(self):
        """Terminate the model update."""
        self.model_update_flag = False
        if self.update_trained_model:
            self.model_updater.join()
