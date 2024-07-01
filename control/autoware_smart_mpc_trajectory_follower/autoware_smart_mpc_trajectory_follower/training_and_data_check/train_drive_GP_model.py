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

# cspell: ignore lengthscale savez

import GPy
from autoware_smart_mpc_trajectory_follower.training_and_data_check import (
    add_training_data_from_csv,
)
import numpy as np


class train_drive_GP_model(add_training_data_from_csv.add_data_from_csv):
    """Class for training the Gaussian process from driving data."""

    def __init__(self):
        super(train_drive_GP_model, self).__init__()

    def get_optimized_GP(self, i, save_name, inducting_num=100):
        """Train Gaussian processes and save the information needed for vehicle control."""
        X_input = np.array(self.X_input_list)
        Y_output = np.array(self.Y_output_list)
        induction_index = np.random.choice(X_input.shape[0], inducting_num, replace=False)
        kernel = GPy.kern.RBF(input_dim=X_input.shape[1], ARD=True)
        m = GPy.models.SparseGPRegression(
            X_input, Y_output[:, [i]], kernel, Z=X_input[induction_index]
        )
        m.optimize()
        theta_1 = kernel.variance[0]
        theta_2 = 1 / np.array(np.array(kernel.lengthscale) ** 2)
        Z_post = np.array(m.inducing_inputs)

        K_MM = kernel.K(Z_post, Z_post)
        K_MN = kernel.K(Z_post, X_input)
        K_MM_inv = np.linalg.inv(K_MM)
        Lam_with_noise_inv = np.zeros(K_MN.shape[1])
        for n in range(K_MN.shape[1]):
            Lam_with_noise_inv[n] = 1 / (
                kernel.variance
                - np.dot(K_MM_inv @ K_MN[:, n], K_MN[:, n])
                + m.Gaussian_noise.variance
            )
        Q_MM = K_MM + K_MN @ np.diag(Lam_with_noise_inv) @ K_MN.T
        Q_MM_inv = np.linalg.inv(Q_MM)
        U = K_MM @ Q_MM_inv @ K_MN @ (np.diag(Lam_with_noise_inv) @ Y_output[:, 4])
        K_MM_invUT = (K_MM_inv @ U).T

        np.savez(
            save_name,
            theta_1=theta_1,
            theta_2=theta_2,
            Z=Z_post,
            mean_mtr=K_MM_invUT,
            cov_mtr=K_MM_inv - Q_MM_inv,
        )

    def get_optimized_GP_x(self, dir_name=".", inducting_num=100):
        """Save the information of the Gaussian process about the prediction error of the longitudinal position."""
        self.get_optimized_GP(0, dir_name + "/GP_x_info", inducting_num)

    def get_optimized_GP_y(self, dir_name=".", inducting_num=100):
        """Save the information of the Gaussian process about the prediction error of the lateral position."""
        self.get_optimized_GP(1, dir_name + "/GP_y_info", inducting_num)

    def get_optimized_GP_v(self, dir_name=".", inducting_num=100):
        """Save the information of the Gaussian process about the prediction error of the velocity."""
        self.get_optimized_GP(2, dir_name + "/GP_v_info", inducting_num)

    def get_optimized_GP_theta(self, dir_name=".", inducting_num=100):
        """Save the information of the Gaussian process about the prediction error of the yaw angle."""
        self.get_optimized_GP(3, dir_name + "/GP_theta_info", inducting_num)

    def get_optimized_GP_acc(self, dir_name=".", inducting_num=100):
        """Save the information of the Gaussian process about the prediction error of the acceleration."""
        self.get_optimized_GP(4, dir_name + "/GP_acc_info", inducting_num)

    def get_optimized_GP_steer(self, dir_name=".", inducting_num=100):
        """Save the information of the Gaussian process about the prediction error of the steering angle."""
        self.get_optimized_GP(5, dir_name + "/GP_steer_info", inducting_num)
