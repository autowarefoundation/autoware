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

import warnings

from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
import numpy as np
from sklearn import linear_model

warnings.simplefilter("ignore")


class pure_pursuit_gain_updater:
    def __init__(self, max_time=10.0, max_input_delay=1.5):
        self.vel_queue = []
        self.yaw_queue = []
        self.acc_queue = []
        self.steer_queue = []
        self.acc_input_queue = []
        self.steer_input_queue = []
        self.max_queue_size = round(max_time / drive_functions.ctrl_time_step)
        self.max_input_delay_size = round(max_input_delay / drive_functions.ctrl_time_step)
        self.A_wheel_base = np.zeros((2, 2))
        self.B_wheel_base = np.zeros(2)

    def state_queue_updater(self, vel, yaw, acc, steer):
        self.vel_queue.append(vel)
        self.yaw_queue.append(yaw)
        self.acc_queue.append(acc)
        self.steer_queue.append(steer)
        if len(self.vel_queue) > self.max_queue_size:
            self.vel_queue.pop(0)
            self.yaw_queue.pop(0)
            self.acc_queue.pop(0)
            self.steer_queue.pop(0)
        if len(self.vel_queue) > 1:
            numerator = self.vel_queue[-2] * np.tan(self.steer_queue[-2])
            yaw_dot_error = (
                self.yaw_queue[-1] - self.yaw_queue[-2]
            ) / drive_functions.ctrl_time_step
            yaw_dot_error -= numerator / drive_functions.L

            X_wheel_base = np.array([[1.0, numerator]])
            self.A_wheel_base += X_wheel_base.T @ X_wheel_base + 1e-5 * np.eye(2)
            self.B_wheel_base += yaw_dot_error * X_wheel_base[0]

    def input_queue_updater(self, acc_input, steer_input):
        self.acc_input_queue.append(acc_input)
        self.steer_input_queue.append(steer_input)
        if len(self.acc_input_queue) > self.max_queue_size - 1:
            self.acc_input_queue.pop(0)
            self.steer_input_queue.pop(0)

    def get_acc_gain_scaling(self):
        if len(self.vel_queue) < self.max_queue_size:
            return 1.0
        acc_array = np.array(self.acc_queue)
        acc_input_array = np.array(self.acc_input_queue)

        acc_dot = (acc_array[1:] - acc_array[:-1]) / drive_functions.ctrl_time_step
        acc_estimation_error = np.inf
        best_acc_coef = np.array(
            [
                -drive_functions.ctrl_time_step / drive_functions.acc_time_constant,
                drive_functions.ctrl_time_step / drive_functions.acc_time_constant,
            ]
        )
        for i in range(self.max_input_delay_size):
            clf_acc = linear_model.ElasticNet(
                fit_intercept=False, alpha=1e-10, l1_ratio=0.5, max_iter=100000
            )
            clf_acc.fit(
                np.stack(
                    (
                        acc_array[self.max_input_delay_size - 1 : -1],
                        acc_input_array[
                            self.max_input_delay_size - 1 - i : len(acc_input_array) - i
                        ],
                    ),
                    1,
                ),
                acc_dot[self.max_input_delay_size - 1 :],
            )
            acc_error = (
                (
                    clf_acc.coef_[0] * acc_array[self.max_input_delay_size - 1 : -1]
                    + clf_acc.coef_[1]
                    * acc_input_array[self.max_input_delay_size - 1 - i : len(acc_input_array) - i]
                    - acc_dot[self.max_input_delay_size - 1 :]
                )
                ** 2
            ).sum()

            if acc_error < acc_estimation_error and clf_acc.coef_[0] < 0 and clf_acc.coef_[1] > 0:
                acc_estimation_error = acc_error
                best_acc_coef = clf_acc.coef_
            X_acc = np.stack(
                (
                    acc_array[self.max_input_delay_size - 1 : -1],
                    acc_input_array[self.max_input_delay_size - 1 - i : len(acc_input_array) - i],
                ),
                1,
            )
            X_acc = np.hstack((np.ones((X_acc.shape[0], 1)), X_acc))

            A = X_acc.T @ X_acc + 1e-15 * X_acc.shape[0] * np.eye(3)

            B = X_acc.T @ acc_dot[self.max_input_delay_size - 1 :]

            coef = np.linalg.solve(A, B)
            acc_error = (
                (
                    coef[1] * acc_array[self.max_input_delay_size - 1 : -1]
                    + coef[2]
                    * acc_input_array[self.max_input_delay_size - 1 - i : len(acc_input_array) - i]
                    + coef[0]
                    - acc_dot[self.max_input_delay_size - 1 :]
                )
                ** 2
            ).sum()

            if acc_error < acc_estimation_error and coef[1] < 0 and coef[2] > 0:
                acc_estimation_error = acc_error
                best_acc_coef = coef[[1, 2]]
        estimated_acc_scaling = np.clip(-best_acc_coef[1] / best_acc_coef[0], 0.1, 10.0)

        return 1 / estimated_acc_scaling

    def get_steer_gain_scaling(self):
        if len(self.vel_queue) < self.max_queue_size:
            return 1.0

        steer_array = np.array(self.steer_queue)
        steer_input_array = np.array(self.steer_input_queue)

        estimated_wheel_base_scaling = 1.0 / np.clip(
            1 + np.linalg.solve(self.A_wheel_base, self.B_wheel_base)[1] * drive_functions.L,
            0.1,
            1.0,
        )

        steer_dot = (steer_array[1:] - steer_array[:-1]) / drive_functions.ctrl_time_step

        steer_estimation_error = np.inf
        best_steer_coef = np.array(
            [
                -drive_functions.ctrl_time_step / drive_functions.steer_time_constant,
                drive_functions.ctrl_time_step / drive_functions.steer_time_constant,
            ]
        )
        for i in range(self.max_input_delay_size):
            X_steer = np.stack(
                (
                    steer_array[self.max_input_delay_size - 1 : -1],
                    steer_input_array[
                        self.max_input_delay_size - 1 - i : len(steer_input_array) - i
                    ],
                ),
                1,
            )
            X_steer = np.hstack((np.ones((X_steer.shape[0], 1)), X_steer))

            A = X_steer.T @ X_steer + 1e-15 * X_steer.shape[0] * np.eye(3)

            B = X_steer.T @ steer_dot[self.max_input_delay_size - 1 :]

            coef = np.linalg.solve(A, B)
            steer_error = (
                (
                    coef[1] * steer_array[self.max_input_delay_size - 1 : -1]
                    + coef[2]
                    * steer_input_array[
                        self.max_input_delay_size - 1 - i : len(steer_input_array) - i
                    ]
                    + coef[0]
                    - steer_dot[self.max_input_delay_size - 1 :]
                )
                ** 2
            ).sum()

            if steer_error < steer_estimation_error and coef[1] < 0 and coef[2] > 0:
                steer_estimation_error = steer_error
                best_steer_coef = coef[[1, 2]]

        estimated_steer_scaling = np.clip(-best_steer_coef[1] / best_steer_coef[0], 0.1, 10.0)

        return np.clip(estimated_wheel_base_scaling / estimated_steer_scaling, 0.1, 10.0)
