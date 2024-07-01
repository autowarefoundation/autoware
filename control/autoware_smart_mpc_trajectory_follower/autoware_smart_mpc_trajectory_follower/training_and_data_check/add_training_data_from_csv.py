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

# cspell: ignore ndimage usecols ndim interp savez


"""Class for preprocessing CSV data to obtain training data for training."""

import csv
import glob
import json
import os
from pathlib import Path
import subprocess

from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
import numpy as np
import scipy.interpolate
from scipy.ndimage import gaussian_filter
from scipy.spatial.transform import Rotation
from sklearn import linear_model
from sklearn.preprocessing import PolynomialFeatures

ctrl_index_for_polynomial_reg = np.concatenate(
    (
        np.arange(3),
        3
        + max(drive_functions.acc_delay_step - drive_functions.mpc_freq, 0)
        + np.arange(drive_functions.mpc_freq),
        3
        + drive_functions.acc_ctrl_queue_size
        + max(drive_functions.steer_delay_step - drive_functions.mpc_freq, 0)
        + np.arange(drive_functions.mpc_freq),
    )
)


if drive_functions.use_memory_for_training:
    default_add_mode = "divide"
else:
    default_add_mode = "as_train"


def data_smoothing(data: np.ndarray, sigma: float) -> np.ndarray:
    """Apply a Gaussian filter to the data."""
    data_ = gaussian_filter(data, sigma)
    return data_


def yaw_transform(raw_yaw: np.ndarray) -> np.ndarray:
    """Adjust and transform within a period of 2π so that the yaw angle is continuous."""
    transformed_yaw = np.zeros(raw_yaw.shape)
    transformed_yaw[0] = raw_yaw[0]
    for i in range(raw_yaw.shape[0] - 1):
        rotate_num = (raw_yaw[i + 1] - transformed_yaw[i]) // (2 * np.pi)
        if raw_yaw[i + 1] - transformed_yaw[i] - 2 * rotate_num * np.pi < np.pi:
            transformed_yaw[i + 1] = raw_yaw[i + 1] - 2 * rotate_num * np.pi
        else:
            transformed_yaw[i + 1] = raw_yaw[i + 1] - 2 * (rotate_num + 1) * np.pi
    return transformed_yaw


class add_data_from_csv:
    """Class for loading csv files containing driving data, converting them into teacher data for the NN training, and storing them in lists."""

    X_input_list: list[np.ndarray]
    """Input side of NN teacher data."""

    Y_output_list: list[np.ndarray]
    """Output side of NN teacher data."""

    def __init__(
        self, alpha_1_for_polynomial_regression=0.1**5, alpha_2_for_polynomial_regression=0.1**5
    ):
        self.X_input_list = []
        self.Y_output_list = []
        self.X_val_list = []
        self.Y_val_list = []
        self.division_indices = []
        self.division_indices_val = []
        self.alpha_1_for_polynomial_regression = alpha_1_for_polynomial_regression
        self.alpha_2_for_polynomial_regression = alpha_2_for_polynomial_regression

    def clear_data(self):
        """Clear the teacher data for the training."""
        self.X_input_list.clear()
        self.Y_output_list.clear()
        self.X_val_list.clear()
        self.Y_val_list.clear()

    def transform_rosbag_to_csv(self, dir_name: str, delete_csv_first: bool = True) -> None:
        """Convert rosbag file to CSV format."""
        package_path_json = str(Path(__file__).parent.parent) + "/package_path.json"
        with open(package_path_json, "r") as file:
            package_path = json.load(file)
        dir_exe = os.getcwd()
        subprocess.run(
            "cp "
            + package_path["path"]
            + "/autoware_smart_mpc_trajectory_follower/training_and_data_check/rosbag2.bash "
            + dir_name,
            shell=True,
        )
        os.chdir(dir_name)
        if len(glob.glob("*.csv")) > 0 and delete_csv_first:
            subprocess.run("rm *.csv", shell=True)
        subprocess.run("bash rosbag2.bash", shell=True)
        os.chdir(dir_exe)

    def add_data_from_csv(self, dir_name: str, add_mode=default_add_mode) -> None:
        """Adding teacher data for training from a CSV file."""
        acc_ctrl_queue_size = drive_functions.acc_ctrl_queue_size
        steer_ctrl_queue_size = drive_functions.steer_ctrl_queue_size
        ctrl_time_step = drive_functions.ctrl_time_step
        mpc_time_step = drive_functions.mpc_time_step
        mpc_freq = drive_functions.mpc_freq
        acc_delay_step_ctrl = drive_functions.acc_delay_step
        steer_delay_step_ctrl = drive_functions.steer_delay_step
        acc_time_constant_ctrl = drive_functions.acc_time_constant
        steer_time_constant_ctrl = drive_functions.steer_time_constant

        kinematic = np.loadtxt(
            dir_name + "/kinematic_state.csv", delimiter=",", usecols=[0, 1, 4, 5, 7, 8, 9, 10, 47]
        )
        pose_position_x = kinematic[:, 2]
        pose_position_y = kinematic[:, 3]
        vel = kinematic[:, 8]
        raw_yaw = Rotation.from_quat(kinematic[:, 4:8]).as_euler("xyz")[:, 2]
        yaw = yaw_transform(raw_yaw)

        loc_acc = np.loadtxt(dir_name + "/acceleration.csv", delimiter=",", usecols=[0, 1, 3])
        acc = loc_acc[:, 2]

        acc_sm = data_smoothing(acc, drive_functions.acc_sigma_for_learning)

        vehicle_steer = np.loadtxt(
            dir_name + "/steering_status.csv", delimiter=",", usecols=[0, 1, 2]
        )
        steer = vehicle_steer[:, 2]
        steer_sm = steer.copy()
        steer_sm = data_smoothing(steer, drive_functions.steer_sigma_for_learning)

        control_cmd = np.loadtxt(
            dir_name + "/control_cmd_orig.csv", delimiter=",", usecols=[0, 1, 8, 16]
        )
        acc_des = control_cmd[:, 3]
        acc_des_sm = data_smoothing(acc_des, drive_functions.acc_des_sigma_for_learning)

        steer_des = control_cmd[:, 2]
        steer_des_sm = steer_des.copy()
        steer_des_sm = data_smoothing(steer_des, drive_functions.steer_des_sigma_for_learning)

        operation_mode = np.loadtxt(
            dir_name + "/system_operation_mode_state.csv", delimiter=",", usecols=[0, 1, 2]
        )
        if operation_mode.ndim == 1:
            operation_mode = operation_mode.reshape(1, -1)
        with open(dir_name + "/system_operation_mode_state.csv") as f:
            reader = csv.reader(f, delimiter=",")
            autoware_control_enabled_str = np.array([row[3] for row in reader])

        proxima_control_enabled = np.zeros(operation_mode.shape[0])
        for i in range(operation_mode.shape[0]):
            if operation_mode[i, 2] > 1.5 and autoware_control_enabled_str[i] == "True":
                proxima_control_enabled[i] = 1.0
        for i in range(operation_mode.shape[0] - 1):
            if proxima_control_enabled[i] < 0.5 and proxima_control_enabled[i + 1] > 0.5:
                operation_start_time = operation_mode[i + 1, 0] + 1e-9 * operation_mode[i + 1, 1]
            elif proxima_control_enabled[i] > 0.5 and proxima_control_enabled[i + 1] < 0.5:
                operation_end_time = operation_mode[i + 1, 0] + 1e-9 * operation_mode[i + 1, 1]
                break
            operation_end_time = kinematic[-1, 0] + 1e-9 * kinematic[-1, 1]
        if operation_mode.shape[0] == 1:
            operation_end_time = kinematic[-1, 0] + 1e-9 * kinematic[-1, 1]
        if proxima_control_enabled[0] > 0.5:
            operation_start_time = operation_mode[0, 0] + 1e-9 * operation_mode[0, 1]
        print("operation_start_time", operation_start_time)
        print("operation_end_time", operation_end_time)
        min_time_stamp = max(
            [
                operation_start_time,
                kinematic[0, 0] + 1e-9 * kinematic[0, 1],
                loc_acc[0, 0] + 1e-9 * loc_acc[0, 1],
                vehicle_steer[0, 0] + 1e-9 * vehicle_steer[0, 1],
                control_cmd[0, 0] + 1e-9 * control_cmd[0, 1],
            ]
        )
        max_time_stamp = min(
            [
                operation_end_time,
                kinematic[-1, 0] + 1e-9 * kinematic[-1, 1],
                loc_acc[-1, 0] + 1e-9 * loc_acc[-1, 1],
                vehicle_steer[-1, 0] + 1e-9 * vehicle_steer[-1, 1],
                control_cmd[-1, 0] + 1e-9 * control_cmd[-1, 1],
            ]
        )

        trajectory_interpolator_list = []
        trajectory_interpolator_list.append(
            scipy.interpolate.interp1d(kinematic[:, 0] + 1e-9 * kinematic[:, 1], pose_position_x)
        )
        trajectory_interpolator_list.append(
            scipy.interpolate.interp1d(kinematic[:, 0] + 1e-9 * kinematic[:, 1], pose_position_y)
        )
        trajectory_interpolator_list.append(
            scipy.interpolate.interp1d(kinematic[:, 0] + 1e-9 * kinematic[:, 1], vel)
        )
        trajectory_interpolator_list.append(
            scipy.interpolate.interp1d(kinematic[:, 0] + 1e-9 * kinematic[:, 1], yaw)
        )
        trajectory_interpolator_list.append(
            scipy.interpolate.interp1d(loc_acc[:, 0] + 1e-9 * loc_acc[:, 1], acc_sm)
        )
        trajectory_interpolator_list.append(
            scipy.interpolate.interp1d(vehicle_steer[:, 0] + 1e-9 * vehicle_steer[:, 1], steer_sm)
        )
        trajectory_interpolator_list.append(
            scipy.interpolate.interp1d(control_cmd[:, 0] + 1e-9 * control_cmd[:, 1], acc_des_sm)
        )
        trajectory_interpolator_list.append(
            scipy.interpolate.interp1d(control_cmd[:, 0] + 1e-9 * control_cmd[:, 1], steer_des_sm)
        )

        def get_interpolated_state(s):
            x_current = np.zeros(6)
            for i in range(6):
                x_current[i] = trajectory_interpolator_list[i](s)
            return x_current

        def get_interpolated_input(s):
            u_input = np.zeros(2)
            for i in range(2):
                u_input[i] = trajectory_interpolator_list[6 + i](s)
            return u_input

        s = min_time_stamp
        X = []
        U = []

        Timestamp = []
        while True:
            if s > max_time_stamp:
                break
            X.append(get_interpolated_state(s))

            U.append(get_interpolated_input(s))

            Timestamp.append(s)
            s += ctrl_time_step

        X_array = np.array(X)
        U_array = np.array(U)
        X_input_list = []
        Y_output_list = []
        Timestamp_learn = []
        for i in range(
            max(acc_ctrl_queue_size, steer_ctrl_queue_size) + 1, X_array.shape[0] - mpc_freq - 1
        ):
            acc_input_queue = U_array[i - acc_ctrl_queue_size : i, 0].copy()
            steer_input_queue = U_array[i - steer_ctrl_queue_size : i, 1].copy()
            x_current = X_array[i + 3]
            x_old = X_array[i]

            Timestamp_learn.append(Timestamp[i])
            X_input_list.append(
                np.concatenate((x_old[[2, 4, 5]], acc_input_queue[::-1], steer_input_queue[::-1]))
            )
            acc_start = acc_ctrl_queue_size - acc_delay_step_ctrl
            acc_end = acc_ctrl_queue_size - acc_delay_step_ctrl + mpc_freq
            steer_start = steer_ctrl_queue_size - steer_delay_step_ctrl
            steer_end = steer_ctrl_queue_size - steer_delay_step_ctrl + mpc_freq

            u_for_predict_x_current = np.zeros((mpc_freq, 2))
            u_for_predict_x_current[:, 0] = acc_input_queue[acc_start:acc_end]
            u_for_predict_x_current[:, 1] = steer_input_queue[steer_start:steer_end]
            var_dot = x_current - drive_functions.F_multiple(
                x_old, u_for_predict_x_current, acc_time_constant_ctrl, steer_time_constant_ctrl
            )
            var_dot /= mpc_time_step
            if var_dot[3] > 1:
                print(i)
            Y_output_list.append(drive_functions.rotate_data(x_old, var_dot))

        Y_output_list_array = np.array(Y_output_list)
        Y_smooth = Y_output_list_array.copy()
        Y_smooth[:, 0] = data_smoothing(
            Y_output_list_array[:, 0], drive_functions.x_out_sigma_for_learning
        )
        Y_smooth[:, 1] = data_smoothing(
            Y_output_list_array[:, 1], drive_functions.y_out_sigma_for_learning
        )
        Y_smooth[:, 2] = data_smoothing(
            Y_output_list_array[:, 2], drive_functions.v_out_sigma_for_learning
        )
        Y_smooth[:, 3] = data_smoothing(
            Y_output_list_array[:, 3], drive_functions.theta_out_sigma_for_learning
        )
        Y_smooth[:, 4] = data_smoothing(
            Y_output_list_array[:, 4], drive_functions.acc_out_sigma_for_learning
        )
        Y_smooth[:, 5] = data_smoothing(
            Y_output_list_array[:, 5], drive_functions.steer_out_sigma_for_learning
        )
        if add_mode == "divide":
            for i in range(len(X_input_list)):
                if i < 3 * len(X_input_list) / 4:
                    self.X_input_list.append(X_input_list[i])
                    self.Y_output_list.append(Y_smooth[i])
                else:
                    self.X_val_list.append(X_input_list[i])
                    self.Y_val_list.append(Y_smooth[i])

            self.division_indices.append(len(self.X_input_list))
            self.division_indices_val.append(len(self.X_val_list))

        elif add_mode == "as_train":
            for i in range(len(X_input_list)):
                self.X_input_list.append(X_input_list[i])
                self.Y_output_list.append(Y_smooth[i])

            self.division_indices.append(len(self.X_input_list))
        if add_mode == "as_val":
            for i in range(len(X_input_list)):
                self.X_val_list.append(X_input_list[i])
                self.Y_val_list.append(Y_smooth[i])

            self.division_indices_val.append(len(self.X_val_list))

    def save_train_data(self, save_dir=".") -> None:
        """Save neural net teacher data."""
        np.savez(
            save_dir + "/train_data",
            X_input=np.array(self.X_input_list),
            Y_output=np.array(self.Y_output_list),
        )

    def get_polynomial_regression_result(
        self,
        X_input_list: list[np.ndarray],
        Y_output_list: list[np.ndarray],
        use_polynomial_reg: bool,
        use_selected_polynomial: bool,
        deg: int,
        fit_intercept: bool,
        use_intercept: bool,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Get the results of a polynomial regression."""
        X_input = np.array(X_input_list)
        Y_output = np.array(Y_output_list)
        if use_selected_polynomial:
            self.deg = 2
        else:
            self.deg = deg
        self.A, self.b, Y_output_minus = polynomial_regression(
            X_input,
            Y_output,
            self.alpha_1_for_polynomial_regression,
            self.alpha_2_for_polynomial_regression,
            use_polynomial_reg,
            use_selected_polynomial,
            self.deg,
            fit_intercept,
            use_intercept,
        )
        return X_input, Y_output_minus


def polynomial_regression(
    X,
    Y,
    alpha_1,
    alpha_2,
    use_polynomial_reg,
    use_selected_polynomial,
    deg,
    fit_intercept,
    use_intercept,
    fit_acc_steer=False,
):
    polynomial_features = PolynomialFeatures(degree=deg, include_bias=False)
    alpha = alpha_1 + alpha_2
    if use_polynomial_reg:
        if use_selected_polynomial:
            clf_1 = linear_model.ElasticNet(
                fit_intercept=fit_intercept, alpha=alpha, l1_ratio=alpha_1 / alpha, max_iter=100000
            )
            clf_2 = linear_model.ElasticNet(
                fit_intercept=fit_intercept, alpha=alpha, l1_ratio=alpha_1 / alpha, max_iter=100000
            )
            clf_3 = linear_model.ElasticNet(
                fit_intercept=fit_intercept, alpha=alpha, l1_ratio=alpha_1 / alpha, max_iter=100000
            )
            X_poly = polynomial_features.fit_transform(X[:, ctrl_index_for_polynomial_reg])
            A = np.zeros((Y.shape[1], X_poly.shape[1]))
            b = np.zeros(Y.shape[1])
            clf_1.fit(X_poly[:, [0, ctrl_index_for_polynomial_reg.shape[0] + 2]], Y[:, [3]])
            # clf_2.fit(X_poly[:, [1, 3,4,5]], Y[:, [4]])

            acc_input_sum = np.zeros((3, 1))
            acc_input_sum[0, 0] = drive_functions.ctrl_time_step / drive_functions.acc_time_constant
            acc_input_sum[1, 0] = acc_input_sum[0, 0] * (1 - acc_input_sum[0, 0])
            acc_input_sum[2, 0] = acc_input_sum[1, 0] * (1 - acc_input_sum[0, 0])
            clf_2.fit(X_poly[:, [3, 4, 5]] @ acc_input_sum, Y[:, [4]])

            # clf_3.fit(X_poly[:, [2, 6,7,8]], Y[:, [5]])

            steer_input_sum = np.zeros((3, 1))
            steer_input_sum[0, 0] = (
                drive_functions.ctrl_time_step / drive_functions.steer_time_constant
            )
            steer_input_sum[1, 0] = steer_input_sum[0, 0] * (1 - steer_input_sum[0, 0])
            steer_input_sum[2, 0] = steer_input_sum[1, 0] * (1 - steer_input_sum[0, 0])
            clf_3.fit(X_poly[:, [6, 7, 8]] @ steer_input_sum, Y[:, [5]])

            A[3, [0, ctrl_index_for_polynomial_reg.shape[0] + 2]] = clf_1.coef_
            # A[4, [1, 3,4,5]] = clf_2.coef_
            if fit_acc_steer:
                A[4, [3, 4, 5]] = clf_2.coef_ * acc_input_sum.flatten()

            # A[5, [2, 6,7,8]] = clf_3.coef_
            if fit_acc_steer:
                A[5, [6, 7, 8]] = clf_3.coef_ * steer_input_sum.flatten()

            if fit_intercept:
                b[3] = clf_1.intercept_
                if fit_acc_steer:
                    b[4] = clf_2.intercept_
                    b[5] = clf_3.intercept_

            if not use_intercept:
                b = 0 * b
            return A, b, Y - X_poly @ A.T - b
        else:
            clf = linear_model.ElasticNet(
                fit_intercept=fit_intercept, alpha=alpha, l1_ratio=alpha_1 / alpha, max_iter=100000
            )
            X_poly = polynomial_features.fit_transform(X[:, ctrl_index_for_polynomial_reg])
            clf.fit(X_poly, Y)
            if fit_intercept:
                if use_intercept:
                    return (
                        clf.coef_,
                        clf.intercept_,
                        Y - clf.predict(X_poly),
                    )
                else:
                    return (
                        clf.coef_,
                        0 * clf.intercept_,
                        Y - clf.predict(X_poly) + clf.intercept_,
                    )
            else:
                return clf.coef_, np.zeros(Y.shape[1]), Y - clf.predict(X_poly)
    else:
        poly_dim = polynomial_features.fit_transform(
            X[0, ctrl_index_for_polynomial_reg].reshape(1, -1)
        ).shape[1]
        return np.zeros((Y.shape[1], poly_dim)), np.zeros(Y.shape[1]), Y
