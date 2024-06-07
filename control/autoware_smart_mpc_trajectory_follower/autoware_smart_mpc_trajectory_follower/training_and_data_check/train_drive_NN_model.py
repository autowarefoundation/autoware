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

# cspell: ignore optim savez suptitle

"""Class for training neural nets from driving data."""
from autoware_smart_mpc_trajectory_follower.scripts import drive_NN
from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
from autoware_smart_mpc_trajectory_follower.training_and_data_check import (
    add_training_data_from_csv,
)
import matplotlib.pyplot as plt
import numpy as np
from sklearn import linear_model
from sklearn.preprocessing import PolynomialFeatures
import torch
from torch import nn
from torch.utils.data import DataLoader
from torch.utils.data import TensorDataset

ctrl_index_for_polynomial_reg = np.concatenate(
    (
        np.arange(3),
        [3 + drive_functions.acc_delay_step],
        [3 + drive_functions.acc_ctrl_queue_size + drive_functions.steer_delay_step],
    )
)


def nominal_model_acc(v: float, alpha_0: float, alpha: float) -> np.ndarray:
    """Predicted acceleration value according to the nominal model."""
    nominal_pred = alpha_0
    nominal_v = v + nominal_pred * drive_functions.ctrl_time_step
    nominal_pred = (
        nominal_pred
        + (alpha - nominal_pred)
        * drive_functions.ctrl_time_step
        / drive_functions.acc_time_constant
    )
    nominal_v = v + nominal_pred * drive_functions.ctrl_time_step
    nominal_pred = (
        nominal_pred
        + (alpha - nominal_pred)
        * drive_functions.ctrl_time_step
        / drive_functions.acc_time_constant
    )
    nominal_v = v + nominal_pred * drive_functions.ctrl_time_step
    nominal_pred = (
        nominal_pred
        + (alpha - nominal_pred)
        * drive_functions.ctrl_time_step
        / drive_functions.acc_time_constant
    )
    return np.array([nominal_v, nominal_pred])


def nominal_model_steer(delta_0: float, delta: float) -> float:
    """Predicted steer angle according to the nominal model."""
    nominal_pred = delta_0
    nominal_pred = (
        nominal_pred
        + (delta - nominal_pred)
        * drive_functions.ctrl_time_step
        / drive_functions.steer_time_constant
    )
    nominal_pred = (
        nominal_pred
        + (delta - nominal_pred)
        * drive_functions.ctrl_time_step
        / drive_functions.steer_time_constant
    )
    nominal_pred = (
        nominal_pred
        + (delta - nominal_pred)
        * drive_functions.ctrl_time_step
        / drive_functions.steer_time_constant
    )
    return nominal_pred


class train_drive_NN_model(add_training_data_from_csv.add_data_from_csv):
    """Class for training neural nets from driving data."""

    tanh_gain: float
    """Gain of tanh in the NN cost function"""

    lam: float
    """Weights of the tanh term in the NN cost function."""

    tol: float
    """Tolerances for terminating training iterations."""

    alpha_1: float
    """L¹ regularization weights in the NN cost function"""

    alpha_2: float
    """L² regularization weights in the NN cost function."""

    alpha_1_for_polynomial_regression: float
    """L¹ regularization weights for polynomial regression"""

    alpha_2_for_polynomial_regression: float
    """L² regularization weights for polynomial regression"""

    x_loss: list[float]
    """List of values for the loss function of the x component in the NN training."""

    y_loss: list[float]
    """List of values for the loss function of the y component in the NN training."""

    v_loss: list[float]
    """List of values for the loss function of the velocity component in the NN training."""

    theta_loss: list[float]
    """List of values for the loss function of the yaw angle component in the NN training."""

    acc_loss: list[float]
    """List of values for the loss function of the acceleration component in the NN training."""

    steer_loss: list[float]
    """List of values for the loss function of the steer angle component in the NN training."""

    steer_loss_plus_tanh: list[float]
    """List of values of the loss function for the steer angle component with tanh term in the NN training."""

    total_loss: list[float]
    """In NN training x_loss, ... , acc_loss, steer_loss_plus_tanh summed list of values"""

    model: drive_NN.DriveNeuralNetwork
    """trained neural network model"""

    A: np.ndarray
    """Coefficient matrix of polynomial regression model."""

    b: np.ndarray
    """Constant terms in polynomial regression models (bias)"""

    def __init__(
        self,
        tanh_gain=10,
        lam=0.1,
        tol=0.00001,
        alpha_1=0.1**7,
        alpha_2=0.1**7,
        alpha_1_for_polynomial_regression=0.1**5,
        alpha_2_for_polynomial_regression=0.1**5,
    ):
        super(train_drive_NN_model, self).__init__()
        self.tanh_gain = tanh_gain
        self.lam = lam
        self.tol = tol
        self.alpha_1 = alpha_1
        self.alpha_2 = alpha_2
        self.alpha_1_for_polynomial_regression = alpha_1_for_polynomial_regression
        self.alpha_2_for_polynomial_regression = alpha_2_for_polynomial_regression
        self.x_loss = []
        self.y_loss = []
        self.v_loss = []
        self.theta_loss = []
        self.acc_loss = []
        self.steer_loss = []
        self.steer_loss_plus_tanh = []
        self.total_loss = []

    def train_model(
        self,
        model: drive_NN.DriveNeuralNetwork,
        X_input: np.ndarray,
        Y_output: np.ndarray,
        learning_rates: list[float],
        patience: int = 10,
        batch_size: int | None = 50,
        max_iter: int = 100000,
    ) -> None:
        """Train the model."""
        if batch_size is None:
            batch_size = X_input.shape[0] // 50
        self.x_loss.clear()
        self.y_loss.clear()
        self.v_loss.clear()
        self.theta_loss.clear()
        self.acc_loss.clear()
        self.steer_loss.clear()
        self.steer_loss_plus_tanh.clear()
        self.total_loss.clear()
        sample_size = X_input.shape[0]
        num_train = int(3 * sample_size / 4)
        id_all = np.random.choice(sample_size, sample_size, replace=False)
        id_train = id_all[:num_train]
        id_val = id_all[num_train:]
        X_tensor = torch.tensor(X_input.astype(np.float32)).clone()
        Y_tensor = torch.tensor(Y_output.astype(np.float32)).clone()
        X_train = X_tensor[id_train]
        Y_train = Y_tensor[id_train]
        X_val = X_tensor[id_val]
        Y_val = Y_tensor[id_val]

        print("sample_size: ", X_input.shape[0])
        print("patience:", patience)
        loss_fn = torch.nn.L1Loss()
        learning_rate = learning_rates[0]
        optimizer = torch.optim.Adam(params=model.parameters(), lr=learning_rate)

        drive_data = DataLoader(
            TensorDataset(X_train, Y_train), batch_size=batch_size, shuffle=True
        )
        initial_loss = drive_NN.loss_fn_plus_tanh(
            loss_fn,
            torch.tensor(np.zeros(Y_val.shape), dtype=torch.float32),
            Y_val,
            self.tanh_gain,
            self.lam,
        )
        print("initial_loss:", initial_loss.detach().item())
        early_stopping = drive_NN.EarlyStopping(
            initial_loss=initial_loss, tol=self.tol, patience=patience
        )
        k = 0
        print("learning_rate:", learning_rates[0])
        for i in range(max_iter):
            model.train()
            for X_batch, y_batch in drive_data:
                y_pred = model(X_batch)
                loss = drive_NN.loss_fn_plus_tanh(
                    loss_fn, y_pred, y_batch, self.tanh_gain, self.lam
                )
                for w in model.parameters():
                    loss = (
                        loss + self.alpha_1 * torch.norm(w, p=1) + self.alpha_2 * torch.norm(w) ** 2
                    )
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

            model.eval()
            pred = model(X_val)
            self.total_loss.append(
                drive_NN.loss_fn_plus_tanh(loss_fn, pred, Y_val, self.tanh_gain, self.lam)
                .detach()
                .item()
            )
            self.x_loss.append(loss_fn(pred[:, [0]], Y_val[:, [0]]).detach().item())
            self.y_loss.append(loss_fn(pred[:, [1]], Y_val[:, [1]]).detach().item())
            self.v_loss.append(loss_fn(pred[:, [2]], Y_val[:, [2]]).detach().item())
            self.theta_loss.append(loss_fn(pred[:, [3]], Y_val[:, [3]]).detach().item())
            self.acc_loss.append(loss_fn(pred[:, [4]], Y_val[:, [4]]).detach().item())
            self.steer_loss.append(loss_fn(pred[:, [5]], Y_val[:, [5]]).detach().item())
            self.steer_loss_plus_tanh.append(
                loss_fn(pred[:, [5]], Y_val[:, [5]]).detach().item()
                + self.lam
                * loss_fn(
                    torch.tanh(self.tanh_gain * (pred[:, -1] - Y_val[:, -1])),
                    torch.zeros(Y_val.shape[0]),
                )
                .detach()
                .item()
            )
            if i % 10 == 1:
                current_loss = drive_NN.loss_fn_plus_tanh(
                    loss_fn, model(X_val), Y_val, self.tanh_gain, self.lam
                )
                print(current_loss.detach().item(), i)
                if early_stopping(current_loss):
                    k += 1
                    if k == len(learning_rates):
                        break
                    else:
                        learning_rate = learning_rates[k]
                        print("update learning_rate to ", learning_rates[k])
                        optimizer = torch.optim.Adam(params=model.parameters(), lr=learning_rate)
                        early_stopping = drive_NN.EarlyStopping(
                            initial_loss=initial_loss, tol=self.tol, patience=patience
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
        X_input_core = X_input[
            :,
            : 3 + drive_functions.acc_ctrl_queue_size + drive_functions.steer_ctrl_queue_size_core,
        ].copy()
        Y_output = np.array(Y_output_list)
        if use_selected_polynomial:
            self.deg = 2
        else:
            self.deg = deg
        self.A, self.b, Y_output_minus = polynomial_regression(
            X_input_core,
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

    def plot_trained_result(
        self, show_flag=False, save_dir=".", plot_range=np.arange(500, 1200)
    ) -> None:
        """Plot the results of the training."""
        polynomial_features = PolynomialFeatures(degree=self.deg, include_bias=False)

        X_input = np.array(self.X_input_list)
        Y_output = np.array(self.Y_output_list)
        X_tensor = torch.tensor(X_input.astype(np.float32)).clone()
        acc_ctrl_queue_size = drive_functions.acc_ctrl_queue_size
        steer_ctrl_queue_size = drive_functions.steer_ctrl_queue_size
        ctrl_time_step = drive_functions.ctrl_time_step
        Y_pred = (
            self.model(X_tensor).detach().numpy()
            + polynomial_features.fit_transform(X_input[:, ctrl_index_for_polynomial_reg])
            @ (self.A).T
            + self.b
        )
        x = max(acc_ctrl_queue_size, steer_ctrl_queue_size) * ctrl_time_step
        y_labels = [
            "x_error [m]",
            "y_error [m]",
            "vel_error [m/s]",
            "yaw_error [rad]",
            "acc_error [m/s^2]",
            "steer_error [rad]",
        ]

        plt.figure(figsize=(24, 15), tight_layout=True)
        plt.subplot(231)
        ax = []
        for i in range(6):
            ax.append(plt.subplot(2, 3, i + 1))
            ax[-1].plot(
                drive_functions.ctrl_time_step * plot_range + x,
                Y_output[plot_range, i] * drive_functions.mpc_time_step,
                label="nominal_error",
            )
            ax[-1].plot(
                drive_functions.ctrl_time_step * plot_range + x,
                Y_pred[plot_range, i] * drive_functions.mpc_time_step,
                label="pred_error",
            )
            ax[-1].set_xlabel("sec")
            ax[-1].set_ylabel(y_labels[i])
            ax[-1].legend()
        if show_flag:
            plt.show()
        else:
            plt.savefig(save_dir + "/train_drive_NN_model_fig.png")
        plt.close()

    def plot_loss(self, show_flag=False, save_dir=".") -> None:
        """Plot the progression of values of the loss function of the training."""
        plt.figure(figsize=(24, 15), tight_layout=True)

        y_loss_labels = [
            "total_loss",
            "x_loss",
            "y_loss",
            "vel_loss",
            "yaw_loss",
            "acc_loss",
            "steer_loss",
            "steer_plus_tanh_loss",
        ]
        plt.subplot(241)
        ax_2 = []
        loss_list = [
            np.array(self.total_loss),
            np.array(self.x_loss),
            np.array(self.y_loss),
            np.array(self.v_loss),
            np.array(self.theta_loss),
            np.array(self.acc_loss),
            np.array(self.steer_loss),
            np.array(self.steer_loss_plus_tanh),
        ]
        for i in range(8):
            ax_2.append(plt.subplot(2, 4, i + 1))
            ax_2[-1].plot(
                np.arange(loss_list[i].shape[0]),
                loss_list[i],
                label="loss",
            )
            ax_2[-1].set_xlabel("iteration")
            ax_2[-1].set_ylabel(y_loss_labels[i])
            ax_2[-1].legend()
        if show_flag:
            plt.show()
        else:
            plt.savefig(save_dir + "/loss.png")
        plt.close()

    def get_trained_model(
        self,
        learning_rates: list[float] = [1e-3, 1e-4, 1e-5, 1e-6],
        randomize=0.01,
        acc_drop_out=0.0,
        steer_drop_out=0.0,
        patience: int = 10,
        batch_size: int | None = 50,
        max_iter=100000,
        use_polynomial_reg=False,
        use_selected_polynomial=True,
        force_NN_model_to_zero=False,
        fit_intercept=True,
        use_intercept=None,
        deg: int = 2,
    ):
        """Train on a model for which initial values are randomly given in a suitable range."""
        if use_intercept is None:
            if force_NN_model_to_zero:
                use_intercept = True
            else:
                use_intercept = False
        X_input, Y_output_minus = self.get_polynomial_regression_result(
            self.X_input_list,
            self.Y_output_list,
            use_polynomial_reg,
            use_selected_polynomial,
            deg,
            fit_intercept,
            use_intercept,
        )
        self.model = drive_NN.DriveNeuralNetwork(
            randomize=randomize,
            acc_drop_out=acc_drop_out,
            steer_drop_out=steer_drop_out,
            acc_queue_size=drive_functions.acc_ctrl_queue_size,
            steer_queue_size=drive_functions.steer_ctrl_queue_size,
        )
        if force_NN_model_to_zero:
            for w in self.model.parameters():
                w.data = nn.Parameter(torch.zeros(w.shape))
        else:
            self.train_model(
                self.model, X_input, Y_output_minus, learning_rates, patience, batch_size, max_iter
            )

    def update_trained_model(
        self,
        learning_rates=[1e-4, 1e-5, 1e-6],
        patience=5,
        batch_size=50,
        max_iter=100000,
        use_polynomial_reg=False,
        use_selected_polynomial=True,
        force_NN_model_to_zero=False,
        fit_intercept=True,
        use_intercept=None,
        deg: int = 2,
    ):
        """Update `self.model` with additional learning."""
        if use_intercept is None:
            if force_NN_model_to_zero:
                use_intercept = True
            else:
                use_intercept = False
        X_input, Y_output_minus = self.get_polynomial_regression_result(
            self.X_input_list,
            self.Y_output_list,
            use_polynomial_reg,
            use_selected_polynomial,
            deg,
            fit_intercept,
            use_intercept,
        )
        if force_NN_model_to_zero:
            for w in self.model.parameters():
                w.data = nn.Parameter(torch.zeros(w.shape))
        else:
            self.train_model(
                self.model, X_input, Y_output_minus, learning_rates, patience, batch_size, max_iter
            )

    def update_saved_trained_model(
        self,
        path="model_for_test_drive.pth",
        learning_rates: list[float] = [1e-4, 1e-5, 1e-6],
        patience=5,
        batch_size: int | None = 50,
        max_iter=100000,
        use_polynomial_reg=False,
        use_selected_polynomial=True,
        force_NN_model_to_zero=False,
        fit_intercept=True,
        use_intercept=None,
        deg: int = 2,
    ) -> None:
        """Load the saved model and update the model with additional training."""
        if use_intercept is None:
            if force_NN_model_to_zero:
                use_intercept = True
            else:
                use_intercept = False
        X_input, Y_output_minus = self.get_polynomial_regression_result(
            self.X_input_list,
            self.Y_output_list,
            use_polynomial_reg,
            use_selected_polynomial,
            deg,
            fit_intercept,
            use_intercept,
        )
        self.model = torch.load(path)
        if force_NN_model_to_zero:
            for w in self.model.parameters():
                w.data = nn.Parameter(torch.zeros(w.shape))
        else:
            self.train_model(
                self.model, X_input, Y_output_minus, learning_rates, patience, batch_size, max_iter
            )

    def save_model(self, save_dir=".", path="model_for_test_drive.pth") -> None:
        """Save trained NN models."""
        torch.save(self.model, save_dir + "/" + path)

    def save_polynomial_reg_info(self, save_dir=".", path="polynomial_reg_info") -> None:
        """Save the coefficients and degree of the resulting polynomial regression."""
        np.savez(save_dir + "/" + path, A=self.A, b=self.b, deg=self.deg)

    def save_models(
        self,
        save_dir=".",
        model_name="model_for_test_drive.pth",
        polynomial_reg_info_name="polynomial_reg_info",
    ) -> None:
        """Run save_model and save_polynomial_reg_info."""
        self.save_model(save_dir, model_name)
        self.save_polynomial_reg_info(save_dir, polynomial_reg_info_name)

    def predict_error(
        self, v: float, alpha_0: float, delta_0: float, alpha: float, delta: float
    ) -> np.ndarray:
        """Predicts the prediction error when following a nominal model."""
        polynomial_features = PolynomialFeatures(degree=self.deg, include_bias=False)
        x_input = np.zeros(
            3 + drive_functions.acc_ctrl_queue_size + drive_functions.steer_ctrl_queue_size
        )
        x_input[0] = v
        x_input[1] = alpha_0
        x_input[2] = delta_0
        x_input[3 : 3 + drive_functions.acc_ctrl_queue_size] += alpha
        x_input[3 + drive_functions.acc_ctrl_queue_size :] += delta
        x_input_tensor = torch.tensor(x_input.reshape(1, -1).astype(np.float32))
        return (
            self.model(x_input_tensor)[0].detach().numpy()
            + self.A
            @ polynomial_features.fit_transform(
                x_input[ctrl_index_for_polynomial_reg].reshape(1, -1)
            )[0]
            + self.b
        )

    def sim(
        self, v: float, alpha_0: float, alpha: float, delta_0: float, delta: float, iter_times: int
    ) -> tuple[float, float, float]:
        """Simulate velocity, acceleration and steer changes according to a trained model."""
        v_pred = v
        alpha_pred = alpha_0
        delta_pred = delta_0
        for i in range(iter_times):
            pred = (
                np.concatenate(
                    (
                        nominal_model_acc(v_pred, alpha_pred, alpha),
                        np.array([nominal_model_steer(delta_pred, delta)]),
                    )
                )
                + self.predict_error(v_pred, alpha_pred, delta_pred, alpha, delta)[[2, 4, 5]]
                * drive_functions.mpc_time_step
            )
            v_pred = pred[0]
            alpha_pred = pred[1]
            steer_pred = pred[2]
        return v_pred, alpha_pred, steer_pred

    def plot_acc_map(self, iter_times: int, starting_steer=0.0, target_steer=0.0) -> None:
        """Run `sim` `iter_times` times. The simulation time is approximately 0.1*iter_times.

        Draw the resulting acceleration map.
        """
        acc = np.arange(-1, 1, 0.05)
        v_tests = np.arange(0, 12, 0.3)
        acc_result = np.zeros((acc.shape[0], v_tests.shape[0]))
        v_result = np.zeros((acc.shape[0], v_tests.shape[0]))
        steer_result = np.zeros((acc.shape[0], v_tests.shape[0]))
        for i in range(acc.shape[0]):
            v_tests_tmp = v_tests - 0.1 * iter_times * acc[i]
            for j in range(v_tests.shape[0]):
                v_result[i, j], acc_result[i, j], steer_result[i, j] = self.sim(
                    v_tests_tmp[j], acc[i], acc[i], starting_steer, target_steer, iter_times
                )
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot_surface(np.tile(acc, (v_tests.shape[0], 1)).T, v_result, acc_result)
        ax.set_xlabel("acc_input [m/s^2]")
        ax.set_ylabel("vel [m/s]")
        ax.set_zlabel("acc_sim [m/s^2]")
        fig.suptitle(str(iter_times * drive_functions.mpc_time_step) + " sec simulation")
        plt.show()

    def plot_steer_map(self, iter_times: int, starting_acc=0.0, target_acc=0.0) -> None:
        """Run `sim` `iter_times` times. The simulation time is approximately 0.1*iter_times.

        Draw the resulting steer map.
        """
        steer = np.arange(-1, 1, 0.05)
        v_tests = np.arange(0, 12, 0.3)
        acc_result = np.zeros((steer.shape[0], v_tests.shape[0]))
        v_result = np.zeros((steer.shape[0], v_tests.shape[0]))
        steer_result = np.zeros((steer.shape[0], v_tests.shape[0]))
        for i in range(steer.shape[0]):
            for j in range(v_tests.shape[0]):
                v_result[i, j], acc_result[i, j], steer_result[i, j] = self.sim(
                    v_tests[j], starting_acc, target_acc, steer[i], steer[i], iter_times
                )
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot_surface(np.tile(steer, (v_tests.shape[0], 1)).T, v_result, steer_result)
        ax.set_xlabel("steer_input [rad]")
        ax.set_ylabel("vel [m/s]")
        ax.set_zlabel("steer_sim [rad]")
        fig.suptitle(str(iter_times * drive_functions.mpc_time_step) + " sec simulation")
        plt.show()


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
            Y_error = Y.copy()
            A = np.zeros((Y.shape[1], X_poly.shape[1]))
            b = np.zeros(Y.shape[1])
            clf_1.fit(X_poly[:, [0, 7]], Y[:, [3]])
            clf_2.fit(X_poly[:, [1, 3]], Y[:, [4]])
            clf_3.fit(X_poly[:, [2, 4]], Y[:, [5]])
            A[3, [0, 7]] = clf_1.coef_
            A[4, [1, 3]] = clf_2.coef_
            A[5, [2, 4]] = clf_3.coef_
            if fit_intercept:
                b[3] = clf_1.intercept_
                b[4] = clf_2.intercept_
                b[5] = clf_3.intercept_
            Y_error[:, 3] -= clf_1.predict(X_poly[:, [0, 7]])
            Y_error[:, 4] -= clf_2.predict(X_poly[:, [1, 3]])
            Y_error[:, 5] -= clf_3.predict(X_poly[:, [2, 4]])
            if not use_intercept:
                Y_error += b
                b = 0 * b
            return A, b, Y_error
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
