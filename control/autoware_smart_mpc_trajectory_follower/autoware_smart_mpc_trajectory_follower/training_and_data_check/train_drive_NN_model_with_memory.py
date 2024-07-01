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

"""Class for training neural nets with memory from driving data."""
from autoware_smart_mpc_trajectory_follower.scripts import drive_NN
from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
from autoware_smart_mpc_trajectory_follower.training_and_data_check import (
    add_training_data_from_csv,
)
import matplotlib.pyplot as plt  # type: ignore
import numpy as np
from sklearn.preprocessing import PolynomialFeatures  # type: ignore
import torch
from torch import nn
from torch.utils.data import DataLoader
from torch.utils.data import TensorDataset

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


def transform_to_sequence_data(X, seq_size, indices):
    X_seq = []
    start_num = 0
    for i in range(len(indices)):
        j = 0
        while start_num + j + drive_functions.mpc_freq * seq_size <= indices[i]:
            X_seq.append(X[start_num + j + drive_functions.mpc_freq * np.arange(seq_size)])
            j += 1
        start_num = indices[i]

    return np.array(X_seq)


class train_drive_NN_model_with_memory(add_training_data_from_csv.add_data_from_csv):
    """Class for training neural nets from driving data."""

    tanh_gain: float
    """Gain of tanh in the NN cost function."""

    lam: float
    """Weights of the tanh term in the NN cost function."""

    tol: float
    """Tolerances for terminating training iterations."""

    alpha_1: float
    """L¹ regularization weights in the NN cost function."""

    alpha_2: float
    """L² regularization weights in the NN cost function."""

    alpha_1_for_polynomial_regression: float
    """L¹ regularization weights for polynomial regression."""

    alpha_2_for_polynomial_regression: float
    """L² regularization weights for polynomial regression."""

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
    """In NN training x_loss, ... , acc_loss, steer_loss_plus_tanh summed list of values."""

    model: drive_NN.DriveNeuralNetworkWithMemory
    """trained neural network model."""

    A: np.ndarray
    """Coefficient matrix of polynomial regression model."""

    b: np.ndarray
    """Constant terms in polynomial regression models (bias)."""

    def __init__(
        self,
        tanh_gain=10,
        lam=0.1,
        tol=0.00001,
        alpha_1=0.1**7,
        alpha_2=0.1**7,
        alpha_1_for_polynomial_regression=0.1**5,
        alpha_2_for_polynomial_regression=0.1**5,
        seq_size=2 * drive_functions.N + 1,
        first_order_weight=1.0,
        second_order_weight=1.0,
    ):
        super(train_drive_NN_model_with_memory, self).__init__()
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
        self.seq_size = seq_size
        self.first_order_weight = first_order_weight
        self.second_order_weight = second_order_weight

    def train_model(
        self,
        model: drive_NN.DriveNeuralNetworkWithMemory,
        X_input: np.ndarray,
        Y_output: np.ndarray,
        learning_rates: list[float],
        patience: int = 10,
        batch_size: int | None = 5,
        max_iter: int = 100000,
        X_val_np=None,
        Y_val_np=None,
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

        if X_val_np is None:
            X_tensor = torch.tensor(
                transform_to_sequence_data(X_input, self.seq_size, self.division_indices).astype(
                    np.float32
                )
            ).clone()
            Y_tensor = torch.tensor(
                transform_to_sequence_data(Y_output, self.seq_size, self.division_indices).astype(
                    np.float32
                )
            ).clone()
            sample_size = X_tensor.shape[0]
            num_train = int(3 * sample_size / 4)
            id_all = np.random.choice(sample_size, sample_size, replace=False)
            id_train = id_all[:num_train]
            id_val = id_all[num_train:]
            X_train = X_tensor[id_train]
            Y_train = Y_tensor[id_train]
            X_val = X_tensor[id_val]
            Y_val = Y_tensor[id_val]
            print("sample_size: ", X_input.shape[0])
        else:
            X_train = torch.tensor(
                transform_to_sequence_data(X_input, self.seq_size, self.division_indices).astype(
                    np.float32
                )
            ).clone()
            Y_train = torch.tensor(
                transform_to_sequence_data(Y_output, self.seq_size, self.division_indices).astype(
                    np.float32
                )
            ).clone()
            X_val = torch.tensor(
                transform_to_sequence_data(
                    X_val_np, self.seq_size, self.division_indices_val
                ).astype(np.float32)
            ).clone()
            Y_val = torch.tensor(
                transform_to_sequence_data(
                    Y_val_np, self.seq_size, self.division_indices_val
                ).astype(np.float32)
            ).clone()
            print("sample_size", X_input.shape[0] + X_val_np.shape[0])
        print("patience:", patience)
        loss_fn = torch.nn.L1Loss()
        learning_rate = learning_rates[0]
        optimizer = torch.optim.Adam(params=model.parameters(), lr=learning_rate)

        drive_data = DataLoader(
            TensorDataset(X_train, Y_train), batch_size=batch_size, shuffle=True
        )
        initial_loss = drive_NN.loss_fn_plus_tanh_with_memory(
            loss_fn,
            torch.tensor(np.zeros(Y_val.shape)[:, self.seq_size // 2 :], dtype=torch.float32),
            Y_val[:, self.seq_size // 2 :],
            self.tanh_gain,
            self.lam,
            self.first_order_weight,
            self.second_order_weight,
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
                loss = drive_NN.loss_fn_plus_tanh_with_memory(
                    loss_fn,
                    y_pred[:, self.seq_size // 2 :],
                    y_batch[:, self.seq_size // 2 :],
                    self.tanh_gain,
                    self.lam,
                    self.first_order_weight,
                    self.second_order_weight,
                )
                for w in model.named_parameters():
                    loss = (
                        loss
                        + self.alpha_1 * torch.norm(w[1], p=1)
                        + self.alpha_2 * torch.norm(w[1]) ** 2
                    )
                    if w[0] in ["finalize.weight", "finalize.bias"]:
                        loss = loss + (drive_functions.finalize_x_weight - 1) * (
                            self.alpha_1 * torch.norm(w[1][0], p=1)
                            + self.alpha_2 * torch.norm(w[1][0]) ** 2
                        )
                        loss = loss + (drive_functions.finalize_y_weight - 1) * (
                            self.alpha_1 * torch.norm(w[1][1], p=1)
                            + self.alpha_2 * torch.norm(w[1][1]) ** 2
                        )
                        loss = loss + (drive_functions.finalize_v_weight - 1) * (
                            self.alpha_1 * torch.norm(w[1][2], p=1)
                            + self.alpha_2 * torch.norm(w[1][2]) ** 2
                        )
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

            model.eval()
            pred = model(X_val)
            self.total_loss.append(
                drive_NN.loss_fn_plus_tanh_with_memory(
                    loss_fn,
                    pred[:, self.seq_size // 2 :],
                    Y_val[:, self.seq_size // 2 :],
                    self.tanh_gain,
                    self.lam,
                    self.first_order_weight,
                    self.second_order_weight,
                )
                .detach()
                .item()
            )
            self.x_loss.append(
                loss_fn(pred[:, self.seq_size // 2 :, [0]], Y_val[:, self.seq_size // 2 :, [0]])
                .detach()
                .item()
            )
            self.y_loss.append(
                loss_fn(pred[:, self.seq_size // 2 :, [1]], Y_val[:, self.seq_size // 2 :, [1]])
                .detach()
                .item()
            )
            self.v_loss.append(
                loss_fn(pred[:, self.seq_size // 2 :, [2]], Y_val[:, self.seq_size // 2 :, [2]])
                .detach()
                .item()
            )
            self.theta_loss.append(
                loss_fn(pred[:, self.seq_size // 2 :, [3]], Y_val[:, self.seq_size // 2 :, [3]])
                .detach()
                .item()
            )
            self.acc_loss.append(
                loss_fn(pred[:, self.seq_size // 2 :, [4]], Y_val[:, self.seq_size // 2 :, [4]])
                .detach()
                .item()
            )
            self.steer_loss.append(
                loss_fn(pred[:, self.seq_size // 2 :, [5]], Y_val[:, self.seq_size // 2 :, [5]])
                .detach()
                .item()
            )
            self.steer_loss_plus_tanh.append(
                loss_fn(pred[:, self.seq_size // 2 :, [5]], Y_val[:, self.seq_size // 2 :, [5]])
                .detach()
                .item()
                + self.lam
                * loss_fn(
                    torch.tanh(
                        self.tanh_gain
                        * (pred[:, self.seq_size // 2 :, -1] - Y_val[:, self.seq_size // 2 :, -1])
                    ),
                    torch.zeros((Y_val.shape[0], Y_val.shape[1] - self.seq_size // 2)),
                )
                .detach()
                .item()
            )

            current_loss = drive_NN.loss_fn_plus_tanh_with_memory(
                loss_fn,
                model(X_val)[:, self.seq_size // 2 :],
                Y_val[:, self.seq_size // 2 :],
                self.tanh_gain,
                self.lam,
                self.first_order_weight,
                self.second_order_weight,
            )
            if i % 10 == 1:
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
        plt.subplot(2, 4, 1)
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
        Y_pred = np.zeros(Y_output.shape)
        start_index = 0
        for i in range(len(self.division_indices)):
            Y_pred[start_index : self.division_indices[i]] = (
                self.model(X_tensor[start_index : self.division_indices[i]].unsqueeze(dim=0))[0]
                .detach()
                .numpy()
                + polynomial_features.fit_transform(
                    X_input[start_index : self.division_indices[i], ctrl_index_for_polynomial_reg]
                )
                @ (self.A).T
                + self.b
            )
            start_index = self.division_indices[i]
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
        plt.subplot(2, 3, 1)
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

    def get_trained_model(
        self,
        hidden_layer_sizes=(16, 16),
        hidden_layer_lstm=8,
        learning_rates: list[float] = [1e-3, 1e-4, 1e-5, 1e-6],
        randomize=0.01,
        acc_drop_out=0.0,
        steer_drop_out=0.0,
        patience: int = 10,
        batch_size: int | None = 5,
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

        if len(self.X_val_list) > 0:
            X_val = np.array(self.X_val_list)
            Y_val = np.array(self.Y_val_list)
            polynomial_features = PolynomialFeatures(degree=self.deg, include_bias=False)
            Y_val = (
                Y_val
                - polynomial_features.fit_transform(X_val[:, ctrl_index_for_polynomial_reg])
                @ (self.A).T
                - self.b
            )
        else:
            X_val = None
            Y_val = None
        self.model = drive_NN.DriveNeuralNetworkWithMemory(
            hidden_layer_sizes=hidden_layer_sizes,
            hidden_layer_lstm=hidden_layer_lstm,
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
                self.model,
                X_input,
                Y_output_minus,
                learning_rates,
                patience,
                batch_size,
                max_iter,
                X_val,
                Y_val,
            )

    def update_trained_model(
        self,
        learning_rates=[1e-4, 1e-5, 1e-6],
        patience=10,
        batch_size=5,
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
        if len(self.X_val_list) > 0:
            X_val = np.array(self.X_val_list)
            Y_val = np.array(self.Y_val_list)
            polynomial_features = PolynomialFeatures(degree=self.deg, include_bias=False)
            Y_val = (
                Y_val
                - polynomial_features.fit_transform(X_val[:, ctrl_index_for_polynomial_reg])
                @ (self.A).T
                - self.b
            )
        else:
            X_val = None
            Y_val = None
        if force_NN_model_to_zero:
            for w in self.model.parameters():
                w.data = nn.Parameter(torch.zeros(w.shape))
        else:
            self.train_model(
                self.model,
                X_input,
                Y_output_minus,
                learning_rates,
                patience,
                batch_size,
                max_iter,
                X_val,
                Y_val,
            )

    def update_saved_trained_model(
        self,
        path="model_for_test_drive.pth",
        learning_rates: list[float] = [1e-4, 1e-5, 1e-6],
        patience=10,
        batch_size: int | None = 5,
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
        if len(self.X_val_list) > 0:
            X_val = np.array(self.X_val_list)
            Y_val = np.array(self.Y_val_list)
            polynomial_features = PolynomialFeatures(degree=self.deg, include_bias=False)
            Y_val = (
                Y_val
                - polynomial_features.fit_transform(X_val[:, ctrl_index_for_polynomial_reg])
                @ (self.A).T
                - self.b
            )
        else:
            X_val = None
            Y_val = None
        self.model = torch.load(path)
        if force_NN_model_to_zero:
            for w in self.model.parameters():
                w.data = nn.Parameter(torch.zeros(w.shape))
        else:
            self.train_model(
                self.model,
                X_input,
                Y_output_minus,
                learning_rates,
                patience,
                batch_size,
                max_iter,
                X_val,
                Y_val,
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
