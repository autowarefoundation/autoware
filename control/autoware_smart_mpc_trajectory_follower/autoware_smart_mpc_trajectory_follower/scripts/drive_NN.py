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


from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
from autoware_smart_mpc_trajectory_follower.scripts import proxima_calc
import numpy as np
import torch
from torch import nn

dim_steer_layer_1_head = 32
dim_steer_layer_1_tail = 16
dim_steer_layer_2 = 8
dim_acc_layer_1 = 16
dim_acc_layer_2 = 16


def nominal_model_input(Var, lam, step):
    """Calculate prediction with nominal model that takes into account time constants for the first order and time delays related to the input."""
    nominal_pred = Var[:, 0]
    nominal_pred = (
        nominal_pred + (Var[:, step] - nominal_pred) * drive_functions.ctrl_time_step / lam
    )
    nominal_pred = (
        nominal_pred + (Var[:, step - 1] - nominal_pred) * drive_functions.ctrl_time_step / lam
    )
    nominal_pred = (
        nominal_pred + (Var[:, step - 2] - nominal_pred) * drive_functions.ctrl_time_step / lam
    )
    return nominal_pred


def loss_fn_plus_tanh(loss_fn, pred, Y, tanh_gain, tanh_weight):
    """Compute the loss function to be used in the training."""
    loss = loss_fn(pred, Y)
    loss += tanh_weight * loss_fn(
        torch.tanh(tanh_gain * (pred[:, -1] - Y[:, -1])), torch.zeros(Y.shape[0])
    )
    return loss


# example usage: `model = DriveNeuralNetwork(params...).to("cpu")`
class DriveNeuralNetwork(nn.Module):
    """Define the neural net model to be used in vehicle control."""

    def __init__(
        self,
        hidden_layer_sizes=(32, 16),
        randomize=0.01,
        acc_drop_out=0.0,
        steer_drop_out=0.0,
        acc_delay_step=drive_functions.acc_delay_step,
        steer_delay_step=drive_functions.steer_delay_step,
        acc_time_constant_ctrl=drive_functions.acc_time_constant,
        steer_time_constant_ctrl=drive_functions.steer_time_constant,
        acc_queue_size=drive_functions.acc_ctrl_queue_size,
        steer_queue_size=drive_functions.steer_ctrl_queue_size,
        steer_queue_size_core=drive_functions.steer_ctrl_queue_size_core,
    ):
        super().__init__()
        self.acc_time_constant_ctrl = acc_time_constant_ctrl
        self.acc_delay_step = acc_delay_step
        self.steer_time_constant_ctrl = steer_time_constant_ctrl
        self.steer_delay_step = steer_delay_step

        lb = -randomize
        ub = randomize
        self.acc_input_index = np.concatenate(([1], np.arange(acc_queue_size) + 3))
        self.steer_input_index = np.concatenate(
            ([2], np.arange(steer_queue_size_core) + acc_queue_size + 3)
        )
        self.acc_input_index_ = np.arange(acc_queue_size) + 3
        self.steer_input_index_ = np.arange(steer_queue_size_core) + acc_queue_size + 3
        self.steer_input_index_full = np.arange(steer_queue_size) + acc_queue_size + 3
        self.acc_layer_1 = nn.Sequential(
            nn.Linear(self.acc_input_index.shape[0], dim_acc_layer_1),
            nn.ReLU(),
        )
        nn.init.uniform_(self.acc_layer_1[0].weight, a=lb, b=ub)
        nn.init.uniform_(self.acc_layer_1[0].bias, a=lb, b=ub)
        self.steer_layer_1_head = nn.Sequential(
            nn.Linear(self.steer_input_index.shape[0], dim_steer_layer_1_head),
            nn.ReLU(),
        )
        nn.init.uniform_(self.steer_layer_1_head[0].weight, a=lb, b=ub)
        nn.init.uniform_(self.steer_layer_1_head[0].bias, a=lb, b=ub)

        self.steer_layer_1_tail = nn.Sequential(
            nn.Linear(self.steer_input_index_full.shape[0], dim_steer_layer_1_tail), nn.ReLU()
        )
        nn.init.uniform_(self.steer_layer_1_tail[0].weight, a=lb, b=ub)
        nn.init.uniform_(self.steer_layer_1_tail[0].bias, a=lb, b=ub)

        self.acc_layer_2 = nn.Sequential(nn.Linear(dim_acc_layer_1, dim_acc_layer_2), nn.Tanh())
        nn.init.uniform_(self.acc_layer_2[0].weight, a=lb, b=ub)
        nn.init.uniform_(self.acc_layer_2[0].bias, a=lb, b=ub)

        self.steer_layer_2 = nn.Sequential(
            nn.Linear(dim_steer_layer_1_head + dim_steer_layer_1_tail, dim_steer_layer_2), nn.Tanh()
        )
        nn.init.uniform_(self.steer_layer_2[0].weight, a=lb, b=ub)
        nn.init.uniform_(self.steer_layer_2[0].bias, a=lb, b=ub)

        self.linear_relu_stack = nn.Sequential(
            nn.Linear(1 + dim_acc_layer_2 + dim_steer_layer_2, hidden_layer_sizes[0]),
            nn.ReLU(),
            nn.Linear(hidden_layer_sizes[0], hidden_layer_sizes[1]),
            nn.ReLU(),
        )
        nn.init.uniform_(self.linear_relu_stack[0].weight, a=lb, b=ub)
        nn.init.uniform_(self.linear_relu_stack[0].bias, a=lb, b=ub)
        nn.init.uniform_(self.linear_relu_stack[2].weight, a=lb, b=ub)
        nn.init.uniform_(self.linear_relu_stack[2].bias, a=lb, b=ub)

        self.finalize = nn.Linear(hidden_layer_sizes[1] + dim_acc_layer_2 + dim_steer_layer_2, 6)

        nn.init.uniform_(self.finalize.weight, a=lb, b=ub)
        nn.init.uniform_(self.finalize.bias, a=lb, b=ub)

        self.acc_dropout = nn.Dropout(acc_drop_out)
        self.steer_dropout = nn.Dropout(steer_drop_out)

    def forward(self, x):
        acc_layer_1 = self.acc_layer_1(x[:, self.acc_input_index])
        steer_layer_1 = torch.cat(
            (
                self.steer_layer_1_head(x[:, self.steer_input_index]),
                self.steer_layer_1_tail(x[:, self.steer_input_index_full]),
            ),
            dim=1,
        )
        acc_layer_2 = self.acc_layer_2(self.acc_dropout(acc_layer_1))
        steer_layer_2 = self.steer_layer_2(self.steer_dropout(steer_layer_1))

        pre_pred = self.linear_relu_stack(torch.cat((x[:, [0]], acc_layer_2, steer_layer_2), dim=1))
        pred = self.finalize(torch.cat((pre_pred, acc_layer_2, steer_layer_2), dim=1))
        return pred


class EarlyStopping:
    """Class for early stopping in NN training."""

    def __init__(self, initial_loss, tol=0.01, patience=30):
        self.epoch = 0  # Initialise the counter for the number of epochs being monitored.
        self.best_loss = float("inf")  # Initialise loss of comparison with infinity 'inf'.
        self.patience = patience  # Initialise the number of epochs to be monitored with a parameter
        self.initial_loss = initial_loss.detach().item()
        self.tol = tol

    def __call__(self, current_loss):
        current_loss_num = current_loss.detach().item()
        if current_loss_num + self.tol * self.initial_loss > self.best_loss:
            self.epoch += 1
        else:
            self.epoch = 0
        if current_loss_num < self.best_loss:
            self.best_loss = current_loss_num
        if self.epoch >= self.patience:
            return True
        return False


class transform_model_to_c:
    """Pass the necessary information to the C++ program to call the trained model at high speed."""

    def __init__(
        self,
        model,
        A_for_linear_reg,
        b_for_linear_reg,
        deg,
        acc_time_constant,
        acc_delay_step,
        steer_time_constant,
        steer_delay_step,
        acc_queue_size,
        steer_queue_size,
        steer_queue_size_core,
    ):
        transformer = proxima_calc.transform_model_to_eigen()

        numpy_weight_list = []
        numpy_bias_list = []
        for i, w in enumerate(model.parameters()):
            if i % 2 == 0:
                numpy_weight_list.append(w.detach().numpy().astype(np.float64))
            else:
                numpy_bias_list.append(w.detach().numpy().astype(np.float64))
        array_weight_list_0 = numpy_weight_list[0]
        array_bias_list_0 = numpy_bias_list[0]
        array_weight_list_1 = numpy_weight_list[1]
        array_bias_list_1 = numpy_bias_list[1]
        array_weight_list_2 = numpy_weight_list[2]
        array_bias_list_2 = numpy_bias_list[2]
        array_weight_list_3 = numpy_weight_list[3]
        array_bias_list_3 = numpy_bias_list[3]
        array_weight_list_4 = numpy_weight_list[4]
        array_bias_list_4 = numpy_bias_list[4]
        array_weight_list_5 = numpy_weight_list[5]
        array_bias_list_5 = numpy_bias_list[5]
        array_weight_list_6 = numpy_weight_list[6]
        array_bias_list_6 = numpy_bias_list[6]
        array_weight_list_7 = numpy_weight_list[7]
        array_bias_list_7 = numpy_bias_list[7]

        transformer.set_params(
            array_weight_list_0,
            array_weight_list_1,
            array_weight_list_2,
            array_weight_list_3,
            array_weight_list_4,
            array_weight_list_5,
            array_weight_list_6,
            array_weight_list_7,
            array_bias_list_0,
            array_bias_list_1,
            array_bias_list_2,
            array_bias_list_3,
            array_bias_list_4,
            array_bias_list_5,
            array_bias_list_6,
            array_bias_list_7,
            A_for_linear_reg,
            b_for_linear_reg,
            deg,
            acc_time_constant,
            acc_delay_step,
            steer_time_constant,
            steer_delay_step,
            acc_queue_size,
            steer_queue_size,
            steer_queue_size_core,
        )
        self.pred = transformer.rot_and_d_rot_error_prediction
        self.pred_with_diff = transformer.rot_and_d_rot_error_prediction_with_diff
        self.Pred = transformer.Rotated_error_prediction
