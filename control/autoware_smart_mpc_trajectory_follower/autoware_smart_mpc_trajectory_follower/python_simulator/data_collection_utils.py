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

import csv

from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
from numba import njit  # type: ignore
import numpy as np
from numpy import arctan
from numpy import cos
from numpy import pi
from numpy import sin

RANDOM_SEED_STEP_RESPONSE = 42


def get_feedforward_nominal_input(t, trajectory_data):
    """Calculate the nominal input for feed-forward driving."""
    total_time = trajectory_data[-1, 0]
    t_current = t - (t // total_time) * total_time
    nearest_index = np.argmin(np.abs(trajectory_data[:, 0] - t_current))
    return trajectory_data[nearest_index, [5, 6]]


def create_additional_sine_data(
    seed,
    t_range,
    acc_amp_range,
    acc_period_range,
    steer_amp_range,
    steer_period_range,
    large_steer_amp_range,
    large_steer_period_range,
    start_large_steer_time,
):
    """Create sine wave data to be added randomly to feed-forward runs."""
    np.random.seed(seed=seed)
    t_acc = 0.0
    t_steer = 0.0
    t_large_steer = 0.0
    t_acc_list = []
    t_steer_list = []
    t_large_steer_list = []
    t_acc_list.append(t_acc)
    t_steer_list.append(t_steer)
    t_large_steer_list.append(t_large_steer)
    t_large_steer += start_large_steer_time
    t_large_steer_list.append(t_large_steer)
    amp_acc_list = []
    amp_steer_list = []
    amp_large_steer_list = []
    amp_large_steer_list.append(0)
    while True:
        if max(t_acc, t_large_steer) >= t_steer:
            period = (
                steer_period_range[1] - steer_period_range[0]
            ) * np.random.uniform() + steer_period_range[0]
            t_steer += period
            t_steer_list.append(t_steer)
            amp_steer_list.append(steer_amp_range * np.random.uniform())
        elif t_large_steer >= t_acc:
            period = (
                acc_period_range[1] - acc_period_range[0]
            ) * np.random.uniform() + acc_period_range[0]
            t_acc += period
            t_acc_list.append(t_acc)
            amp_acc_list.append(acc_amp_range * np.random.uniform())
        else:
            period = (
                large_steer_period_range[1] - large_steer_period_range[0]
            ) * np.random.uniform() + large_steer_period_range[0]
            t_large_steer += period
            t_large_steer_list.append(t_large_steer)
            amp_large_steer_list.append(large_steer_amp_range * np.random.uniform())
        if t_acc >= t_range[1] and t_steer >= t_range[1] and t_large_steer >= t_range[1]:
            break
    return (
        np.array(t_acc_list),
        np.array(amp_acc_list),
        np.array(t_steer_list),
        np.array(amp_steer_list),
        np.array(t_large_steer_list),
        np.array(amp_large_steer_list),
    )


# cSpell:ignore njit fastmath numba
@njit(cache=False, fastmath=True)
def get_current_additional_sine(
    t,
    t_acc_array,
    amp_acc_array,
    t_steer_array,
    amp_steer_array,
    t_large_steer_array,
    amp_large_steer_array,
):
    """Calculate current values from already created sine wave data."""
    acc_index = 0
    steer_index = 0
    large_steer_index = 0
    for i in range(t_acc_array.shape[0] - 1):
        if t < t_acc_array[i + 1]:
            break
        acc_index += 1
    for i in range(t_steer_array.shape[0] - 1):
        if t < t_steer_array[i + 1]:
            break
        steer_index += 1
    for i in range(t_large_steer_array.shape[0] - 1):
        if t < t_large_steer_array[i + 1]:
            break
        large_steer_index += 1
    acc = amp_acc_array[acc_index] * np.sin(
        2
        * np.pi
        * (t - t_acc_array[acc_index])
        / (t_acc_array[acc_index + 1] - t_acc_array[acc_index])
    )
    steer = amp_steer_array[steer_index] * np.sin(
        2
        * np.pi
        * (t - t_steer_array[steer_index])
        / (t_steer_array[steer_index + 1] - t_steer_array[steer_index])
    )
    steer += amp_large_steer_array[large_steer_index] * np.sin(
        2
        * np.pi
        * (t - t_large_steer_array[large_steer_index])
        / (t_large_steer_array[large_steer_index + 1] - t_large_steer_array[large_steer_index])
    )
    return np.array([acc, steer])


@njit(cache=False, fastmath=True)
def get_vel_sine(t, v_mid, v_range, period, constant_vel_time):
    """Calculate current target velocity values already created sine wave data."""
    if t < period / 4:
        vel = v_mid + v_range * np.sin(2 * np.pi * t / period)
    elif t < period / 4 + constant_vel_time:
        vel = v_mid + v_range
    elif t < 3 * period / 4 + constant_vel_time:
        vel = v_mid + v_range * np.sin(2 * np.pi * (t - constant_vel_time) / period)
    elif t < 3 * period / 4 + 2 * constant_vel_time:
        vel = v_mid - v_range
    else:
        vel = v_mid + v_range * np.sin(2 * np.pi * (t - 2 * constant_vel_time) / period)

    return vel


def get_periodic_count(counter, split_size):
    return split_size - 0.5 - np.abs(counter % (2 * split_size - 1) - split_size + 0.5)


def compute_curvature_radius(trajectory_position_data, trajectory_yaw_data):
    d_step = 5
    curvature_radius = []
    for i in range(len(trajectory_position_data)):
        tmp_pos = trajectory_position_data[i]
        tmp_yaw = trajectory_yaw_data[i]
        tmp_computed_flag = False
        for j in range(i, len(trajectory_position_data)):
            distance = np.sqrt(((tmp_pos[:2] - trajectory_position_data[j, :2]) ** 2).sum())
            if distance >= d_step:
                diff_yaw = tmp_yaw - trajectory_yaw_data[j]
                if diff_yaw > np.pi:
                    diff_yaw -= 2 * np.pi
                if diff_yaw < -np.pi:
                    diff_yaw += 2 * np.pi
                curvature_radius.append(distance / (1e-12 + np.abs(diff_yaw)))
                tmp_computed_flag = True
                break
        if tmp_computed_flag is False:
            curvature_radius.append(1 * curvature_radius[-1])
    curvature_radius = np.array(curvature_radius)
    return curvature_radius


def compute_curvature_radius_loop_trajectory(trajectory_position_data, trajectory_yaw_data):
    data_length = len(trajectory_position_data)
    augmented_trajectory_position_data = np.vstack(
        [trajectory_position_data, trajectory_position_data[: data_length // 2]]
    )
    augmented_trajectory_yaw_data = np.hstack(
        [trajectory_yaw_data, trajectory_yaw_data[: data_length // 2]]
    )
    return compute_curvature_radius(
        augmented_trajectory_position_data, augmented_trajectory_yaw_data
    )[:data_length]


class StraightLine:
    """Straight line target velocity."""

    def __init__(self, v_min=1.0, v_max=11.0, acc_max=1.2, constant_vel_time=5.0, split_size=5):
        self.v_min = v_min
        self.v_max = v_max
        self.v_mid = 0.5 * (v_min + v_max)
        self.period = 2 * np.pi * (v_max - self.v_mid) / acc_max
        self.constant_vel_time = constant_vel_time
        self.split_size = split_size
        self.break_flag = False

    def get_current_velocity(self, t):
        index = int(t / (self.period + 2 * self.constant_vel_time))
        t1 = t - (self.period + 2 * self.constant_vel_time) * index
        if index < 2 * self.split_size:
            adjust = 0.0
            if index >= self.split_size:
                adjust = 0.5
            v_range = (
                (self.v_max - self.v_mid)
                * (get_periodic_count(index, self.split_size) + 1 - adjust)
                / self.split_size
            )
            return get_vel_sine(t1, self.v_mid, v_range, self.period, self.constant_vel_time)
        else:
            if t1 > 2 * self.constant_vel_time:
                self.break_flag = True
            return self.v_mid


class FigureEight:
    """Figure eight trajectory."""

    def __init__(
        self,
        y_length: float,
        x_length: float,
        v_min=1.0,
        v_max=11.0,
        split_size=5,
        acc_max=1.2,
        constant_vel_time=5.0,
        smoothing_trajectory_data_flag=False,
    ):
        if y_length >= x_length:
            raise Exception("invalid argument: y_length must be less than x_length")
        self.y_length = y_length
        self.x_length = x_length
        self.v_min = v_min
        self.v_max = v_max
        self.split_size = split_size

        self.v_mid = 0.5 * (v_min + v_max)
        self.v_start = self.v_mid

        self.period = 2 * np.pi * (v_max - self.v_mid) / acc_max
        self.constant_vel_time = constant_vel_time

        self.counter = 0
        self.previous_circle = "left_circle"
        self.break_flag = False
        self.accel_mode = 1
        self.smoothing_trajectory_data_flag = smoothing_trajectory_data_flag

    @property
    def total_distance(self) -> float:
        a = self.y_length
        b = self.x_length
        arc = a * pi
        diagonal = 2 * np.sqrt((b - a) ** 2 + a**2)
        return arc + diagonal

    def get_trajectory_points(self, step: float):
        """Get the position and yaw angle in world coordinates of the figure eight.

        The return value is a 2-dimensional array of positions and a 1-dimensional array of yaw angles corresponding to `t`.
        """
        a = self.y_length
        b = self.x_length

        t_array = np.arange(start=0.0, stop=self.total_distance, step=step).astype("float")
        x = t_array.copy()
        y = t_array.copy()
        yaw = t_array.copy()
        curvature_radius = t_array.copy()
        parts = []
        achievement_rates = []

        # Boundary points between circular and linear trajectory
        C = [-(b - a) / 2, -a / 2]
        D = [(b - a) / 2, -a / 2]

        R = a / 2  # radius of the circle
        OL = [-(b - a) / 2, 0]  # center of the left circle
        OR = [(b - a) / 2, 0]  # center of the right circle
        OB = np.sqrt((b - a) ** 2 + a**2) / 2  # half length of the linear trajectory
        AD = 2 * OB
        θB = arctan(a / (b - a))  # Angle that OB makes with respect to x-axis
        BD = pi * a / 2  # the length of arc BD
        AC = BD
        CO = OB

        i_end = t_array.shape[0]
        for i, t in enumerate(t_array):
            if t > OB + BD + AD + AC + CO:
                i_end = i
                break
            if 0 <= t and t <= OB:
                x[i] = (b - a) * t / (2 * OB)
                y[i] = a * t / (2 * OB)
                yaw[i] = θB
                curvature_radius[i] = 1e12
                parts.append("linear_positive")
                achievement_rates.append(t / (2 * OB) + 0.5)
            if OB <= t and t <= OB + BD:
                t1 = t - OB
                t1_rad = t1 / R
                x[i] = OR[0] + R * cos(pi / 2 - t1_rad)
                y[i] = OR[1] + R * sin(pi / 2 - t1_rad)
                yaw[i] = -t1_rad
                curvature_radius[i] = R
                parts.append("right_circle")
                achievement_rates.append(0.0)
            if OB + BD <= t and t <= OB + BD + AD:
                t2 = t - (OB + BD)
                x[i] = D[0] - (b - a) * t2 / (2 * OB)
                y[i] = D[1] + a * t2 / (2 * OB)
                yaw[i] = pi - θB
                curvature_radius[i] = 1e12
                parts.append("linear_negative")
                achievement_rates.append(t2 / (2 * OB))
            if OB + BD + AD <= t and t <= OB + BD + AD + AC:
                t3 = t - (OB + BD + AD)
                t3_rad = t3 / R
                x[i] = OL[0] + R * cos(pi / 2 + t3_rad)
                y[i] = OL[1] + R * sin(pi / 2 + t3_rad)
                yaw[i] = pi + t3_rad
                curvature_radius[i] = R
                parts.append("left_circle")
                achievement_rates.append(0.0)
            if OB + BD + AD + AC <= t and t <= OB + BD + AD + AC + CO:
                t4 = t - (OB + BD + AD + AC)
                x[i] = C[0] + (b - a) * t4 / (2 * OB)
                y[i] = C[1] + a * t4 / (2 * OB)
                yaw[i] = θB
                curvature_radius[i] = 1e12
                parts.append("linear_positive")
                achievement_rates.append(t4 / (2 * OB))

        # drop rest
        x = x[:i_end]
        y = y[:i_end]
        trajectory_position_data = np.array([x, y]).T
        trajectory_yaw_data = yaw[:i_end]
        curvature_radius = curvature_radius[:i_end]
        parts = parts[:i_end]
        achievement_rates = achievement_rates[:i_end]

        if self.smoothing_trajectory_data_flag:
            window = 1000
            w = np.ones(window) / window
            augmented_position_data = np.vstack(
                [
                    trajectory_position_data[-window:],
                    trajectory_position_data,
                    trajectory_position_data[:window],
                ]
            )
            trajectory_position_data[:, 0] = (
                1 * np.convolve(augmented_position_data[:, 0], w, mode="same")[window:-window]
            )
            trajectory_position_data[:, 1] = (
                1 * np.convolve(augmented_position_data[:, 1], w, mode="same")[window:-window]
            )
            augmented_yaw_data = np.hstack(
                [
                    trajectory_yaw_data[-window:],
                    trajectory_yaw_data,
                    trajectory_yaw_data[:window],
                ]
            )
            smoothed_trajectory_yaw_data = trajectory_yaw_data.copy()
            for i in range(len(trajectory_yaw_data)):
                tmp_yaw = trajectory_yaw_data[i]
                tmp_data = (
                    augmented_yaw_data[window + (i - window // 2) : window + (i + window // 2)]
                    - tmp_yaw
                )
                for j in range(len(tmp_data)):
                    if tmp_data[j] > np.pi:
                        tmp_data[j] -= 2 * np.pi
                    if tmp_data[j] < -np.pi:
                        tmp_data[j] += 2 * np.pi
                tmp_data = np.convolve(tmp_data, w, mode="same")
                smoothed_trajectory_yaw_data[i] = (
                    tmp_yaw + np.convolve(tmp_data, w, mode="same")[window // 2]
                )
                if smoothed_trajectory_yaw_data[i] > np.pi:
                    smoothed_trajectory_yaw_data[i] -= 2 * np.pi
                if smoothed_trajectory_yaw_data[i] < -np.pi:
                    smoothed_trajectory_yaw_data[i] += 2 * np.pi

            trajectory_yaw_data = smoothed_trajectory_yaw_data.copy()

        return (
            trajectory_position_data,
            trajectory_yaw_data,
            curvature_radius,
            parts,
            np.array(achievement_rates),
        )

    def get_current_velocity(self, t):
        index = int(t / (self.period + 2 * self.constant_vel_time))
        t1 = t - (self.period + 2 * self.constant_vel_time) * index
        if index < 2 * self.split_size:
            adjust = 0.0
            if index >= self.split_size:
                adjust = 0.5
            v_range = (
                (self.v_max - self.v_mid)
                * (get_periodic_count(index, self.split_size) + 1 - adjust)
                / self.split_size
            )
            return get_vel_sine(t1, self.v_mid, v_range, self.period, self.constant_vel_time)
        else:
            self.break_flag = True
            return self.v_mid


def get_pure_pursuit_info(x_current, trajectory_position_data, trajectory_yaw_data, previous_index):
    """Calculate the target position and yaw angle required for pure pursuit."""
    search_range = (
        np.arange(
            previous_index - trajectory_position_data.shape[0] // 4,
            previous_index + trajectory_position_data.shape[0] // 4,
        )
        % trajectory_position_data.shape[0]
    )
    nearest_index = np.argmin(
        ((trajectory_position_data[search_range] - x_current[:2].reshape(1, 2)) ** 2).sum(axis=1)
    )
    return (
        trajectory_position_data[search_range[nearest_index]],
        trajectory_yaw_data[search_range[nearest_index]],
        search_range[nearest_index],
    )


def get_naive_pure_pursuit_info(
    x_current, trajectory_position_data, trajectory_yaw_data, previous_index
):
    """Calculate the target position and yaw angle required for naive pure pursuit."""
    search_range = (
        np.arange(
            previous_index - trajectory_position_data.shape[0] // 4,
            previous_index + trajectory_position_data.shape[0] // 4,
        )
        % trajectory_position_data.shape[0]
    )
    nearest_index = np.argmin(
        ((trajectory_position_data[search_range] - x_current[:2].reshape(1, 2)) ** 2).sum(axis=1)
    )
    lookahead_distance = (
        drive_functions.naive_pure_pursuit_lookahead_coef * x_current[2]
        + drive_functions.naive_pure_pursuit_lookahead_intercept
    )

    aug_trajectory_position_data = np.vstack([trajectory_position_data, trajectory_position_data])
    i = 0
    while True:
        tmp_distance = np.sqrt(
            (
                (aug_trajectory_position_data[search_range[nearest_index] + i] - x_current[:2]) ** 2
            ).sum()
        )
        if tmp_distance > lookahead_distance:
            break
        if (search_range[nearest_index] + i) == (len(aug_trajectory_position_data) - 1):
            break
        i += 1
    return (
        trajectory_position_data[search_range[nearest_index]],
        trajectory_yaw_data[search_range[nearest_index]],
        search_range[nearest_index],
        aug_trajectory_position_data[search_range[nearest_index] + i],
    )


def restrict_target_vel(delta, v_m=2.0, v_M=15.0, steer_m=0.01, steer_M=0.3):
    if np.abs(delta) < steer_m:
        return v_M
    elif np.abs(delta) < steer_M:
        return (-(v_M - v_m) * delta + (v_M * steer_M - v_m * steer_m)) / (steer_M - steer_m)
    else:
        return v_m


def step_response(
    t: float,
    start_time: float,
    interval: float,
    max_input: float,
    max_length: float,
    min_length: float,
) -> float:
    """Calculate the value of the step response."""
    if t < start_time:
        return 0.0

    step = int((t - start_time) // interval)
    step_start_time = step * interval + start_time

    np.random.seed(seed=step + RANDOM_SEED_STEP_RESPONSE)

    if max_length > interval:
        print(f"warning: max_length = {max_length} > interval = {interval}")

    length = np.random.uniform(min_length, min(max_length, interval))
    input_u = np.random.uniform(-max_input, max_input)

    if (t - step_start_time) >= length:
        return 0.0

    return input_u


class driving_log_updater:
    """Class for updating logs when driving on the Python simulator."""

    def __init__(self):
        self.X_history = []
        self.U_history = []
        self.control_cmd_time_stamp_list = []
        self.control_cmd_steer_list = []
        self.control_cmd_acc_list = []
        self.kinematic_state_list = []
        self.acceleration_list = []
        self.steering_status_list = []
        self.control_cmd_orig_list = []
        self.operation_mode_list = []

    def update(self, t_current, x_current, u_current):
        """Update logs."""
        self.X_history.append(x_current)
        self.U_history.append(u_current)
        self.control_cmd_time_stamp_list.append(t_current)
        self.control_cmd_steer_list.append(u_current[1])
        self.control_cmd_acc_list.append(u_current[0])
        if self.control_cmd_time_stamp_list[-1] - self.control_cmd_time_stamp_list[0] > 3.0:
            self.control_cmd_time_stamp_list.pop(0)
            self.control_cmd_steer_list.pop(0)
            self.control_cmd_acc_list.pop(0)
        t_sec = int(t_current)
        t_n_sec = int(1e9 * (t_current - t_sec))
        kinematic_state = np.zeros(7)
        acceleration = np.zeros(4)
        steering_status = np.zeros(3)
        control_cmd_orig = np.zeros(17)
        operation_mode = np.zeros(3)
        kinematic_state[0] = t_sec
        kinematic_state[1] = t_n_sec
        kinematic_state[2] = x_current[0]
        kinematic_state[3] = x_current[1]
        kinematic_state[4] = np.sin(0.5 * x_current[3])
        kinematic_state[5] = np.cos(0.5 * x_current[3])
        kinematic_state[6] = x_current[2]
        self.kinematic_state_list.append(kinematic_state)
        acceleration[0] = t_sec
        acceleration[1] = t_n_sec
        acceleration[3] = x_current[4]
        self.acceleration_list.append(acceleration)
        steering_status[0] = t_sec
        steering_status[1] = t_n_sec
        steering_status[2] = x_current[5]
        self.steering_status_list.append(steering_status)
        control_cmd_orig[0] = t_sec
        control_cmd_orig[1] = t_n_sec
        control_cmd_orig[8] = u_current[1]
        control_cmd_orig[16] = u_current[0]
        self.control_cmd_orig_list.append(control_cmd_orig)
        operation_mode[0] = t_sec
        operation_mode[1] = t_n_sec
        operation_mode[2] = 2.0
        self.operation_mode_list.append(operation_mode)

    def save(self, save_dir):
        """Save logs in csv format."""
        kinematic_states = np.zeros((len(self.kinematic_state_list), 48))
        kinematic_states[:, [0, 1, 4, 5, 9, 10, 47]] = np.array(self.kinematic_state_list)
        np.savetxt(save_dir + "/kinematic_state.csv", kinematic_states, delimiter=",")
        np.savetxt(save_dir + "/acceleration.csv", np.array(self.acceleration_list), delimiter=",")
        np.savetxt(
            save_dir + "/steering_status.csv",
            np.array(self.steering_status_list),
            delimiter=",",
        )
        np.savetxt(
            save_dir + "/control_cmd_orig.csv",
            np.array(self.control_cmd_orig_list),
            delimiter=",",
        )
        np.savetxt(
            save_dir + "/system_operation_mode_state.csv",
            np.array(self.operation_mode_list),
            delimiter=",",
        )
        with open(save_dir + "/system_operation_mode_state.csv", "w") as f:
            writer = csv.writer(f)
            for i in range(len(self.operation_mode_list)):
                operation_mode_plus_true = self.operation_mode_list[i].tolist()
                operation_mode_plus_true.append("True")
                writer.writerow(operation_mode_plus_true)
