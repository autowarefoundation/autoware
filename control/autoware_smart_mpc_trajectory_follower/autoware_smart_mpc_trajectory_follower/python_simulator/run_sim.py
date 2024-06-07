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

# cspell: ignore oneline

import argparse
from enum import Enum
from importlib import reload as ir
import json
import os
import time
import traceback
from typing import Dict

from autoware_smart_mpc_trajectory_follower.training_and_data_check import train_drive_NN_model
import numpy as np
import python_simulator

parser = argparse.ArgumentParser()
parser.add_argument("--param_name", default=None)
args = parser.parse_args()

P1 = 0.8
P2 = 1.2

USE_TRAINED_MODEL_DIFF = True
DATA_COLLECTION_MODE = "pp"  # option: "pp": pure_pursuit, "ff": feed_forward, "mpc": smart_mpc
USE_POLYNOMIAL_REGRESSION = True
USE_SELECTED_POLYNOMIAL = True
FORCE_NN_MODEL_TO_ZERO = False


FIT_INTERCEPT = True  # Should be True if FORCE_NN_MODEL_TO_ZERO is True
USE_INTERCEPT = False  # Should be True if FORCE_NN_MODEL_TO_ZERO is True


class ChangeParam(Enum):
    """Parameters to be changed when running the simulation."""

    steer_bias = [
        -1.0 * np.pi / 180.0,
        -0.8 * np.pi / 180.0,
        -0.6 * np.pi / 180.0,
        -0.4 * np.pi / 180.0,
        -0.2 * np.pi / 180.0,
        0.0,
        0.2 * np.pi / 180.0,
        0.4 * np.pi / 180.0,
        0.6 * np.pi / 180.0,
        0.8 * np.pi / 180.0,
        1.0 * np.pi / 180.0,
    ]
    """steer midpoint (soft + hard)"""

    steer_rate_lim = [0.020, 0.050, 0.100, 0.150, 0.200, 0.300, 0.400, 0.500]
    """Maximum steer angular velocity"""

    vel_rate_lim = [0.5, 1.0, 3.0, 5.0, 7.0, 9.0]
    """Maximum acceleration/deceleration"""

    wheel_base = [0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 7.0]
    """wheel base"""

    steer_dead_band = [0.0000, 0.0012, 0.0025, 0.0050, 0.01]
    """steer dead band"""

    adaptive_gear_ratio_coef = [
        [15.713, 0.053, 0.042, 15.713, 0.053, 0.042],
        [15.713, 0.053, 0.042, P1 * 15.713, 0.053, 0.042],
        [15.713, 0.053, 0.042, P2 * 15.713, 0.053, 0.042],
        [15.713, 0.053, 0.042, 15.713, P1 * 0.053, 0.042],
        [15.713, 0.053, 0.042, P1 * 15.713, P1 * 0.053, 0.042],
        [15.713, 0.053, 0.042, P2 * 15.713, P1 * 0.053, 0.042],
        [15.713, 0.053, 0.042, 15.713, P2 * 0.053, 0.042],
        [15.713, 0.053, 0.042, P1 * 15.713, P2 * 0.053, 0.042],
        [15.713, 0.053, 0.042, P2 * 15.713, P2 * 0.053, 0.042],
        [15.713, 0.053, 0.042, 15.713, 0.053, P1 * 0.042],
        [15.713, 0.053, 0.042, P1 * 15.713, 0.053, P1 * 0.042],
        [15.713, 0.053, 0.042, P2 * 15.713, 0.053, P1 * 0.042],
        [15.713, 0.053, 0.042, 15.713, P1 * 0.053, P1 * 0.042],
        [15.713, 0.053, 0.042, P1 * 15.713, P1 * 0.053, P1 * 0.042],
        [15.713, 0.053, 0.042, P2 * 15.713, P1 * 0.053, P1 * 0.042],
        [15.713, 0.053, 0.042, 15.713, P2 * 0.053, P1 * 0.042],
        [15.713, 0.053, 0.042, P1 * 15.713, P2 * 0.053, P1 * 0.042],
        [15.713, 0.053, 0.042, P2 * 15.713, P2 * 0.053, P1 * 0.042],
        [15.713, 0.053, 0.042, 15.713, 0.053, P2 * 0.042],
        [15.713, 0.053, 0.042, P1 * 15.713, 0.053, P2 * 0.042],
        [15.713, 0.053, 0.042, P2 * 15.713, 0.053, P2 * 0.042],
        [15.713, 0.053, 0.042, 15.713, P1 * 0.053, P2 * 0.042],
        [15.713, 0.053, 0.042, P1 * 15.713, P1 * 0.053, P2 * 0.042],
        [15.713, 0.053, 0.042, P2 * 15.713, P1 * 0.053, P2 * 0.042],
        [15.713, 0.053, 0.042, 15.713, P2 * 0.053, P2 * 0.042],
        [15.713, 0.053, 0.042, P1 * 15.713, P2 * 0.053, P2 * 0.042],
        [15.713, 0.053, 0.042, P2 * 15.713, P2 * 0.053, P2 * 0.042],
    ]
    """velocity-dependent gear ratio"""

    acc_time_delay = [0.00, 0.1, 0.27, 0.40, 0.60, 0.80, 1.01]
    """acc time delay"""

    steer_time_delay = [0.00, 0.1, 0.27, 0.40, 0.60, 0.80, 1.02]
    """steer time delay"""

    acc_time_constant = [0.01, 0.1, 0.20, 0.24, 0.40, 0.60, 0.80, 1.01]
    """time constant"""

    steer_time_constant = [0.01, 0.1, 0.20, 0.24, 0.40, 0.60, 0.80, 1.02]
    """time constant"""

    accel_map_scale = [0.2, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0]
    """pedal - real acceleration correspondence"""

    acc_scaling = [0.2, 0.5, 1.0, 2.0, 3.0, 4.0, 5.01]
    """Acceleration scaling coefficient"""

    steer_scaling = [0.2, 0.5, 1.0, 2.0, 3.0, 4.0, 5.02]
    """Steer scaling coefficient"""

    vehicle_type = [1, 2, 3, 4]
    """change to other vehicle parameters"""


def run_simulator(change_param: ChangeParam):
    """Run the simulator."""
    # initialize parameters
    print("reset sim_setting_json")
    data: Dict[str, float] = {}
    with open("sim_setting.json", "w") as f:
        json.dump(data, f)

    param_val_list = change_param.value
    start_time = time.time()

    for j in range(len(param_val_list)):
        i = j + 0
        with open("sim_setting.json", "r") as f:
            data = json.load(f)

        data[change_param.name] = param_val_list[j]
        with open("sim_setting.json", "w") as f:
            json.dump(data, f)
        ir(python_simulator)

        try:
            initial_error = np.array(
                [0.001, 0.03, 0.01, -0.001, 0, 2 * python_simulator.measurement_steer_bias]
            )
            if DATA_COLLECTION_MODE in ["ff", "pp"]:
                if DATA_COLLECTION_MODE == "ff":
                    save_dir = "test_feedforward_sim" + change_param.name + str(i)
                else:
                    save_dir = "test_pure_pursuit_sim" + change_param.name + str(i)
                python_simulator.slalom_drive(
                    save_dir=save_dir,
                    control_type=DATA_COLLECTION_MODE,
                    t_range=[0, 200.0],
                    acc_width_range=0.005,
                    acc_period_range=[5.0, 20.0],
                    steer_width_range=0.005,
                    steer_period_range=[5.0, 20.0],
                    large_steer_width_range=0.05,
                    large_steer_period_range=[10.0, 20.0],
                    start_large_steer_time=150.0,
                )
                if FORCE_NN_MODEL_TO_ZERO:
                    model_trainer = train_drive_NN_model.train_drive_NN_model()
                else:
                    model_trainer = train_drive_NN_model.train_drive_NN_model(
                        alpha_1_for_polynomial_regression=0.1**5,
                        alpha_2_for_polynomial_regression=0.1**5,
                    )

                model_trainer.add_data_from_csv(save_dir)
                start_time_learning = time.time()
                model_trainer.get_trained_model(
                    use_polynomial_reg=USE_POLYNOMIAL_REGRESSION,
                    use_selected_polynomial=USE_SELECTED_POLYNOMIAL,
                    force_NN_model_to_zero=FORCE_NN_MODEL_TO_ZERO,
                    fit_intercept=FIT_INTERCEPT,
                    use_intercept=USE_INTERCEPT,
                )
                learning_computation_time = time.time() - start_time_learning
                model_trainer.plot_trained_result(save_dir=save_dir)
                model_trainer.save_models(save_dir=save_dir)
                load_dir = save_dir
                if DATA_COLLECTION_MODE == "ff":
                    save_dir = "test_python_ff_aided_sim_" + change_param.name + str(i)
                elif DATA_COLLECTION_MODE == "pp":
                    save_dir = "test_python_pp_aided_sim_" + change_param.name + str(i)
                auto_test_performance_result_list = python_simulator.slalom_drive(
                    load_dir=load_dir,
                    save_dir=save_dir,
                    use_trained_model=True,
                    use_trained_model_diff=USE_TRAINED_MODEL_DIFF,
                    initial_error=initial_error,
                )

                print("learning_computation_time:", learning_computation_time)
                if DATA_COLLECTION_MODE == "ff":
                    f = open(save_dir + "/computational_time_learning_from_ff_data.txt", "w")
                    f.write(str(learning_computation_time))
                    f.close()
                    f = open(
                        "auto_test_result_intermediate_model_control_trained_with_data_collected_by_ff_control.csv",
                        mode="a",
                    )
                elif DATA_COLLECTION_MODE == "pp":
                    f = open(save_dir + "/computational_time_learning_from_pp_data.txt", "w")
                    f.write(str(learning_computation_time))
                    f.close()
                    f = open(
                        "auto_test_result_intermediate_model_control_trained_with_data_collected_by_pp_control.csv",
                        mode="a",
                    )
            else:
                save_dir = "test_python_sim_" + change_param.name + str(i)
                auto_test_performance_result_list = python_simulator.slalom_drive(
                    save_dir=save_dir,
                    initial_error=initial_error,
                )
                f = open("auto_test_result_nominal_model_control.csv", mode="a")

            print(
                change_param.name,
                str(param_val_list[j]).replace(",", "_"),
                *auto_test_performance_result_list,
                sep=",",
                file=f
            )
            f.close()

            print(i, "auto_test_performance_result_list", auto_test_performance_result_list)

            skip_learning_for_developing_testcase = False
            if not skip_learning_for_developing_testcase:
                ir(train_drive_NN_model)
                model_trainer = train_drive_NN_model.train_drive_NN_model()
                learning_computation_time = None
                if DATA_COLLECTION_MODE in ["ff", "pp"]:
                    model_trainer.add_data_from_csv(load_dir)
                    model_trainer.add_data_from_csv(save_dir)
                    start_time_learning = time.time()
                    model_trainer.update_saved_trained_model(
                        path=load_dir + "/model_for_test_drive.pth",
                        use_polynomial_reg=USE_POLYNOMIAL_REGRESSION,
                        use_selected_polynomial=USE_SELECTED_POLYNOMIAL,
                        force_NN_model_to_zero=FORCE_NN_MODEL_TO_ZERO,
                        fit_intercept=FIT_INTERCEPT,
                        use_intercept=USE_INTERCEPT,
                    )
                    learning_computation_time = time.time() - start_time_learning
                    model_trainer.plot_trained_result(save_dir=save_dir)
                    model_trainer.save_models(save_dir=save_dir)
                    if DATA_COLLECTION_MODE == "ff":
                        load_dir = "test_python_ff_aided_sim_" + change_param.name + str(i)
                        save_dir = "test_python_ff_aided_sim_trained_" + change_param.name + str(i)
                    elif DATA_COLLECTION_MODE == "pp":
                        load_dir = "test_python_pp_aided_sim_" + change_param.name + str(i)
                        save_dir = "test_python_pp_aided_sim_trained_" + change_param.name + str(i)
                else:
                    model_trainer.add_data_from_csv(save_dir)
                    start_time_learning = time.time()
                    model_trainer.get_trained_model(
                        use_polynomial_reg=USE_POLYNOMIAL_REGRESSION,
                        use_selected_polynomial=USE_SELECTED_POLYNOMIAL,
                        force_NN_model_to_zero=FORCE_NN_MODEL_TO_ZERO,
                        fit_intercept=FIT_INTERCEPT,
                        use_intercept=USE_INTERCEPT,
                    )
                    learning_computation_time = time.time() - start_time_learning
                    model_trainer.plot_trained_result(save_dir=save_dir)
                    model_trainer.save_models(save_dir=save_dir)

                    load_dir = "test_python_sim_" + change_param.name + str(i)
                    save_dir = "test_python_sim_trained_" + change_param.name + str(i)
                auto_test_performance_result_list = python_simulator.slalom_drive(
                    load_dir=load_dir,
                    use_trained_model=True,
                    use_trained_model_diff=USE_TRAINED_MODEL_DIFF,
                    save_dir=save_dir,
                    initial_error=initial_error,
                )
                print("learning_computation_time: ", learning_computation_time)
                f = open(save_dir + "/test_info.txt", "w")
                f.write("commit id: " + str(os.popen("git log --oneline -1").read()) + "\n")
                f.write(
                    "computational_time_learning_from_total_data: "
                    + str(learning_computation_time)
                    + "\n"
                )
                f.write("USE_TRAINED_MODEL_DIFF: " + str(USE_TRAINED_MODEL_DIFF) + "\n")
                f.write("USE_POLYNOMIAL_REGRESSION: " + str(USE_POLYNOMIAL_REGRESSION) + "\n")
                f.write("USE_SELECTED_POLYNOMIAL: " + str(USE_SELECTED_POLYNOMIAL) + "\n")
                f.write("FORCE_NN_MODEL_TO_ZERO: " + str(FORCE_NN_MODEL_TO_ZERO) + "\n")
                f.write("FIT_INTERCEPT: " + str(FIT_INTERCEPT) + "\n")
                f.write("USE_INTERCEPT: " + str(USE_INTERCEPT) + "\n")
                f.close()

                if DATA_COLLECTION_MODE == "ff":
                    f = open(
                        "auto_test_result_final_model_control_trained_with_data_collected_by_ff_control.csv",
                        mode="a",
                    )
                elif DATA_COLLECTION_MODE == "pp":
                    f = open(
                        "auto_test_result_final_model_control_trained_with_data_collected_by_pp_control.csv",
                        mode="a",
                    )
                else:
                    f = open(
                        "auto_test_result_final_model_control_trained_with_data_collected_by_nominal_control.csv",
                        mode="a",
                    )
                print(
                    change_param.name,
                    str(param_val_list[j]).replace(",", "_"),
                    *auto_test_performance_result_list,
                    sep=",",
                    file=f
                )
                f.close()
            print("experiment success")
        except Exception:
            print("# Catch Exception #")
            print(traceback.print_exc())
            auto_test_performance_result_list = [1e16] * 11
            if DATA_COLLECTION_MODE == "ff":
                f = open(
                    "auto_test_result_final_model_control_trained_with_data_collected_by_ff_control.csv",
                    mode="a",
                )
            elif DATA_COLLECTION_MODE == "pp":
                f = open(
                    "auto_test_result_final_model_control_trained_with_data_collected_by_pp_control.csv",
                    mode="a",
                )
            else:
                f = open(
                    "auto_test_result_final_model_control_trained_with_data_collected_by_nominal_control.csv",
                    mode="a",
                )
            print(
                change_param.name,
                str(param_val_list[j]).replace(",", "_"),
                *auto_test_performance_result_list,
                sep=",",
                file=f
            )
            f.close()
            print("experiment failure")
        print("data:", data)

    print("total_time: ", time.time() - start_time)


def yes_no_input():
    """Let the user enter yes or no. Default is no."""
    choice = input("Please respond with 'yes' or 'no' [y/N]: ").lower()
    if choice in ["y", "ye", "yes"]:
        return True
    return False


if __name__ == "__main__":
    if args.param_name is None:
        print("Do you want to run the simulation for all parameters at once?")
        if yes_no_input():
            for _, change_param in ChangeParam.__members__.items():
                run_simulator(change_param)
        else:
            print("Enter the name of the parameter for which the simulation is to be run.")
            input_string = input()
            for name, change_param in ChangeParam.__members__.items():
                if name == input_string:
                    run_simulator(change_param)
                    break
    else:
        for name, change_param in ChangeParam.__members__.items():
            if name == args.param_name:
                run_simulator(change_param)
                break
