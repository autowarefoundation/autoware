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
import argparse
import datetime
from importlib import reload as ir
import json
import os
import time
import traceback
from typing import Dict

from assets import ChangeParam  # type: ignore
from assets import ControlType
from assets import DataCollectionMode
from assets import DirGenerator
from autoware_smart_mpc_trajectory_follower.training_and_data_check import train_drive_NN_model
import numpy as np
import python_simulator  # type: ignore

USE_TRAINED_MODEL_DIFF = True
DATA_COLLECTION_MODE = DataCollectionMode.pp
CONTROL_TYPE_TO_SKIP = [ControlType.pp_straight]
USE_POLYNOMIAL_REGRESSION = True
USE_SELECTED_POLYNOMIAL = True
FORCE_NN_MODEL_TO_ZERO = False


FIT_INTERCEPT = True  # Should be True if FORCE_NN_MODEL_TO_ZERO is True
USE_INTERCEPT = False  # Should be True if FORCE_NN_MODEL_TO_ZERO is True


def run_simulator(
    change_param: ChangeParam,
    root: str = ".",
    batch_size=5,
    patience_1=10,
    patience_2=10,
    acc_amp_range=0.05,
    acc_period_range=[5.0, 20.0],
    hidden_layer_sizes=(16, 16),
    hidden_layer_lstm=8,
    steer_amp_range=0.05,
    steer_period_range=[5.0, 30.0],
    acc_max=1.2,
    constant_vel_time=5.0,
    split_size=5,
    step_response_max_input=0.01,
    step_response_max_length=1.5,
    step_response_start_time_ratio=0.0,
    step_response_interval=5.0,
    step_response_min_length=0.5,
    use_memory_diff=True,  # False,
    skip_data_collection=False,
    smoothing_trajectory_data_flag=True,
):
    """Run the simulator."""
    # initialize parameters
    print("reset sim_setting_json")
    data: Dict[str, float] = {}
    with open("sim_setting.json", "w") as f:
        json.dump(data, f)

    param_val_list = change_param.value
    start_time = time.time()

    dir_generator = DirGenerator(root=root)

    for j in range(len(param_val_list)):
        i = j + 0
        with open("sim_setting.json", "r") as f:
            data = json.load(f)

        data[change_param.name] = param_val_list[j]
        with open("sim_setting.json", "w") as f:
            json.dump(data, f)
        ir(python_simulator)
        training_data_dirs = []
        val_data_dirs = []

        try:
            initial_error = np.array(
                [0.001, 0.03, 0.01, 0.0, 0, python_simulator.measurement_steer_bias]
            )
            if DATA_COLLECTION_MODE in [
                DataCollectionMode.ff,
                DataCollectionMode.pp,
                DataCollectionMode.npp,
            ]:
                if FORCE_NN_MODEL_TO_ZERO:
                    model_trainer = train_drive_NN_model.train_drive_NN_model()
                else:
                    model_trainer = train_drive_NN_model.train_drive_NN_model(
                        alpha_1_for_polynomial_regression=0.1**5,
                        alpha_2_for_polynomial_regression=0.1**5,
                    )

                for control_type in DATA_COLLECTION_MODE.toControlTypes():
                    if control_type in CONTROL_TYPE_TO_SKIP:
                        continue
                    add_mode = ["as_val", "as_train"]
                    y_length = 60.0
                    x_length = 120.0
                    initial_error_diff = np.zeros(6)
                    if control_type == ControlType.pp_eight:
                        initial_error_diff[0] = -(x_length - y_length) / 4.0
                        initial_error_diff[1] = -y_length / 4.0
                    else:
                        initial_error_diff[1] = -0.05
                    for k in range(2):
                        save_dir = dir_generator.test_dir_name(
                            control_type=control_type,
                            change_param=change_param,
                            index=i,
                            validation_flag=(1 - k),
                        )
                        if not skip_data_collection:
                            python_simulator.drive_sim(
                                save_dir=save_dir,
                                control_type=control_type,
                                seed=k + 1,
                                t_range=[0, 900.0],
                                acc_amp_range=acc_amp_range,
                                acc_period_range=acc_period_range,
                                steer_amp_range=steer_amp_range,
                                steer_period_range=steer_period_range,
                                large_steer_amp_range=0.0,
                                large_steer_period_range=[10.0, 20.0],
                                start_large_steer_time=150.0,
                                acc_max=acc_max,
                                constant_vel_time=constant_vel_time,
                                split_size=split_size,
                                y_length=(0.9 ** (1 - k)) * y_length,
                                x_length=(0.9 ** (1 - k)) * x_length,
                                step_response_max_input=step_response_max_input,
                                step_response_max_length=step_response_max_length,
                                step_response_start_time_ratio=step_response_start_time_ratio,
                                step_response_interval=step_response_interval,
                                step_response_min_length=step_response_min_length,
                                initial_error=initial_error + (1 - k) * initial_error_diff,
                                smoothing_trajectory_data_flag=smoothing_trajectory_data_flag,
                            )
                            model_trainer.add_data_from_csv(save_dir, add_mode=add_mode[k])
                        if k == 0:
                            val_data_dirs.append(save_dir)
                        else:
                            training_data_dirs.append(save_dir)

                if not skip_data_collection:
                    start_time_learning = time.time()
                    model_trainer.get_trained_model(
                        use_polynomial_reg=USE_POLYNOMIAL_REGRESSION,
                        use_selected_polynomial=USE_SELECTED_POLYNOMIAL,
                        force_NN_model_to_zero=FORCE_NN_MODEL_TO_ZERO,
                        fit_intercept=FIT_INTERCEPT,
                        use_intercept=USE_INTERCEPT,
                        hidden_layer_sizes=hidden_layer_sizes,
                        hidden_layer_lstm=hidden_layer_lstm,
                        batch_size=batch_size,
                        patience=patience_1,
                    )
                    learning_computation_time = time.time() - start_time_learning
                    model_trainer.plot_trained_result(save_dir=save_dir)
                    model_trainer.save_models(save_dir=save_dir)
                    print("learning_computation_time:", learning_computation_time)
                    f = open(
                        save_dir
                        + f"/computational_time_learning_from_{DATA_COLLECTION_MODE}_data.txt",
                        "w",
                    )
                    f.write(str(learning_computation_time))
                    f.close()
                load_dir = save_dir
                save_dir = dir_generator.test_dir_name(
                    data_collection_mode=DATA_COLLECTION_MODE,
                    change_param=change_param,
                    index=i,
                    use_memory_diff=use_memory_diff,
                )
                auto_test_performance_result_list = python_simulator.drive_sim(
                    load_dir=load_dir,
                    save_dir=save_dir,
                    use_trained_model=True,
                    use_trained_model_diff=USE_TRAINED_MODEL_DIFF,
                    initial_error=initial_error,
                    use_memory_diff=use_memory_diff,
                )
                training_data_dirs.append(save_dir)

                f = open(
                    f"auto_test_result_intermediate_model_control_trained_with_data_collected_by_{control_type}_control.csv",
                    mode="a",
                )
            else:
                save_dir = dir_generator.test_dir_name(
                    change_param=change_param,
                    index=i,
                )
                auto_test_performance_result_list = python_simulator.drive_sim(
                    save_dir=save_dir,
                    initial_error=initial_error,
                )
                f = open("auto_test_result_nominal_model_control.csv", mode="a")

            print(
                change_param.name,
                str(param_val_list[j]).replace(",", "_"),
                *auto_test_performance_result_list,
                sep=",",
                file=f,
            )
            f.close()

            print(i, "auto_test_performance_result_list", auto_test_performance_result_list)

            skip_learning_for_developing_testcase = False
            if not skip_learning_for_developing_testcase:
                ir(train_drive_NN_model)
                model_trainer = train_drive_NN_model.train_drive_NN_model()
                learning_computation_time = None  # type: ignore
                if DATA_COLLECTION_MODE in [
                    DataCollectionMode.ff,
                    DataCollectionMode.pp,
                    DataCollectionMode.npp,
                ]:
                    for dir_name in val_data_dirs:
                        model_trainer.add_data_from_csv(dir_name, add_mode="as_val")
                    for dir_name in training_data_dirs:
                        model_trainer.add_data_from_csv(dir_name, add_mode="as_train")
                    start_time_learning = time.time()
                    model_trainer.update_saved_trained_model(
                        path=load_dir + "/model_for_test_drive.pth",
                        use_polynomial_reg=USE_POLYNOMIAL_REGRESSION,
                        use_selected_polynomial=USE_SELECTED_POLYNOMIAL,
                        force_NN_model_to_zero=FORCE_NN_MODEL_TO_ZERO,
                        fit_intercept=FIT_INTERCEPT,
                        use_intercept=USE_INTERCEPT,
                        batch_size=batch_size,
                        patience=patience_2,
                    )
                    learning_computation_time = time.time() - start_time_learning
                    model_trainer.plot_trained_result(save_dir=save_dir)
                    model_trainer.save_models(save_dir=save_dir)
                    load_dir = dir_generator.test_dir_name(
                        data_collection_mode=DATA_COLLECTION_MODE,
                        change_param=change_param,
                        index=i,
                        use_memory_diff=use_memory_diff,
                    )
                    save_dir = dir_generator.test_dir_name(
                        data_collection_mode=DATA_COLLECTION_MODE,
                        trained=True,
                        change_param=change_param,
                        index=i,
                        use_memory_diff=use_memory_diff,
                    )
                else:
                    model_trainer.add_data_from_csv(save_dir)
                    start_time_learning = time.time()
                    model_trainer.get_trained_model(
                        use_polynomial_reg=USE_POLYNOMIAL_REGRESSION,
                        use_selected_polynomial=USE_SELECTED_POLYNOMIAL,
                        force_NN_model_to_zero=FORCE_NN_MODEL_TO_ZERO,
                        fit_intercept=FIT_INTERCEPT,
                        use_intercept=USE_INTERCEPT,
                        hidden_layer_sizes=hidden_layer_sizes,
                        hidden_layer_lstm=hidden_layer_lstm,
                        batch_size=batch_size,
                        patience=patience_1,
                    )
                    learning_computation_time = time.time() - start_time_learning
                    model_trainer.plot_trained_result(save_dir=save_dir)
                    model_trainer.save_models(save_dir=save_dir)

                    load_dir = dir_generator.test_dir_name(
                        change_param=change_param,
                        index=i,
                        use_memory_diff=use_memory_diff,
                    )
                    save_dir = dir_generator.test_dir_name(
                        trained=True,
                        change_param=change_param,
                        index=i,
                        use_memory_diff=use_memory_diff,
                    )
                auto_test_performance_result_list = python_simulator.drive_sim(
                    load_dir=load_dir,
                    use_trained_model=True,
                    use_trained_model_diff=USE_TRAINED_MODEL_DIFF,
                    save_dir=save_dir,
                    initial_error=initial_error,
                    use_memory_diff=use_memory_diff,
                )
                # cSpell:ignore oneline
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

                f = open(
                    f"auto_test_result_final_model_control_trained_with_data_collected_by_{DATA_COLLECTION_MODE}_control.csv",
                    mode="a",
                )
                print(
                    change_param.name,
                    str(param_val_list[j]).replace(",", "_"),
                    *auto_test_performance_result_list,
                    sep=",",
                    file=f,
                )
                f.close()
            print("experiment success")
        except Exception:
            print("# Catch Exception #")
            traceback.print_exc()
            auto_test_performance_result_list = [1e16] * 11
            f = open(
                f"auto_test_result_final_model_control_trained_with_data_collected_by_{DATA_COLLECTION_MODE}_control.csv",
                mode="a",
            )
            print(
                change_param.name,
                str(param_val_list[j]).replace(",", "_"),
                *auto_test_performance_result_list,
                sep=",",
                file=f,
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--param_name", default=None)
    parser.add_argument("--root", default=".")
    args = parser.parse_args()

    if args.root == "time":
        args.root = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.root = "test_run_sim_" + args.root

    if args.param_name is None:
        print("Do you want to run the simulation for all parameters at once?")
        if yes_no_input():
            for _, change_param in ChangeParam.__members__.items():
                run_simulator(change_param, root=args.root)
        else:
            print("Enter the name of the parameter for which the simulation is to be run.")
            input_string = input()
            for name, change_param in ChangeParam.__members__.items():
                if name == input_string:
                    run_simulator(change_param, root=args.root)
                    break
    else:
        for name, change_param in ChangeParam.__members__.items():
            if name == args.param_name:
                run_simulator(change_param, root=args.root)
                break
