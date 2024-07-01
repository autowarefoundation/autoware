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

import sys

from assets import ControlType
from autoware_smart_mpc_trajectory_follower.training_and_data_check import train_drive_NN_model
import numpy as np
import python_simulator

initial_error = np.array(
    [0.001, 0.03, 0.01, -0.001, 0, 2 * python_simulator.measurement_steer_bias]
)


if len(sys.argv) > 1:
    if sys.argv[1] == "nominal_test":
        save_dir = "test_python_nominal_sim"
        python_simulator.drive_sim(save_dir=save_dir, initial_error=initial_error)
    else:
        print("Invalid argument")
        sys.exit(1)
else:
    train_dir = "test_python_pure_pursuit_train"
    val_dir = "test_python_pure_pursuit_val"

    python_simulator.drive_sim(
        seed=0,
        t_range=[0, 900.0],
        control_type=ControlType.pp_eight,
        save_dir=train_dir,
        initial_error=initial_error,
    )
    python_simulator.drive_sim(
        seed=1,
        t_range=[0, 900.0],
        control_type=ControlType.pp_eight,
        save_dir=val_dir,
        initial_error=initial_error,
    )

    model_trainer = train_drive_NN_model.train_drive_NN_model()
    model_trainer.add_data_from_csv(train_dir, add_mode="as_train")
    model_trainer.add_data_from_csv(val_dir, add_mode="as_val")
    model_trainer.get_trained_model(use_polynomial_reg=True)
    model_trainer.save_models(save_dir=train_dir)

    save_dir = "test_python_trained_sim"
    python_simulator.drive_sim(
        save_dir=save_dir, load_dir=train_dir, use_trained_model=True, initial_error=initial_error
    )
