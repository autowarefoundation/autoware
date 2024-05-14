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

import numpy as np
import python_simulator
from smart_mpc_trajectory_follower.training_and_data_check import train_drive_NN_model

initial_error = np.array(
    [0.001, 0.03, 0.01, -0.001, 0, 2 * python_simulator.measurement_steer_bias]
)

save_dir = "test_python_sim"
python_simulator.slalom_drive(save_dir=save_dir, initial_error=initial_error)

model_trainer = train_drive_NN_model.train_drive_NN_model()
model_trainer.add_data_from_csv(save_dir)
model_trainer.save_train_data(save_dir)
model_trainer.get_trained_model()
model_trainer.save_models(save_dir=save_dir)

save_dir = "test_python_trained_sim"
load_dir = "test_python_sim"
python_simulator.slalom_drive(
    save_dir=save_dir, load_dir=load_dir, use_trained_model=True, initial_error=initial_error
)
