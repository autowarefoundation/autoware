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

import subprocess

if __name__ == "__main__":
    params = [
        "steer_bias",
        "steer_dead_band",
        "wheel_base",
        "acc_time_delay",
        "steer_time_delay",
        "acc_time_constant",
        "steer_time_constant",
        "accel_map_scale",
        "acc_scaling",
        "steer_scaling",
        "vehicle_type",
    ]
    for param in params:
        str_run = "python3 run_sim.py " + f" --param_name {param}" + " --root auto_test"
        subprocess.run(str_run, shell=True)
