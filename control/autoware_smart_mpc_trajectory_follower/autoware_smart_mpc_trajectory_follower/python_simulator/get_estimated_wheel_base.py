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
import numpy as np


def get_estimated_wheel_base_coef(dir_name: str) -> float:
    A = np.load(dir_name + "/polynomial_reg_info.npz")["A"]
    estimated_wheel_base = 1 / (1 / drive_functions.L + A[3, 11])
    print("estimated wheel base:", estimated_wheel_base)
    return estimated_wheel_base / drive_functions.L
