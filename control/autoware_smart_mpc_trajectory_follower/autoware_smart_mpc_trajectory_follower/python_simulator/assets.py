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
from enum import Enum
import os

import numpy as np

CHANGE_PARAM_P1 = 0.8
CHANGE_PARAM_P2 = 1.2


class ControlType(Enum):
    ff = "feed_forward"
    pp_eight = "pure_pursuit_figure_eight"
    pp_straight = "pure_pursuit_straight"
    npp_eight = "naive_pure_pursuit_figure_eight"
    mpc = "smart_mpc"

    def __str__(self) -> str:
        return self.name


class DataCollectionMode(Enum):
    ff = "feed_forward"
    pp = "pure_pursuit"
    npp = "naive_pure_pursuit"
    mpc = "nominal_mpc"

    def __str__(self) -> str:
        return self.name

    def toControlTypes(self) -> list[ControlType]:
        if self.name == "pp":
            return [ControlType.pp_eight, ControlType.pp_straight]
        elif self.name == "npp":
            return [ControlType.npp_eight]
        elif self.name == "ff":
            return [ControlType.ff]
        elif self.name == "mpc":
            return [ControlType.mpc]
        else:
            print(f"Error: unexpected DataCollectionMode: {self}")
            raise Exception


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
        [15.713, 0.053, 0.042, CHANGE_PARAM_P1 * 15.713, 0.053, 0.042],
        [15.713, 0.053, 0.042, CHANGE_PARAM_P2 * 15.713, 0.053, 0.042],
        [15.713, 0.053, 0.042, 15.713, CHANGE_PARAM_P1 * 0.053, 0.042],
        [15.713, 0.053, 0.042, CHANGE_PARAM_P1 * 15.713, CHANGE_PARAM_P1 * 0.053, 0.042],
        [15.713, 0.053, 0.042, CHANGE_PARAM_P2 * 15.713, CHANGE_PARAM_P1 * 0.053, 0.042],
        [15.713, 0.053, 0.042, 15.713, CHANGE_PARAM_P2 * 0.053, 0.042],
        [15.713, 0.053, 0.042, CHANGE_PARAM_P1 * 15.713, CHANGE_PARAM_P2 * 0.053, 0.042],
        [15.713, 0.053, 0.042, CHANGE_PARAM_P2 * 15.713, CHANGE_PARAM_P2 * 0.053, 0.042],
        [15.713, 0.053, 0.042, 15.713, 0.053, CHANGE_PARAM_P1 * 0.042],
        [15.713, 0.053, 0.042, CHANGE_PARAM_P1 * 15.713, 0.053, CHANGE_PARAM_P1 * 0.042],
        [15.713, 0.053, 0.042, CHANGE_PARAM_P2 * 15.713, 0.053, CHANGE_PARAM_P1 * 0.042],
        [15.713, 0.053, 0.042, 15.713, CHANGE_PARAM_P1 * 0.053, CHANGE_PARAM_P1 * 0.042],
        [
            15.713,
            0.053,
            0.042,
            CHANGE_PARAM_P1 * 15.713,
            CHANGE_PARAM_P1 * 0.053,
            CHANGE_PARAM_P1 * 0.042,
        ],
        [
            15.713,
            0.053,
            0.042,
            CHANGE_PARAM_P2 * 15.713,
            CHANGE_PARAM_P1 * 0.053,
            CHANGE_PARAM_P1 * 0.042,
        ],
        [15.713, 0.053, 0.042, 15.713, CHANGE_PARAM_P2 * 0.053, CHANGE_PARAM_P1 * 0.042],
        [
            15.713,
            0.053,
            0.042,
            CHANGE_PARAM_P1 * 15.713,
            CHANGE_PARAM_P2 * 0.053,
            CHANGE_PARAM_P1 * 0.042,
        ],
        [
            15.713,
            0.053,
            0.042,
            CHANGE_PARAM_P2 * 15.713,
            CHANGE_PARAM_P2 * 0.053,
            CHANGE_PARAM_P1 * 0.042,
        ],
        [15.713, 0.053, 0.042, 15.713, 0.053, CHANGE_PARAM_P2 * 0.042],
        [15.713, 0.053, 0.042, CHANGE_PARAM_P1 * 15.713, 0.053, CHANGE_PARAM_P2 * 0.042],
        [15.713, 0.053, 0.042, CHANGE_PARAM_P2 * 15.713, 0.053, CHANGE_PARAM_P2 * 0.042],
        [15.713, 0.053, 0.042, 15.713, CHANGE_PARAM_P1 * 0.053, CHANGE_PARAM_P2 * 0.042],
        [
            15.713,
            0.053,
            0.042,
            CHANGE_PARAM_P1 * 15.713,
            CHANGE_PARAM_P1 * 0.053,
            CHANGE_PARAM_P2 * 0.042,
        ],
        [
            15.713,
            0.053,
            0.042,
            CHANGE_PARAM_P2 * 15.713,
            CHANGE_PARAM_P1 * 0.053,
            CHANGE_PARAM_P2 * 0.042,
        ],
        [15.713, 0.053, 0.042, 15.713, CHANGE_PARAM_P2 * 0.053, CHANGE_PARAM_P2 * 0.042],
        [
            15.713,
            0.053,
            0.042,
            CHANGE_PARAM_P1 * 15.713,
            CHANGE_PARAM_P2 * 0.053,
            CHANGE_PARAM_P2 * 0.042,
        ],
        [
            15.713,
            0.053,
            0.042,
            CHANGE_PARAM_P2 * 15.713,
            CHANGE_PARAM_P2 * 0.053,
            CHANGE_PARAM_P2 * 0.042,
        ],
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

    vehicle_type = [0, 1, 2, 3, 4]
    """change to other vehicle parameters"""

    test_vehicle = [0, 1, 2, 3, 4, 5, 6, 7]


def test_dir_name(
    root: str = ".",
    data_collection_mode: DataCollectionMode | None = None,
    control_type: ControlType | None = None,
    trained: bool = False,
    change_param: ChangeParam | None = None,
    index: int | None = None,
    validation_flag: bool = False,
    use_memory_diff: bool = False,
) -> str:
    """Generate string for directory name."""
    dir_name = root + "/test"
    if control_type is not None:
        dir_name += f"_{control_type.value}_sim"
    elif data_collection_mode is not None:
        dir_name += f"_{data_collection_mode}_aided_sim"
    else:
        dir_name += "_sim"

    if trained:
        dir_name += "_trained"
    if change_param is not None:
        dir_name += f"_{change_param.name}"
    if index is not None:
        dir_name += f"_{index}th"
    if validation_flag:
        dir_name += "_val"
    if use_memory_diff:
        dir_name += "_mem_diff"
    return dir_name


class DirGenerator:
    """Class to store parameters for `test_dir_name`."""

    def __init__(self, root: str):
        # create directory if not exist
        if not os.path.isdir(root):
            os.mkdir(root)
        self.root = root

    def test_dir_name(
        self,
        data_collection_mode: DataCollectionMode | None = None,
        control_type: ControlType | None = None,
        trained: bool = False,
        change_param: ChangeParam | None = None,
        index: int | None = None,
        validation_flag: bool = False,
        use_memory_diff: bool = False,
    ):
        return test_dir_name(
            root=self.root,
            data_collection_mode=data_collection_mode,
            control_type=control_type,
            trained=trained,
            change_param=change_param,
            index=index,
            validation_flag=validation_flag,
            use_memory_diff=use_memory_diff,
        )
