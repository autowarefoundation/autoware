#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2021 Tier IV, Inc. All rights reserved.
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

# config of file index
TS = "timestamp"
VEL = "velocity"
RAW_ACC = "accel"
PITCH_ACC = "pitch_comp_accel"
ACC = "final_accel"
A_PED = "accel_pedal"
B_PED = "brake_pedal"
A_PED_SPD = "accel_pedal_speed"
B_PED_SPD = "brake_pedal_speed"
PITCH = "pitch"
JERK = "jerk"
STEER = "steer"

# config of accel / brake map
VEL_LIST = np.array([0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50])  # km
PEDAL_LIST = np.array(
    [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
)
VEL_MIN = VEL_LIST[0]
VEL_MAX = VEL_LIST[-1]
VEL_SPAN = (VEL_MAX - VEL_MIN) / (len(VEL_LIST) - 1)
PEDAL_MIN = PEDAL_LIST[0]
PEDAL_MAX = PEDAL_LIST[-1]
PEDAL_SPAN = (PEDAL_MAX - PEDAL_MIN) / (len(PEDAL_LIST) - 1)
