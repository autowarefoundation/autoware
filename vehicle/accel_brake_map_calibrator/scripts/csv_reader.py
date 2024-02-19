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

import glob

import config as CF
import numpy as np
import pandas as pd

# pre-defined
NUMERIC_LIMITS = 1e-02


class CSVReader(object):
    def __init__(self, csv, csv_type="dir"):
        if csv_type == "dir":
            csv_files = glob.glob(csv + "/*.csv")
            csv_list = []
            for cf in csv_files:
                csv_list.append(pd.read_csv(cf, engine="python"))
            self.csv_data = pd.concat(csv_list, ignore_index=True)
        else:
            self.csv_data = pd.read_csv(csv, engine="python")

    def removeUnusedData(
        self,
        min_vel_thr,
        max_steer_thr,
        max_pitch_thr,
        max_pedal_vel_thr,
        max_jerk_thr,
        remove_by_invalid_pedal=True,
        remove_by_vel=True,
        remove_by_steer=True,
        remove_by_pitch=True,
        remove_by_pedal_speed=True,
        remove_by_jerk=True,
    ):
        # remove unused data
        raw_size = len(self.csv_data)

        for i in range(0, raw_size)[::-1]:
            vel = self.csv_data[CF.VEL][i]
            steer = self.csv_data[CF.STEER][i]
            accel_pedal = self.csv_data[CF.A_PED][i]
            brake_pedal = self.csv_data[CF.B_PED][i]
            accel_pedal_speed = self.csv_data[CF.A_PED_SPD][i]
            brake_pedal_speed = self.csv_data[CF.B_PED_SPD][i]
            jerk = self.csv_data[CF.JERK][i]
            pitch = self.csv_data[CF.PITCH][i]

            # invalid pedal
            if (
                remove_by_invalid_pedal
                and accel_pedal > NUMERIC_LIMITS
                and brake_pedal > NUMERIC_LIMITS
            ):
                self.csv_data = self.csv_data.drop(i)
                continue

            # low velocity
            if remove_by_vel and vel < min_vel_thr:
                self.csv_data = self.csv_data.drop(i)
                continue

            # high steer
            if remove_by_steer and np.abs(steer) > max_steer_thr:
                self.csv_data = self.csv_data.drop(i)
                continue

            # high pitch
            if remove_by_pitch and np.abs(pitch) > max_pitch_thr:
                self.csv_data = self.csv_data.drop(i)
                continue

            # high pedal speed
            if (
                remove_by_pedal_speed
                and np.abs(accel_pedal_speed) > max_pedal_vel_thr
                or np.abs(brake_pedal_speed) > max_pedal_vel_thr
            ):
                self.csv_data = self.csv_data.drop(i)
                continue

            # max_jerk_thr
            if remove_by_jerk and np.abs(jerk) > max_jerk_thr:
                self.csv_data = self.csv_data.drop(i)
                continue

        return self.csv_data

    def getVelData(self):
        vel_data = np.array(self.csv_data[CF.VEL])
        return vel_data

    def getPedalData(self):
        pedal_data = np.array(self.csv_data[CF.A_PED]) - np.array(self.csv_data[CF.B_PED])
        return pedal_data

    def getAccData(self):
        acc_data = np.array(self.csv_data[CF.ACC])
        return acc_data

    def getPitchData(self):
        pitch_data = np.array(self.csv_data[CF.PITCH])
        return pitch_data

    def extractPedalRangeWithDelay(self, delay_step, pedal_value, pedal_diff_thr):
        csv_data = self.csv_data.reset_index(drop=True)

        for i in range(delay_step, len(self.csv_data))[::-1]:
            pedal = csv_data[CF.A_PED][i - delay_step] - csv_data[CF.B_PED][i - delay_step]
            # extract data of pedal = pedal_value
            if pedal > pedal_value + pedal_diff_thr or pedal < pedal_value - pedal_diff_thr:
                csv_data = csv_data.drop(i)
                continue

        return csv_data
