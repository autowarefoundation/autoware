#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright 2015-2019 Autoware Foundation
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

import rospy
from constants import autoware_health_checker
from threading import Lock
import copy

class RateChecker:
    def __init__(self, buffer_length, warn_rate, error_rate, fatal_rate, description):
        self.__buffer_length = buffer_length
        self.__warn_rate = warn_rate
        self.__error_rate = error_rate
        self.__fatal_rate = fatal_rate
        self.description = description
        self.__start_time = rospy.Time.now()
        self.__lock = Lock()
        self.__data = []

    def check(self):
        self.__update()
        with self.__lock:
            self.__data.append(rospy.Time.now())

    def getErrorLevelAndRate(self):
        rate = self.getRate()
        if not rate:
            ret = [autoware_health_checker.LEVEL_ERROR, 0]
        elif rate < self.__fatal_rate:
            ret = [autoware_health_checker.LEVEL_FATAL, rate]
        elif rate < self.__error_rate:
            ret = [autoware_health_checker.LEVEL_ERROR, rate]
        elif rate < self.__warn_rate:
            ret = [autoware_health_checker.LEVEL_WARN, rate]
        else:
            ret = [autoware_health_checker.LEVEL_OK, rate]
        return ret

    def getErrorLevel(self):
        rate = self.getRate()
        if not rate:
            return autoware_health_checker.LEVEL_ERROR
        elif rate < self.__fatal_rate:
            return autoware_health_checker.LEVEL_FATAL
        elif rate < self.__error_rate:
            return autoware_health_checker.LEVEL_ERROR
        elif rate < self.__warn_rate:
            return autoware_health_checker.LEVEL_WARN
        else:
            return autoware_health_checker.LEVEL_OK

    def getRate(self):
        if rospy.Time.now() - self.__start_time < rospy.Duration(self.__buffer_length):
            return None
        self.__update()
        with self.__lock:
            rate = len(self.__data)/self.__buffer_length
        return rate

    def __update(self):
        with self.__lock:
            buffer = []
            for data_itr in self.__data:
                if data_itr > rospy.Time.now() - rospy.Duration(self.__buffer_length):
                    buffer.append(data_itr)
            self.__data = buffer
        return
