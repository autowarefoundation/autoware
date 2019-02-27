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
from autoware_system_msgs.msg import DiagnosticStatusArray
from threading import Lock


class DiagBuffer:

    def __init__(self, key, type, description, buffer_length):
        self.key_ = key
        self.type = type
        self.description = description
        self.__buffer = {autoware_health_checker.LEVEL_FATAL:DiagnosticStatusArray(),\
                        autoware_health_checker.LEVEL_ERROR:DiagnosticStatusArray(),\
                        autoware_health_checker.LEVEL_WARN:DiagnosticStatusArray(),\
                        autoware_health_checker.LEVEL_OK:DiagnosticStatusArray(),\
                        autoware_health_checker.LEVEL_UNDEFINED:DiagnosticStatusArray()}
        self.__buffer_length = rospy.Duration(buffer_length)
        self.__lock = Lock()

    def addDiag(self, status):
        with self.__lock:
            self.__buffer[status.level].status.append(status)
            self.updateBuffer()

    def getAndClearData(self):
        with self.__lock:
            data = self.__buffer[autoware_health_checker.LEVEL_FATAL]
            data.status.extend(self.__buffer[autoware_health_checker.LEVEL_ERROR].status)
            data.status.extend(self.__buffer[autoware_health_checker.LEVEL_WARN].status)
            data.status.extend(self.__buffer[autoware_health_checker.LEVEL_OK].status)
            data.status.extend(self.__buffer[autoware_health_checker.LEVEL_UNDEFINED].status)
            data.status.sort(cmp=self.isOlderTimestamp)
            self.__buffer = {autoware_health_checker.LEVEL_FATAL:DiagnosticStatusArray(),\
                            autoware_health_checker.LEVEL_ERROR:DiagnosticStatusArray(),\
                            autoware_health_checker.LEVEL_WARN:DiagnosticStatusArray(),\
                            autoware_health_checker.LEVEL_OK:DiagnosticStatusArray(),\
                            autoware_health_checker.LEVEL_UNDEFINED:DiagnosticStatusArray()} # self.__buffer.clear()
            return data

    def getErrorLevel(self):
        with self.__lock:
            self.updateBuffer()
            if len(self.__buffer[autoware_health_checker.LEVEL_FATAL].status) != 0:
                return autoware_health_checker.LEVEL_FATAL
            elif len(self.__buffer[autoware_health_checker.LEVEL_ERROR].status) != 0:
                return autoware_health_checker.LEVEL_ERROR
            elif len(self.__buffer[autoware_health_checker.LEVEL_WARN].status) != 0:
                return autoware_health_checker.LEVEL_WARN
            else:
                return autoware_health_checker.LEVEL_OK

    # // filter data from timestamp and level
    def filterBuffer(self, now, level):
        filterd_data = DiagnosticStatusArray()
        ret = DiagnosticStatusArray()
        if self.__buffer.keys().count(level) != 0:
            filterd_data = self.__buffer[level]
        for data_itr in filterd_data.status:
            if data_itr.header.stamp > (now - self.__buffer_length):
                ret.status.append(data_itr)
        return ret

    def updateBuffer(self):
        now = rospy.Time.now()
        self.__buffer[autoware_health_checker.LEVEL_FATAL] = self.filterBuffer(now, autoware_health_checker.LEVEL_FATAL)
        self.__buffer[autoware_health_checker.LEVEL_ERROR] = self.filterBuffer(now, autoware_health_checker.LEVEL_ERROR)
        self.__buffer[autoware_health_checker.LEVEL_WARN] = self.filterBuffer(now, autoware_health_checker.LEVEL_WARN)
        self.__buffer[autoware_health_checker.LEVEL_OK] = self.filterBuffer(now, autoware_health_checker.LEVEL_OK)
        self.__buffer[autoware_health_checker.LEVEL_UNDEFINED] = self.filterBuffer(now, autoware_health_checker.LEVEL_UNDEFINED)

    def isOlderTimestamp(self, a, b):
        return cmp(a.header.stamp, b.header.stamp)
