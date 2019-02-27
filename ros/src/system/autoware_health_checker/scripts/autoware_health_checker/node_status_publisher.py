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
from autoware_system_msgs.msg import NodeStatus
from constants import autoware_health_checker
from rate_checker import RateChecker
from autoware_system_msgs.msg import DiagnosticStatusArray
from autoware_system_msgs.msg import DiagnosticStatus
from diag_buffer import DiagBuffer
import json
import threading
from threading import Lock

class NodeStatusPublisher:
    def __init__(self):
        self.__node_activated = False
        self.__status_pub = rospy.Publisher('node_status', NodeStatus, queue_size=10)
        self.__lock = Lock()
        self.__rate_checkers = {}
        self.__diag_buffers = {}

    def publishStatus(self):
        rate = rospy.Rate(autoware_health_checker.UPDATE_RATE)
        while not rospy.is_shutdown():
            with self.__lock:
                status = NodeStatus()
                status.node_activated = self.__node_activated
                now = rospy.Time.now()
                status.header.stamp = now
                status.node_name = rospy.get_name()
                checker_keys = self.getRateCheckerKeys()
                #  iterate Rate checker and publish rate_check result
                for key_itr in checker_keys:
                    diag_array = DiagnosticStatusArray()
                    diag       = DiagnosticStatus()
                    diag.type = DiagnosticStatus.RATE_IS_SLOW
                    result = self.__rate_checkers[key_itr].getErrorLevelAndRate()
                    diag.level = result[0]
                    diag.key = key_itr
                    diag.value = self.doubleToJson(result[1])
                    diag.description = self.__rate_checkers[key_itr].description
                    diag.header.stamp = now
                    diag_array.status.append(diag)
                    status.status.append(diag_array)
                keys = self.getKeys()
                for key_itr in keys:
                    status.status.append(self.__diag_buffers[key_itr].getAndClearData())
                self.__status_pub.publish(status)
            rate.sleep()
        return

    def ENABLE(self):
        publish_thread = threading.Thread(target=self.publishStatus)
        publish_thread.start()

    def getKeys(self):
        keys = []
        checker_keys = self.getRateCheckerKeys()
        for buf_itr_first,buf_itr_second in self.__diag_buffers.items():
            matched = False
            for checker_key_itr in checker_keys:
                if checker_key_itr == buf_itr_first:
                    matched = True
            if not matched:
                keys.append(buf_itr_first)
        return keys

    def getRateCheckerKeys(self):
        keys = []
        for checker_itr_first, checker_itr_second in self.__rate_checkers.items():
            keys.append(checker_itr_first)
        return keys

    def keyExist(self, key):
        return key in self.__diag_buffers

    def addNewBuffer(self, key, type, description):
        if not self.keyExist(key):
            buf = DiagBuffer(key, type, description, autoware_health_checker.BUFFER_LENGTH)
            self.__diag_buffers[key] = buf

    def CHECK_MIN_VALUE(self, key, value, warn_value, error_value, fatal_value, description):
        self.addNewBuffer(key, DiagnosticStatus.OUT_OF_RANGE, description)
        new_status = DiagnosticStatus()
        new_status.type = DiagnosticStatus.OUT_OF_RANGE
        if value < fatal_value:
            new_status.level = DiagnosticStatus.FATAL
        elif value < error_value:
            new_status.level = DiagnosticStatus.ERROR
        elif value < warn_value:
            new_status.level = DiagnosticStatus.WARN
        else:
            new_status.level = DiagnosticStatus.OK
        new_status.description = description
        new_status.value = self.doubleToJson(value)
        new_status.header.stamp = rospy.Time.now()
        self.__diag_buffers[key].addDiag(new_status)
        return new_status.level

    def CHECK_MAX_VALUE(self, key, value, warn_value, error_value, fatal_value, description):
        self.addNewBuffer(key, DiagnosticStatus.OUT_OF_RANGE, description)
        new_status = DiagnosticStatus()
        new_status.type = DiagnosticStatus.OUT_OF_RANGE
        if value > fatal_value:
            new_status.level = DiagnosticStatus.FATAL
        elif value > error_value:
            new_status.level = DiagnosticStatus.ERROR
        elif value > warn_value:
            new_status.level = DiagnosticStatus.WARN
        else:
            new_status.level = DiagnosticStatus.OK
        new_status.description = description
        new_status.value = self.doubleToJson(value)
        new_status.header.stamp = rospy.Time.now()
        self.__diag_buffers[key].addDiag(new_status)
        return new_status.level

    def CHECK_RANGE(self,key, value, warn_value, error_value, fatal_value, description):
        self.addNewBuffer(key, DiagnosticStatus.OUT_OF_RANGE, description)
        new_status = DiagnosticStatus()
        new_status.type = DiagnosticStatus.OUT_OF_RANGE
        if value < fatal_value[0] or value > fatal_value[1]:
            new_status.level = DiagnosticStatus.FATAL
        elif value < error_value[0] or value > error_value[1]:
            new_status.level = DiagnosticStatus.ERROR
        elif value < warn_value[0] or value > warn_value[1]:
            new_status.level = DiagnosticStatus.WARN
        else:
            new_status.level = DiagnosticStatus.OK
        new_status.description = description
        new_status.value = self.doubleToJson(value)
        new_status.header.stamp = rospy.Time.now()
        self.__diag_buffers[key].addDiag(new_status)
        return new_status.level

    def CHECK_VALUE(self,key, value, check_func, value_json_func, description):
        self.addNewBuffer(key, DiagnosticStatus.OUT_OF_RANGE, description)
        check_result = check_func(value)
        pt = value_json_func(value)
        ss = json.dumps(pt)
        new_status = DiagnosticStatus()
        new_status.type = DiagnosticStatus.OUT_OF_RANGE
        new_status.level= check_result
        new_status.description = description
        new_status.description = ss
        new_status.header.stamp = rospy.Time.now()
        self.__diag_buffers[key].addDiag(new_status)
        return new_status.level


    def CHECK_RATE(self,key, warn_rate, error_rate, fatal_rate, description):
        if not self.keyExist(key):
            self.__rate_checkers[key] = RateChecker(autoware_health_checker.BUFFER_LENGTH,warn_rate,error_rate,fatal_rate,description)
        self.addNewBuffer(key,DiagnosticStatus.RATE_IS_SLOW,description)
        self.__rate_checkers[key].check()

    def NODE_ACTIVATE(self):
        with self.__lock:
            self.__node_activated = True

    def NODE_DEACTIVATE(self):
        with self.__lock:
            self.__node_activated  = False

    def getNodeStatus(self):
        return self.__node_activated

    def doubleToJson(self,value):
        return json.dumps({"value.double": value})
