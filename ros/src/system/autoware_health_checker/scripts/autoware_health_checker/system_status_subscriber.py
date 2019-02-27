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
from autoware_system_msgs.msg import SystemStatus

class SystemStatusSubscriber:
    def __init__(self):
        self.__functions=[]

    def enable(self):
        rate = rospy.Rate(1)
        self.__status_sub = rospy.Subscriber("/system_status", SystemStatus, self.systemStatusCallback)
        #while not rospy.is_shutdown():
        #    rate.sleep()
        rospy.spin()

    def systemStatusCallback(self,msg):
        for function_itr in self.__functions:
            function_itr(msg)

    def addCallback(self,func):
        self.__functions.append(func)
