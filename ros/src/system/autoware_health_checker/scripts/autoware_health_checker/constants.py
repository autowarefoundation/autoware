#!/usr/bin/python
# -*- coding: utf-8 -*-

#
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
from autoware_system_msgs.msg import DiagnosticStatus


class autoware_health_checker:
    LEVEL_UNDEFINED = DiagnosticStatus.UNDEFINED
    LEVEL_OK = DiagnosticStatus.OK
    LEVEL_WARN = DiagnosticStatus.WARN
    LEVEL_ERROR = DiagnosticStatus.ERROR
    LEVEL_FATAL = DiagnosticStatus.FATAL

    TYPE_UNDEFINED = DiagnosticStatus.UNDEFINED
    TYPE_OUT_OF_RANGE = DiagnosticStatus.OUT_OF_RANGE
    TYPE_RATE_IS_SLOW = DiagnosticStatus.RATE_IS_SLOW

    BUFFER_LENGTH = 5.0
    UPDATE_RATE = 10.0
