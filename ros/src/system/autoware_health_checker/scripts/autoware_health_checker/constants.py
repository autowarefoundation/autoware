#!/usr/bin/python
# -*- coding: utf-8 -*-

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
