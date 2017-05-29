#!/usr/bin/env python
# -*- coding: utf-8 -*-

import inspect

import rospy


def _log_msg_with_called_location(msg):
    try:
        return '[{cls}::{method}] {msg}'.format(
            cls=inspect.stack()[2][0].f_locals['self'].__class__.__name__,
            method=inspect.stack()[2][0].f_code.co_name,
            msg=msg)
    except KeyError:
        return msg


def jsk_logdebug(msg):
    rospy.loginfo(_log_msg_with_called_location(msg))


def jsk_loginfo(msg):
    rospy.loginfo(_log_msg_with_called_location(msg))


def jsk_logwarn(msg):
    rospy.logwarn(_log_msg_with_called_location(msg))


def jsk_logerr(msg):
    rospy.logerr(_log_msg_with_called_location(msg))


def jsk_logfatal(msg):
    rospy.logfatal(_log_msg_with_called_location(msg))
