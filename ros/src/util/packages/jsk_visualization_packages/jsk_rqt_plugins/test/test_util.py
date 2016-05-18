#!/usr/bin/env python
# -*- coding: utf-8 -*-

from nose.tools import assert_equal

from roslib.message import get_message_class
from jsk_rqt_plugins.util import get_slot_type_field_names


def test_get_slot_type_field_names():
    # test for type as slot_type
    msg = get_message_class('jsk_rviz_plugins/OverlayText')
    field_names = get_slot_type_field_names(msg, slot_type='string')
    assert_equal(field_names, ['/font', '/text'])
    # test for msg as slot_type
    field_names = get_slot_type_field_names(msg,
                                            slot_type='std_msgs/ColorRGBA')
    assert_equal(field_names, ['/bg_color', '/fg_color'])
    # test for type array
    msg = get_message_class('jsk_recognition_msgs/Histogram')
    field_names = get_slot_type_field_names(msg, slot_type='float64[]')
    assert_equal(field_names, ['/histogram'])
    # test for msg array
    msg = get_message_class('diagnostic_msgs/DiagnosticArray')
    field_names = get_slot_type_field_names(msg, slot_type='string')
    assert_equal(field_names, ['/header/frame_id', '/status[]/name',
                               '/status[]/message', '/status[]/hardware_id',
                               '/status[]/values[]/key',
                               '/status[]/values[]/value'])
