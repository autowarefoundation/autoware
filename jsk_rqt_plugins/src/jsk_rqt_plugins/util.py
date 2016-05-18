#!/usr/bin/env python
# -*- coding: utf-8 -*-

from roslib.message import get_message_class


def get_slot_type_field_names(msg, slot_type, field_name=None, found=None):
    if field_name is None:
        field_name = ''
    if found is None:
        found = []
    if msg is None:
        return []

    for slot, slot_t in zip(msg.__slots__, msg._slot_types):
        deeper_field_name = field_name + '/' + slot
        if slot_t == slot_type:
            found.append(deeper_field_name)
        elif slot_t == slot_type + '[]':
            # supports array of type field like string[]
            deeper_field_name += '[]'
            found.append(deeper_field_name)
        try:
            if slot_t.endswith('[]'):
                # supports array of ros message like std_msgs/Header[]
                deeper_field_name += '[]'
                slot_t = slot_t.rstrip('[]')
            msg_impl = get_message_class(slot_t)
        except ValueError:
            continue
        found = get_slot_type_field_names(msg_impl, slot_type,
                                          deeper_field_name, found)
    return found
