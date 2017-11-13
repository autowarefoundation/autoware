#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String, Header
import message_filters
from visualization_msgs.msg import Marker
import math
import time
import pcl
import sys

import pprint
pp = pprint.PrettyPrinter(indent=2).pprint


class MarkerDownSampler(object):
    def __init__(self):
        self.__previous_time = time.time()
        self.__period = None  # [sec]

    def callback(self, raw_data, republisher):
        current_time = time.time()
        if self.__period < current_time - self.__previous_time:
            self.__previous_time += (1+int((current_time - self.__previous_time)/self.__period)) * self.__period
            republisher.publish(raw_data)

    def setup(self, topic_name, period=1.0):
        self.__period = period
        topic_name = topic_name if topic_name[0] != "/" else topic_name[1:]
        marker_subscriber = message_filters.Subscriber('/{}'.format(topic_name), Marker)
        marker_republisher = rospy.Publisher('/downsampled_{}'.format(topic_name), Marker, queue_size=3)

        marker_subscriber.registerCallback(self.callback, marker_republisher)


if __name__ == '__main__':
    rospy.init_node("marker_downsampler", anonymous=True)
    MarkerDownSampler().setup("/trajectory_circle_mark", 1.5)
    MarkerDownSampler().setup("/next_target_mark")
    rospy.spin()
