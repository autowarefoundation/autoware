#!/usr/bin/env python

from jsk_recognition_msgs.msg import PlotData
import numpy as np
import rospy
from numpy.random import *
import math

if __name__ == "__main__":
    rospy.init_node("sample_2d_plot")
    pub = rospy.Publisher("~output", PlotData)
    r = rospy.Rate(10)
    offset = 0
    while not rospy.is_shutdown():
        msg = PlotData()
        msg.xs = randn(50)
        msg.ys = randn(50)

        pub.publish(msg)
        offset = offset + 0.1
        r.sleep()
