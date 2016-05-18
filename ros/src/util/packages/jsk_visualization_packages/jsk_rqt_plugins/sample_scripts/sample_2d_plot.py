#!/usr/bin/env python

from jsk_recognition_msgs.msg import PlotData
import numpy as np
import rospy
import math

if __name__ == "__main__":
    rospy.init_node("sample_2d_plot")
    pub = rospy.Publisher("~output", PlotData)
    r = rospy.Rate(10)
    offset = 0
    while not rospy.is_shutdown():
        msg = PlotData()
        msg.xs = np.arange(0, 5, 0.1)
        msg.ys = [math.sin(x + offset) for x in msg.xs]
        pub.publish(msg)
        offset = offset + 0.1
        r.sleep()
