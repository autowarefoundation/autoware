#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from jsk_recognition_msgs.msg import HistogramWithRange, HistogramWithRangeBin
import numpy as np

if __name__ == "__main__":
    rospy.init_node("sample_hist_pub")
    pub = rospy.Publisher("normal_array", Float32MultiArray)
    pub_range = rospy.Publisher("range_array", HistogramWithRange)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        data = np.random.normal(size=1000)
        range_msg = HistogramWithRange()
        hist, bins = np.histogram(data, bins=50)
        for v, min, max in zip(hist, bins[:-1], bins[1:]):
            msg_bin = HistogramWithRangeBin()
            msg_bin.max_value = max
            msg_bin.min_value = min
            msg_bin.count = v
            range_msg.bins.append(msg_bin)
        msg = Float32MultiArray()
        msg.data = hist
        pub.publish(msg)
        pub_range.publish(range_msg)
        r.sleep()
