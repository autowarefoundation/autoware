#!/usr/bin/env python
import os
import sys

import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", 
                                                "scripts")))
from topic_compare import ROSTopicCompare

import rospy
import time

import numpy as np

try:
    from std_msgs.msg import Float32
except:
    import roslib; roslib.load_manifest("jsk_topic_tools")
    from std_msgs.msg import Float32

def eps_equal(a, b, err=0.001):
    return abs(a - b) < err

hz_msg = None

def topic_cb(msg):
    global hz_msg
    hz_msg = msg


class TestHzMeasure(unittest.TestCase):
    def test_hz(self):
        global hz_msg
        while hz_msg == None:
            rospy.loginfo('wait for msg...')
            if not rospy.is_shutdown():
                rospy.sleep(5.0)          #wait for 5 sec
        # should be 30Hz
        msgs = []
        while len(msgs) < 30:
            msgs.append(hz_msg.data)
            # rospy.loginfo('hz of msg %s'%hz_msg.data)
            rospy.sleep(0.1)
        hz = np.median(msgs)
        rospy.loginfo('average hz of msg %s'%np.mean(msgs))
        rospy.loginfo('median  hz of msg %s'%hz)
        self.assertTrue(eps_equal(hz, 30, 1))

if __name__ == "__main__":
    import rostest
    rospy.init_node("test_hz_measure")
    s = rospy.Subscriber("/hz/output", Float32, topic_cb)
    rostest.rosrun("jsk_topic_tools", "test_hz_measure", TestHzMeasure)
