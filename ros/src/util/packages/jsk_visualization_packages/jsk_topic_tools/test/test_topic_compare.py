#!/usr/bin/env python
import os
import sys

import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", 
                                                "scripts")))
from topic_compare import ROSTopicCompare

import rospy
import time

def eps_equal(a, b, err=0.001):
    return abs(a - b) < err

# subscribing three topics
#   * /origin
#   * /origin (the same topic)
#   * /half
class TestTopicCompare(unittest.TestCase):
    def test_same_topic(self):
        while not tc.isAllTopicAvailable(20):
            rospy.sleep(1)
        print tc.getTotalBytes(0) / (tc.getEndTime(0) - tc.getStartTime(0))
        print tc.getTotalBytes(1) / (tc.getEndTime(1) - tc.getStartTime(1))
        self.assertTrue(eps_equal(tc.getBandwidth(0), tc.getBandwidth(1), 20))
    def test_half_topic(self):
        while not tc.isAllTopicAvailable(20):
            rospy.sleep(1)
        print tc.getTotalBytes(0) / (tc.getEndTime(0) - tc.getStartTime(0))
        print tc.getTotalBytes(2) / (tc.getEndTime(2) - tc.getStartTime(2))
        self.assertTrue(eps_equal(tc.getBandwidth(0), 2 * tc.getBandwidth(2), 20))

if __name__ == "__main__":
    import rostest
    rospy.init_node("test_topic_compare")
    tc = ROSTopicCompare()
    tc.registerTopic("/origin")
    tc.registerTopic("/origin")
    tc.registerTopic("/half")
    rostest.rosrun("jsk_topic_tools", "test_topic_compare", TestTopicCompare)


