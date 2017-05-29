#!/usr/bin/env python

import os
import sys

import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", 
                                                "scripts")))

import rospy

try:
    from std_msgs.msg import String
except:
    import roslib; roslib.load_manifest("jsk_topic_tools")
    from std_msgs.msg import String


class TestBlock(unittest.TestCase):
    input_msg = None
    subscriber = None
    def reset_subscribers(self):
        global running
        if self.subscriber:
            self.subscriber.unregister()
            seff.subscriber = None
        self._input_msg = None
        running = False
    def cb(self, msg):
        self.input_msg = msg
    def test_no_(self):
        global running
        # do not subscribe
        self.reset_subscribers()
        rospy.loginfo("wait 10 seconds...")
        rospy.sleep(10)
        self.assertTrue(self.input_msg == None)
    def test_sub(self):
        global running
        self.reset_subscribers()
        self.subscribe = rospy.Subscriber("/output", String, self.cb)
        rospy.loginfo("wait 10 seconds...")
        rospy.sleep(10)
        self.assertFalse(self.input_msg == None)

running = False
output_original_pub = None
def topic_cb(msg):
    global running
    running = True
    output_original_pub.publish(msg)

if __name__ == "__main__":
    import rostest
    rospy.init_node("test_blocke")
    output_original_pub = rospy.Publisher("/output_original", String)
    s = rospy.Subscriber("/input_original", String, topic_cb)
    rostest.rosrun("jsk_topic_tools", "test_block", TestBlock)

