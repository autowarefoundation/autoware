#!/usr/bin/env python

import os
import sys

import unittest

import rosgraph
import rospy
import rosmsg
import roslib


class TestConnection(unittest.TestCase):

    def test_no_subscribers(self):
        check_connected_topics = rospy.get_param('~check_connected_topics')
        master = rosgraph.Master('/test_connection')
        _, sub, _ = master.getSystemState()
        # Check assumed topics are not there
        master = rosgraph.Master('test_connection')
        _, subscriptions, _ = master.getSystemState()
        for check_topic in check_connected_topics:
            for topic, sub_node in subscriptions:
                if topic == rospy.get_namespace() + check_topic:
                    raise ValueError('Found topic: {}'.format(check_topic))

    def test_subscriber_appears(self):
        topic_type = rospy.get_param('~input_topic_type')
        check_connected_topics = rospy.get_param('~check_connected_topics')
        wait_time = rospy.get_param('~wait_for_connection', 0)
        msg_class = roslib.message.get_message_class(topic_type)
        # Subscribe topic and bond connection
        sub = rospy.Subscriber('~input', msg_class,
                               self._cb_test_subscriber_appears)
        print('Waiting for connection for {} sec.'.format(wait_time))
        rospy.sleep(wait_time)
        # Check assumed topics are there
        master = rosgraph.Master('test_connection')
        _, subscriptions, _ = master.getSystemState()
        for check_topic in check_connected_topics:
            for topic, sub_node in subscriptions:
                if topic == rospy.get_namespace() + check_topic:
                    break
            else:
                raise ValueError('Not found topic: {}'.format(check_topic))

    def _cb_test_subscriber_appears(self, msg):
        pass


if __name__ == "__main__":
    import rostest
    rospy.init_node('test_connection')
    rostest.rosrun("jsk_topic_tools", "test_connection", TestConnection)
