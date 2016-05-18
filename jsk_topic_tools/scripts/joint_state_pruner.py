#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

class JointStatePruner():
    def __init__(self):
        rospy.init_node('joint_state_pruner', anonymous=True)
        self.pruned_pub = rospy.Publisher("/joint_states_pruned", JointState)
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        rospy.spin()


    def joint_state_callback(self,js):
        js.effort = []
        js.velocity = []
        self.pruned_pub.publish(js)

if __name__ == '__main__':
    jsp = JointStatePruner()
