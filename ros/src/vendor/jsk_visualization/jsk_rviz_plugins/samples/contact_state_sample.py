#!/usr/bin/env python

import rospy
from hrpsys_ros_bridge.msg import ContactState, ContactStateStamped, ContactStatesStamped
from random import random
if __name__ == "__main__":
    rospy.init_node("contact_state_sample")
    pub = rospy.Publisher("~output", ContactStatesStamped)
    link_names = rospy.get_param("~links", ["r_shoulder_pan_link"])
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        states = ContactStatesStamped()
        states.header.stamp = rospy.Time.now()
        for link_name in link_names:
            state = ContactStateStamped()
            state.header.frame_id = link_name
            state.header.stamp = rospy.Time.now()
            if random() < 0.5:
                state.state.state = ContactState.ON
            else:
                state.state.state = ContactState.OFF
            states.states.append(state)
        pub.publish(states)
        rate.sleep()
