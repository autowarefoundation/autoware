#!/usr/bin/env python

import sys
import rospy
import std_msgs.msg

if __name__ == "__main__":
	s = sys.argv[1] if len(sys.argv) > 1 else 'hello'
	rospy.init_node('test_pub', anonymous=True)
	pub = rospy.Publisher('to_rtmgr', std_msgs.msg.String, queue_size=10)
	r = rospy.Rate(10)
	r.sleep()
        pub.publish(s)
	r.sleep()

# EOF
