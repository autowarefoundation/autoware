#!/usr/bin/env python


import roslib
roslib.load_manifest('tf')
import rospy, tf
from geometry_msgs.msg import PointStamped
from math import *

pub = rospy.Publisher('point_test', PointStamped)
rospy.init_node('point_test')
r = rospy.Rate(10)
br = tf.TransformBroadcaster()

count = 0;
while not rospy.is_shutdown():
    t = count/10.0
    br.sendTransform((3*sin(t/10),3*cos(t/10),sin(t/20)),
                     tf.transformations.quaternion_from_euler(0,0,0),
                     rospy.Time.now(),
                     "/map", "/point")
    msg = PointStamped();
    msg.header.frame_id = "/point"
    msg.header.stamp = rospy.Time.now();
    msg.point.x = 2*sin(t);
    msg.point.y = 2*cos(t);
    msg.point.z = 0;
    print msg
    pub.publish(msg);
    r.sleep()
    count += 1;

