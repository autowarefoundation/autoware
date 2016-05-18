#!/usr/bin/env python

import rospy
import tf
from tf.transformations import *
import numpy as np
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA
rospy.init_node("tf_diff")

src1 = rospy.get_param("~src1")
src2 = rospy.get_param("~src2")
listener = tf.TransformListener()
r = rospy.Rate(1)
pub = rospy.Publisher("diff_text", OverlayText)
while not rospy.is_shutdown():
    try:
        (pos, rot) = listener.lookupTransform(src1, src2, rospy.Time(0))
        pos_diff = np.linalg.norm(pos)
        # quaternion to rpy
        rpy = euler_from_quaternion(rot)
        rot_diff = np.linalg.norm(rpy)
        print (pos_diff, rot_diff)
        msg = OverlayText()
        msg.width = 1000
        msg.height = 200
        msg.left = 10
        msg.top = 10
        msg.text_size = 20
        msg.line_width = 2
        msg.font = "DejaVu Sans Mono"
        msg.text = """%s <-> %s
pos: %f
rot: %f
        """ % (src1, src2, pos_diff, rot_diff)
        msg.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        msg.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.0)
        pub.publish(msg)
    except:
        rospy.logerr("ignore error")
    finally:
        r.sleep()
