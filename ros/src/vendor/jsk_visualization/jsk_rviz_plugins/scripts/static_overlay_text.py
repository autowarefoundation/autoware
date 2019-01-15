#!/usr/bin/env python

# it depends on jsk_rviz_plugins

import rospy
from jsk_rviz_plugins.msg import OverlayText
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface
def publish_text(event):
    text_interface.publish(str(text))

if __name__ == "__main__":
    rospy.init_node("static_overlay_text")
    text = rospy.get_param("~text")
    text_interface = OverlayTextInterface("~output")
    rospy.Timer(rospy.Duration(0.1), publish_text)
    rospy.spin()
