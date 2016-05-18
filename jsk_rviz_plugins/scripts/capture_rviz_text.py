#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import Int32


def callback(msg):
    global pub
    count = msg.data
    text = OverlayText()
    color = (52, 152, 219)
    text.fg_color.r = color[0] / 255.0
    text.fg_color.g = color[1] / 255.0
    text.fg_color.b = color[2] / 255.0
    text.fg_color.a = 1.0
    text.bg_color.a = 0.0
    text.text = "Samples: %d" % (count)
    text.width = 500
    text.height = 100
    text.left = 10
    text.top = 10
    text.text_size = 30
    pub.publish(text)

def main():
    global pub
    pub = rospy.Publisher("capture_text", OverlayText)
    sub = rospy.Subscriber("capture_count", Int32, callback)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("capture_rviz_text")
    main()
