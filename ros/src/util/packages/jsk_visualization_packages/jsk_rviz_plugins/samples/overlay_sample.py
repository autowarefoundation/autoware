#!/usr/bin/env python
try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

from std_msgs.msg import ColorRGBA, Float32
import rospy
import math
rospy.init_node("overlay_sample")

text_pub = rospy.Publisher("text_sample", OverlayText)
value_pub = rospy.Publisher("value_sample", Float32)
counter = 0
rate = 100
r = rospy.Rate(rate)
import random, math
while not rospy.is_shutdown():
  counter = counter + 1
  text = OverlayText()
  theta = counter % 255 / 255.0
  text.width = 400
  text.height = 600
  #text.height = 600
  text.left = 10
  text.top = 10
  text.text_size = 12
  text.line_width = 2
  text.font = "DejaVu Sans Mono"
  text.text = """This is OverlayText plugin.
The update rate is %d Hz.
You can write several text to show to the operators.
New line is supported and automatical wrapping text is also supported.
And you can choose font, this text is now rendered by '%s'

You can specify background color and foreground color separatelly.

Of course, the text is not needed to be fixed, see the counter: %d.
  """ % (rate, text.font, counter)
  text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
  text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
  text_pub.publish(text)
  value_pub.publish(math.sin(counter * math.pi * 2 / 100))
  r.sleep()

