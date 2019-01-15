#!/usr/bin/env python
try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, Float32
import rospy

text_pub = rospy.Publisher("output_text", OverlayText)

def cloud_cb(cloud):
  point_num = cloud.width * cloud.height

  point_type = ""
  for field in cloud.fields:
    point_type +=field.name
    frame_id = cloud.header.frame_id

  text = OverlayText()
  text.width = 500
  text.height = 80
  text.left = 10
  text.top = 10
  text.text_size = 12
  text.line_width = 2
  text.font = "DejaVu Sans Mono"
  text.text = """Point Cloud Num : %d
PointType       : %s
PointFrame      : %s
""" % (point_num, point_type, frame_id)
  text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
  text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
  text_pub.publish(text)


if __name__ == "__main__":
  rospy.init_node("pointcloud_information_text")
  rospy.Subscriber("input", PointCloud2, cloud_cb)
  rospy.spin()
