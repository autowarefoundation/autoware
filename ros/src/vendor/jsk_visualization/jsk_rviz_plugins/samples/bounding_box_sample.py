#!/usr/bin/env python


import rospy

from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from tf.transformations import *
rospy.init_node("bbox_sample")
pub = rospy.Publisher("bbox", BoundingBoxArray)
r = rospy.Rate(24)
counter = 0
while not rospy.is_shutdown():
  box_a = BoundingBox()
  box_b = BoundingBox()
  box_a.label = 2
  box_b.label = 5
  box_arr = BoundingBoxArray()
  now = rospy.Time.now()
  box_a.header.stamp = now
  box_b.header.stamp = now
  box_arr.header.stamp = now
  box_a.header.frame_id = "map"
  box_b.header.frame_id = "map"
  box_arr.header.frame_id = "map"
  q = quaternion_about_axis((counter % 100) * math.pi * 2 / 100.0, [0, 0, 1])
  box_a.pose.orientation.x = q[0]
  box_a.pose.orientation.y = q[1]
  box_a.pose.orientation.z = q[2]
  box_a.pose.orientation.w = q[3]
  box_b.pose.orientation.w = 1
  box_b.pose.position.y = 2
  box_b.dimensions.x = (counter % 10 + 1) * 0.1
  box_b.dimensions.y = ((counter + 1) % 10 + 1) * 0.1
  box_b.dimensions.z = ((counter + 2) % 10 + 1) * 0.1
  box_a.dimensions.x = 1
  box_a.dimensions.y = 1
  box_a.dimensions.z = 1
  box_a.value = (counter % 100) / 100.0
  box_b.value = 1 - (counter % 100) / 100.0
  box_arr.boxes.append(box_a)
  box_arr.boxes.append(box_b)
  pub.publish(box_arr)
  r.sleep()
  counter = counter + 1
  
