#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import TorusArray, Torus
from geometry_msgs.msg import Pose
import math

rospy.init_node("test_torus")
p = rospy.Publisher("test_torus", TorusArray)
r = rospy.Rate(5)
counter = 0
while not rospy.is_shutdown():
  torus_array = TorusArray()
  torus1 = Torus()
  torus1.header.frame_id = "base_link"
  torus1.large_radius = 4
  torus1.small_radius = 1
  p1 = Pose()
  p1.position.x = 2
  p1.position.z = 4
  p1.orientation.x = math.sqrt(0.5)
  p1.orientation.y = math.sqrt(0.5)
  torus1.pose = p1

  torus2 = Torus()
  torus2.header.frame_id = "base_link"
  torus2.large_radius = 5
  torus2.small_radius = 1
  p2 = Pose()
  p2.position.x = 3
  p2.position.y = -4
  p2.orientation.z = math.sqrt(0.5)
  p2.orientation.y = math.sqrt(0.5)
  torus2.pose = p2

  torus_array.header.frame_id = "base_link"

  torus_array.toruses.append(torus1)
  torus_array.toruses.append(torus2)

  p.publish(torus_array)
  r.sleep()
