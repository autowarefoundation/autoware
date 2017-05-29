#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import OverlayMenu

rospy.init_node("test_menu")
p = rospy.Publisher("test_menu", OverlayMenu)
r = rospy.Rate(5)
counter = 0
while not rospy.is_shutdown():
  menu = OverlayMenu()
  menu.title = "The Beatles"
  menu.menus = ["John Lennon", "Paul McCartney", "George Harrison",
                "Ringo Starr"]
  menu.current_index = counter % len(menu.menus)
  if counter % 100 == 0:
    menu.action = OverlayMenu.ACTION_CLOSE
  p.publish(menu)
  counter = counter + 1
  r.sleep()
