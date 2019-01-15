#!/usr/bin/env python
"""
Call snapshot service of rviz (provided by ScreenshotListener tool)
when a topic is published.

This script is useful to automatically record result of ros processing.

NOTE:
  rviz should be in fron of other windows because 
"""

import rospy
from jsk_rviz_plugins.srv import Screenshot

def callback(msg):
    global counter
    rospy.loginfo('received a message, save a screenshot to {0}'.format(file_format.format(counter)))
    try:
        screenshot_srv(file_format.format(counter))
        counter = counter + 1
    except rospy.ServiceException, e:
        rospy.logerr('Failed to call screenshot service call. Have you add ScreenshotListener to rviz and file_format is correct? file_format is "{0}"'.format(file_format))
        
    
if __name__ == '__main__':
    counter = 0
    rospy.init_node('relay_screenshot')
    screenshot_srv = rospy.ServiceProxy('/rviz/screenshot', Screenshot)
    file_format = rospy.get_param('~file_format', 'rviz_screenshot_{0:0>5}.png')
    sub = rospy.Subscriber('~input', rospy.msg.AnyMsg, callback)
    rospy.spin()
