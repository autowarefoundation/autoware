#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo

class RelayCameraInfo():
    def __init__(self):
        rospy.init_node('relay_camera_info')
        self.frame_id = rospy.get_param('~frame_id')
        self.pub = rospy.Publisher("output", CameraInfo)
        rospy.Subscriber("input", CameraInfo, self.callback)
        rospy.spin()

    def callback(self, info):
        info.header.frame_id = self.frame_id
        self.pub.publish(info)

if __name__ == '__main__':
    RelayCameraInfo()
