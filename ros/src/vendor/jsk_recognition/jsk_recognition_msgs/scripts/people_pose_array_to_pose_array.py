#!/usr/bin/env python

from geometry_msgs.msg import PoseArray
from jsk_topic_tools import ConnectionBasedTransport
import rospy

from jsk_recognition_msgs.msg import PeoplePoseArray


class PeoplePoseArrayToPoseArray(ConnectionBasedTransport):

    def __init__(self):
        super(PeoplePoseArrayToPoseArray, self).__init__()
        self.pub = self.advertise('~output', PoseArray, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', PeoplePoseArray, self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, msg):
        out_msg = PoseArray()
        for person_pose in msg.poses:
            out_msg.poses.extend(person_pose.poses)
        out_msg.header = msg.header
        self.pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('people_pose_array_to_pose_array')
    app = PeoplePoseArrayToPoseArray()
    rospy.spin()
