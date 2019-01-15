#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import dynamic_reconfigure.server
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from jsk_topic_tools import ConnectionBasedTransport
from jsk_topic_tools.log_utils import logerr_throttle
from jsk_recognition_utils.cfg import PoseArrayToPoseConfig
import rospy


class PoseArrayToPose(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.pub = self.advertise('~output', PoseStamped, queue_size=1)
        dynamic_reconfigure.server.Server(PoseArrayToPoseConfig,
                                          self._config_callback)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', PoseArray, self._convert)

    def unsubscribe(self):
        self.sub.unregister()

    def _config_callback(self, config, level):
        self.index = config.index
        return config

    def _convert(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        if self.index < 0:
            return
        elif self.index < len(msg.poses):
            pose_msg.pose = msg.poses[self.index]
            self.pub.publish(pose_msg)
        else:
            logerr_throttle(10,
                            'Invalid index {} is specified '
                            'for pose array whose size is {}'
                            .format(self.index, len(msg.poses)))


if __name__ == '__main__':
    rospy.init_node('pose_array_to_pose')
    PoseArrayToPose()
    rospy.spin()
