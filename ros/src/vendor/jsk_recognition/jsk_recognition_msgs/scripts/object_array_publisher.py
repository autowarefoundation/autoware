#!/usr/bin/env python

import rospy
import genpy
from jsk_recognition_msgs.msg import Object
from jsk_recognition_msgs.msg import ObjectArray


class ObjectArrayPublisher(object):

    def __init__(self):
        objects = rospy.get_param('~objects')
        self.msg = ObjectArray()
        for obj in objects:
            obj_msg = Object()
            genpy.message.fill_message_args(obj_msg, obj)
            self.msg.objects.append(obj_msg)

        latch = rospy.get_param('~latch', False)
        self._pub = rospy.Publisher('~output', ObjectArray, queue_size=1,
                                    latch=True)
        self._timer = rospy.Timer(rospy.Duration(1), self._timer_cb,
                                  oneshot=latch)

    def _timer_cb(self, event):
        self.msg.header.stamp = event.current_real
        self._pub.publish(self.msg)


if __name__ == '__main__':
    rospy.init_node('object_array_publisher')
    app = ObjectArrayPublisher()
    rospy.spin()
