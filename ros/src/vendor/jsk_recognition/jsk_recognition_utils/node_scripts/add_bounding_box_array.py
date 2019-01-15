#!/usr/bin/env python

from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy


class AddBoundingBoxArray(ConnectionBasedTransport):
    def __init__(self):
        super(AddBoundingBoxArray, self).__init__()
        self._pub = self.advertise('~output', BoundingBoxArray, queue_size=1)

    def subscribe(self):
        topics = rospy.get_param('~topics')
        self.subs = []
        for tp in topics:
            sub = message_filters.Subscriber(tp, BoundingBoxArray)
            self.subs.append(sub)
        use_async = rospy.get_param('~approximate_sync', False)
        queue_size = rospy.get_param('~queue_size', 100)
        if use_async:
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, queue_size, slop)
        else:
            sync = message_filters.TimeSynchronizer(self.subs, queue_size)
        sync.registerCallback(self._decompose)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _decompose(self, *bboxes_msgs):
        out_msg = BoundingBoxArray()
        out_msg.header = bboxes_msgs[0].header
        for bboxes_msg in bboxes_msgs:
            out_msg.boxes.extend(bboxes_msg.boxes)
        self._pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('add_bounding_box_array')
    app = AddBoundingBoxArray()
    rospy.spin()
